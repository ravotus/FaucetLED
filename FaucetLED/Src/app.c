#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include "cmsis_os.h"
#include "stm32l4xx_nucleo_32.h"

#include "app.h"
#include "drivers/led.h"
#include "main.h"

// No idea why this isn't included in any headers from ST...
#define VREFINT_CAL	((volatile uint16_t*)0x1FFF75AAU)

#define FLASH_PAGE_127   ((uint32_t)0x0803F800) /* Base @ of Page 127, 2 Kbytes */

#define MAGIC_FLASH_WRITTEN 0xA5A5A5A5A5A5A5A5

#define NOTIFY_ADC_COMPLETE			0x01
#define NOTIFY_ADC_INJ_COMPLETE		0x02

static TaskHandle_t adc_task_handle = NULL;
static uint32_t adc_cal_value;

volatile static int16_t adc_data[NUM_ADC_SAMPLES];
static float adc_data_f[NUM_ADC_SAMPLES];

struct FlashData_t
{
	uint64_t flash_code;
	float piezo_rms_hist[128];
	uint32_t index;
};

volatile struct FlashData_t sample_data;

#define FLASH_DATA_IN_FLASH ((volatile struct FlashData_t *)FLASH_PAGE_127)

SleepType_E app_get_sleep_capability(void)
{
	uint32_t adc_state = HAL_ADC_GetState(&ADC_DEV);
	if ((adc_state & HAL_ADC_STATE_REG_BUSY) || (adc_state & HAL_ADC_STATE_INJ_BUSY))
	{
		return SLEEP_NONE;
	}
	else if (led_get_active())
	{
		return SLEEP_LOW_POWER;
	}
	else
	{
		return SLEEP_STOP;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	BaseType_t higher_prio_task_woken = pdFALSE;

	if (adc_task_handle)
	{
		xTaskNotifyFromISR(adc_task_handle, NOTIFY_ADC_COMPLETE,
				eSetBits, &higher_prio_task_woken);
		portYIELD_FROM_ISR(higher_prio_task_woken);
	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	BaseType_t higher_prio_task_woken = pdFALSE;

	if (adc_task_handle)
	{
		// We only want to set the complete bit once the entire sequence of injected
		// conversions is finished. If the "End of Conversion Selection" happens to be
		// set to End of Single Conversion (or both), we will get an interrupt for every
		// injected channel that completes.
		if ( __HAL_ADC_GET_FLAG(hadc, ADC_FLAG_JEOS))
		{
			xTaskNotifyFromISR(adc_task_handle, NOTIFY_ADC_INJ_COMPLETE,
					eSetBits, &higher_prio_task_woken);
		}
		portYIELD_FROM_ISR(higher_prio_task_woken);
	}
}

static void compute_led_color(float temp_C, struct led_color *output)
{
	if (!output)
	{
		return;
	}

	if (temp_C > 26.0f)
	{
		// Hot (26-60C)
		const float M = -7.5f;
		const float B = 450.0f;
		output->red = 255;
		output->green = (uint8_t)(powf(2.0f, (temp_C * M + B) * 8.0f / 255.0f) - 1);
		output->blue = output->green;

	}
	else
	{
		// Cold (10-26C)
		const float M = 15.9375f;
		const float B = -159.375f;
		output->red = (uint8_t)(powf(2.0f, (temp_C * M + B) * 8.0f / 255.0f) - 1);
		output->green = output->red;
		output->blue = 255;
	}
}

void AdcReaderTask(const void *arg)
{
	uint32_t task_notify_val;
	uint32_t last_wake_time;
	uint32_t last_led_update;
	uint8_t num_shocks_last = 0;

	adc_task_handle = xTaskGetCurrentTaskHandle();

	if (HAL_ADCEx_Calibration_Start(&ADC_DEV, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	adc_cal_value = HAL_ADCEx_Calibration_GetValue(&ADC_DEV, ADC_SINGLE_ENDED);
	HAL_ADC_DeInit(&ADC_DEV);

	if (FLASH_DATA_IN_FLASH->flash_code == MAGIC_FLASH_WRITTEN)
	{
		for (size_t i=0; i<10; ++i)
		{
			BSP_LED_Toggle(LED3);
			osDelay(500);
		}

		HAL_FLASH_Unlock();

		FLASH_EraseInitTypeDef EraseInitStruct;
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks       = FLASH_BANK_1;
		EraseInitStruct.Page        = 127;
		EraseInitStruct.NbPages     = 1;

		uint32_t err;
		if (HAL_FLASHEx_Erase(&EraseInitStruct, &err) != HAL_OK)
		{
			Error_Handler();
		}

		HAL_FLASH_Lock();
	}

	sample_data.index = 0;
	BSP_LED_On(LED3);
	osDelay(5000);
	BSP_LED_Off(LED3);

	last_wake_time = osKernelSysTick();
	last_led_update = last_wake_time;

	while (1)
	{
		// Power the thermistor to measure temperature
		HAL_GPIO_WritePin(THERM_PWR_GPIO_Port, THERM_PWR_Pin, GPIO_PIN_SET);

		// The thermistor has a long rise time due to the cable.
		osDelay(20);

		// Enable the external opamp and give it some time to settle. It needs 3.5us min.
		HAL_GPIO_WritePin(AMP_SHDN_GPIO_Port, AMP_SHDN_Pin, GPIO_PIN_SET);
		// Start the internal opamp which buffers Vcc/2.
		HAL_OPAMP_Start(&OPAMP_DEV);

		extern void MX_ADC1_Init(void);
		MX_ADC1_Init();

		// Workaround HAL bug which requires the ADC to be enabled to set the calibration
		// but yet there is no way to fully enable it without starting a conversion.
		if (ADC_Enable(&ADC_DEV) != HAL_OK)
		{
			Error_Handler();
		}

		if (HAL_ADCEx_Calibration_SetValue(&ADC_DEV, ADC_SINGLE_ENDED, adc_cal_value) != HAL_OK)
		{
			Error_Handler();
		}

		// First read the injected group which includes the internal voltage reference
		// and the external temperature sensor (thermistor).
		if (HAL_ADCEx_InjectedStart_IT(&ADC_DEV) != HAL_OK)
		{
			Error_Handler();
		}

		if (xTaskNotifyWait(0, ULONG_MAX, &task_notify_val, 90))
		{
			if (! (task_notify_val & NOTIFY_ADC_INJ_COMPLETE))
			{
				Error_Handler();
			}
		}
		else
		{
			// Timeout
			Error_Handler();
		}

		// Because oversampling is enabled for injected channels, the left align bit is ignored,
		// so this data is right aligned
		uint16_t vrefint_value = HAL_ADCEx_InjectedGetValue(&ADC_DEV, ADC_INJ_CHANNEL_VREF);
		uint16_t exttemp_value = HAL_ADCEx_InjectedGetValue(&ADC_DEV, ADC_INJ_CHANNEL_EXTTEMP);

		(void)HAL_ADCEx_InjectedStop_IT(&ADC_DEV);

		// No need for the thermistor now.
		HAL_GPIO_WritePin(THERM_PWR_GPIO_Port, THERM_PWR_Pin, GPIO_PIN_RESET);

		// Calculate the value of the internal reference.
		float vdda_V = 3.0 * (*VREFINT_CAL) / vrefint_value;

		// Calculation of input voltage
		float temp_input_V = exttemp_value * vdda_V / 4096.0f;

		// Calculate thermistor R = Vdda * 10k / Vin - 10k
		float thermistor_R = vdda_V * THERMISTOR_R_DIVIDER / temp_input_V - THERMISTOR_R_DIVIDER;

		// Finally use Steinhart-Hart approximation
		// R_Kelvin = 1/(1/T0 + 1/B*ln(Rtherm/R_T0)), B = 3984, T0 = 25C
		float temperature_C = 1/(1/298.15f + (1.0f/THERMISTOR_B)*logf(thermistor_R / THERMISTOR_T0)) - 273.15f;

		// Wait a bit to ensure any transients from disabling the thermistor supply have gone.
		osDelay(5);

		// Begin reading the shock sensor.
		if (HAL_ADC_Start_DMA(&ADC_DEV, (uint32_t *)adc_data, NUM_ADC_SAMPLES) != HAL_OK)
		{
			Error_Handler();
		}

		if (xTaskNotifyWait(0, ULONG_MAX, &task_notify_val, 90))
		{
			if (! (task_notify_val & NOTIFY_ADC_COMPLETE))
			{
				Error_Handler();
			}
		}
		else
		{
			// Timeout
			Error_Handler();
		}

		// For safety...
		(void)HAL_ADC_Stop_DMA(&ADC_DEV);

		(void)HAL_ADC_DeInit(&ADC_DEV);
		(void)HAL_ADCEx_DisableVoltageRegulator(&ADC_DEV);
		(void)HAL_ADCEx_EnterADCDeepPowerDownMode(&ADC_DEV);

		HAL_GPIO_WritePin(AMP_SHDN_GPIO_Port, AMP_SHDN_Pin, GPIO_PIN_RESET);
		HAL_OPAMP_Stop(&OPAMP_DEV);

		// ADC data is required to be right-aligned because the injected channels
		// have oversampling enabled.
		arm_q15_to_float((int16_t *)adc_data, (float *)adc_data_f,
				         NUM_ADC_SAMPLES);
		// We need to scale the data by 8 because the adc is 12-bit for a range
		// of [0, 4096), and arm_q15_to_float() expects a Q15 value in [0, 32768),
		// hence (32768/4096) = 8.
		arm_scale_f32((float *)adc_data_f, vdda_V * 8.0f,
				      (float *)adc_data_f, NUM_ADC_SAMPLES);

		// Calculate the standard deviation of the samples, which it turns out
		// is mathematically the same as calculating the RMS of the signal with
		// the DC component (ie, the mean) removed. This gives a good
		// approximation of the amplitude of the differential signal.
		float stddev_V;
		arm_std_f32((float *)adc_data_f, NUM_ADC_SAMPLES, &stddev_V);

		sample_data.piezo_rms_hist[sample_data.index] = stddev_V;
		++sample_data.index;
		if (sample_data.index == 50)
		{
			sample_data.index = 60;
			BSP_LED_On(LED3);
			osDelay(5000);
			BSP_LED_Off(LED3);
		}
		else if(sample_data.index == 110)
		{
			sample_data.flash_code = MAGIC_FLASH_WRITTEN;
			HAL_FLASH_Unlock();
			__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
			uint64_t *flash_data = (uint64_t *)&sample_data;
			for (size_t i=0; i<(sizeof(sample_data) + 1); i += sizeof(uint64_t))
			{
				if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, FLASH_PAGE_127 + i, *flash_data) != HAL_OK)
				{
					Error_Handler();
				}

				++flash_data;
			}

			HAL_FLASH_Lock();

			while(1)
			{
				BSP_LED_Toggle(LED3);
				osDelay(500);
			}
		}
#if 0
		if (stddev_V > ADC_TRIGGER_STDEV_V)
		{
			if (num_shocks_last < NUM_SAMPLES_FOR_TRIGGER)
			{
				num_shocks_last++;
			}
			else if ((osKernelSysTick() - last_led_update) > 1000)
			{
				struct led_color color;
				compute_led_color(temperature_C, &color);
				led_set(&color);
				last_led_update = osKernelSysTick();
			}
		}
		else
		{
			if (num_shocks_last > 0)
			{
				num_shocks_last--;
			}
			else
			{
				led_disable();
				last_led_update = 0;
			}
		}
#endif
		osDelayUntil(&last_wake_time, 100);
	}
}
