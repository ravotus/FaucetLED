#include <limits.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "arm_math.h"
#include "arm_const_structs.h"
#include "cmsis_os.h"

#include "app.h"
#include "drivers/led.h"
#include "main.h"

// No idea why this isn't included in any headers from ST...
#define VREFINT_CAL	((volatile uint16_t*)0x1FFF75AAU)

#define NOTIFY_ADC_COMPLETE			0x01

static TaskHandle_t adc_task_handle = NULL;
static uint32_t adc_cal_value;

volatile static int16_t adc_data[NUM_ADC_SAMPLES];
static float adc_data_f[NUM_ADC_SAMPLES];

float piezo_samples[NUM_SAMPLES_PIEZO_CAL];

static const struct led_color green = {
	.red = 0,
	.green = 128,
	.blue = 0
};

inline void piezo_amp_enable(void)
{
	// Enable the external opamp and give it some time to settle. It needs 3.5us min.
	HAL_GPIO_WritePin(AMP_SHDN_GPIO_Port, AMP_SHDN_Pin, GPIO_PIN_SET);
	// Start the internal opamp which buffers Vcc/2.
	HAL_OPAMP_Start(&OPAMP_DEV);
}

inline void piezo_amp_disable(void)
{
	HAL_GPIO_WritePin(AMP_SHDN_GPIO_Port, AMP_SHDN_Pin, GPIO_PIN_RESET);
	HAL_OPAMP_Stop(&OPAMP_DEV);
}

inline void thermistor_enable(void)
{
	HAL_GPIO_WritePin(THERM_PWR_GPIO_Port, THERM_PWR_Pin, GPIO_PIN_SET);
	// The thermistor has a long rise time due to the cable.
	osDelay(20);
}

inline void thermistor_disable(void)
{
	HAL_GPIO_WritePin(THERM_PWR_GPIO_Port, THERM_PWR_Pin, GPIO_PIN_RESET);
}

static void adc_select_channel(uint32_t channel, uint32_t sampling_time)
{
	ADC_ChannelConfTypeDef adc_channel_config;
	adc_channel_config.Channel = channel;
	adc_channel_config.Rank = 1;
	adc_channel_config.SamplingTime = sampling_time;
	adc_channel_config.SingleDiff = ADC_SINGLE_ENDED;
	adc_channel_config.OffsetNumber = ADC_OFFSET_NONE;
	adc_channel_config.Offset = 0;

	if (HAL_ADC_ConfigChannel(&ADC_DEV, &adc_channel_config) != HAL_OK)
	{
		Error_Handler();
	}
}

static void wait_for_adc_conversion(void)
{
	uint32_t task_notify_val;
	if (xTaskNotifyWait(0, NOTIFY_ADC_COMPLETE, &task_notify_val, 90))
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
}

static void adc_perform_conversion(void)
{
	if (HAL_ADC_Start_IT(&ADC_DEV) != HAL_OK)
	{
		Error_Handler();
	}
	wait_for_adc_conversion();
	(void)HAL_ADC_Stop_IT(&ADC_DEV);
}

static void adc_perform_conversion_dma(void)
{
	if (HAL_ADC_Start_DMA(&ADC_DEV, (uint32_t *)adc_data, NUM_ADC_SAMPLES) != HAL_OK)
	{
		Error_Handler();
	}
	wait_for_adc_conversion();
	(void)HAL_ADC_Stop_DMA(&ADC_DEV);
}

static float compute_sample_stdev(int16_t *samples, uint16_t adc_ref_V)
{
	// ADC data is required to be right-aligned because the injected channels
	// have oversampling enabled.
	arm_q15_to_float(samples, (float *)adc_data_f, NUM_ADC_SAMPLES);
	// We need to scale the data by 8 because the adc is 12-bit for a range
	// of [0, 4096), and arm_q15_to_float() expects a Q15 value in [0, 32768),
	// hence (32768/4096) = 8.
	arm_scale_f32((float *)adc_data_f, adc_ref_V * 8.0f,
			      (float *)adc_data_f, NUM_ADC_SAMPLES);

	// Calculate the standard deviation of the samples, which it turns out
	// is mathematically the same as calculating the RMS of the signal with
	// the DC component (ie, the mean) removed. This gives a good
	// approximation of the amplitude of the differential signal.
	float stdev_V;
	arm_std_f32((float *)adc_data_f, NUM_ADC_SAMPLES, &stdev_V);

	return stdev_V;
}

static float compute_thermistor_temp_C(uint16_t adc_counts, float adc_ref_V)
{
	// Calculation of input voltage
	float temp_input_V = adc_counts * adc_ref_V / 4096.0f;

	// Calculate thermistor R = Vdda * 10k / Vin - 10k
	float thermistor_R = adc_ref_V * THERMISTOR_R_DIVIDER / temp_input_V - THERMISTOR_R_DIVIDER;

	// Finally use Steinhart-Hart approximation
	// R_Kelvin = 1/(1/T0 + 1/B*ln(Rtherm/R_T0)), B = 3984, T0 = 25C
	float temperature_C = 1/(1/298.15f + (1.0f/THERMISTOR_B)*logf(thermistor_R / THERMISTOR_T0)) - 273.15f;

	return temperature_C;
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

void AdcReaderTask(const void *arg)
{
	uint32_t last_wake_time;
	uint32_t last_led_change;
	float piezo_cal_stdev_V = -1.0f;
	size_t piezo_cal_index = 0;
	size_t piezo_buffer_index = 0;

	adc_task_handle = xTaskGetCurrentTaskHandle();

	// ADC is already initialized and active from main().
	if (HAL_ADCEx_Calibration_Start(&ADC_DEV, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}
	adc_cal_value = HAL_ADCEx_Calibration_GetValue(&ADC_DEV, ADC_SINGLE_ENDED);
	HAL_ADC_DeInit(&ADC_DEV);

	led_set(&green);

	last_wake_time = osKernelSysTick();
	last_led_change = last_wake_time;
	piezo_amp_enable();

	while (1)
	{
		// TODO
		//piezo_amp_enable();

		extern void MX_ADC1_Init(void);
		MX_ADC1_Init();

		// Configure the internal reference channel while the ADC is disabled (required by HAL).
		adc_select_channel(ADC_CHANNEL_VREFINT, ADC_SAMPLETIME_2CYCLES_5);

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

		adc_perform_conversion();

		// Calculate the value of the internal reference.
		float adc_ref_V = 3.0f * (*VREFINT_CAL) / HAL_ADC_GetValue(&ADC_DEV);

		adc_select_channel(ADC_CHANNEL_PIEZO_AMP, ADC_SAMPLETIME_6CYCLES_5);
		adc_perform_conversion_dma();
		// TODO
		//piezo_amp_disable();

		float stdev_V = PIEZO_STDEV_SCALE_FACT * compute_sample_stdev((int16_t *)adc_data, adc_ref_V);

		if (piezo_cal_index < NUM_SAMPLES_PIEZO_CAL)
		{
			piezo_samples[piezo_cal_index] = stdev_V;
			if (++piezo_cal_index == NUM_SAMPLES_PIEZO_CAL)
			{
				arm_mean_f32(piezo_samples, NUM_SAMPLES_PIEZO_CAL, &piezo_cal_stdev_V);
				piezo_cal_stdev_V *= PIEZO_CAL_SCALE_FACT;

				memset(piezo_samples, 0, sizeof(piezo_samples));
				led_disable();
			}
		}
		else
		{
			piezo_samples[piezo_buffer_index] = stdev_V - piezo_cal_stdev_V;
			if (++piezo_buffer_index >= NUM_SAMPLES_PIEZO_HIST)
			{
				piezo_buffer_index = 0;
			}

			uint32_t ticks = osKernelSysTick();
			if ((ticks - last_led_change) > 1000)
			{
				unsigned active = 0;
				for (unsigned i=0; i<NUM_SAMPLES_PIEZO_HIST; ++i)
				{
					if (piezo_samples[i] > ADC_TRIGGER_THRESH_V)
					{
						active++;
					}
				}
				if (active >= NUM_SAMPLES_PIEZO_THRESH)
				{
					thermistor_enable();
					adc_select_channel(ADC_CHANNEL_THERMISTOR, ADC_SAMPLETIME_24CYCLES_5);
					adc_perform_conversion();
					thermistor_disable();

					float temperature_C = compute_thermistor_temp_C(HAL_ADC_GetValue(&ADC_DEV), adc_ref_V);

					struct led_color color;
					compute_led_color(temperature_C, &color);
					led_set(&color);
					last_led_change = ticks;
				}
				else if (led_get_active())
				{
					led_disable();
					last_led_change = ticks;
				}
			}
		}

		(void)HAL_ADC_DeInit(&ADC_DEV);
		(void)HAL_ADCEx_DisableVoltageRegulator(&ADC_DEV);
		(void)HAL_ADCEx_EnterADCDeepPowerDownMode(&ADC_DEV);

		osDelayUntil(&last_wake_time, 100);
	}
}
