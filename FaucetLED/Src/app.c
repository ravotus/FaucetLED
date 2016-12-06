#include <limits.h>
#include <math.h>
#include <string.h>
#include "arm_math.h"
#include "arm_const_structs.h"
#include "cmsis_os.h"

#include "app.h"
#include "drivers/ds18b20.h"
#include "drivers/led.h"
#include "main.h"

#include "stm32l4xx_hal_crc.h"

// No idea why this isn't included in any headers from ST...
#define VREFINT_CAL	((volatile uint16_t*)0x1FFF75AAU)

#define NOTIFY_ADC_COMPLETE			0x01
#define NOTIFY_ADC_INJ_COMPLETE		0x02

static TaskHandle_t adc_task_handle = NULL;

volatile static int16_t adc_data[NUM_ADC_SAMPLES];
static float adc_data_f[NUM_ADC_SAMPLES];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	BaseType_t higher_prio_task_woken = pdFALSE;

	if (adc_task_handle)
	{
		xTaskNotifyFromISR(adc_task_handle, NOTIFY_ADC_COMPLETE,
				eSetBits, &higher_prio_task_woken);
		portYIELD_FROM_ISR(higher_prio_task_woken);
	}
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	BaseType_t higher_prio_task_woken = pdFALSE;

	if (adc_task_handle)
	{
		xTaskNotifyFromISR(adc_task_handle, NOTIFY_ADC_INJ_COMPLETE,
				eSetBits, &higher_prio_task_woken);
		portYIELD_FROM_ISR(higher_prio_task_woken);
	}
}

void AdcReaderTask(const void *arg)
{
	uint32_t last_wake_time;
	uint32_t task_notify_val;
	uint32_t vrefint_value;
	float vdda_value;

	struct led_color red = {.red = 128, .green = 0, .blue = 0};
	struct led_color blue = {.red = 0, .green = 0, .blue = 128};
	struct led_color green = {.red = 0, .green = 128, .blue = 0};
	struct led_color black = {.red = 0, .green = 0, .blue = 0};

	adc_task_handle = xTaskGetCurrentTaskHandle();
	led_init(1000);

	if (HAL_ADCEx_Calibration_Start(&ADC_DEV, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	if (HAL_ADCEx_InjectedStart_IT(&ADC_DEV) != HAL_OK)
	{
		Error_Handler();
	}

	if (xTaskNotifyWait(0, ULONG_MAX, &task_notify_val, 100))
	{
		if (task_notify_val != NOTIFY_ADC_INJ_COMPLETE)
		{
			Error_Handler();
		}
	}

	// Since all ADC data is left-aligned, we need to convert it back to a right-aligned integer.
	vrefint_value = (HAL_ADCEx_InjectedGetValue(&ADC_DEV, ADC_INJECTED_RANK_1) >> 4) & 0x0fff;

	// Calculate the value of the internal reference.
	vdda_value = 3.0 * (*VREFINT_CAL) / vrefint_value;

	last_wake_time = osKernelSysTick();

	while (1)
	{
		if (HAL_ADC_Start_DMA(&ADC_DEV, (uint32_t *)adc_data, NUM_ADC_SAMPLES) != HAL_OK)
		{
			Error_Handler();
		}

		if (xTaskNotifyWait(0, ULONG_MAX, &task_notify_val, 90))
		{
			if (task_notify_val != NOTIFY_ADC_COMPLETE)
			{
				Error_Handler();
			}
		}

		// Avoid having to shift all the data left by 3 here by configuring the
		// ADC in left-aligned data mode with offset enabled but set to 0.
		// This produces a left-aligned 12-bit integer with one sign-extend bit,
		// for 13 bits total. arm_q15_to_float() divides by 32768 (right shift 3).
		arm_q15_to_float((int16_t *)adc_data, (float *)adc_data_f,
				         NUM_ADC_SAMPLES);
		arm_scale_f32((float *)adc_data_f, vdda_value,
				      (float *)adc_data_f, NUM_ADC_SAMPLES);

		float stddev_v;
		arm_std_f32((float *)adc_data_f, NUM_ADC_SAMPLES, &stddev_v);

		// Relatively arbitrary levels, just for testing.
		if (stddev_v > 0.2)
		{
			led_set(&blue);
		}
		else if (stddev_v > 0.125)
		{
			led_set(&green);
		}
		else if (stddev_v > 0.05)
		{
			led_set (&red);
		}
		else
		{
			led_set(&black);
		}

		osDelayUntil(&last_wake_time, 100);
	}
}


static uint8_t ds18b20_rom_buf[DS18B20_READ_ROM_BUF_LEN];

void TemperatureTask(const void *arg)
{
	uint32_t last_wake_time;
	float temp_C;
	volatile enum ds18b20_error err;

	if (DS18B20_EOK != ds18b20_init(&DS18B20_UART, &CRC_DEV))
	{
		Error_Handler();
	}

	last_wake_time = osKernelSysTick();

	while(1)
	{
		memset(ds18b20_rom_buf, 0, sizeof(ds18b20_rom_buf));
		err = ds18b20_read_rom(ds18b20_rom_buf, DS18B20_READ_ROM_BUF_LEN);
		err = ds18b20_read_temp(&temp_C);
		(void)err;
		osDelayUntil(&last_wake_time, 1000);
	}
}
