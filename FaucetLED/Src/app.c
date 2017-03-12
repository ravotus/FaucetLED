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
#if defined(STM32L432xx)
#define VREFINT_CAL	((volatile uint16_t*)0x1FFF75AAU)
#elif defined(STM32L052xx)
#define VREFINT_CAL	((volatile uint16_t*)0x1FF80078U)
#endif

#define NOTIFY_ADC_COMPLETE			0x01

extern QueueHandle_t TouchDataQHandle;
extern QueueHandle_t LedCmdQHandle;

static TaskHandle_t adc_task_handle = NULL;

static const struct led_color green = {
	.red = 0,
	.green = 128,
	.blue = 0
};

static inline void thermistor_enable(void)
{
	HAL_GPIO_WritePin(THERM_SW_GPIO_Port, THERM_SW_Pin, GPIO_PIN_SET);
	// The thermistor has a long rise time due to the cable.
	osDelay(20);
}

static inline void thermistor_disable(void)
{
	HAL_GPIO_WritePin(THERM_SW_GPIO_Port, THERM_SW_Pin, GPIO_PIN_RESET);
}

static void adc_select_channel(uint32_t channel)
{
	ADC_ChannelConfTypeDef adc_channel_config;
	adc_channel_config.Channel = channel;
	adc_channel_config.Rank = 1;

	// Deselect any current channels
	// TODO: Turn off vrefint?
	ADC_DEV.Instance->CHSELR &= ~ADC_CHANNEL_MASK;

	if (HAL_ADC_ConfigChannel(&ADC_DEV, &adc_channel_config) != HAL_OK)
	{
		Error_Handler();
	}
}

static void adc_perform_conversion(void)
{
	if (HAL_ADC_Start_IT(&ADC_DEV) != HAL_OK)
	{
		Error_Handler();
	}

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

	(void)HAL_ADC_Stop_IT(&ADC_DEV);
}

static uint16_t read_touch_sense(void)
{
	uint16_t tsc_value;

	HAL_TSC_IODischarge(&TSC_DEV, ENABLE);
	osDelay(1);

	if (HAL_TSC_Start_IT(&TSC_DEV) != HAL_OK)
	{
		Error_Handler();
	}

	if (! xQueueReceive(TouchDataQHandle, &tsc_value, 50))
	{
		Error_Handler();
	}

	(void)HAL_TSC_Stop_IT(&TSC_DEV);

	return tsc_value;
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
	if ((adc_state & HAL_ADC_STATE_REG_BUSY) || (adc_state & HAL_ADC_STATE_INJ_BUSY) ||
		(HAL_TSC_GroupGetStatus(&TSC_DEV, TOUCH_SENSE_GROUP) != TSC_GROUP_COMPLETED) || led_get_active())
	{
		return SLEEP_NONE;
	}
	// TODO: Low-power sleep support while LED is on.
	// When complete, remove led_get_active() call from check above.
#if 0
	else if (led_get_active())
	{
		return SLEEP_LOW_POWER;
	}
#endif
	else
	{
		return SLEEP_STOP;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	(void)hadc;
	BaseType_t higher_prio_task_woken = pdFALSE;

	if (adc_task_handle)
	{
		xTaskNotifyFromISR(adc_task_handle, NOTIFY_ADC_COMPLETE,
				eSetBits, &higher_prio_task_woken);
		portYIELD_FROM_ISR(higher_prio_task_woken);
	}
}

void HAL_TSC_ConvCpltCallback(TSC_HandleTypeDef* htsc)
{
	BaseType_t higher_prio_task_woken = pdFALSE;
	HAL_TSC_IODischarge(htsc, ENABLE);

	if (HAL_TSC_GroupGetStatus(htsc, TOUCH_SENSE_GROUP) == TSC_GROUP_COMPLETED)
	{
		if (adc_task_handle)
		{
			uint16_t tsc_val = (uint16_t)HAL_TSC_GroupGetValue(htsc, TOUCH_SENSE_GROUP);
			(void)xQueueSendFromISR(TouchDataQHandle, &tsc_val, &higher_prio_task_woken);
			portYIELD_FROM_ISR(higher_prio_task_woken);
		}
	}
}

void HAL_TSC_ErrorCallback(TSC_HandleTypeDef* htsc)
{
	// TODO
	(void)htsc;
}

uint32_t touch_cal = 0;

void AdcReaderTask(const void *arg)
{
	(void)arg;
	uint32_t last_wake_time;
	uint32_t last_led_change;
	uint16_t touch_val;
	uint32_t ticks;
	LedCmd_S led_cmd;

	adc_task_handle = xTaskGetCurrentTaskHandle();

	// ADC is already initialized and active from main().
	if (HAL_ADCEx_Calibration_Start(&ADC_DEV, ADC_SINGLE_ENDED) != HAL_OK)
	{
		Error_Handler();
	}

	for (unsigned i=0; i<TOUCH_NUM_SAMPLES_CAL; ++i)
	{
		osDelay(10);
		touch_cal += read_touch_sense();
	}

	touch_cal /= TOUCH_NUM_SAMPLES_CAL;

	led_cmd.id = LED_CMD_SET;
	led_cmd.color = green;
	(void)xQueueSend(LedCmdQHandle, &led_cmd, 0);

	last_wake_time = osKernelSysTick();
	last_led_change = last_wake_time;

	while (1)
	{
		touch_val = read_touch_sense();
		ticks = osKernelSysTick();

		// Check if the read value is at least 10% less than the calibration value.
		// TODO: Compensate for drift in touch sensor value.
		if ((touch_val < touch_cal) && ((10*touch_cal / touch_val)) >= 11)
		{
			if ((ticks - last_led_change) > 1000)
			{
				// Configure the internal reference channel while the ADC is disabled (required by HAL).
				adc_select_channel(ADC_CHANNEL_VREFINT);
				// Work around errata 2.2.1 (perform two conversions back-to-back since first may be invalid).
				adc_perform_conversion();
				adc_perform_conversion();

				// Calculate the value of the internal reference.
				float adc_ref_V = 3.0f * (*VREFINT_CAL) / HAL_ADC_GetValue(&ADC_DEV);

				thermistor_enable();
				adc_select_channel(ADC_CHANNEL_THERMISTOR);
				adc_perform_conversion();
				thermistor_disable();

				float temperature_C = compute_thermistor_temp_C(HAL_ADC_GetValue(&ADC_DEV), adc_ref_V);

				led_cmd.id = LED_CMD_FADE;
				compute_led_color(temperature_C, &led_cmd.color);
				(void)xQueueSend(LedCmdQHandle, &led_cmd, 0);
				last_led_change = ticks;
			}
		}
		else
		{
			if (led_get_active() && ((ticks - last_led_change) > 500))
			{
				led_cmd.id = LED_CMD_DISABLE;
				(void)xQueueSend(LedCmdQHandle, &led_cmd, 0);
				last_led_change = ticks;
			}
		}

		osDelayUntil(&last_wake_time, 200);
	}
}

static inline int calc_color(int old_color, int new_color, int inc, int increments)
{
	// Important: This math relies on signed values
	return old_color + (((new_color - old_color) * inc) / increments);
}

void LedTask(void const *arg)
{
	(void)arg;
	LedCmd_S command;
	struct led_color old_color, fade_color;

	while (xQueueReceive(LedCmdQHandle, &command, portMAX_DELAY))
	{
		switch (command.id)
		{
		case LED_CMD_DISABLE:
			led_disable();
			break;

		case LED_CMD_SET:
			led_set(&command.color);
			break;

		case LED_CMD_FADE:
			if (led_get_active())
			{
				led_get(&old_color);

				for (int i=1; i<=LED_FADE_INCREMENTS; ++i)
				{
					fade_color.red = calc_color(old_color.red, command.color.red, i, LED_FADE_INCREMENTS);
					fade_color.green = calc_color(old_color.green, command.color.green, i, LED_FADE_INCREMENTS);
					fade_color.blue = calc_color(old_color.blue, command.color.blue, i, LED_FADE_INCREMENTS);
					led_set(&fade_color);

					osDelay(25);
				}
			}
			else
			{
				led_set(&command.color);
			}
			break;

		default:
			break;
		}
	}
}
