/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2016, 2017 Nicholas Graumann
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

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

static volatile TouchCal_s *touch_data_store = (volatile TouchCal_s *)TOUCH_CAL_BASE;

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
	adc_channel_config.Rank = ADC_RANK_CHANNEL_NUMBER;

	// TODO: Check for and disable Vrefint if needed.
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

static float compute_thermistor_temp_C(uint16_t adc_counts)
{
	float counts_f = adc_counts;
	float temperature_C =  THERMISTOR_X2*(counts_f*counts_f) +
						   THERMISTOR_X1*(counts_f) +
						  (THERMISTOR_X0 +
						   THERMISTOR_OFFSET);
	return temperature_C;
}

static void compute_led_color(int32_t temp_C, struct led_color *output)
{
	if (!output)
	{
		return;
	}

	memset(output, 0, sizeof(*output));

	if (temp_C >= 55)
	{
		output->red = 255;
	}
	else if (temp_C >= 28)
	{
		// Linear fit 28<=x<55C to 0<x<8, then convert log scale with 2**x.
		// y = (8-0)/(55-28)*(x-28) => y = 8*(x-28)/27
		output->red = 1<<((8*(temp_C-28))/27);
	}
	else if (temp_C > 10)
	{
		// Linear fit 10<x<=27C to 8>x>0, then convert to log scale.
		// y = (0-8)/(27-10)*(x-26) => y = 8*(27-x)/17
		output->blue = 1<<(8*(27-temp_C)/17);
	}
	else
	{
		output->blue = 255;
	}
}

static void update_touch_data(TouchCal_s *touch_data, TouchCal_s *data_store_item)
{
	HAL_FLASHEx_DATAEEPROM_Unlock();
	HAL_FLASHEx_DATAEEPROM_EnableFixedTimeProgram();

	for (unsigned i=0; i<sizeof(TouchCal_s); i+=sizeof(uint32_t))
	{
		uint32_t address = ((uint32_t)data_store_item) + i;
		uint32_t data = *(uint32_t *)(((uint8_t *)touch_data) + i);
		HAL_FLASHEx_DATAEEPROM_Erase(address);
		HAL_FLASHEx_DATAEEPROM_Program(FLASH_TYPEPROGRAMDATA_WORD, address, data);
	}

	HAL_FLASHEx_DATAEEPROM_Lock();
}

SleepType_E app_get_sleep_capability(void)
{
	if ((HAL_ADC_GetState(&ADC_DEV) & HAL_ADC_STATE_REG_BUSY) ||
		(HAL_TSC_GroupGetStatus(&TSC_DEV, TOUCH_SENSE_GROUP) != TSC_GROUP_COMPLETED) ||
		led_get_active())
	{
		return SLEEP_NONE;
	}
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
	uint32_t last_touch_update;
	uint16_t touch_val;
	uint32_t ticks;
	LedCmd_S led_cmd;
	TouchCal_s app_touch_data;
	volatile TouchCal_s *touch_data_item;

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

	if (TOUCH_CAL_EMPTY == touch_cal)
	{
		Error_Handler();
	}

	touch_data_item = touch_data_store;

	for (unsigned i=0; i<TOUCH_CAL_HISTORY_LEN; ++i)
	{
		if (TOUCH_CAL_EMPTY == touch_data_store[i].cal_value)
		{
			// Found an open spot.
			touch_data_item = &touch_data_store[i];
			break;
		}
	}


	HAL_FLASHEx_DATAEEPROM_Unlock();

	// Erase the next item in the list
	if ((touch_data_item+1) == &touch_data_store[TOUCH_CAL_HISTORY_LEN])
	{
		HAL_FLASHEx_DATAEEPROM_Erase((uint32_t)touch_data_store);
	}
	else
	{
		HAL_FLASHEx_DATAEEPROM_Erase((uint32_t)(touch_data_item+1));
	}

	HAL_FLASHEx_DATAEEPROM_Lock();

	app_touch_data.cal_value = touch_cal;
	app_touch_data.unused = 0;
	app_touch_data.touch_high = 0;
	app_touch_data.touch_low = 0xffff;

	update_touch_data(&app_touch_data, (TouchCal_s *)touch_data_item);

	led_cmd.id = LED_CMD_SET;
	led_cmd.color = green;
	(void)xQueueSend(LedCmdQHandle, &led_cmd, 0);

	last_wake_time = osKernelSysTick();
	last_led_change = last_wake_time;
	last_touch_update = last_wake_time;

	while (1)
	{
		touch_val = read_touch_sense();

		ticks = osKernelSysTick();

		// Check if the read value is at least 20% less than the calibration value.
		// TODO: Compensate for drift in touch sensor value.
		if ((touch_val < touch_cal) && ((10*touch_cal / touch_val)) >= 12)
		{
			if ((ticks - last_led_change) > LED_UPDATE_DURATION_MS)
			{
				thermistor_enable();
				//adc_select_channel(ADC_CHANNEL_THERMISTOR);
				adc_perform_conversion();
				adc_perform_conversion();
				thermistor_disable();

				float temperature_C = compute_thermistor_temp_C(HAL_ADC_GetValue(&ADC_DEV));

				led_cmd.id = LED_CMD_FADE;
				compute_led_color((uint32_t)temperature_C, &led_cmd.color);
				(void)xQueueSend(LedCmdQHandle, &led_cmd, 0);
				last_led_change = ticks;
			}
		}
		else if (led_get_active() && ((ticks - last_led_change) > LED_DISABLE_DURATION_MS))
		{
			led_cmd.id = LED_CMD_DISABLE;
			(void)xQueueSend(LedCmdQHandle, &led_cmd, 0);
			last_led_change = ticks;
		}

		// Update logged touch information
		if (touch_val < app_touch_data.touch_low)
		{
			app_touch_data.touch_low = touch_val;
		}
		if (touch_val > app_touch_data.touch_high)
		{
			app_touch_data.touch_high = touch_val;
		}
		if ((ticks - last_touch_update) > TOUCH_DATA_UPDATE_MS)
		{
			if ((app_touch_data.touch_low != touch_data_item->touch_low) ||
		        (app_touch_data.touch_high != touch_data_item->touch_high))
			{
				update_touch_data(&app_touch_data, (TouchCal_s *)touch_data_item);
			}
		}

		osDelayUntil(&last_wake_time, 200);
	}
}

static inline int calc_color(int old_color, int new_color, int inc, int increments)
{
	// Important: This math relies on signed values
	return ((old_color * (increments - inc)) + (new_color * inc)) / increments;
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

				// Only update the color if the color has changed to help prevent flicker.
				if (old_color.red != command.color.red ||
					old_color.green != command.color.green ||
					old_color.blue != command.color.blue)
				{
					for (int i=1; i<=LED_FADE_INCREMENTS; ++i)
					{
						int r = calc_color(old_color.red, command.color.red, i, LED_FADE_INCREMENTS);
						int g = calc_color(old_color.green, command.color.green, i, LED_FADE_INCREMENTS);
						int b = calc_color(old_color.blue, command.color.blue, i, LED_FADE_INCREMENTS);

						configASSERT(r >= 0 && g >= 0 && b >= 0 && r <= 255 && g <= 255 && b <= 255);

						// When fading between colors, don't set to fully black to avoid the appearance
						// of the LED turning off then on again.
						if ((r == 0 && g == 0 && b == 0) &&
						    (command.color.red != 0 || command.color.green != 0 || command.color.blue != 0))
						{
							continue;
						}

						fade_color.red = r;
						fade_color.green = g;
						fade_color.blue = b;
						led_set(&fade_color);

						osDelay(LED_FADE_DELAY_MS);
					}
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
