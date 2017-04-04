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

#include "stm32l0xx_hal.h"
#include "drivers/led.h"

static bool active = false;
static bool gpio_initialized = false;
static uint32_t timer_period = 0;
static TIM_HandleTypeDef *timer_dev = NULL;
static struct led_color current_color;

static inline enum led_error hal_to_led_error(HAL_StatusTypeDef hal_status)
{
	return (enum led_error)hal_status;
}

enum led_error led_init(TIM_HandleTypeDef *dev, uint32_t period)
{
	if (period > 16843009)
	{
		// Disallow due to uint32_t overflow in period calculation of channels
		// in led_set().
		return LED_EINVAL;
	}

	timer_dev = dev;
	timer_period = period;
	return LED_EOK;
}

enum led_error led_deinit(void)
{
	timer_dev = NULL;
	timer_period = 0;
	return LED_EOK;
}

void led_disable(void)
{
	if (! timer_dev)
	{
		return;
	}

	active = false;
	gpio_initialized = false;

	(void)HAL_TIM_PWM_Stop(timer_dev, TIM_CHANNEL_1);
	(void)HAL_TIM_PWM_Stop(timer_dev, TIM_CHANNEL_2);
	(void)HAL_TIM_PWM_Stop(timer_dev, TIM_CHANNEL_3);

	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_5);
}

bool led_get_active(void)
{
	if (!timer_dev)
	{
		return false;
	}
	else
	{
		return active;
	}
}

enum led_error led_get(struct led_color *color)
{
	if (! timer_dev || (0 == timer_period))
	{
		return LED_EDISABLED;
	}
	else if (NULL == color)
	{
		return LED_EINVAL;
	}

	*color = current_color;

	return LED_EOK;
}

enum led_error led_set(const struct led_color *color)
{
	TIM_OC_InitTypeDef sConfigOC;
	HAL_StatusTypeDef hal_status;
	enum led_error error_ret = LED_EOK;

	if (! timer_dev || (0 == timer_period))
	{
		return LED_EDISABLED;
	}
	else if (NULL == color)
	{
		return LED_EINVAL;
	}
	else if (active &&
			 current_color.red == color->red &&
			 current_color.green == color->green &&
			 current_color.blue == color->blue)
	{
		return LED_EOK;
	}

	active = true;

	if (!gpio_initialized)
	{
		extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
		HAL_TIM_MspPostInit(timer_dev);
		gpio_initialized = true;
	}

	// Enable shadow register for reload value to prevent weird flickering
	// when adjusting PWM value.
	timer_dev->Instance->CR1 |= TIM_CR1_ARPE;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;

	// Red channel
	sConfigOC.Pulse = (color->red * timer_period) / 255;
	if ((hal_status = HAL_TIM_PWM_ConfigChannel(timer_dev, &sConfigOC, TIM_CHANNEL_1)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	// Green channel
	sConfigOC.Pulse = (color->green * timer_period) / 255;
	if ((hal_status = HAL_TIM_PWM_ConfigChannel(timer_dev, &sConfigOC, TIM_CHANNEL_2)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	// Blue channel
	sConfigOC.Pulse = (color->blue * timer_period) / 255;
	if ((hal_status = HAL_TIM_PWM_ConfigChannel(timer_dev, &sConfigOC, TIM_CHANNEL_3)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	// Enable outputs
	if ((hal_status = HAL_TIM_PWM_Start(timer_dev, TIM_CHANNEL_1)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	if ((hal_status = HAL_TIM_PWM_Start(timer_dev, TIM_CHANNEL_2)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	if ((hal_status = HAL_TIM_PWM_Start(timer_dev, TIM_CHANNEL_3)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	current_color = *color;

	return error_ret;

out_disable:
	(void)HAL_TIM_PWM_Stop(timer_dev, TIM_CHANNEL_1);
	(void)HAL_TIM_PWM_Stop(timer_dev, TIM_CHANNEL_2);
	(void)HAL_TIM_PWM_Stop(timer_dev, TIM_CHANNEL_3);

	return error_ret;
}
