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

#ifndef __LED_DRIVER_H__
#define __LED_DRIVER_H__

#include <stdbool.h>
#include <stdint.h>
#include "stm32l0xx_hal_def.h"

#define LED_COLOR_MIN	0
#define LED_COLOR_MAX	UINT8_MAX

// For now, this enum should be castable to HAL_StatusTypeDef.
enum led_error
{
	LED_EOK = HAL_OK,
	LED_EERROR = HAL_ERROR,
	LED_EBUSY = HAL_BUSY,
	LED_ETIMEOUT = HAL_TIMEOUT,
	LED_EINVAL,
	LED_EDISABLED,
};

struct led_color
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
};

enum led_error led_init(TIM_HandleTypeDef *dev, uint32_t period);
enum led_error led_deinit(void);
void led_disable(void);
bool led_get_active(void);
enum led_error led_get(struct led_color *color);
enum led_error led_set(const struct led_color *color);

#endif //__LED_DRIVER_H__
