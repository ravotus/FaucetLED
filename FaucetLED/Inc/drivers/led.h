#ifndef __LED_DRIVER_H__
#define __LED_DRIVER_H__

#include <stdbool.h>
#include <stdint.h>
#include "stm32l4xx_hal_def.h"

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
enum led_error led_set(const struct led_color *color);

#endif //__LED_DRIVER_H__
