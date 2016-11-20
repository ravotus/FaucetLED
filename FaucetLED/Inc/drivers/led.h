#ifndef __LED_DRIVER_H__
#define __LED_DRIVER_H__

#include <stdint.h>

#include "stm32l4xx_hal_def.h"

#define LED_COLOR_MIN	0
#define LED_COLOR_MAX	UINT8_MAX

// Note: At least for now, this type should be castable to HAL_StatusTypeDef.
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

enum led_error led_init(uint32_t period);
enum led_error led_deinit(void);
enum led_error led_set(struct led_color *color);

#endif //__LED_DRIVER_H__
