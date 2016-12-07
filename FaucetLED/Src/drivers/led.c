#include "stm32l4xx_hal.h"
#include "drivers/led.h"

extern TIM_HandleTypeDef htim1;

static uint32_t timer_period = 0;

static inline enum led_error hal_to_led_error(HAL_StatusTypeDef hal_status)
{
	return (enum led_error)hal_status;
}

enum led_error led_init(uint32_t period)
{
	if (period > 16843009)
	{
		// Disallow due to uint32_t overflow in period calculation of channels
		// in led_set().
		return LED_EINVAL;
	}

	timer_period = period;
	return LED_EOK;
}

enum led_error led_deinit(void)
{
	timer_period = 0;
	return LED_EOK;
}

enum led_error led_set(const struct led_color *color)
{
	TIM_OC_InitTypeDef sConfigOC;
	HAL_StatusTypeDef hal_status;
	enum led_error error_ret = LED_EOK;

	if (0 == timer_period)
	{
		return LED_EDISABLED;
	}
	else if (NULL == color)
	{
		return LED_EINVAL;
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	// Red channel
	sConfigOC.Pulse = (color->red * timer_period) / 255;
	if ((hal_status = HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	// Green channel
	sConfigOC.Pulse = (color->green * timer_period) / 255;
	if ((hal_status = HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	// Blue channel
	sConfigOC.Pulse = (color->blue * timer_period) / 255;
	if ((hal_status = HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	// Enable outputs
	if ((hal_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	if ((hal_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	if ((hal_status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3)) != HAL_OK)
	{
		error_ret = hal_to_led_error(hal_status);
		goto out_disable;
	}

	return error_ret;

out_disable:
	(void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	(void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	(void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);

	return error_ret;
}
