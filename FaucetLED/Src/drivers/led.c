#include "stm32l4xx_hal.h"
#include "drivers/led.h"

static bool active = false;
static bool gpio_initialized = false;
static uint32_t timer_period = 0;
static TIM_HandleTypeDef *timer_dev = NULL;

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

	active = true;

	if (!gpio_initialized)
	{
		extern void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);
		HAL_TIM_MspPostInit(timer_dev);
		gpio_initialized = true;
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

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

	return error_ret;

out_disable:
	(void)HAL_TIM_PWM_Stop(timer_dev, TIM_CHANNEL_1);
	(void)HAL_TIM_PWM_Stop(timer_dev, TIM_CHANNEL_2);
	(void)HAL_TIM_PWM_Stop(timer_dev, TIM_CHANNEL_3);

	return error_ret;
}
