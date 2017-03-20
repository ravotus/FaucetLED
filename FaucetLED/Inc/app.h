#ifndef __APP_H__
#define __APP_H__

#include "stm32l0xx_hal.h"

#include "drivers/led.h"

#define ADC_CHANNEL_THERMISTOR		ADC_CHANNEL_3
#define ADC_CHANNEL_BATT_2			ADC_CHANNEL_9

/*
# Coefficients generated with numpy:
import numpy as np
Rmin = 2874
Rmax = 20566
AdcMin = 10000/(10000+Rmax)*4096  # 55C
AdcMax = 10000/(10000+Rmin)*4096   # 10C
x = np.linspace(AdcMin, AdcMax)
Rx = 4096*10000/x-10000
# Note: Need to subtract 273.15 from y to convert to C.
y = 1/(1/298.15 + (1/3984)*np.log(Rx / 10000))
p2 = np.poly1d(np.polyfit(x, y, 2))

>>> max(abs(y-p2(x)))
0.95293751583449193
 */
#define THERMISTOR_X2				(  3.42794262e-06f )
#define THERMISTOR_X1				(  8.55486576e-03f )
#define THERMISTOR_X0				(  2.65963480e+02f )
#define THERMISTOR_OFFSET			( -273.15f )

#define TOUCH_NUM_SAMPLES_CAL		10
#define TOUCH_SENSE_GROUP			TSC_GROUP5_IDX

#define ADC_DEV				hadc
#define LED_TIMER_DEV		htim2
#define LPTIM_DEV			hlptim1
#define HAL_TICK_TIM_DEV	htim6
#define HAL_TICK_TIM_IRQ	TIM6_IRQn
#define OPAMP_DEV			hopamp1
#define TSC_DEV				htsc

extern ADC_HandleTypeDef 	ADC_DEV;
extern TIM_HandleTypeDef 	LED_TIMER_DEV;
extern LPTIM_HandleTypeDef	LPTIM_DEV;
extern TIM_HandleTypeDef	HAL_TICK_TIM_DEV;
extern TSC_HandleTypeDef	TSC_DEV;

#define APP_LOW_POWER_TICK_HZ	400000
#define APP_LOW_POWER_MSI_RANGE	RCC_MSIRANGE_2

#define LPTIM_PERIOD		0xffff
#define LPTIM_CLK_HZ		40000
#define LPTIM_CLK_DIV		1

typedef enum
{
	SLEEP_NONE,
	SLEEP_LOW_POWER,
	SLEEP_STOP,
	NUM_SLEEP_TYPES
} SleepType_E;

#define LED_UPDATE_DURATION_MS	1000
#define LED_DISABLE_DURATION_MS	500

// The total time to fade the LED should be < LED_UPDATE_DURATION_MS
#define LED_FADE_DELAY_MS	20
#define LED_FADE_INCREMENTS	40

#define LED_CMD_DISABLE		0x01
#define LED_CMD_SET			0x02
#define LED_CMD_FADE		0x03

typedef struct
{
	uint8_t id;

	union
	{
		struct led_color color;
	};
} LedCmd_S;

SleepType_E app_get_sleep_capability(void);

#endif /* __APP_H__ */
