#ifndef __APP_H__
#define __APP_H__

#include "stm32l4xx_hal.h"

#define NUM_ADC_SAMPLES				128
#define ADC_TRIGGER_STDEV_V			0.1f
#define NUM_SAMPLES_FOR_TRIGGER		5

#define ADC_CHANNEL_PIEZO_AMP		ADC_CHANNEL_9
#define ADC_CHANNEL_THERMISTOR		ADC_CHANNEL_11

#define THERMISTOR_R_DIVIDER		10000
#define THERMISTOR_T0				10000
#define THERMISTOR_B				3984

#define ADC_DEV				hadc1
#define CRC_DEV				hcrc
#define LED_TIMER_DEV		htim2
#define LPTIM_DEV			hlptim1
#define HAL_TICK_TIM_DEV	htim7
#define HAL_TICK_TIM_IRQ	TIM7_IRQn
#define OPAMP_DEV			hopamp1

extern ADC_HandleTypeDef 	ADC_DEV;
extern TIM_HandleTypeDef 	LED_TIMER_DEV;
extern LPTIM_HandleTypeDef	LPTIM_DEV;
extern TIM_HandleTypeDef	HAL_TICK_TIM_DEV;
extern OPAMP_HandleTypeDef	OPAMP_DEV;

#define APP_LOW_POWER_TICK_HZ	400000
#define APP_LOW_POWER_MSI_RANGE	RCC_MSIRANGE_2

#define LPTIM_PERIOD	0xffff
#define LPTIM_CLK_HZ	32000
#define LPTIM_CLK_DIV	1

typedef enum
{
	SLEEP_NONE,
	SLEEP_LOW_POWER,
	SLEEP_STOP,
	NUM_SLEEP_TYPES
} SleepType_E;

SleepType_E app_get_sleep_capability(void);

#endif /* __APP_H__ */
