#ifndef __APP_H__
#define __APP_H__

#include "stm32l4xx_hal.h"

#define NUM_ADC_SAMPLES				128
#define ADC_TRIGGER_STDEV_V			0.1f
#define NUM_SAMPLES_FOR_TRIGGER		5

#define ADC_INJ_CHANNEL_VREF 		ADC_INJECTED_RANK_1
#define ADC_INJ_CHANNEL_EXTTEMP		ADC_INJECTED_RANK_2

#define THERMISTOR_R_DIVIDER		10000
#define THERMISTOR_T0				10000
#define THERMISTOR_B				3984

#define ADC_DEV				hadc1
#define CRC_DEV				hcrc
#define LED_TIMER_DEV		htim2
#define HAL_TICK_TIM_DEV	htim7
#define HAL_TICK_TIM_IRQ	TIM7_IRQn
#define OPAMP_DEV			hopamp1

extern ADC_HandleTypeDef 	ADC_DEV;
extern CRC_HandleTypeDef 	CRC_DEV;
extern TIM_HandleTypeDef 	LED_TIMER_DEV;
extern TIM_HandleTypeDef	HAL_TICK_TIM_DEV;
extern OPAMP_HandleTypeDef	OPAMP_DEV;

#define APP_LOW_POWER_TICK_HZ	400000
#define APP_LOW_POWER_MSI_RANGE	RCC_MSIRANGE_2

uint8_t app_can_low_power_sleep(void);

#endif /* __APP_H__ */
