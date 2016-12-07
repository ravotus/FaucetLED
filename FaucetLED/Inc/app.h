#ifndef __APP_H__
#define __APP_H__

#include "stm32l4xx_hal.h"

#define NUM_ADC_SAMPLES				128
#define ADC_TRIGGER_STDEV_V			0.1f
#define NUM_SAMPLES_FOR_TRIGGER		5

#define ADC_DEV				hadc1
#define DS18B20_UART		huart1
#define CRC_DEV				hcrc

extern ADC_HandleTypeDef ADC_DEV;
extern CRC_HandleTypeDef CRC_DEV;
extern UART_HandleTypeDef DS18B20_UART;

#endif /* __APP_H__ */
