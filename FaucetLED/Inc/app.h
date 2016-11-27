#ifndef __APP_H__
#define __APP_H__

#include "stm32l4xx_hal.h"

#define DS18B20_UART	huart1
#define CRC_DEV			hcrc

extern CRC_HandleTypeDef CRC_DEV;
extern UART_HandleTypeDef DS18B20_UART;

#endif /* __APP_H__ */
