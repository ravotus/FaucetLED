#ifndef __DS18B20_H__
#define __DS18B20_H__

#include <stdint.h>
#include "stm32l4xx_hal_def.h"

// For now, this enum should be castable to HAL_StatusTypeDef.
enum ds18b20_error
{
	DS18B20_EOK = 0,
	DS18B20_EERROR = HAL_ERROR,
	DS18B20_EBUSY = HAL_BUSY,
	DS18B20_ETIMEOUT = HAL_TIMEOUT,
	DS18B20_EINVAL,
	DS18B20_EDISABLED,
	DS18B20_NOTFOUND,
	DS18B20_CRC_FAIL,
};

#define DS18B20_READ_ROM_BUF_LEN	8

// Initialize the DS18B20 device, including associated peripherals.
// Parameters:
// 	`uart_dev`: The UART device. This driver will take full control of the device.
//  `crc_dev`: The CRC device. This driver will share the CRC peripheral and is
//             currently not set up to initialize/deinitialize it.
enum ds18b20_error ds18b20_init(UART_HandleTypeDef *uart_dev, CRC_HandleTypeDef *crc_dev);
enum ds18b20_error ds18b20_deinit(void);

enum ds18b20_error ds18b20_read_rom(uint8_t *rom_buf, size_t buf_len);

#endif //__DS18B20_H__
