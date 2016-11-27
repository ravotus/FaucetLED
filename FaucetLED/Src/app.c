#include "cmsis_os.h"

#include "app.h"
#include "drivers/ds18b20.h"
#include "main.h"

#include "stm32l4xx_hal_crc.h"

static uint8_t ds18b20_rom_buf[OW_READ_ROM_BUF_LEN];

void TemperatureTask(const void *arg)
{
	uint32_t last_wake_time;
	volatile enum ds18b20_error err;

	if (DS18B20_EOK != ds18b20_init(&DS18B20_UART, &CRC_DEV))
	{
		Error_Handler();
	}

	last_wake_time = osKernelSysTick();

	while(1)
	{
		err = ds18b20_read_rom(ds18b20_rom_buf, OW_READ_ROM_BUF_LEN);
		osDelayUntil(&last_wake_time, 10);
	}
}