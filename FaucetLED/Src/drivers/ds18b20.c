#include "stm32l4xx_hal.h"

#include "drivers/ds18b20.h"

#define DS18B20_TIMEOUT_MS			100

#define OW_RESET_CMD				0xf0
#define OW_RESET_VAL				OW_RESET_CMD

// These values from Maxium Tutorial 214
#define OW_RESET_MIN				0x90
#define OW_RESET_MAX				0xe0
#define OW_BIT_HIGH_THRESH			0xfe

#define OW_0						0x00
#define OW_1						0xff
#define OW_R						0xff

#define DS18B20_ROM_CRC_BYTE		0x07
#define DS18B20_READ_ROM_CMD		0x33
#define DS18B20_CONVERT_TEMP_CMD	0x44

// Currently only a single instance supported
static UART_HandleTypeDef *ds18b20_uart_dev = NULL;
static CRC_HandleTypeDef *ds18b20_crc_dev = NULL;

static const uint8_t ds18b20_read_rom_cmd[] = {
		OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, OW_0, OW_0,
};

static inline enum ds18b20_error hal_to_ds18b20_error(HAL_StatusTypeDef hal_status)
{
	return (enum ds18b20_error)hal_status;
}

// Convert an array of one-wire bits read to an array of bytes containing
// the actual data. A bit is considered high if it is >= OW_BIT_HIGH_THRESH.
static void ow_bits_to_bytes(uint8_t *bits_in, uint8_t *bytes_out, size_t len)
{
	for (unsigned i=0; i<len; ++i)
	{
		if (bits_in[i] >= OW_BIT_HIGH_THRESH)
		{
			bytes_out[(unsigned)(i/8)] |= 1<<(i%8);
		}
	}
}

// Reset the device on the 1-wire bus
static enum ds18b20_error ds18b20_ow_reset(UART_HandleTypeDef *uart_dev)
{
	enum ds18b20_error err_ret = DS18B20_EOK;
	HAL_StatusTypeDef hal_status;

	uint8_t reset_cmd = OW_RESET_CMD;
	uint8_t reset_result;

	// First, initialize the UART to 9600 baud to send the appropriate-
	// length reset pulse.
	uart_dev->Instance = USART1;
	uart_dev->Init.BaudRate = 9600;
	uart_dev->Init.WordLength = UART_WORDLENGTH_8B;
	uart_dev->Init.StopBits = UART_STOPBITS_1;
	uart_dev->Init.Parity = UART_PARITY_NONE;
	uart_dev->Init.Mode = UART_MODE_TX_RX;
	uart_dev->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart_dev->Init.OverSampling = UART_OVERSAMPLING_16;
	uart_dev->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uart_dev->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if ((hal_status = HAL_HalfDuplex_Init(uart_dev)) != HAL_OK)
	{
		err_ret = hal_to_ds18b20_error(hal_status);
		goto out_err;
	}
	// Transmit reset command at lower baud
	if ((hal_status = HAL_UART_Transmit(uart_dev, &reset_cmd, 1, 100)) != HAL_OK)
	{
		err_ret = hal_to_ds18b20_error(hal_status);
		goto out_err;
	}
	if ((hal_status = HAL_UART_Receive(uart_dev, &reset_result, 1, 100)) != HAL_OK)
	{
		err_ret = hal_to_ds18b20_error(hal_status);
		goto out_err;
	}
	if ((reset_result < OW_RESET_MIN) || (reset_result > OW_RESET_MAX))
	{
		err_ret = DS18B20_NOTFOUND;
		goto out_err;
	}

	// Now disable the device, then configure for proper 115200 baud
	if ((hal_status = HAL_UART_DeInit(uart_dev)) != HAL_OK)
	{
		err_ret = hal_to_ds18b20_error(hal_status);
		goto out_err;
	}
	uart_dev->Instance = USART1;
	uart_dev->Init.BaudRate = 115200;
	uart_dev->Init.WordLength = UART_WORDLENGTH_8B;
	uart_dev->Init.StopBits = UART_STOPBITS_1;
	uart_dev->Init.Parity = UART_PARITY_NONE;
	uart_dev->Init.Mode = UART_MODE_TX_RX;
	uart_dev->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	uart_dev->Init.OverSampling = UART_OVERSAMPLING_16;
	uart_dev->Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	uart_dev->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if ((hal_status = HAL_HalfDuplex_Init(uart_dev)) != HAL_OK)
	{
		err_ret = hal_to_ds18b20_error(hal_status);
		goto out_err;
	}
	return err_ret;

out_err:
	(void)HAL_UART_DeInit(uart_dev);
	return err_ret;
}

enum ds18b20_error ds18b20_init(UART_HandleTypeDef *uart_dev,
		CRC_HandleTypeDef *crc_dev)
{
	enum ds18b20_error err_ret = DS18B20_EOK;

	if (ds18b20_uart_dev)
	{
		return DS18B20_EBUSY;
	}
	else if (!uart_dev || !crc_dev)
	{
		return DS18B20_EINVAL;
	}

	if ((err_ret = ds18b20_ow_reset(uart_dev)) != DS18B20_EOK)
	{
		goto out_err;
	}

	ds18b20_uart_dev = uart_dev;
	ds18b20_crc_dev = crc_dev;
	return err_ret;

out_err:
	(void)HAL_UART_DeInit(uart_dev);
	return err_ret;
}

enum ds18b20_error ds18b20_deinit(void)
{
	enum ds18b20_error err_ret = DS18B20_EOK;

	if (ds18b20_uart_dev)
	{
		err_ret = hal_to_ds18b20_error(HAL_UART_DeInit(ds18b20_uart_dev));
		ds18b20_uart_dev = NULL;
	}

	ds18b20_crc_dev = NULL;

	return err_ret;
}

enum ds18b20_error ds18b20_read_rom(uint8_t *rom_buf, size_t buf_len)
{
	enum ds18b20_error err_ret = DS18B20_EOK;
	HAL_StatusTypeDef hal_status;
	uint8_t tmp_read_buf[OW_READ_ROM_BUF_LEN * 8];

	if (!ds18b20_uart_dev)
	{
		return DS18B20_EDISABLED;
	}
	else if (!rom_buf)
	{
		return DS18B20_EINVAL;
	}
	else if (buf_len < OW_READ_ROM_BUF_LEN)
	{
		return DS18B20_EINVAL;
	}

	if ((err_ret = ds18b20_ow_reset(ds18b20_uart_dev)) != DS18B20_EOK)
	{
		goto out;
	}

	for (unsigned i=0; i<OW_READ_ROM_BUF_LEN; ++i)
	{
		if ((hal_status = HAL_UART_Transmit(ds18b20_uart_dev,
				(uint8_t *)&ds18b20_read_rom_cmd[i], 1, DS18B20_TIMEOUT_MS)) != HAL_OK)
		{
			err_ret = hal_to_ds18b20_error(hal_status);
			goto out;
		}
		else if ((hal_status = HAL_UART_Receive(ds18b20_uart_dev,
				&tmp_read_buf[i], 1, DS18B20_TIMEOUT_MS)) != HAL_OK)
		{
			err_ret = hal_to_ds18b20_error(hal_status);
			goto out;
		}

		rom_buf[i] = 0;
	}

	// Receive 8 bytes of ROM data, one bit at a time
	for (unsigned i=0; i<(OW_READ_ROM_BUF_LEN * 8); ++i)
	{
		if ((hal_status = HAL_UART_Transmit(ds18b20_uart_dev,
				(uint8_t *)&ds18b20_read_rom_cmd, 1, DS18B20_TIMEOUT_MS)) != HAL_OK)
		{
			err_ret = hal_to_ds18b20_error(hal_status);
			goto out;
		}
		else if ((hal_status = HAL_UART_Receive(ds18b20_uart_dev,
				&tmp_read_buf[i], 1, DS18B20_TIMEOUT_MS)) != HAL_OK)
		{
			err_ret = hal_to_ds18b20_error(hal_status);
			goto out;
		}
	}

	// Convert the sequence of OW bits back to actual byte data.
	ow_bits_to_bytes(tmp_read_buf, rom_buf,	OW_READ_ROM_BUF_LEN * 8);

	// The last byte of the ROM contains the CRC of the data
	if (HAL_CRC_Calculate(ds18b20_crc_dev, (uint32_t *)rom_buf,
			OW_READ_ROM_BUF_LEN - 1) != rom_buf[DS18B20_ROM_CRC_BYTE])
	{
		err_ret = DS18B20_CRC_FAIL;
		goto out;
	}

out:
	return err_ret;
}
