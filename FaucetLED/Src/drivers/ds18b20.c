#include <limits.h>

#include "FreeRTOS.h"
#include "task.h"
#include "stm32l4xx_hal.h"

#include "drivers/ds18b20.h"

#define DMA_RX_COMPLETE				0x01
#define DMA_TX_COMPLETE 			0x02

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
// The maximum number of bytes read by one command
#define OW_MAX_CMD_LEN_BYTES		(9 * 8)

#define DS18B20_ROM_CRC_BYTE		7

#define DS18B20_READ_ROM_CMD		0x33
#define DS18B20_SKIP_ROM_CMD		0xCC
#define DS18B20_CONVERT_TEMP_CMD	0x44
#define DS18B20_READ_SCRATCHPAD_CMD	0xBE

#define DS18B20_SCRATCHPAD_LEN		9
#define DS18B20_SCRATCHPAD_CRC_BYTE	8

#define DS18B20_TEMP_RAW12_TO_C		0.0625f

// Currently only a single instance supported
static UART_HandleTypeDef *ds18b20_uart_dev = NULL;
static CRC_HandleTypeDef *ds18b20_crc_dev = NULL;

static uint8_t read_buf[OW_MAX_CMD_LEN_BYTES];
static TaskHandle_t task_handle = NULL;

// Each value below corresponds to the command, in bits, LSB first.
static const uint8_t ds18b20_read_rom_cmd[] = {
		OW_1, OW_1, OW_0, OW_0, OW_1, OW_1, OW_0, OW_0,
};

static const uint8_t ds18b20_skip_rom_cmd[] = {
		OW_0, OW_0, OW_1, OW_1, OW_0, OW_0, OW_1, OW_1,
};

static const uint8_t ds18b20_convert_temp_cmd[] = {
		OW_0, OW_0, OW_1, OW_0, OW_0, OW_0, OW_1, OW_0,
};

static const uint8_t ds18b20_read_scratchpad_cmd[] = {
		OW_0, OW_1, OW_1, OW_1, OW_1, OW_1, OW_0, OW_1,
};

// ST's HAL doesn't provide a good way to enable or disable memory
// auto-increment, so we provide our own.
static inline void hal_dma_disable_meminc(DMA_HandleTypeDef *hdma)
{
	__HAL_DMA_DISABLE(hdma);
	if (hdma)
	{
		hdma->Instance->CCR &= ~DMA_MINC_ENABLE;
	}
}

static inline void hal_dma_enable_meminc(DMA_HandleTypeDef *hdma)
{
	__HAL_DMA_DISABLE(hdma);
	if (hdma)
	{
		hdma->Instance->CCR |= DMA_MINC_ENABLE;
	}
}

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
		else
		{
			bytes_out[(unsigned)(i/8)] &= ~(1<<(i%8));
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

	// Now disable the device, then configure for 115200 baud
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

static enum ds18b20_error ds18b20_transaction_dma(const uint8_t *tx_buf,
		uint8_t *rx_buf, uint32_t len, uint32_t timeout)
{
	enum ds18b20_error err_ret = DS18B20_EOK;
	HAL_StatusTypeDef hal_status;

	task_handle = xTaskGetCurrentTaskHandle();

	if ((hal_status = HAL_UART_Transmit_DMA(ds18b20_uart_dev, (uint8_t *)tx_buf, len)) != HAL_OK)
	{
		err_ret = hal_to_ds18b20_error(hal_status);
		goto out;
	}

	if ((hal_status = HAL_UART_Receive_DMA(ds18b20_uart_dev, rx_buf, len)) != HAL_OK)
	{
		err_ret = hal_to_ds18b20_error(hal_status);
		goto out;
	}

	uint32_t notify_val;
	uint32_t wait_bits = DMA_RX_COMPLETE | DMA_TX_COMPLETE;
	uint32_t start_ticks = xTaskGetTickCount();
	while ((wait_bits != 0) && ((xTaskGetTickCount() - start_ticks) < timeout))
	{
		if (xTaskNotifyWait(0, ULONG_MAX, &notify_val, (timeout/4) || 1))
		{
			wait_bits &= ~notify_val;
		}
	}

	if (wait_bits != 0)
	{
		err_ret = DS18B20_ETIMEOUT;
		goto out;
	}

out:
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
	uint8_t ow_read = OW_R;

	if (!ds18b20_uart_dev)
	{
		return DS18B20_EDISABLED;
	}
	else if (!rom_buf)
	{
		return DS18B20_EINVAL;
	}
	else if (buf_len < DS18B20_READ_ROM_BUF_LEN)
	{
		return DS18B20_EINVAL;
	}

	if ((err_ret = ds18b20_ow_reset(ds18b20_uart_dev)) != DS18B20_EOK)
	{
		goto out;
	}

	// Issue read ROM command
	hal_dma_enable_meminc(ds18b20_uart_dev->hdmatx);
	if ((err_ret = ds18b20_transaction_dma(ds18b20_read_rom_cmd, read_buf,
			sizeof(ds18b20_read_rom_cmd), DS18B20_TIMEOUT_MS)) != DS18B20_EOK)
	{
		goto out;
	}

	// Receive 8 bytes of ROM data, one bit at a time
	hal_dma_disable_meminc(ds18b20_uart_dev->hdmatx);
	if ((err_ret = ds18b20_transaction_dma(&ow_read, read_buf,
			DS18B20_READ_ROM_BUF_LEN * 8, DS18B20_TIMEOUT_MS)) != DS18B20_EOK)
	{
		goto out;
	}

	// Convert the sequence of OW bits back to actual byte data.
	ow_bits_to_bytes(read_buf, rom_buf,	DS18B20_READ_ROM_BUF_LEN * 8);

	// The last byte of the ROM contains the CRC of the data
	if (HAL_CRC_Calculate(ds18b20_crc_dev, (uint32_t *)rom_buf,
			DS18B20_READ_ROM_BUF_LEN - 1) != rom_buf[DS18B20_ROM_CRC_BYTE])
	{
		err_ret = DS18B20_CRC_FAIL;
		goto out;
	}

out:
	return err_ret;
}

enum ds18b20_error ds18b20_read_temp(float *temperature_C)
{
	enum ds18b20_error err_ret = DS18B20_EOK;
	uint8_t ow_read = OW_R;

	if (!ds18b20_uart_dev)
	{
		return DS18B20_EDISABLED;
	}
	else if (!temperature_C)
	{
		return DS18B20_EINVAL;
	}

	// Reset
	if ((err_ret = ds18b20_ow_reset(ds18b20_uart_dev)) != DS18B20_EOK)
	{
		goto out;
	}

	// Issue skip ROM command
	hal_dma_enable_meminc(ds18b20_uart_dev->hdmatx);
	if ((err_ret = ds18b20_transaction_dma(ds18b20_skip_rom_cmd, read_buf,
			sizeof(ds18b20_skip_rom_cmd), DS18B20_TIMEOUT_MS)) != DS18B20_EOK)
	{
		goto out;
	}

	// Issue Convert T command
	if ((err_ret = ds18b20_transaction_dma(ds18b20_convert_temp_cmd, read_buf,
			sizeof(ds18b20_convert_temp_cmd), DS18B20_TIMEOUT_MS)) != DS18B20_EOK)
	{
		goto out;
	}

	vTaskDelay(750);

	// Reset
	if ((err_ret = ds18b20_ow_reset(ds18b20_uart_dev)) != DS18B20_EOK)
	{
		goto out;
	}

	// Issue skip ROM command
	if ((err_ret = ds18b20_transaction_dma(ds18b20_skip_rom_cmd, read_buf,
			sizeof(ds18b20_skip_rom_cmd), DS18B20_TIMEOUT_MS)) != DS18B20_EOK)
	{
		goto out;
	}

	// Issue read scratchpad command
	if ((err_ret = ds18b20_transaction_dma(ds18b20_read_scratchpad_cmd, read_buf,
			sizeof(ds18b20_read_scratchpad_cmd), DS18B20_TIMEOUT_MS)) != DS18B20_EOK)
	{
		goto out;
	}

	// Read scratchpad: Receive 9 bytes of scratchpad data, one bit at a time
	hal_dma_disable_meminc(ds18b20_uart_dev->hdmatx);
	if ((err_ret = ds18b20_transaction_dma(&ow_read, read_buf,
			DS18B20_SCRATCHPAD_LEN * 8, DS18B20_TIMEOUT_MS)) != DS18B20_EOK)
	{
		goto out;
	}

	// Convert the sequence of OW bits back to actual byte data.
	uint8_t scratchpad_buf[DS18B20_SCRATCHPAD_LEN];
	ow_bits_to_bytes(read_buf, scratchpad_buf,	DS18B20_SCRATCHPAD_LEN * 8);

	// The last byte of the scratchpad contains the CRC of the data
	if (HAL_CRC_Calculate(ds18b20_crc_dev, (uint32_t *)scratchpad_buf,
			DS18B20_SCRATCHPAD_LEN - 1) != scratchpad_buf[DS18B20_SCRATCHPAD_CRC_BYTE])
	{
		err_ret = DS18B20_CRC_FAIL;
		goto out;
	}

	int16_t temperature = ((scratchpad_buf[1] << 8) & 0xff00) | (scratchpad_buf[0] & 0xff);
	*temperature_C = temperature * DS18B20_TEMP_RAW12_TO_C;

out:
	return err_ret;
}

// ISR Handlers
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t higher_prio_task_woken = pdFALSE;

	if (task_handle)
	{
		xTaskNotifyFromISR(task_handle, DMA_RX_COMPLETE, eSetBits, &higher_prio_task_woken);
		portYIELD_FROM_ISR(higher_prio_task_woken);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	BaseType_t higher_prio_task_woken = pdFALSE;

	if (task_handle)
	{
		xTaskNotifyFromISR(task_handle, DMA_TX_COMPLETE, eSetBits, &higher_prio_task_woken);
		portYIELD_FROM_ISR(higher_prio_task_woken);
	}
}
