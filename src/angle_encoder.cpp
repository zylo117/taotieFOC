#include "angle_encoder.h"
#include "at32f403a_407_board.h"
#include "FreeRTOS.h"
#include "task.h"

namespace encoder_common
{
    void encoder_write_gpio(gpio_type* port, uint16_t pin, bool state)
    {
        if (state)
        {
            port->scr = pin;
        }
        else
        {
            port->clr = pin;
        }
    }

    void encoder_gpio_config_input(gpio_type* port, uint16_t pin)
    {
        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_mode = GPIO_MODE_INPUT;
        gpio_init_struct.gpio_pins = pin;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init(port, &gpio_init_struct);
    }

    void encoder_gpio_config_output(gpio_type* port, uint16_t pin)
    {
        gpio_init_type gpio_init_struct;
        gpio_default_para_init(&gpio_init_struct);
        gpio_init_struct.gpio_drive_strength = GPIO_DRIVE_STRENGTH_STRONGER;
        gpio_init_struct.gpio_out_type = GPIO_OUTPUT_PUSH_PULL;
        gpio_init_struct.gpio_mode = GPIO_MODE_OUTPUT;
        gpio_init_struct.gpio_pins = pin;
        gpio_init_struct.gpio_pull = GPIO_PULL_NONE;
        gpio_init(port, &gpio_init_struct);
    }

    uint16_t encoder_spi2_rw16(uint16_t tx_data)
    {
        // 【超时预算必须与中断周期同量级】
        // 本函数跑在 TMR5 采样中断里，中断周期 32us（估算：32us × 250MHz = 8000 cycles）。
        // 正常完成一次 16bit 传输只需要 16 / 3.9MHz ≈ 4.1us ≈ 1025 cycles，
        // 空转循环每次约 2~3 cycles，所以 2000 次 ≈ 24us 已经是 6 倍余量。
        //
        // 旧值 100000 次 ≈ 1.2ms，是总周期的 37 倍：MISO 断线/CS 失效/编码器掉电时，
        // 每拍中断都会在这里卡 1.2ms，同优先级的 DMA1 电流中断被堵死，进中断风暴死机。
        constexpr uint32_t spi_timeout = 2000U;
        uint32_t timeout = spi_timeout;
        while (spi_i2s_flag_get(KTH7823_SPI, SPI_I2S_TDBE_FLAG) == RESET && timeout > 0U)
        {
            --timeout;
        }
        if (timeout == 0U)
        {
            return 0xFFFFU;
        }
        spi_i2s_data_transmit(KTH7823_SPI, tx_data);

        timeout = spi_timeout;
        while (spi_i2s_flag_get(KTH7823_SPI, SPI_I2S_RDBF_FLAG) == RESET && timeout > 0U)
        {
            --timeout;
        }
        if (timeout == 0U)
        {
            return 0xFFFFU;
        }
        return static_cast<uint16_t>(spi_i2s_data_receive(KTH7823_SPI));
    }
}

void AngleEncoder::loadFromFlash(void)
{
	for(uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++)
	{
		m_calData[i].value = m_nvmFlashCal[i];
		m_calData[i].error = CALIBRATION_MIN_ERROR;
	}
	calibration_min_ = m_nvmFlashMeta->MIN;
	calibration_max_ = m_nvmFlashMeta->MAX;
}

bool AngleEncoder::saveToFlash(bool yield_between_pages)
{
	// 【已知行为，暂时不改】这里一置 false，isNonlinearCalValid() 立刻变成 false，
	// readRaw() 在整个写入期间（FastCal 128KB，约 5 秒）都不会做非线性校正，
	// 角度会带系统偏差；而且 flash_halfword_program 会 stall 总线，
	// TMR4/TMR5 中断都被推迟。
	// 当前不修的理由：校准流程本身不跑电机，偏差和中断延迟都不影响结果；
	// 改成“新表写完再原子切换”需要重排 Flash 写入顺序，引入新 bug 的风险更大。
	// 如果以后要在运动过程中保存，必须先改成原子切换。
	m_fastCalValid = false;
	static uint16_t main_table[CALIBRATION_TABLE_SIZE];
	uint16_t minimum = m_calData[0].value;
	uint16_t maximum = minimum;
	for (uint16_t i = 0U; i < CALIBRATION_TABLE_SIZE; ++i)
	{
		const uint16_t value = m_calData[i].value;
		main_table[i] = value;
		if (value < minimum) minimum = value;
		if (value > maximum) maximum = value;
	}

	FlashCalMetadata_t metadata = {
		CALIBRATION_STORAGE_MAGIC,
		CalStatus::invalid,
		minimum,
		maximum,
		0xFFFFU
	};
	calibration_min_ = minimum;
	calibration_max_ = maximum;
	if (flash_write(MAINCAL_META_BASE, reinterpret_cast<uint16_t*>(&metadata), sizeof(metadata) / 2U) != SUCCESS)
	{
		return false;
	}
	if (yield_between_pages) vTaskDelay(1U);
	if (flash_write(MAINCAL_FLASH_BASE, main_table, CALIBRATION_TABLE_SIZE) != SUCCESS)
	{
		return false;
	}
	if (yield_between_pages) vTaskDelay(1U);
	if (!createFastCal(yield_between_pages))
	{
		return false;
	}

	metadata.fast_checksum = calibration_checksum_;
	metadata.status = CalStatus::valid;
	if (flash_write(MAINCAL_META_BASE, reinterpret_cast<uint16_t*>(&metadata), sizeof(metadata) / 2U) != SUCCESS)
	{
		return false;
	}
	calibration_min_ = minimum;
	calibration_max_ = maximum;
	m_fastCalValid = true;
	return true;
}

bool AngleEncoder::createFastCal(bool yield_between_pages)
{
	static uint16_t data[FLASH_ROW_SIZE];
	uint16_t checksum = 0U;
	for (uint32_t i = 0U; i < 65536U; ++i)
	{
		data[i % FLASH_ROW_SIZE] = reverseLookup(static_cast<uint16_t>(i));
		checksum = static_cast<uint16_t>(checksum + data[i % FLASH_ROW_SIZE]);
		if ((i % FLASH_ROW_SIZE) == FLASH_ROW_SIZE - 1U)
		{
			const uint32_t page_address = FASTCAL_FLASH_BASE + (i + 1U - FLASH_ROW_SIZE) * 2U;
			if (flash_write(page_address, data, FLASH_ROW_SIZE) != SUCCESS)
			{
				m_fastCalValid = false;
				return false;
			}
			if (yield_between_pages) vTaskDelay(1U);
		}
	}
	calibration_checksum_ = checksum;
	return true;
}

void AngleEncoder::updateFastCalCheck(void)
{
	uint32_t i;
	uint16_t checkSum = 0;
	bool NonZero = false;
	for(i=0; i < 65536U; i++)
	{
		checkSum += m_nvmFastCal->angle[i];
		if(m_nvmFastCal->angle[i] != 0U)
		{
			NonZero = true;
		}
	}
	const uint16_t stored_checksum = m_nvmFlashMeta->fast_checksum;

	if(m_nvmFlashMeta->magic != CALIBRATION_STORAGE_MAGIC ||
	   m_nvmFlashMeta->status != CalStatus::valid ||
	   checkSum != stored_checksum || NonZero != true)
	{
		if (m_nvmFlashMeta->magic == CALIBRATION_STORAGE_MAGIC &&
		    m_nvmFlashMeta->status == CalStatus::valid)
		{
			loadFromFlash();
			saveToFlash(false);
		}
	}
	else
	{
		calibration_checksum_ = stored_checksum;
		calibration_min_ = m_nvmFlashMeta->MIN;
		calibration_max_ = m_nvmFlashMeta->MAX;
		m_fastCalValid = true;
	}
}

void AngleEncoder::calibrationInit(void)
{
	uint16_t i;
	if(m_nvmFlashMeta->magic == CALIBRATION_STORAGE_MAGIC &&
	   m_nvmFlashMeta->status == CalStatus::valid)
	{
		loadFromFlash();
		updateFastCalCheck();
	}else
	{
		for(i=0; i < CALIBRATION_TABLE_SIZE; i++)
		{
			m_calData[i].value = 0;
			m_calData[i].error = CALIBRATION_ERROR_NOT_SET;
		}
        m_fastCalValid = false;
	}
}