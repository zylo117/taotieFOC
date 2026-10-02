#include "angle_encoder.h"
#include "at32f403a_407_board.h"

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
        constexpr uint32_t spi_timeout = 100000U;
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
		m_calData[i].value = m_nvmFlashCal->FlashCalData[i];
		m_calData[i].error = CALIBRATION_MIN_ERROR;
	}
}

void AngleEncoder::saveToFlash(void)
{
	uint16_t i = 0;
	uint16_t min = 0, max = 0;
	FlashCalData_t data;

	max = min = m_calData[0].value;
	for (i=0; i < CALIBRATION_TABLE_SIZE; i++ )
	{
		if(m_calData[i].value < min)	{min = m_calData[i].value;}
		if(m_calData[i].value > max)	{max = m_calData[i].value;}
		data.FlashCalData[i] = m_calData[i].value;
	}
	data.status = CalStatus::valid;
	data.MIN = min;
	data.MAX = max;

	flash_write(MAINCAL_FLASH_BASE, reinterpret_cast<uint16_t*>(&data), sizeof(FlashCalData_t)/2U);
	createFastCal();
}

void AngleEncoder::createFastCal(void)
{
	uint32_t i,j;
	uint16_t checkSum = 0;
	uint16_t data[FLASH_ROW_SIZE];
	for (i=0,j=0; i < 65536U; i++)
	{
		uint16_t x = reverseLookup(static_cast<uint16_t>(i));
		data[j] = x;
		j++;
		if (j >= FLASH_ROW_SIZE)
		{
			uint32_t dst_addr = FASTCAL_FLASH_BASE + ((i + 1U - FLASH_ROW_SIZE) * 2U);
			flash_write(dst_addr, data, FLASH_ROW_SIZE);
			j=0;
		}
		checkSum += x;
	}
	if(j>0)
	{
		uint32_t dst_addr = FASTCAL_FLASH_BASE + (i - j)*2U;
		flash_write(dst_addr, data, j);
	}
	flash_write(FASTCAL_CHECKSUM_ADDR, &checkSum, 1U);
	m_fastCalValid = true;
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
	uint16_t stored_checksum;
    flash_read(FASTCAL_CHECKSUM_ADDR, &stored_checksum,1U);

	if(checkSum != stored_checksum || NonZero != true)
	{
		saveToFlash();
	}
	else
	{
		m_fastCalValid = true;
	}
}

void AngleEncoder::calibrationInit(void)
{
	uint16_t i;
	if(m_nvmFlashCal->status == CalStatus::valid)
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