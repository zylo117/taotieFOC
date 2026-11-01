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
	// ===========================================================================
	// 正向涂表，取代原来的“逐 raw 反查”
	//
	// 原实现：for raw in 0..65535: reverseLookup(raw)。而 reverseLookup 从 index 0
	// 往上扫配对、第一个匹配就 return。一旦主表的头尾两段认领了同一段 raw
	//（整圈进给不闭合时必然发生），同一个 raw 就会有两个相差几百度的答案，
	// 快表里出现悬崖——实测 raw 65463 -> 352.870°、raw 65465 -> 0.005°，
	// 现象就是“锁在 0 度时角度偶尔跳到 350 多度”。
	//
	// 新做法：把主表展开成单调序列，按 raw 升序一次性涂满 65536 项：
	//   1) 逐格取环绕步长累加得到总推进 span；
	//   2) 用 65536/span 归一化 raw 轴 —— 一圈对不齐的“比例”部分本质上不可能用
	//      65536 项周期表表示（头尾必然重叠），先把这部分比例摊掉，表只留真正的非线性残差；
	//      比例误差由 EncoderCalibrator 的 [CAL] closure 单独报出，归属运动刻度；
	//   3) 每个 raw 只会被一个区间写到，构造上就不可能出现悬崖；
	//   4) 顺带比原来快很多（原来每个 raw 最多扫 1024 个配对，整表要几秒）。
	// ===========================================================================
	static uint16_t data[FLASH_ROW_SIZE];
	uint16_t checksum = 0U;

	const int32_t nodeCount = static_cast<int32_t>(CALIBRATION_TABLE_SIZE);
	const int32_t idealStep = static_cast<int32_t>(CALIBRATION_STEPS / CALIBRATION_TABLE_SIZE);

	// ---- 第一趟：展开成单调序列，求总推进 span ----
	int32_t prevRaw = static_cast<int32_t>(m_calData[0].value);
	int32_t span = 0;
	for (int32_t i = 1; i < nodeCount; ++i)
	{
		const int32_t cur = static_cast<int32_t>(m_calData[i].value);
		int32_t step = cur - prevRaw;
		while (step > CALIBRATION_WRAP)
		{
			step -= static_cast<int32_t>(CALIBRATION_STEPS);
		}
		while (step < -CALIBRATION_WRAP)
		{
			step += static_cast<int32_t>(CALIBRATION_STEPS);
		}
		span += step;
		prevRaw = cur;
	}
	// 展开失败（方向错、严重不闭合）时不建表；更细的闭合/单值判据在 EncoderCalibrator
	if (span <= static_cast<int32_t>(CALIBRATION_STEPS / 4U) ||
	    span >= static_cast<int32_t>(CALIBRATION_STEPS * 3U / 2U))
	{
		m_fastCalValid = false;
		return false;
	}

	// ---- 第二趟：按归一化后的 raw 升序涂表 ----
	int32_t pageBase = 0;   // data[] 当前页对应的全局 raw 起点
	int32_t cum = 0;        // 展开后的累计 raw 推进
	int32_t paintedQ = 0;   // 已涂到的归一化 raw 位置
	prevRaw = static_cast<int32_t>(m_calData[0].value);
	for (int32_t i = 1; i <= nodeCount; ++i)
	{
		int32_t targetQ;
		if (i < nodeCount)
		{
			const int32_t cur = static_cast<int32_t>(m_calData[i].value);
			int32_t step = cur - prevRaw;
			while (step > CALIBRATION_WRAP)
			{
				step -= static_cast<int32_t>(CALIBRATION_STEPS);
			}
			while (step < -CALIBRATION_WRAP)
			{
				step += static_cast<int32_t>(CALIBRATION_STEPS);
			}
			if (step <= 0)
			{
				// 展开后不再单调：这段 raw 没有唯一归属，表不可信
				m_fastCalValid = false;
				return false;
			}
			cum += step;
			prevRaw = cur;
			targetQ = static_cast<int32_t>((static_cast<int64_t>(cum) *
			                                static_cast<int64_t>(CALIBRATION_STEPS)) /
			                               static_cast<int64_t>(span));
			if (targetQ > static_cast<int32_t>(CALIBRATION_STEPS))
			{
				targetQ = static_cast<int32_t>(CALIBRATION_STEPS);
			}
		}
		else
		{
			targetQ = static_cast<int32_t>(CALIBRATION_STEPS);   // 归一化后必定正好收尾
		}

		const int32_t angleStart = (i - 1) * idealStep;
		const int32_t angleEnd = i * idealStep;
		const int32_t seg = targetQ - paintedQ;
		if (seg < 0)
		{
			m_fastCalValid = false;
			return false;
		}
		for (int32_t q = paintedQ; q < targetQ; ++q)
		{
			const int32_t off = q - paintedQ;
			const int32_t value = (seg == 0)
				? angleEnd
				: (angleStart + static_cast<int32_t>((static_cast<int64_t>(off) *
				                                    static_cast<int64_t>(angleEnd - angleStart)) / seg));
			const int32_t slot = q - pageBase;
			data[slot] = static_cast<uint16_t>(value & 0xFFFFU);
			checksum = static_cast<uint16_t>(checksum + data[slot]);
			if (((q + 1) % static_cast<int32_t>(FLASH_ROW_SIZE)) == 0)
			{
				if (flash_write(FASTCAL_FLASH_BASE + static_cast<uint32_t>(pageBase) * 2U,
				                data, FLASH_ROW_SIZE) != SUCCESS)
				{
					m_fastCalValid = false;
					return false;
				}
				if (yield_between_pages)
				{
					vTaskDelay(1U);
				}
				pageBase = q + 1;
			}
		}
		paintedQ = targetQ;
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