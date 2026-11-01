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
	// 正向涂表：按 raw 下标把 65536 项一次涂满，取代原来的“逐 raw 调 reverseLookup”
	//
	// 原做法的问题：reverseLookup 从 index 0 往上扫配对、第一个匹配就 return。主表一旦
	// 出现“同一段 raw 被两段索引区间同时认领”（整圈进给不闭合时必然发生），同一个 raw
	// 就有两个相差几百度的答案，谁先被扫到谁说了算 —— 快表里就此出现悬崖。实测
	// raw 65463 -> 352.870°、raw 65465 -> 0.005°（4 个 LSB 内跳 352°），表现就是
	// “锁在 0 度附近时角度偶发跳到 358/352 度” —— 与电流、PID、机构、传感器无关。
	//
	// 新做法有两个关键，都是这次踩过的位置：
	//   1) 表的下标是 raw，所以必须按 raw 定位。扫描推进量只是中间坐标：
	//      真角 0° 对应的 raw 是 m_calData[0].value（一般不是 0），拿推进量当下标
	//      会把整张表错位这么多 LSB，并在 raw 跨零点处又暴露成跳变。
	//   2) 先把“一圈对不齐”的比例部分用归一化摊掉：65536 项周期表在数学上表示不了
	//      不闭合的进给（头尾必然重叠）；归一化后表必然单调、单值、构造上无悬崖。
	//      这个比例误差由 EncoderCalibrator 的 [CAL] 报出来，归运动刻度处理。
	// 顺带：建表从“每个 raw 最多扫 N 个配对”改成整表线性扫描，快几个量级。
	// ===========================================================================
	static uint16_t data[FLASH_ROW_SIZE];
	static int32_t qBound[CALIBRATION_TABLE_SIZE + 1];
	uint16_t checksum = 0U;

	const int32_t nodeCount = static_cast<int32_t>(CALIBRATION_TABLE_SIZE);
	const int32_t idealStep = static_cast<int32_t>(CALIBRATION_STEPS / CALIBRATION_TABLE_SIZE);
	const int32_t rawSpan = static_cast<int32_t>(CALIBRATION_STEPS);
	const int32_t rawBase = static_cast<int32_t>(m_calData[0].value);   // 真角 0° 时的 raw

	// ---- 第一趟：按环绕步长把主表展开成单调序列，qBound[i] 暂存累计推进 ----
	int32_t prevRaw = rawBase;
	int32_t cum = 0;
	qBound[0] = 0;
	for (int32_t i = 1; i < nodeCount; ++i)
	{
		const int32_t cur = static_cast<int32_t>(m_calData[i].value);
		int32_t step = cur - prevRaw;
		while (step > CALIBRATION_WRAP)
		{
			step -= rawSpan;
		}
		while (step < -CALIBRATION_WRAP)
		{
			step += rawSpan;
		}
		if (step <= 0)
		{
			// 展开后不单调：这段 raw 没有唯一归属，表不可信
			m_fastCalValid = false;
			return false;
		}
		cum += step;
		prevRaw = cur;
		qBound[i] = cum;
	}
	if (cum <= (rawSpan / 4) || cum >= (rawSpan / 2 * 3))
	{
		m_fastCalValid = false;
		return false;
	}
	// 归一化：把整圈推进摊成正好 65536，最后一格必定正好收尾、不留未定义项
	for (int32_t i = 0; i < nodeCount; ++i)
	{
		qBound[i] = static_cast<int32_t>((static_cast<int64_t>(qBound[i]) * rawSpan) / cum);
	}
	qBound[nodeCount] = rawSpan;

	// ---- 第二趟：按 raw 升序涂表（raw 才是表的数组下标）----
	bool write_failed = false;
	auto paintRange = [&](int32_t rangeBegin, int32_t rangeEnd)
	{
		int32_t seg = 0;
		for (int32_t rawPos = rangeBegin; rawPos < rangeEnd; ++rawPos)
		{
			// 扫描从 rawBase 起绕一圈：raw < rawBase 属于后半圈
			int32_t q = rawPos - rawBase;
			if (q < 0)
			{
				q += rawSpan;
			}
			while (seg + 1 < nodeCount && q >= qBound[seg + 1])
			{
				++seg;
			}
			const int32_t segLen = qBound[seg + 1] - qBound[seg];
			const int32_t angle = (segLen <= 0)
				? ((seg + 1) * idealStep)
				: (seg * idealStep +
				   static_cast<int32_t>((static_cast<int64_t>(q - qBound[seg]) *
				                       static_cast<int64_t>(idealStep)) / segLen));
			const int32_t slot = rawPos % static_cast<int32_t>(FLASH_ROW_SIZE);
			data[slot] = static_cast<uint16_t>(angle & 0xFFFFU);
			checksum = static_cast<uint16_t>(checksum + data[slot]);
			if (((rawPos + 1) % static_cast<int32_t>(FLASH_ROW_SIZE)) == 0)
			{
				const uint32_t page_address = FASTCAL_FLASH_BASE +
				    static_cast<uint32_t>(rawPos + 1 - static_cast<int32_t>(FLASH_ROW_SIZE)) * 2U;
				if (flash_write(page_address, data, FLASH_ROW_SIZE) != SUCCESS)
				{
					write_failed = true;
					return;
				}
				if (yield_between_pages)
				{
					vTaskDelay(1U);
				}
			}
		}
	};
	// 两段各自内部 q 单调递增，段游标只前进。跳 rawBase 的那一页由两次调用互补填满：
	// 页内槽位按 rawPos % FLASH_ROW_SIZE 定位，所以仍是一个完整 1024 项才 flush 一次。
	paintRange(0, rawBase);
	if (!write_failed)
	{
		paintRange(rawBase, rawSpan);
	}
	if (write_failed)
	{
		m_fastCalValid = false;
		return false;
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