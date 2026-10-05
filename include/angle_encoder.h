#ifndef ANGLE_ENCODER_H
#define ANGLE_ENCODER_H

#include <stdbool.h>
#include <stdint.h>

#include "at32f403a_407.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "flash.h"
#ifdef __cplusplus
}
#endif

//==================== 校准参数（对齐原版MKS‑SERVO42B，适配16bit）====================
#define	CALIBRATION_TABLE_SIZE		    (1024U)
#define CALIBRATION_WRAP 			    ((int32_t)32768)
#define CALIBRATION_STEPS 			    ((uint32_t)65536U)
#define ANGLE_WRAP 					    ((int32_t)32768)
#define ANGLE_STEPS 				    ((uint32_t)65536U)
#define ANGLE_MAX 					    ((uint16_t)65535U)

#define CALIBRATION_ERROR_NOT_SET 	    (-1)
#define CALIBRATION_MIN_ERROR 		    (2)

#define DIVIDE_WITH_ROUND(x,y) 		    ( ( (x) + ((y) >> 1) ) / (y) )

#define FLASH_ROW_SIZE                  (1024U)  // 一个完整2KB Flash扇区，单位为halfword

//==== Flash分配 AT32F403A 0x08000000 base ====
// FastCal：芯片末尾完整128KB连续空间，全部归FastCal，不能插入任何其他数据
#define FASTCAL_FLASH_BASE              (0x08020000U)  // 128k
// MainCal主表、checksum放在FastCal区域【前面】，不属于FastCal块内部
#define MAINCAL_FLASH_BASE              (0x0801F800U)  // 兼容保留原主表地址，独占2KB扇区
#define MAINCAL_META_BASE               (0x0801F000U)  // 与主表分离的2KB元数据扇区
#define FASTCAL_CHECKSUM_ADDR           (MAINCAL_META_BASE + 8U)
#define CALIBRATION_STORAGE_MAGIC       (0x2333U)

enum class CalStatus : uint16_t
{
    invalid = 0,
    valid = 1
};

typedef struct {
    uint16_t value;
    int16_t  error;
} CalData_t;

typedef struct {
    uint16_t magic;
    CalStatus status;
    uint16_t MIN;
    uint16_t MAX;
	uint16_t fast_checksum;
} FlashCalMetadata_t;
static_assert(sizeof(FlashCalMetadata_t) == 10U, "Flash calibration metadata layout must remain halfword-packed");

typedef struct
{
    uint16_t angle[65536U];
} FastCalTable_t;

//==== 硬件引脚定义 ====
#define KTH7823_SPI              SPI2
#define KTH7823_SCLK_PORT        GPIOB
#define KTH7823_SCLK_PIN         GPIO_PINS_13
#define KTH7823_MISO_PORT        GPIOB
#define KTH7823_MISO_PIN         GPIO_PINS_14
#define KTH7823_MOSI_PORT        GPIOB
#define KTH7823_MOSI_PIN         GPIO_PINS_15
#define KTH7823_CS_PORT          GPIOB
#define KTH7823_CS_PIN           GPIO_PINS_12
#define KTH7823_MGH_PORT         GPIOB
#define KTH7823_MGH_PIN          GPIO_PINS_1
#define KTH7823_MGL_PORT         GPIOA
#define KTH7823_MGL_PIN          GPIO_PINS_2

struct EncoderCalibrationConfig
{
    bool enable_offset_calibration;
    bool enable_direction_calibration;
    bool enable_noise_calibration;
    bool enable_walk_calibration;
    uint16_t sample_count;
    float noise_threshold;
    float walk_threshold;
    int16_t offset_correction;
    bool invert_direction;
};

struct EncoderCalibrationResult
{
    bool offset_ok;
    bool direction_ok;
    bool noise_ok;
    bool walk_ok;
    int32_t offset_correction;
    float noise_rms;
    float walk_peak;
};

namespace encoder_common
{
    void encoder_write_gpio(gpio_type* port, uint16_t pin, bool state);
    void encoder_gpio_config_input(gpio_type* port, uint16_t pin);
    void encoder_gpio_config_output(gpio_type* port, uint16_t pin);
    uint16_t encoder_spi2_rw16(uint16_t tx_data);
}

struct EncoderFilterConfig
{
    static constexpr uint16_t kDefaultWindow = 2;
    uint16_t window_size = kDefaultWindow;
};

/**
 * @brief AngleEncoder基类：内置完整原版MKS‑SERVO42B非线性校准算法
 *  1. RAM稀疏表CalData[1024]
 *  2. Flash主表FlashCalData_t
 *  3. Flash FastCal稠密表65536点 + checksum校验
 *  4. fastReverseLookup优先O(1)，校验失败自动回退reverseLookup慢速遍历
 */
class AngleEncoder
{
public:
    virtual ~AngleEncoder() = default;

    //==== 硬件读写虚接口，子类实现 ====
    virtual bool init() = 0;
    virtual uint16_t readRaw() = 0;
    virtual uint16_t readFilteredRaw();
    virtual float readFilteredAngle();
    virtual bool updateFilteredSample()
    {
        (void)readRaw();
        return true;
    }
    virtual bool magneticFieldHigh() const { return false; }
    virtual bool magneticFieldLow() const { return false; }
    virtual void setZero(float zero_angle) = 0;
    virtual bool calibrate(const EncoderCalibrationConfig& config, EncoderCalibrationResult* result) = 0;

    //==================== 【校准对外API】全部基类实现，子类无需重写 ====================
    bool isNonlinearCalValid(void) const;
    uint16_t getCorrectedRaw(uint16_t raw);
    void feedCalibrationSample(uint16_t stepIndex, uint16_t raw);
    bool saveCalibration(void);
    void clearCalibration(void);
	uint16_t calibrationMainValue(uint16_t index) const;
	uint16_t calibrationFastValue(uint32_t index) const;
	uint16_t calibrationChecksum() const;

protected:
    //==== RAM运行时校准存储 ====
    volatile CalData_t m_calData[CALIBRATION_TABLE_SIZE];
    volatile bool m_fastCalValid = false;
	uint16_t calibration_checksum_ = 0U;

    //==== Flash常量指针，直接映射物理地址 ====
	const uint16_t* const m_nvmFlashCal = reinterpret_cast<const uint16_t*>(MAINCAL_FLASH_BASE);
	const FlashCalMetadata_t* const m_nvmFlashMeta = reinterpret_cast<const FlashCalMetadata_t*>(MAINCAL_META_BASE);
    const FastCalTable_t* const m_nvmFastCal  = reinterpret_cast<const FastCalTable_t*>(FASTCAL_FLASH_BASE);
	uint16_t calibration_min_ = 0U;
	uint16_t calibration_max_ = ANGLE_MAX;

    //==================== 【原版校准内部工具函数，基类protected实现】====================
    int32_t fastAbs(int32_t v) const;
    uint16_t getTableIndex(uint16_t value) const;
    bool updateTableValue(uint16_t index, uint16_t value);
    bool checkCalTableComplete(void) const;

    static uint16_t interp(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x);
    static uint16_t interp2(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x);

    uint16_t reverseLookup(uint16_t encoderAngle) const;
    uint16_t fastReverseLookup(uint16_t encoderAngle) const;
    uint16_t getCal(uint16_t actualAngle) const;

    void loadFromFlash(void);
	bool saveToFlash(bool yield_between_pages = false);
	bool createFastCal(bool yield_between_pages = false);
    void updateFastCalCheck(void);
    void calibrationInit(void);
};

//==================== 基类inline实现，放在头文件 ====================
inline int32_t AngleEncoder::fastAbs(int32_t v) const
{
    return v >= 0 ? v : -v;
}

inline uint16_t AngleEncoder::getTableIndex(uint16_t value) const
{
	int32_t x;
	x = ( ( (int32_t)value * CALIBRATION_TABLE_SIZE ) / ANGLE_STEPS );
	return static_cast<uint16_t>(x);
}

inline bool AngleEncoder::updateTableValue(uint16_t index, uint16_t value)
{
	if(index >= CALIBRATION_TABLE_SIZE) return false;
	m_calData[index].value =	value;
	m_calData[index].error = static_cast<int16_t>(CALIBRATION_STEPS / CALIBRATION_TABLE_SIZE);
	return true;
}

inline bool AngleEncoder::checkCalTableComplete(void) const
{
	for (uint16_t i=0; i < CALIBRATION_TABLE_SIZE; i++)
	{
		if (m_calData[i].error == CALIBRATION_ERROR_NOT_SET)
		{
			return false;
		}
	}
	return true;
}

inline uint16_t AngleEncoder::interp(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x)
{
	int32_t dx,dy,dx2,y;
	dx = x2 - x1;
	dy = y2 - y1;
	dx2 = x - x1;
	y = y1 + static_cast<int32_t>(DIVIDE_WITH_ROUND((dx2 * dy),dx));
	if (y < 0)
	{
		y = y + ANGLE_STEPS;
	}
	if (y > ANGLE_MAX)
	{
		y = y - ANGLE_STEPS;
	}
	return static_cast<uint16_t>(y);
}

inline uint16_t AngleEncoder::interp2(int32_t x1, int32_t y1, int32_t x2, int32_t y2, int32_t x)
{
	int32_t dx,dy,dx2,y;
	dx = x2 - x1;
	dy = y2 - y1;
	dx2 = x - x1;
	y = y1 + static_cast<int32_t>(DIVIDE_WITH_ROUND((dx2 * dy),dx));
	if (y < 0)
	{
		y = y + CALIBRATION_STEPS;
	}
	if (y >= static_cast<int32_t>(CALIBRATION_STEPS))
	{
		y = y - CALIBRATION_STEPS;
	}
	return static_cast<uint16_t>(y);
}

inline uint16_t AngleEncoder::reverseLookup(uint16_t encoderAngle) const
{
	int32_t i = 0;
	int32_t a1,a2;
	int32_t b1,b2;
	int32_t x;
	uint16_t y;
	x = static_cast<int32_t>(encoderAngle);
	if (x < static_cast<int32_t>(calibration_min_))
	{
		x = x + CALIBRATION_STEPS;
	}
	i = 0;
	while(i < CALIBRATION_TABLE_SIZE)
	{
		a1 = static_cast<int32_t>(m_calData[i].value);
		if(i == (CALIBRATION_TABLE_SIZE - 1))
		{
			a2 = static_cast<int32_t>(m_calData[0].value);
		}else
		{
			a2 = static_cast<int32_t>(m_calData[i+1].value);
		}
		if (fastAbs(a1 - a2) > CALIBRATION_WRAP)
		{
			if (a1 < a2)
			{
				a1 = a1 + CALIBRATION_STEPS;
			}else
			{
				a2 = a2 + CALIBRATION_STEPS;
			}
		}
		if ( (x >= a1 && x <= a2) || (x >= a2 && x <= a1) )
		{
			b1 = static_cast<int32_t>(DIVIDE_WITH_ROUND(((int64_t)i     * ANGLE_STEPS),CALIBRATION_TABLE_SIZE));
			b2 = static_cast<int32_t>(DIVIDE_WITH_ROUND(((int64_t)(i+1) * ANGLE_STEPS),CALIBRATION_TABLE_SIZE));
			y = interp(a1,b1,a2,b2,x);
			return y;
		}
		i++;
	}
	return 0U;
}

inline uint16_t AngleEncoder::fastReverseLookup(uint16_t encoderAngle) const
{
	if (m_fastCalValid == true)
	{
		return m_nvmFastCal->angle[encoderAngle];
	}else
	{
		return reverseLookup(encoderAngle);
	}
}

inline uint16_t AngleEncoder::getCal(uint16_t actualAngle) const
{
	uint16_t indexLow,indexHigh;
	int32_t x1,x2,y1,y2;
	uint16_t value;
	indexLow  = getTableIndex(actualAngle);
	indexHigh = indexLow + 1;
	x1 = ((int32_t)indexLow  * ANGLE_STEPS) / CALIBRATION_TABLE_SIZE;
	x2 = ((int32_t)indexHigh * ANGLE_STEPS) / CALIBRATION_TABLE_SIZE;

	if(indexHigh >= CALIBRATION_TABLE_SIZE)
	{
		indexHigh -= CALIBRATION_TABLE_SIZE;
	}
	y1 = m_calData[indexLow].value;
	y2 = m_calData[indexHigh].value;

	if (fastAbs(static_cast<int32_t>(y2 - y1)) > CALIBRATION_WRAP)
	{
		if (y2 < y1)
		{
			y2 = y2 + CALIBRATION_STEPS;
		}else
		{
			y1 = y1 + CALIBRATION_STEPS;
		}
	}
	value = interp2(x1, y1, x2, y2, actualAngle);
	return value;
}

//==== 对外API实现 ====
inline bool AngleEncoder::isNonlinearCalValid(void) const
{
	return m_nvmFlashMeta->magic == CALIBRATION_STORAGE_MAGIC &&
		   m_nvmFlashMeta->status == CalStatus::valid && m_fastCalValid;
}

inline uint16_t AngleEncoder::getCorrectedRaw(uint16_t raw)
{
    return fastReverseLookup(raw);
}

inline uint16_t AngleEncoder::calibrationMainValue(uint16_t index) const
{
	return index < CALIBRATION_TABLE_SIZE ? m_nvmFlashCal[index] : 0U;
}

inline uint16_t AngleEncoder::calibrationFastValue(uint32_t index) const
{
	return index < 65536U ? m_nvmFastCal->angle[index] : 0U;
}

inline uint16_t AngleEncoder::calibrationChecksum() const
{
	return m_nvmFlashMeta->fast_checksum;
}

inline void AngleEncoder::feedCalibrationSample(uint16_t stepIndex, uint16_t raw)
{
    updateTableValue(stepIndex, raw);
}

inline bool AngleEncoder::saveCalibration(void)
{
    if(!checkCalTableComplete())
        return false;
	return saveToFlash(true);
}

inline void AngleEncoder::clearCalibration(void)
{
    for(uint32_t i=0; i < CALIBRATION_TABLE_SIZE; i++)
    {
        m_calData[i].value = 0U;
        m_calData[i].error = CALIBRATION_ERROR_NOT_SET;
    }
    m_fastCalValid = false;
}

#endif
