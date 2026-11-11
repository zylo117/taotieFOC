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
// 【点数选什么样的：要整除“每圈微步数”，而不是整除 65536】
//   运动轴：12800 微步/圈（200 整步 × 64 细分），800 点 -> 每段正好 16 微步；
//           32 细分 6400 -> 8 微步；128 细分 25600 -> 32 微步，三种细分全整除。
//   传感器轴：65536/800 = 81.92 不整除 —— 这不再要求整除：建表里的节点角度
//           已改成 (int64)i*65536/N 精确计算，理论推进量也用 kExpectLsb 精确算
//           （旧写法用整数商会把整表角度少算几百 LSB）。
// 【硬上限：N ≤ 1024】主表在 Flash 里只占一个 2KB 扇区（MAINCAL_FLASH_BASE），
// 每项 2 字节 -> 1024 项正好 2048B；再大必须先扩 Flash 布局，否则 0x08020000
// 开始的 128KB FastCal 会被主表写坏。
// 代价对比：256 点=1.406° 间隔，800 点=0.450°（接近旧的 1024 点的 0.352°）；
// RAM：主表 3.2KB + 涂表边界表 3.2KB + 正向缓存 1.6KB；校准时长约 2.6 分钟。
#define	CALIBRATION_TABLE_SIZE		    (800U)
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

    // 上一次 SPI 传输是否超时。调用方靠它判样本有效性，不能再拿 0xFFFF 当错误码
    //（那是个合法角度）。本拍内读、本拍内用；跨拍读会读到后来的传输结果。
    bool encoder_spi2_last_timeout();
}

struct EncoderFilterConfig
{
    // 【窗口默认值 = 4，与中断里的喷发次数一致】
    // 采样在 TMR4 中断里喷发 4 次（SPI 7.8MHz，2.05us/次，总跨度 8.2us）。
    // 窗口 = 4 时，每拍刚好把窗口整个替换一遍：
    //   - 滤波时间跨度 8.2us，远小于 50us 控制周期；
    //   - 没有跨周期历史，也就没有群延迟；
    //   - 降噪 sqrt(4) = 2 倍。
    // 提醒：窗口 > 4 就会开始跨控制周期（每超 1 就多 2.05us 历史），
    // 调参时不要再按旧日“25 = 800us”的习惯填。
    static constexpr uint16_t kDefaultWindow = 4;
    // 窗口上限：也就是 delta_win_ 数组长度。窗口越大越平滑，但 RAM 占用和中断里的
    // 平移环操作都会线性增长。按真实采样周期 32us 算，256 个样本 = 8.2ms，
    // 已经是 20kHz 控制周期的 160 倍，没必要再大。
    // 提醒：默认值 25 样本 = 800us，也是 40 个控制周期，滤波强度偏大；
    // 跟随误差随窗口线性变大，调参时建议先试 8~12。
    static constexpr uint16_t kMaxWindow = 256;
    uint16_t window_size = kDefaultWindow;
};

/**
 * @brief AngleEncoder基类：内置完整原版MKS‑SERVO42B非线性校准算法
 *  1. RAM稀疏表CalData[CALIBRATION_TABLE_SIZE]
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

    // 未经过非线性校准表的滤波原始值。
    // 默认实现直接返回校准后的值（对不实现的子类保持旧行为），
    // Kth7823Encoder 会覆盖它，返回真正的未查表值。
    // 控制器的“行程累计”必须用它：校准表修正的是 INL，修正量的积分不闭合，
    // 用校准后值累加 50 圈会累积出 9° 的比例误差（实测）。
    virtual float readFilteredAngle();
    virtual bool updateFilteredSample()
    {
        (void)readRaw();
        return true;
    }
    virtual bool magneticFieldHigh() const { return false; }

    // 上一次 readRaw() 的 SPI 传输是否成功。默认 true（不支持该信息的子类保持旧行为），
    // Kth7823Encoder 会用真实的超时标志覆盖它。
    // 存在校准/滤波路径上原来用“raw == 0xFFFF”判无效，那会：
    //   1) 把真角 359.99° 的合法样本当垃圾丢掉（接缝节点的去极值平均因此被拉偏）；
    //   2) 把真正的超时样本（查表后不再是 0xFFFF）当有效值写进表。
    virtual bool lastReadValid() const { return true; }
    virtual bool magneticFieldLow() const { return false; }
    virtual void setZero(float zero_angle) = 0;

    // ===== 磁钢极性（方向）修正 =====
    // 换电机/换磁铁时磁钢可能被轴向翻转，KTH7823 读数变成镜像（reading = C - true），
    // 闭环归零会从负反馈变正反馈 —— 转子在 180° 对面来回震荡永远归不了零（实机现象）。
    // 校准入口的极性探针鉴别出反装后置位本标志：readRaw() 在查表之前整体取负，
    // 滤波/校准表/行程累计/角度输出一律工作在同一个"反转坐标系"里；
    // 常数 C（180° 或轴向相关的偏置）由校准归零吸收，无需知道。
    // 标志与校准表同域：校准完成保存表时一并持久化（param_store v7 encoder_invert）。
    void setInvertRaw(bool on) { invert_raw_ = on; }
    bool isInvertRaw() const { return invert_raw_; }

    // 请求切换滤波窗口。只登记请求，真正的重配由实现方在采样中断里做，
    // 否则会和正在跑的滤波状态（环形缓冲/累加和）抢数据。
    virtual void requestFilterWindow(uint16_t window_size) { (void)window_size; }

    //校准对外api
    bool isNonlinearCalValid(void) const;

    // 诊断：true 时 getCorrectedRaw 直接返回原始值，跳过非线性校准表。
    // 运行时可通过寄存器 0x153 切换，用来做“查表 / 不查表”的 A/B 对比。
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
    // 磁钢反装标志（见 public setInvertRaw 注释）。只在转子静止时改（校准探针），
    // 采样中断读它，volatile 足够。
    volatile bool invert_raw_ = false;
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
    // 【诊断宏】在 platformio.ini 的 build_flags 里加
    //   -DDISABLE_ENCODER_CALIBRATION_TABLE
    // 就能完全绕过非线性校准表（所有读取都返回原始值），
    // 用来验证“周期性误差是不是表引入的”：同一份固件只改这一个开关就能 A/B 对比。
#ifdef DISABLE_ENCODER_CALIBRATION_TABLE
    return raw;
#else
    return fastReverseLookup(raw);
#endif
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
