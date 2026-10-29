#ifndef LOAD_ANGLE_H
#define LOAD_ANGLE_H

#include <cstdint>

class LoadAngle
{
public:
    // 【拟合点数上限 = 32】
    // 旧值 1024 在实际使用中完全不必要，而且代价惊人：
    //   points_[1024]  = 8KB（Point 是 uint16+int32，对齐到 8 字节）
    //   received_[1024] = 1KB
    //   load_angle.cpp 里的 load_angle_flash_words[1024*3] = 6KB
    //   合计 15KB，占整个 RAM 的 37%
    // 更严重的是 fit() 要跑 5 层精化 x 20 个 exponent x N 点 x 2 次 std::pow(double)，
    // 1024 点就是 20 万次 pow ≈ 1.6 秒；而 loadFromFlash() 在控制器 init() 里调用，
    // 一旦 Flash 里存了满 1024 点，每次上电都要卡 1.6 秒。
    // 负载角随转速的曲线是单调平滑的，16~32 个点拟合已经完全足够。
    // 改为 32 后：RAM 从 15KB 降到 0.5KB，fit() 降到约 50ms。
    // 兼容性：Flash 里旧数据若点数 > 32，loadFromFlash() 会当作无效处理（重新上传即可）。
    static const uint16_t MAX_POINTS = 32U;

    LoadAngle();

    bool beginUpload(uint16_t count);
    bool setPoint(uint16_t index, uint16_t speed_rpm, int32_t load_angle_mdeg);
    bool fit();
    bool saveToFlash();
    bool loadFromFlash();
    bool getPoint(uint16_t index, uint16_t* speed_rpm, int32_t* load_angle_mdeg) const;

    uint16_t pointCount() const;
    bool isFitted() const;
    float powerCoefficient() const;
    float interceptDeg() const;
    float exponent() const;
    int32_t getLoadAngleMilliDegrees(uint16_t speed_rpm) const;

private:
    struct Point
    {
        uint16_t speed_rpm;
        int32_t load_angle_mdeg;
    };

    static const uint32_t FLASH_DATA_BASE = 0x0801D000U;
    static const uint32_t FLASH_META_BASE = 0x0801C000U;
    static const uint32_t FLASH_MAGIC = 0x4C414E47U;

    Point points_[MAX_POINTS];
    bool received_[MAX_POINTS];
    uint16_t point_count_;
    uint16_t received_count_;
    bool fitted_;
    float power_coefficient_;
    float intercept_deg_;
    float exponent_;
};

#endif