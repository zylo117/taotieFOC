#ifndef LAG_ANGLE_H
#define LAG_ANGLE_H

#include <cstdint>

class LagAngle
{
public:
    // 【拟合点数上限 = 128】
    // 旧值 1024 在实际使用中完全不必要，而且代价惊人：
    //   points_[1024]  = 8KB（Point 是 uint16+int32，对齐到 8 字节）
    //   received_[1024] = 1KB
    //   lag_angle.cpp 里的 lag_angle_flash_words[1024*3] = 6KB
    //   合计 15KB，占整个 RAM 的 37%
    // 更严重的是 fit() 要跑 5 层精化 x 20 个 exponent x N 点 x 2 次 std::pow(double)，
    // 1024 点就是 20 万次 pow ≈ 1.6 秒；而 loadFromFlash() 在控制器 init() 里调用，
    // 一旦 Flash 里存了满 1024 点，每次上电都要卡 1.6 秒。
    // 滞后角随转速的曲线是单调平滑的，16~32 个点拟合已经完全足够。
    // 改为 32 后：RAM 从 15KB 降到 0.5KB，fit() 降到约 50ms。
    // 兼容性：Flash 里旧数据若点数 > 32，loadFromFlash() 会当作无效处理（重新上传即可）。
    // 拟合点数上限。原值 32 是为了省 RAM（Point 8 字节/点 + received 1 字节）。
    // 实测需求：0~3000 RPM、步进 50 = 61 个点，往返后 121 个点，
    // 32 个明显不够（网页会直接报“单程速度点过多”）。
    // 128 点代价：points_ 1024B + received_ 128B + flash 缓冲 768B = 约 1.9KB RAM，
    // 对 229KB 的芯片完全可接受；拟合耗时会从 50ms 涨到约 200~300ms（只在启动时一次）。
    static const uint16_t MAX_POINTS = 128U;

    LagAngle();

    bool beginUpload(uint16_t count);
    bool setPoint(uint16_t index, uint16_t speed_rpm, int32_t lag_angle_mdeg);
    bool fit();
    bool saveToFlash();
    bool loadFromFlash();
    bool getPoint(uint16_t index, uint16_t* speed_rpm, int32_t* lag_angle_mdeg) const;

    uint16_t pointCount() const;
    bool isFitted() const;
    float powerCoefficient() const;
    float interceptDeg() const;
    float exponent() const;
    int32_t getLagAngleMilliDegrees(uint16_t speed_rpm) const;

private:
    struct Point
    {
        uint16_t speed_rpm;
        int32_t lag_angle_mdeg;
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