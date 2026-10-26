#ifndef LOAD_ANGLE_H
#define LOAD_ANGLE_H

#include <cstdint>

class LoadAngle
{
public:
    static const uint16_t MAX_POINTS = 1024U;

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