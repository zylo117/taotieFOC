//
// Created by Jessi on 2026/10/2.
//

#ifndef TAOTIEFOC_UTILS_H
#define TAOTIEFOC_UTILS_H

#ifndef USE_HARD_FLOAT_ACCELERATION
#define USE_HARD_FLOAT_ACCELERATION 1
#endif

#if USE_HARD_FLOAT_ACCELERATION
static inline float fast_abs(float value)
{
    return __builtin_fabsf(value);
}

static inline float fast_clamp(float value, float min_value, float max_value)
{
    return __builtin_fminf(__builtin_fmaxf(value, min_value), max_value);
}
#else
static inline float fast_abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

static inline float fast_clamp(float value, float min_value, float max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    if (value > max_value)
    {
        return max_value;
    }
    return value;
}
#endif

#endif //TAOTIEFOC_UTILS_H
