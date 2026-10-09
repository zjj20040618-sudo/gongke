#ifndef APP_YAW_PROGRESS_BOOST_H
#define APP_YAW_PROGRESS_BOOST_H

#include <stdint.h>
#include <math.h>

/* Mode31 candidate: keep normal correction, then use its existing angular
 * cap if error has not improved0.05deg for400ms. NOT torque sensing or
 * arrival proof; owner retains IMU/stop checks and the12s fault budget. */
typedef struct {
    uint32_t since;
    float reference;
    int8_t direction;
    uint8_t active, boosted;
} YawProgressBoost;
static inline void yaw_progress_reset(YawProgressBoost *s)
{
    s->active = s->boosted = 0u; s->direction = 0;
}
static inline float yaw_progress_floor(YawProgressBoost *s, float error,
                                      uint32_t now, float normal, float cap)
{
    int8_t direction = error > 0.0f ? 1 : -1;
    float magnitude = fabsf(error);
    if (!s->active || direction != s->direction || s->reference - magnitude >= 0.05f - 0.000001f) {
        s->active = 1u; s->boosted = 0u;
        s->direction = direction; s->reference = magnitude; s->since = now;
    } else if ((uint32_t)(now - s->since) >= 400u) s->boosted = 1u;
    return s->boosted ? cap : normal;
}
#endif
