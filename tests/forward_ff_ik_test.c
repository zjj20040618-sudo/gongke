/* Real motion.c IK and wheel-target dispatch, without physical hardware.
 * The nominal 1.25 mm/s body correction is quantized by integer wheel RPM.
 * Neither this model nor its pass result establishes actual ground drift. */
#include <math.h>
#include <stdio.h>
#include "motion.h"
#include "control.h"
#include "imu.h"

static int16_t delivered[4];
static unsigned delivered_mask;
static unsigned delivered_calls;
static unsigned precise_calls;

/* Capture the actual targets passed by motion_vel_set, not a copied IK. */
void ctrl_set_speed(int motor, int16_t rpm)
{
    if (motor >= 0 && motor < 4) {
        delivered[motor] = rpm;
        delivered_mask |= 1u << (unsigned)motor;
    }
    ++delivered_calls;
}

/* Legacy motion_vel_set must retain its integer entry point. */
void ctrl_set_speed_precise(int motor, float rpm)
{
    (void)motor;
    (void)rpm;
    ++precise_calls;
}

/* New low-speed entry only satisfies motion.c linkage in this legacy test. */
void ctrl_set_speed_creep(int motor, float rpm)
{
    ctrl_set_speed_precise(motor, rpm);
}

/* PE host linking still needs these peripheral references from motion.c. */
void ctrl_stop_all(void)
{
    for (int i = 0; i < 4; ++i) delivered[i] = 0;
}
int32_t ctrl_enc_total(int motor) { (void)motor; return 0; }
void ctrl_get_rpm_fast_all(float out[4])
{
    for (int i = 0; i < 4; ++i) out[i] = 0.0f;
}
uint32_t HAL_GetTick(void) { return 0u; }
uint8_t imu_ok(void) { return 0u; }
float imu_heading_deg(void) { return 0.0f; }

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "forward FF IK line %d: %s\n", __LINE__, #expr); \
        return 1; \
    } \
} while (0)

static int check_case(float vx, float vy, const int16_t expected[4])
{
    int16_t ik[4];
    delivered_mask = delivered_calls = precise_calls = 0u;
    for (int i = 0; i < 4; ++i) delivered[i] = -999;
    motion_ik(vx, vy, 0.0f, ik);
    motion_vel_set(vx, vy, 0.0f);
    CHECK(delivered_mask == 15u && delivered_calls == 4u);
    CHECK(precise_calls == 0u);
    for (int i = 0; i < 4; ++i) {
        CHECK(ik[i] == expected[i]);
        CHECK(delivered[i] == expected[i]);
    }
    return 0;
}

/* Reconstruct ideal body vy from the dispatched integer RPM targets using
 * current wheel radius 33 mm and the real IK sign convention, not real wheels. */
static float nominal_lateral_mms(const int16_t rpm[4])
{
    const float mm_per_s_per_rpm = 2.0f * 3.14159f * 33.0f / 60.0f;
    return (-rpm[0] + rpm[1] - rpm[2] + rpm[3]) *
           0.25f * mm_per_s_per_rpm;
}

int main(void)
{
    const int16_t straight[4] = { 28, 28, 28, 28 };
    const int16_t left_ff[4] = { 29, 28, 29, 28 };
    const int16_t right_ff[4] = { 28, 29, 28, 29 };
    const float speed = 100.0f;
    const float ratio = 0.0125f;
    const float commanded_left_mms = -speed * ratio;

    CHECK(check_case(speed, 0.0f, straight) == 0);
    CHECK(check_case(speed, commanded_left_mms, left_ff) == 0);
    CHECK(check_case(speed, -commanded_left_mms, right_ff) == 0);
    CHECK(nominal_lateral_mms(left_ff) < 0.0f);
    CHECK(nominal_lateral_mms(right_ff) > 0.0f);
    CHECK(fabsf(nominal_lateral_mms(left_ff) + 1.7278745f) < 0.0001f);
    CHECK(fabsf(nominal_lateral_mms(right_ff) - 1.7278745f) < 0.0001f);
    CHECK(fabsf(nominal_lateral_mms(left_ff) - commanded_left_mms) > 0.4f);
    /* Quantization plateau: fine ratio changes need not alter wheel commands. */
    CHECK(check_case(speed, -speed * 0.0100f, left_ff) == 0);
    CHECK(check_case(speed, -speed * 0.0150f, left_ff) == 0);
    CHECK(check_case(speed, -speed * 0.0010f, straight) == 0);
    /* Removing FF returns all four wheel targets to their uncorrected values. */
    CHECK(check_case(speed, 0.0f, straight) == 0);

    puts("real forward FF IK: v100 zero/positive/negative and dispatched integer targets passed; nominal quantized vy is not measured ground drift");
    return 0;
}
