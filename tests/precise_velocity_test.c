/* Host-only checks of the real IK, target dispatch and PI/PWM controller.
 * Include the sources so fractional stored targets/history can be inspected
 * without adding a production diagnostics API. No physical acceptance implied.
 * Build this file alone with -Itests/stubs -IApp and -lm. */
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "control.h"
#include "motion.h"
#include "board_pins.h"
#include "imu.h"

enum { HOST_SET, HOST_BRAKE, HOST_COAST };
typedef struct {
    int action;
    int dir;
    int duty;
} HostMotor;
static HostMotor host_motor[4];
static int32_t host_delta[4];
static uint32_t host_ms;

void bp_motor_set(int m, int dir, int duty)
{
    host_motor[m] = (HostMotor){ HOST_SET, dir, duty };
}
void bp_motor_brake(int m)
{
    host_motor[m] = (HostMotor){ HOST_BRAKE, 0, 0 };
}
void bp_motor_stop(int m)
{
    host_motor[m] = (HostMotor){ HOST_COAST, 0, 0 };
}
int32_t bp_enc_delta(int m) { return host_delta[m]; }
void bp_enc_raw_reset_all(void) { }
uint32_t HAL_GetTick(void) { return host_ms; }
uint8_t imu_ok(void) { return 1u; }
float imu_heading_deg(void) { return 0.0f; }

#include "../App/control.c"
#include "../App/motion.c"

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "precise velocity line %d: %s\n", __LINE__, #expr); \
        return 1; \
    } \
} while (0)

static void host_reset(void)
{
    const CtrlTune defaults = { 0.05f, 0.004f, 0.2f, 25u };
    memset(host_delta, 0, sizeof host_delta);
    host_ms = 0u;
    ctrl_init();
    (void)ctrl_tune_set(&defaults);
}

static void host_tick(void)
{
    ++host_ms;
    ctrl_tick_1ms();
}

static int check_precision(void)
{
    float base[4], corrected[4];
    int16_t old_base[4], old_corrected[4];
    const float w = -0.3f * 3.0f * 0.0174533f;
    const float mm_per_s_per_rpm = 2.0f * 3.14159f * 33.0f / 60.0f;
    float forward, sideways;
    host_reset();
    motion_ik_precise(100.0f, -1.25f, 0.0f, base);
    motion_ik_precise(100.0f, -1.25f, w, corrected);
    motion_ik(100.0f, -1.25f, 0.0f, old_base);
    motion_ik(100.0f, -1.25f, w, old_corrected);
    CHECK(memcmp(old_base, old_corrected, sizeof old_base) == 0);
    CHECK(old_base[0] == 29 && old_base[1] == 28 &&
          old_base[2] == 29 && old_base[3] == 28);
    for (int i = 0; i < 4; ++i) CHECK(fabsf(corrected[i] - base[i]) > 0.2f);
    CHECK(corrected[0] < base[0] && corrected[3] < base[3]);
    CHECK(corrected[1] > base[1] && corrected[2] > base[2]);
    motion_vel_set_precise(100.0f, -1.25f, w);
    for (int i = 0; i < 4; ++i) CHECK(s_target[i] == corrected[i]);
    forward = (base[0] + base[1] + base[2] + base[3]) *
              0.25f * mm_per_s_per_rpm;
    sideways = (-base[0] + base[1] - base[2] + base[3]) *
               0.25f * mm_per_s_per_rpm;
    CHECK(fabsf(forward - 100.0f) < 0.0001f);
    CHECK(fabsf(sideways + 1.25f) < 0.0001f);
    CHECK(fabsf(sideways / forward + 0.0125f) < 0.000001f);
    /* A correction smaller than 1 RPM must reach the real PI, not brake. */
    {
        const CtrlTune probe = { 2.0f, 0.0f, 1.0f, 0u };
        CHECK(ctrl_tune_set(&probe));
        ctrl_set_speed_precise(0, 0.75f);
        host_tick();
        CHECK(host_motor[0].action == HOST_SET && host_motor[0].duty == 1);
        ctrl_set_speed(0, 0);
        host_tick();
        CHECK(host_motor[0].action == HOST_BRAKE);
        ctrl_set_speed_precise(0, -0.75f);
        host_tick();
        CHECK(host_motor[0].action == HOST_SET && host_motor[0].duty == 1 &&
              host_motor[0].dir == BP_DIR_REV);
    }
    return 0;
}

static int check_legacy_dispatch(void)
{
    static const struct {
        float vx, vy, w;
        int16_t expected[4];
    } cases[] = {
        { 100.0f, 0.0f, 0.0f, { 28, 28, 28, 28 } },
        { 100.0f, -1.25f, 0.0f, { 29, 28, 29, 28 } },
        { -100.0f, 0.0f, 0.0f, { -28, -28, -28, -28 } },
        { 0.0f, -100.0f, 0.0f, { 28, -28, 28, -28 } },
        { 0.0f, 100.0f, 0.0f, { -28, 28, -28, 28 } },
        { 0.0f, 0.0f, 2.0f, { 28, -28, -28, 28 } },
        { 0.0f, 0.0f, -2.0f, { -28, 28, 28, -28 } },
        { 0.0f, 0.0f, 0.18f, { 2, -2, -2, 2 } },
    };
    host_reset();
    ctrl_set_speed(0, INT16_MIN);
    CHECK(s_target[0] == -32768.0f);
    ctrl_set_speed(0, INT16_MAX);
    CHECK(s_target[0] == 32767.0f);
    for (unsigned c = 0; c < sizeof cases / sizeof cases[0]; ++c) {
        int16_t old[4];
        float precise[4];
        motion_ik(cases[c].vx, cases[c].vy, cases[c].w, old);
        motion_ik_precise(cases[c].vx, cases[c].vy, cases[c].w, precise);
        motion_vel_set(cases[c].vx, cases[c].vy, cases[c].w);
        for (int i = 0; i < 4; ++i) {
            CHECK(old[i] == cases[c].expected[i]);
            CHECK(old[i] == (int16_t)precise[i]);
            CHECK(s_target[i] == (float)cases[c].expected[i]);
        }
    }
    return 0;
}

static int check_integer_controller_compatibility(void)
{
    enum { TICKS = 128 };
    HostMotor old_trace[TICKS][4];
    const int16_t target[4] = { 29, -29, 2, 0 };
    for (int pass = 0; pass < 2; ++pass) {
        host_reset();
        for (int m = 0; m < 4; ++m) {
            if (pass == 0) ctrl_set_speed(m, target[m]);
            else ctrl_set_speed_precise(m, (float)target[m]);
        }
        for (int tick = 0; tick < TICKS; ++tick) {
            for (int m = 0; m < 4; ++m)
                host_delta[m] = (tick + m) % 5 == 0 ? (m == 1 ? -1 : 1) : 0;
            host_tick();
            for (int m = 0; m < 4; ++m) {
                if (pass == 0) old_trace[tick][m] = host_motor[m];
                else {
                    CHECK(host_motor[m].action == old_trace[tick][m].action);
                    CHECK(host_motor[m].dir == old_trace[tick][m].dir);
                    CHECK(host_motor[m].duty == old_trace[tick][m].duty);
                }
            }
        }
    }
    return 0;
}

static int check_stop_and_open_loop(void)
{
    const CtrlTune probe = { 2.0f, 0.1f, 1.0f, 0u };
    host_reset();
    CHECK(ctrl_tune_set(&probe));
    for (int m = 0; m < 4; ++m) ctrl_set_speed_precise(m, 40.5f);
    for (int i = 0; i < 30; ++i) host_tick();
    CHECK(s_ei[0] > 1000.0f);
    ctrl_set_duty_open(0, -45);
    host_tick();
    CHECK(host_motor[0].action == HOST_SET &&
          host_motor[0].dir == BP_DIR_REV && host_motor[0].duty == 45);
    ctrl_set_duty_open(0, 0);
    host_tick();
    CHECK(host_motor[0].action == HOST_BRAKE);
    ctrl_set_duty_open(0, 999);
    host_tick();
    CHECK(host_motor[0].duty == (int)MOTOR_PWM_PERIOD);
    motion_brake();
    for (int m = 0; m < 4; ++m) {
        CHECK(s_target[m] == 0.0f && s_ei[m] == 0.0f && s_e_prev[m] == 0.0f);
        CHECK(host_motor[m].action == HOST_BRAKE);
    }
    host_tick();
    for (int m = 0; m < 4; ++m) CHECK(host_motor[m].action == HOST_BRAKE);
    ctrl_set_speed_precise(0, 0.75f);
    host_tick();
    CHECK(host_motor[0].action == HOST_SET && host_motor[0].duty == 1);
    for (int i = 0; i < 30; ++i) host_tick();
    CHECK(s_ei[0] > 0.0f);
    ctrl_coast_all();
    for (int m = 0; m < 4; ++m) {
        CHECK(s_target[m] == 0.0f && s_ei[m] == 0.0f && s_e_prev[m] == 0.0f);
        CHECK(host_motor[m].action == HOST_COAST);
    }
    host_tick();
    for (int m = 0; m < 4; ++m) CHECK(host_motor[m].action == HOST_COAST);
    ctrl_set_speed_precise(0, 0.75f);
    host_tick();
    CHECK(host_motor[0].action == HOST_SET && host_motor[0].duty == 1);
    return 0;
}

int main(void)
{
    CHECK(check_precision() == 0);
    CHECK(check_legacy_dispatch() == 0);
    CHECK(check_integer_controller_compatibility() == 0);
    CHECK(check_stop_and_open_loop() == 0);
    puts("precise velocity: fractional yaw/FFF reach real PI; legacy integer targets/PWM, open-loop, brake/coast and history reset passed");
    return 0;
}
