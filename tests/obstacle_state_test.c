/* Real nonblocking obstacle module with fake clock/IMU and inert motor IO.
 * These checks establish software transitions, not physical obstacle clearance. */
#include "auto_steps.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t now_ms;
static int aborted, valid = 1, commands, brakes, scripted_pitch;
static float pitch, yaw, gain = 0.3f, last_vx, last_vy, last_w;
uint32_t HAL_GetTick(void) { return now_ms; }
void osDelay(uint32_t ms) { now_ms += ms; }
int run_aborted(void) { return aborted; }
uint8_t imu_ok(void) { return (uint8_t)valid; }
float imu_yaw_deg(void) { return yaw; }
float imu_pitch_deg(void)
{
    return scripted_pitch ? ((now_ms >= 20u && now_ms < 40u) ? 3.0f : 0.0f) : pitch;
}
float step_heading_kp_deg(void) { return gain; }
void motion_brake(void) { brakes++; }
void motion_vel_set(float vx, float vy, float w)
{
    commands++; last_vx = vx; last_vy = vy; last_w = w;
}
#define CHECK(expr) do { if (!(expr)) { fprintf(stderr, "line %d: %s\n", __LINE__, #expr); return 1; } } while (0)

static void reset_case(void)
{
    cross_cancel();
    now_ms = 0u; aborted = 0; valid = 1; commands = brakes = 0;
    pitch = yaw = 0.0f; gain = 0.3f; scripted_pitch = 0;
    (void)cross_tune_set(2.0f, 0.5f);
}

int main(void)
{
    CrossSnapshot s;
    float rise, flat;
    int before;

    reset_case();
    aborted = 1;
    CHECK(!cross_begin(150.0f, 0.0f, 8000u));
    CHECK(cross_status() == CROSS_ABORT && commands == 0 && brakes > 0);

    reset_case();
    CHECK(cross_begin(150.0f, 0.0f, 8000u) && commands == 0);
    CHECK(cross_tick() == CROSS_RUNNING && commands == 1);
    aborted = 1;
    CHECK(cross_tick() == CROSS_ABORT && commands == 1);
    CHECK(cross_tick() == CROSS_ABORT && commands == 1);

    reset_case();
    valid = 0;
    CHECK(!cross_begin(150.0f, 0.0f, 8000u));
    CHECK(cross_status() == CROSS_IMUERR && commands == 0);
    valid = 1;
    CHECK(cross_begin(150.0f, 0.0f, 8000u));
    CHECK(cross_tick() == CROSS_RUNNING);
    before = commands; valid = 0;
    CHECK(cross_tick() == CROSS_IMUERR && commands == before);

    reset_case();
    CHECK(!cross_begin(150.0f, 0.0f, 0u));
    CHECK(cross_status() == CROSS_TIMEOUT && commands == 0);
    CHECK(cross_begin(150.0f, 0.0f, 100u));
    CHECK(cross_tick() == CROSS_RUNNING);
    now_ms = 100u; before = commands;
    CHECK(cross_tick() == CROSS_TIMEOUT && commands == before);

    /* Constant pitch, even a tilted installation, must never fabricate rise. */
    reset_case(); pitch = 7.0f;
    CHECK(cross_begin(150.0f, 0.0f, 3000u));
    for (now_ms = 0u; now_ms < 3000u; now_ms += 20u) CHECK(cross_tick() == CROSS_RUNNING);
    cross_get(&s);
    CHECK(s.window_full && !s.rise_seen && s.pp_deg == 0.0f);
    CHECK(cross_tick() == CROSS_TIMEOUT);

    /* At BT cadence, the actual time window remains one second, not 100*20ms. */
    reset_case(); scripted_pitch = 1;
    CHECK(cross_begin(150.0f, 0.0f, 3000u));
    for (now_ms = 0u; now_ms < 1020u; now_ms += 20u) {
        CHECK(cross_tick() == CROSS_RUNNING);
        cross_get(&s);
        if (now_ms < 1000u) CHECK(!s.window_full);
    }
    before = commands;
    CHECK(cross_tick() == CROSS_DONE && commands == before);
    cross_get(&s);
    CHECK(s.rise_seen && s.window_full && s.elapsed_ms == 1020u);
    CHECK(cross_tick() == CROSS_DONE && commands == before);
    CHECK(strcmp(cross_status_name(s.status), "DONE") == 0);

    /* The unchanged blocking public API uses exactly the same state machine. */
    reset_case(); scripted_pitch = 1;
    CHECK(step_cross_obstacle(150.0f, 0.0f, 3000u) == 1);
    CHECK(now_ms >= 1000u && now_ms <= 1040u && cross_status() == CROSS_DONE);

    /* Negative body directions are preserved, as required by the old API. */
    reset_case();
    CHECK(cross_begin(-150.0f, -20.0f, 3000u));
    CHECK(cross_tick() == CROSS_RUNNING && last_vx == -150.0f && last_vy == -20.0f);
    before = commands; cross_cancel();
    CHECK(!aborted && cross_status() == CROSS_ABORT);
    CHECK(cross_tick() == CROSS_ABORT && commands == before);
    CHECK(cross_begin(150.0f, 0.0f, 3000u));
    CHECK(!cross_begin(-150.0f, 0.0f, 3000u));
    CHECK(cross_tick() == CROSS_RUNNING && last_vx == 150.0f);

    /* Threshold updates are explicit, finite, ordered and stopped-only. */
    CHECK(!cross_tune_set(3.0f, 1.0f));
    cross_cancel();
    CHECK(!cross_tune_set(0.5f, 2.0f));
    CHECK(!cross_tune_set(2.0f, 0.0f));
    CHECK(!cross_tune_set(NAN, 0.5f));
    CHECK(!cross_tune_set(2.0f, NAN));
    CHECK(!cross_tune_set(INFINITY, 0.5f));
    CHECK(!cross_tune_set(21.0f, 1.0f));
    CHECK(cross_tune_set(3.0f, 1.0f));
    cross_tune_get(&rise, &flat);
    CHECK(rise == 3.0f && flat == 1.0f);
    CHECK(!cross_begin(NAN, 0.0f, 1000u) && cross_status() == CROSS_BAD_CONFIG);
    CHECK(!cross_begin(0.0f, 0.0f, 1000u) && cross_status() == CROSS_BAD_CONFIG);

    /* Yaw wrap and clamp prevent a 359->0 crossing producing a huge command. */
    reset_case(); yaw = 359.0f; gain = 5.0f;
    CHECK(cross_begin(150.0f, 0.0f, 3000u));
    yaw = 0.0f; CHECK(cross_tick() == CROSS_RUNNING);
    cross_get(&s); CHECK(s.yaw_error_deg == -1.0f && last_w < 0.0f && last_w > -0.1f);
    yaw = 90.0f; CHECK(cross_tick() == CROSS_RUNNING && last_w == -2.0f);

    /* A long missing execution interval cannot fabricate a complete window. */
    reset_case();
    CHECK(cross_begin(150.0f, 0.0f, 4000u));
    CHECK(cross_tick() == CROSS_RUNNING);
    now_ms = 20u; pitch = 3.0f; CHECK(cross_tick() == CROSS_RUNNING);
    now_ms = 2000u; pitch = 0.0f; CHECK(cross_tick() == CROSS_RUNNING);
    cross_get(&s); CHECK(!s.window_full && !s.rise_seen);

    /* uint32 tick rollover must preserve the deadline check. */
    reset_case(); now_ms = UINT32_MAX - 40u;
    CHECK(cross_begin(150.0f, 0.0f, 100u));
    CHECK(cross_tick() == CROSS_RUNNING);
    now_ms += 100u;
    CHECK(cross_tick() == CROSS_TIMEOUT);
    puts("obstacle state: abort, deadline, IMU loss, 1s window, wrapper, cancel, tuning and yaw guards passed");
    return 0;
}
