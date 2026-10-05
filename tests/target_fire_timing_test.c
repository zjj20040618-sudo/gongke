/* Real steps.c target settle + laser timers, synthetic ticks/pins only.
 * Link with section GC and disable MinGW unwind tables so unrelated
 * arm/vision/motion APIs are not retained or mocked.
 * This verifies requested timing/output order, not physical aim or light power. */
#include "steps.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "target timing line %d: %s tick=%lu\n", \
    __LINE__, #x, (unsigned long)now_ms); return 1; } } while (0)

typedef struct { int on; uint32_t at; } LaserEvent;
static LaserEvent laser_events[8];
static unsigned event_count, issues, brakes, fire_calls, return_calls, delay_calls;
static uint32_t now_ms, started_at, abort_after_ms, abort_at, elapsed_ms;
static int laser_on, moving, phase; /* phase 1=settling, 2=laser, 3=return */

uint32_t HAL_GetTick(void) { return now_ms; }
void motion_brake(void) { moving = 0; brakes++; }
void proto_receive_end(void) { } /* timer abort closes local RX, not camera TX */
void bp_laser_set(int on)
{
    if (event_count >= sizeof laser_events / sizeof laser_events[0]) { issues++; return; }
    laser_events[event_count].on = on != 0;
    laser_events[event_count++].at = now_ms;
    laser_on = on != 0;
    if (laser_on && (phase != 2 || moving || run_aborted() ||
                     (uint32_t)(now_ms - started_at) != 1000u)) issues++;
}

void osDelay(uint32_t ms)
{
    delay_calls++;
    for (uint32_t i = 0u; i < ms; i++) {
        if (moving || (phase == 1 && laser_on) || (phase == 2 && !laser_on)) issues++;
        now_ms++; elapsed_ms++;
        if (abort_after_ms && elapsed_ms == abort_after_ms) {
            abort_at = now_ms;
            run_abort();
        }
    }
}

static void reset_fixture(uint32_t tick, uint32_t stop_after)
{
    memset(laser_events, 0, sizeof laser_events);
    event_count = issues = brakes = fire_calls = return_calls = delay_calls = 0u;
    now_ms = started_at = tick; abort_after_ms = stop_after; abort_at = elapsed_ms = 0u;
    laser_on = 1; moving = 1; phase = 1; /* Helper must first turn off and brake. */
    run_reset();
}

static int run_target_sequence(void)
{
    phase = 1;
    if (!step_target_settle()) return 0;
    phase = 2; fire_calls++;
    if (!step_fire(TARGET_LASER_ON_MS)) return 0;
    if (laser_on || moving || run_aborted()) issues++;
    phase = 3; return_calls++; moving = 1;
    return 1;
}

static int check_success_and_rollover(void)
{
    static const uint32_t starts[] = {0u, UINT32_MAX - 400u};
    CHECK(TARGET_AIM_SETTLE_MS == 1000u && TARGET_LASER_ON_MS == 2000u);
    for (unsigned i = 0u; i < sizeof starts / sizeof starts[0]; i++) {
        reset_fixture(starts[i], 0u);
        CHECK(run_target_sequence() == 1 && issues == 0u && !run_aborted());
        CHECK(event_count == 3u && laser_events[0].on == 0 && laser_events[1].on == 1 && laser_events[2].on == 0);
        CHECK(laser_events[0].at == starts[i]);
        CHECK((uint32_t)(laser_events[1].at - starts[i]) == 1000u);
        CHECK((uint32_t)(laser_events[2].at - laser_events[1].at) == 2000u);
        CHECK(elapsed_ms == 3000u && brakes >= 1u && fire_calls == 1u && return_calls == 1u && !laser_on);
    }
    return 0;
}

static int check_abort_before_and_during_settle(void)
{
    static const uint32_t stops[] = {1u, 500u, 999u, 1000u};
    reset_fixture(0u, 0u); run_abort();
    CHECK(run_target_sequence() == 0 && issues == 0u && brakes >= 1u);
    CHECK(!laser_on && !moving && event_count == 1u && !laser_events[0].on);
    CHECK(now_ms == 0u && delay_calls == 0u && fire_calls == 0u && return_calls == 0u);
    for (unsigned i = 0u; i < sizeof stops / sizeof stops[0]; i++) {
        reset_fixture(100u, stops[i]);
        CHECK(run_target_sequence() == 0 && run_aborted() && issues == 0u);
        CHECK(event_count == 1u && !laser_events[0].on && !laser_on && !moving);
        CHECK(fire_calls == 0u && return_calls == 0u && elapsed_ms <= stops[i] + 1u);
        CHECK((uint32_t)(now_ms - abort_at) <= 1u);
    }
    /* Abort accepted immediately after successful settling must not energize. */
    reset_fixture(0u, 0u); CHECK(step_target_settle() == 1);
    run_abort(); phase = 2;
    CHECK(step_fire(TARGET_LASER_ON_MS) == 0 && !laser_on && event_count == 2u && elapsed_ms == 1000u);
    CHECK(!laser_events[1].on && delay_calls == 500u);
    /* Even a stale/high output at an aborted direct entry must be forced off. */
    reset_fixture(0u, 0u); run_abort(); phase = 2;
    CHECK(step_fire(TARGET_LASER_ON_MS) == 0 && !laser_on && event_count == 1u);
    CHECK(!laser_events[0].on && delay_calls == 0u && now_ms == 0u);
    return 0;
}

static int check_abort_during_laser(void)
{
    static const uint32_t stops[] = {1001u, 1750u, 2999u, 3000u};
    for (unsigned i = 0u; i < sizeof stops / sizeof stops[0]; i++) {
        reset_fixture(700u, stops[i]);
        CHECK(run_target_sequence() == 0 && run_aborted() && issues == 0u);
        CHECK(event_count == 3u && !laser_events[0].on && laser_events[1].on && !laser_events[2].on);
        CHECK(!laser_on && !moving && fire_calls == 1u && return_calls == 0u);
        CHECK(elapsed_ms <= stops[i] + 1u && (uint32_t)(now_ms - abort_at) <= 1u);
        CHECK((uint32_t)(laser_events[2].at - laser_events[1].at) <= 2000u);
    }
    return 0;
}

int main(void)
{
    CHECK(check_success_and_rollover() == 0);
    CHECK(check_abort_before_and_during_settle() == 0);
    CHECK(check_abort_during_laser() == 0);
    puts("target timing: real steps.c stopped/off1000ms -> laser2000ms -> off before continuation; 2 normal/wrap starts, entry/between-phase and 8 timed aborts passed (no hardware)");
    return 0;
}
