/* Real IMU parser/snapshot. IRQ hooks model a pending complete UART frame,
 * so the snapshot must copy pitch/roll/timestamp while interrupts are masked. */
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "../App/imu.h"

static uint32_t host_tick, host_primask;
static unsigned irq_disable_calls, irq_restore_calls;
static uint8_t pending_frame[24];
static unsigned pending_size;
static void dispatch_pending(void);
uint32_t HAL_GetTick(void)
{
    if (!host_primask && pending_size) dispatch_pending();
    return host_tick;
}
static uint32_t __get_PRIMASK(void) { return host_primask; }
static void __disable_irq(void) { host_primask = 1u; irq_disable_calls++; }
static void __set_PRIMASK(uint32_t value)
{
    host_primask = value; irq_restore_calls++;
    if (!value && pending_size) dispatch_pending();
}
#define HOST_TEST_MAIN_H
#include "../App/imu.c"

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "imu tilt line%d: %s\n", __LINE__, #x); return 1; } } while (0)
static int near(float a, float b) { return fabsf(a - b) < 0.001f; }

static unsigned pack(uint8_t *frame, unsigned len, uint8_t cmd,
                     uint16_t yaw100, int16_t pitch100, int16_t roll100)
{
    unsigned yaw = len == 6u ? 0u : 16u;
    unsigned pitch = len == 6u ? 2u : 12u;
    unsigned roll = len == 6u ? 4u : 14u;
    frame[0] = 0xAAu; frame[1] = 0x55u; frame[2] = 0x60u;
    frame[3] = cmd; frame[4] = (uint8_t)len;
    memset(frame + 5u, 0x13, len);
    frame[5u+yaw] = (uint8_t)yaw100; frame[6u+yaw] = (uint8_t)(yaw100 >> 8);
    frame[5u+pitch] = (uint8_t)pitch100; frame[6u+pitch] = (uint8_t)((uint16_t)pitch100 >> 8);
    frame[5u+roll] = (uint8_t)roll100; frame[6u+roll] = (uint8_t)((uint16_t)roll100 >> 8);
    uint8_t sum = 0u;
    for (unsigned i = 2u; i < len + 5u; ++i) sum = (uint8_t)(sum + frame[i]);
    frame[5u+len] = sum;
    return len + 6u;
}

static void feed(const uint8_t *frame, unsigned size)
{
    for (unsigned i = 0u; i < size; ++i) imu_feed(frame[i]);
}

static void dispatch_pending(void)
{
    uint8_t frame[24]; unsigned size = pending_size;
    memcpy(frame, pending_frame, size); pending_size = 0u;
    feed(frame, size);
}

static int snapshot_is(float pitch, float roll, uint32_t stamp)
{
    float p = 999.0f, r = 999.0f; uint32_t ms = UINT32_MAX;
    CHECK(imu_tilt_snapshot(&p, &r, &ms));
    CHECK(near(p, pitch) && near(r, roll) && ms == stamp);
    CHECK(imu_tilt_snapshot(NULL, NULL, NULL));
    return 0;
}

static int check_parser_frame_atomicity_and_zero(void)
{
    uint8_t frame[24];
    imu_init(); host_tick = 100u;
    float p = 11.0f, r = 22.0f; uint32_t stamp = 33u;
    CHECK(!imu_tilt_snapshot(&p, &r, &stamp) && p == 11.0f && r == 22.0f && stamp == 33u);
    unsigned size = pack(frame, 6u, 1u, 9000u, -1250, 245);
    feed(frame, size - 1u);
    CHECK(!imu_tilt_snapshot(&p, &r, &stamp));
    imu_feed(frame[size - 1u]); CHECK(snapshot_is(-12.5f, 2.45f, 100u) == 0);
    CHECK(near(imu_heading_deg(), 90.0f));
    host_tick = 150u;
    size = pack(frame, 18u, 1u, 9200u, 17990, -17960);
    feed(frame, size - 1u); CHECK(snapshot_is(-12.5f, 2.45f, 100u) == 0);
    imu_feed(frame[size - 1u]); CHECK(snapshot_is(179.9f, -179.6f, 150u) == 0);
    CHECK(near(imu_heading_deg(), 92.0f));
    CHECK(imu_zero_leg_heading() && near(imu_leg_heading_deg(), 0.0f) && near(imu_heading_deg(), 92.0f));
    CHECK(snapshot_is(179.9f, -179.6f, 150u) == 0);
    host_tick = 170u;
    size = pack(frame, 6u, 1u, 9300u, -45, 125);
    feed(frame, size); CHECK(snapshot_is(-0.45f, 1.25f, 170u) == 0);
    CHECK(near(imu_leg_heading_deg(), 1.0f));
    CHECK(irq_disable_calls == irq_restore_calls && !host_primask);
    puts("imu snapshot:6/18 layouts and signed hundredths, no partial-frame update, coherent timestamp, yaw zero leaves tilt unchanged passed");
    return 0;
}

static int check_bad_frames_age_and_recovery(void)
{
    uint8_t frame[24]; unsigned size;
    imu_init(); host_tick = 100u;
    size = pack(frame, 6u, 1u, 1000u, 120, -230); feed(frame, size);
    host_tick = 150u;
    size = pack(frame, 18u, 1u, 2000u, 500, 600); frame[size - 1u] ^= 1u;
    feed(frame, size); CHECK(snapshot_is(1.2f, -2.3f, 100u) == 0);
    CHECK(imu_last_valid_age_ms() == 50u);
    host_tick = 200u;
    size = pack(frame, 6u, 0xF0u, 3000u, 700, 800); feed(frame, size);
    CHECK(snapshot_is(1.2f, -2.3f, 100u) == 0 && imu_last_valid_age_ms() == 100u);
    host_tick = 299u; imu_feed(0x23u); imu_tick_parse();
    CHECK(snapshot_is(1.2f, -2.3f, 100u) == 0);
    host_tick = 300u;
    float p = 11.0f, r = 22.0f; uint32_t stamp = 33u;
    CHECK(!imu_ok() && !imu_tilt_snapshot(&p, &r, &stamp));
    CHECK(p == 11.0f && r == 22.0f && stamp == 33u && !imu_zero_leg_heading());
    host_tick = 310u; imu_feed(0x23u); imu_tick_parse();
    CHECK(!imu_tilt_snapshot(&p, &r, &stamp)); /* garbage activity cannot renew a valid pose */
    host_tick = 330u; size = pack(frame, 18u, 1u, 3500u, -150, 275); feed(frame, size);
    CHECK(snapshot_is(-1.5f, 2.75f, 330u) == 0 && imu_ok());
    host_tick = 600u; imu_tick_parse();
    CHECK(!imu_tilt_snapshot(NULL, NULL, NULL));
    puts("imu snapshot:bad checksum/ACK/garbage do not renew valid timestamp,199ms valid/200ms stale with outputs unchanged, fresh recovery passed");
    return 0;
}

static int check_irq_mask_and_tick_wrap(void)
{
    uint8_t frame[24]; unsigned size;
    imu_init(); host_tick = 100u; host_primask = 0u;
    size = pack(frame, 6u, 1u, 1000u, 100, 200); feed(frame, size);
    host_tick = 120u; pending_size = pack(pending_frame, 18u, 1u, 2000u, 300, 400);
    float p = 0.0f, r = 0.0f; uint32_t stamp = 0u;
    unsigned disables = irq_disable_calls, restores = irq_restore_calls;
    CHECK(imu_tilt_snapshot(&p, &r, &stamp));
    CHECK(near(p, 1.0f) && near(r, 2.0f) && stamp == 100u &&
          irq_disable_calls == disables + 1u && irq_restore_calls == restores + 1u && !host_primask);
    /* Pending UART ISR runs only after snapshot copied all old fields. */
    CHECK(!pending_size && snapshot_is(3.0f, 4.0f, 120u) == 0);
    host_primask = 1u; pending_size = pack(pending_frame, 6u, 1u, 2500u, -500, -600);
    CHECK(snapshot_is(3.0f, 4.0f, 120u) == 0 && host_primask == 1u && pending_size);
    __set_PRIMASK(0u); CHECK(snapshot_is(-5.0f, -6.0f, 120u) == 0);

    imu_init(); host_tick = UINT32_MAX - 90u;
    size = pack(frame, 6u, 1u, 3000u, 150, -250); feed(frame, size);
    uint32_t baseline = host_tick;
    host_tick = baseline + 199u;
    CHECK(snapshot_is(1.5f, -2.5f, baseline) == 0 && imu_last_valid_age_ms() == 199u);
    host_tick++;
    CHECK(!imu_tilt_snapshot(NULL, NULL, NULL) && imu_last_valid_age_ms() == 200u);
    size = pack(frame, 18u, 1u, 3500u, 175, -275); feed(frame, size);
    CHECK(snapshot_is(1.75f, -2.75f, host_tick) == 0 && imu_last_valid_age_ms() == 0u);
    puts("imu snapshot:pending UART blocked through entire copy, previous PRIMASK restored, unsigned timestamp age across HAL wrap passed");
    return 0;
}

int main(void)
{
    if (check_parser_frame_atomicity_and_zero() || check_bad_frames_age_and_recovery() ||
        check_irq_mask_and_tick_wrap()) return 1;
    puts("real App/imu.c tilt snapshot host-only regression passed");
    return 0;
}
