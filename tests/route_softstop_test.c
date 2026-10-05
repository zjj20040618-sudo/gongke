/* Actual mode31 Bluetooth state machine + actual motion profile arithmetic.
 * Sensor/actuator plumbing is mocked: these are software contracts, not a
 * vehicle-plant simulation or physical stopping/route acceptance. */
#include <math.h>
#define HOST_REAL_MOTION_PROFILE
#define main old_g_command_stop_main
#include "g_command_stop_test.c"
#undef main

/* Keep the existing state-machine fixture's capture/odometry peripherals.
 * Rename only the other real motion exports, leaving all profile/ramp entry
 * points unchanged. Real IK/dispatch is then available to inspect captured
 * mode31 commands for the sub-1RPM endpoint case. */
#define motion_init real_motion_init
#define motion_ik real_motion_ik
#define motion_vel_set real_motion_vel_set
#define motion_ik_precise real_motion_ik_precise
#define motion_vel_set_precise real_motion_vel_set_precise
#define motion_brake real_motion_brake
#define motion_pose real_motion_pose
#define motion_odo_mm real_motion_odo_mm
#define motion_lateral_odo_mm real_motion_lateral_odo_mm
#define motion_pose_update real_motion_pose_update
#include "../App/motion.c"
#undef motion_init
#undef motion_ik
#undef motion_vel_set
#undef motion_ik_precise
#undef motion_vel_set_precise
#undef motion_brake
#undef motion_pose
#undef motion_odo_mm
#undef motion_lateral_odo_mm
#undef motion_pose_update

static float delivered_rpm[4];
static unsigned delivered_mask;
void ctrl_stop_all(void) { memset(delivered_rpm, 0, sizeof delivered_rpm); }
void ctrl_set_speed_precise(int motor, float rpm)
{
    delivered_rpm[motor] = rpm;
    delivered_mask |= 1u << (unsigned)motor;
}

#undef CHECK
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "route soft-stop line %d: %s\n", __LINE__, #expr); \
    return 1; \
} } while (0)

static int close_to(float a, float b)
{ return fabsf(a - b) < 0.0001f; }

static float captured_axis(void)
{ return dist_lateral() ? last_y : last_x; }

static void poll_20ms(void)
{ host_tick += 20u; test_poll(); }

static void put_remaining(float remaining)
{
    const float sign = s_dist_target < 0.0f ? -1.0f : 1.0f;
    const float position = s_dist_odo0 + s_dist_target - sign * remaining;
    if (dist_lateral()) host_lateral = position;
    else host_fore = position;
}

static int global_matches(const MotionProfileTune *expected)
{
    MotionProfileTune actual;
    motion_profile_get(&actual);
    return actual.acc_mms2 == expected->acc_mms2
        && actual.dec_mms2 == expected->dec_mms2;
}

static int start_stage(unsigned stage, const MotionProfileTune *global)
{
    reset_fixture();
    CHECK(motion_profile_set(global));
    run_cmd("31"); run_cmd("g");
    if (stage) { s_seq_stage = (uint8_t)stage; route_seq_prepare(); }
    CHECK(sequence_start_stage() == 0);
    CHECK(s_seq_state == SQ_RUN && s_dist_precise == 1u);
    CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f);
    CHECK(s_dist_heading_profile == 0u &&
          s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : 0.3f) && host_heading_kp == 0.3f);
    CHECK(s_route_heading_kp == 0.3f);
    CHECK(global_matches(global));
    CHECK(strstr(host_messages, "ROUTE_PROFILE mode=31 source=LOCAL acc=700 dec=350") != NULL);
    return 0;
}

static int check_local_profile_and_distance(void)
{
    static const MotionProfileTune globals[] = { { 0.0f, 0.0f }, { 42.0f, 84.0f } };
    static const unsigned stages[] = { 0u, 1u, 3u, 4u, 5u, 10u };
    static const float targets[] = { -530.0f, -650.0f, -650.0f, 80.0f, -190.0f, 780.0f };
    static const float speeds[] = { 100.0f, 100.0f, 300.0f, 20.0f, 100.0f, 100.0f };
    for (unsigned p = 0; p < sizeof globals / sizeof globals[0]; ++p) {
        for (unsigned s = 0; s < sizeof stages / sizeof stages[0]; ++s) {
            float previous, axis;
            CHECK(start_stage(stages[s], &globals[p]) == 0);
            CHECK(s_dist_target == targets[s] && s_v == speeds[s]);
            axis = captured_axis();
            CHECK(close_to(fabsf(axis), 14.0f) && axis * s_dist_target > 0.0f);
            CHECK(host_precise_calls > 0u && host_integer_calls == 0u);
            for (unsigned i = 0; i < 48u; ++i) {
                previous = fabsf(captured_axis());
                poll_20ms();
                CHECK(fabsf(captured_axis()) >= previous);
                CHECK(fabsf(captured_axis()) - previous <= 14.0001f);
            }
            CHECK(close_to(fabsf(captured_axis()), s_v));

            /* Holding a sensor snapshot lets us inspect the real deceleration
             * recurrence independently of an invented physical plant. */
            put_remaining(5.0f);
            for (unsigned i = 0; i < 96u; ++i) {
                previous = fabsf(captured_axis());
                poll_20ms();
                CHECK(s_round == R_RUN && s_seq_stage == stages[s]);
                CHECK(previous - fabsf(captured_axis()) <= 7.0001f);
                CHECK(fabsf(captured_axis()) <= previous + 0.0001f);
            }
            CHECK(close_to(fabsf(captured_axis()), fminf(s_v, sqrtf(2.0f * 350.0f * 5.0f))));
            CHECK(global_matches(&globals[p]));

            /* Only reverse crossing/forward board contact ignore yaw while slowing down.
             * Every other distance leg retains its route-owned gain. */
            const float expected_kp = stages[s] == 3u || stages[s] == 4u ? 0.0f : 0.3f;
            host_yaw = 3.0f; poll_20ms();
            CHECK(close_to(last_w, -expected_kp * 3.0f * 0.0174533f));
            host_yaw = -3.0f; poll_20ms();
            CHECK(close_to(last_w, expected_kp * 3.0f * 0.0174533f));
            host_yaw = 0.0f;

            /* The integer legacy IK would deliver four zeros before distance
             * completion. The selected precise dispatch keeps all fractional
             * targets nonzero, without pretending that real motors track them. */
            put_remaining(0.005f);
            for (unsigned i = 0; i < 16u; ++i) poll_20ms();
            CHECK(s_round == R_RUN && captured_axis() * s_dist_target > 0.0f);
            CHECK(fabsf(captured_axis()) > 0.0f && fabsf(captured_axis()) < 3.0f);
            {
                int16_t old_rpm[4];
                real_motion_ik(last_x, last_y, last_w, old_rpm);
                delivered_mask = 0u;
                real_motion_vel_set_precise(last_x, last_y, last_w);
                CHECK(delivered_mask == 15u);
                for (int motor = 0; motor < 4; ++motor) {
                    CHECK(old_rpm[motor] == 0);
                    CHECK(fabsf(delivered_rpm[motor]) > 0.0f && fabsf(delivered_rpm[motor]) < 1.0f);
                }
            }

            previous = (float)brake_calls;
            put_remaining(0.0f); poll_20ms();
            CHECK(s_round == R_BRAKE && s_dist_reason == 1u);
            CHECK(s_seq_stage == stages[s] && brake_calls > (int)previous);
            CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
            host_tick += T_DIST_STILL_MS; test_poll();
            fixture_complete_distance_alignment();
            if (s_seq_state == SQ_QR_WAIT) CHECK(sequence_release_qr() == 0);
            CHECK(s_seq_stage == stages[s] + 1u && s_seq_state == SQ_STILL);
            CHECK(global_matches(&globals[p]));
        }
    }
    return 0;
}

static int check_immediate_cancellation(void)
{
    static const char *const keys[] = { "g", "a", "0" };
    const MotionProfileTune global = { 0.0f, 0.0f };
    for (unsigned key = 0; key < sizeof keys / sizeof keys[0]; ++key) {
        CHECK(start_stage(0u, &global) == 0);
        for (unsigned i = 0; i < 8u; ++i) poll_20ms();
        put_remaining(5.0f); poll_20ms();
        CHECK(s_round == R_RUN && fabsf(captured_axis()) < 100.0f);
        run_cmd(keys[key]);
        CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 0u && s_msel == 31);
        CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        for (unsigned i = 0; i < 20u; ++i) poll_20ms();
        CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 0u);
        CHECK(global_matches(&global) && pulse_calls == 0 && servo_calls == 0 && !laser_state);
    }
    CHECK(start_stage(0u, &global) == 0);
    put_remaining(5.0f); host_imu_valid = 0; poll_20ms();
    CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 0u);
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    return 0;
}

static int check_lateral_reverse_and_ff_wheel_dispatch(void)
{
    const MotionProfileTune global = { 0.0f, 0.0f };
    float baseline[4];
    CHECK(start_stage(0u, &global) == 0);
    CHECK(s_msel == 17 && s_dist_target == -530.0f && s_v == 100.0f);
    CHECK(last_x == 0.0f && last_y < 0.0f && last_w == 0.0f && s_dist_ff_ratio == 0.0f);
    delivered_mask = 0u;
    real_motion_vel_set_precise(last_x, last_y, last_w);
    CHECK(delivered_mask == 15u);
    CHECK(delivered_rpm[0] > 0.0f && delivered_rpm[1] < 0.0f &&
          delivered_rpm[2] > 0.0f && delivered_rpm[3] < 0.0f);
    CHECK(close_to(delivered_rpm[0], delivered_rpm[2]) && close_to(delivered_rpm[1], delivered_rpm[3]));
    CHECK(close_to(delivered_rpm[0], -delivered_rpm[1]));
    run_cmd("g");
    /* Right35 was removed from the route; preserve its signed IK coverage
     * as an unchanged manual mode18 command rather than inventing a node. */
    run_cmd("18"); run_cmd("v100"); run_cmd("d35"); run_cmd("g");
    poll_20ms();
    CHECK(s_msel == 18 && s_dist_target == 35.0f && s_v == 100.0f);
    CHECK(last_x == 0.0f && last_y > 0.0f && last_w == 0.0f && s_dist_ff_ratio == 0.0f);
    delivered_mask = 0u;
    real_motion_vel_set_precise(last_x, last_y, last_w);
    CHECK(delivered_mask == 15u);
    CHECK(delivered_rpm[0] < 0.0f && delivered_rpm[1] > 0.0f &&
          delivered_rpm[2] < 0.0f && delivered_rpm[3] > 0.0f);
    CHECK(close_to(delivered_rpm[0], delivered_rpm[2]) && close_to(delivered_rpm[1], delivered_rpm[3]));
    CHECK(close_to(delivered_rpm[0], -delivered_rpm[1]));
    run_cmd("g");
    CHECK(start_stage(5u, &global) == 0);
    CHECK(s_msel == 16 && s_dist_target == -190.0f && s_v == 100.0f);
    CHECK(last_x < 0.0f && last_y < 0.0f && last_w == 0.0f && s_dist_ff_ratio == 0.00625f);
    CHECK(close_to(last_y, -0.00625f * fabsf(last_x)));
    delivered_mask = 0u;
    real_motion_vel_set_precise(last_x, last_y, last_w);
    CHECK(delivered_mask == 15u);
    for (int motor = 0; motor < 4; ++motor) CHECK(delivered_rpm[motor] < 0.0f);
    run_cmd("g");
    CHECK(start_stage(10u, &global) == 0);
    CHECK(s_dist_ff_ratio == -0.00625f && last_x > 0.0f && last_y > 0.0f && last_w == 0.0f);
    CHECK(close_to(last_y, last_x * 0.00625f));
    real_motion_ik_precise(last_x, 0.0f, 0.0f, baseline);
    delivered_mask = 0u;
    real_motion_vel_set_precise(last_x, last_y, last_w);
    CHECK(delivered_mask == 15u);
    for (int motor = 0; motor < 4; ++motor) CHECK(delivered_rpm[motor] > 0.0f);
    CHECK(delivered_rpm[0] < baseline[0] && delivered_rpm[2] < baseline[2]);
    CHECK(delivered_rpm[1] > baseline[1] && delivered_rpm[3] > baseline[3]);
    CHECK(close_to(delivered_rpm[0], delivered_rpm[2]) && close_to(delivered_rpm[1], delivered_rpm[3]));
    run_cmd("g");
    puts("real precise IK: route left530/manual right35 signed lateral targets; reverse190 negative axis with separateleftBFF; defaultfrontFFFnegative ->positive lateral and four changed forwardRPM targets passed");
    return 0;
}

static int check_manual_profile_preservation(void)
{
    static const MotionProfileTune globals[] = { { 0.0f, 0.0f }, { 42.0f, 84.0f } };
    for (unsigned p = 0; p < sizeof globals / sizeof globals[0]; ++p) {
        CHECK(start_stage(0u, &globals[p]) == 0);
        run_cmd("g");
        CHECK(global_matches(&globals[p]));
        run_cmd("15"); run_cmd("v100"); run_cmd("d500"); run_cmd("g");
        CHECK(s_seq_state == SQ_OFF && s_round == R_RUN);
        poll_20ms();
        CHECK(close_to(last_x, p == 0u ? 100.0f : 42.0f * 0.020f));
        CHECK(s_dist_ramp.acc == globals[p].acc_mms2);
        CHECK(global_matches(&globals[p]));
    }
    return 0;
}

int main(void)
{
    CHECK(check_local_profile_and_distance() == 0);
    CHECK(check_immediate_cancellation() == 0);
    CHECK(check_lateral_reverse_and_ff_wheel_dispatch() == 0);
    CHECK(check_manual_profile_preservation() == 0);
    puts("route31 real soft-stop: local700/350/global isolation, exactsix legs, crossing/contact yaw/FF disabled including deceleration, back190 restoresykp0.3 + BFF; post-yaw-before-next, fractional endpoint, g/a/0/IMU cancellation and manualprofile preservation passed");
    return 0;
}
