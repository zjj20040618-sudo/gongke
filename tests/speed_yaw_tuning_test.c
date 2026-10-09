/* Real Bluetooth test.c profile routing/state-machine regression. Reuse the
 * peripheral-only fixture; the injected heading helper is checked separately
 * against real steps.c/motion.c by speed_yaw_heading_test.c. No hardware runs. */
#define main existing_g_router_regression_main
#include "g_command_stop_test.c"
#undef main
#include <math.h>

static void select_speed(int mode, int speed)
{
    char command[20];
    snprintf(command, sizeof command, "%d", mode); run_cmd(command);
    snprintf(command, sizeof command, "v%d", speed); run_cmd(command);
}

static void set_gain(float gain)
{
    char command[24];
    snprintf(command, sizeof command, "ykp%.3f", gain); run_cmd(command);
}

static int gain_is(float expected, int want_profile)
{
    int profile = -1;
    float actual = dist_heading_kp_get(&profile);
    CHECK(fabsf(actual - expected) < 0.00001f && profile == want_profile);
    return 0;
}

static int check_direction_speed_profiles(void)
{
    static const int speeds[] = { 100, 200, 300 };
    reset_fixture();
    for (int mode = 15; mode <= 18; ++mode) {
        for (int k = 0; k < 3; ++k) {
            float gain = 0.5f + 0.1f * (float)((mode - 15) * 3 + k);
            select_speed(mode, speeds[k]); set_gain(gain);
            CHECK(strstr(last_message, "ERR") == NULL);
            CHECK(gain_is(gain, 1) == 0 && host_heading_kp == 0.3f);
        }
    }
    /* Selection order must not leak gain across directions or velocities. */
    for (int mode = 18; mode >= 15; --mode) {
        for (int k = 2; k >= 0; --k) {
            float gain = 0.5f + 0.1f * (float)((mode - 15) * 3 + k);
            char expected[24];
            select_speed(mode, speeds[k]);
            CHECK(gain_is(gain, 1) == 0);
            host_messages[0] = '\0';
            run_cmd("param");
            snprintf(expected, sizeof expected, "ykp=%.3f", gain);
            CHECK(strstr(host_messages, expected) != NULL);
            CHECK(strstr(host_messages, "PARAM kp=") != NULL && strstr(last_message, "source=PROFILE") != NULL);
            select_speed(mode, 150); CHECK(gain_is(0.3f, 0) == 0);
        }
    }
    /* No interpolation/nearest-speed borrowing; unmeasured v150 stays global. */
    select_speed(15, 150); CHECK(gain_is(0.3f, 0) == 0);
    run_cmd("0"); run_cmd("20"); set_gain(1.3f);
    CHECK(fabsf(host_heading_kp - 1.3f) < 0.00001f);
    select_speed(15, 150); CHECK(gain_is(1.3f, 0) == 0);
    select_speed(15, 100); CHECK(gain_is(0.5f, 1) == 0);
    run_cmd("0"); select_speed(15, 100); CHECK(gain_is(0.5f, 1) == 0);
    test_init(); select_speed(15, 100); CHECK(gain_is(1.3f, 0) == 0);
    puts("speed ykp: 12 independent mode/v profiles, exact-key fallback, no interpolation, reset/retention passed");
    return 0;
}

static int check_validation_capacity_and_locks(void)
{
    static const char *const invalid[] = { "ykp-0.1", "ykp5.1", "ykp", "ykpnan", "ykp1x" };
    reset_fixture(); run_cmd("15"); set_gain(2.0f);
    CHECK(strstr(last_message, "ERR") != NULL && gain_is(0.3f, 0) == 0);
    select_speed(15, 100); set_gain(0.0f); CHECK(gain_is(0.0f, 1) == 0);
    for (unsigned k = 0; k < sizeof invalid / sizeof invalid[0]; ++k) {
        run_cmd(invalid[k]);
        CHECK(strstr(last_message, "ERR") != NULL && gain_is(0.0f, 1) == 0);
    }
    run_cmd("v0"); CHECK(s_v == 100.0f);
    run_cmd("v601"); CHECK(s_v == 100.0f);
    reset_fixture();
    for (int speed = 1; speed <= 32; ++speed) {
        select_speed(15, speed); set_gain(1.0f);
        CHECK(strstr(last_message, "ERR") == NULL && gain_is(1.0f, 1) == 0);
    }
    select_speed(15, 33); set_gain(2.0f);
    CHECK(strstr(last_message, "ERR") != NULL && gain_is(0.3f, 0) == 0);
    for (int speed = 1; speed <= 32; ++speed) {
        select_speed(15, speed); CHECK(gain_is(1.0f, 1) == 0);
    }
    select_speed(15, 1); set_gain(2.0f); CHECK(gain_is(2.0f, 1) == 0);

    for (int phase = 0; phase < 3; ++phase) {
        reset_fixture(); select_speed(17, 100); set_gain(2.0f);
        run_cmd("d1000"); run_cmd("g"); tick();
        CHECK(s_round == R_RUN && s_dist_heading_kp == 2.0f);
        if (phase == 1) run_cmd("g");
        if (phase == 2) s_round = R_RET; /* real dispatcher write lock, not an invented return path */
        CHECK(s_round == (phase == 0 ? R_RUN : phase == 1 ? R_BRAKE : R_RET));
        set_gain(4.0f);
        CHECK(strstr(last_message, "ERR STOP_WITH_G") != NULL);
        CHECK(gain_is(2.0f, 1) == 0 && s_dist_heading_kp == 2.0f && host_heading_kp == 0.3f);
    }
    reset_fixture(); select_speed(17, 100); set_gain(2.0f);
    host_state = MS_READ_QR; set_gain(4.0f);
    CHECK(strstr(last_message, "ERR") != NULL && gain_is(2.0f, 1) == 0);
    puts("speed ykp: missing v, invalid/range values, zero, 32-slot no-eviction/update, RUN/BRAKE/RET/mission locks passed");
    return 0;
}

static int check_run_snapshot_and_stop_reports(void)
{
    static const int speeds[] = { 100, 200, 300 };
    for (int mode = 15; mode <= 18; ++mode) {
        for (int k = 0; k < 3; ++k) {
            unsigned before;
            float expected_w;
            reset_fixture(); select_speed(mode, speeds[k]); set_gain(2.0f);
            run_cmd("d1000"); host_messages[0] = '\0'; run_cmd("g");
            CHECK(s_round == R_RUN && s_dist_heading_kp == 2.0f && s_dist_heading_profile == 1);
            CHECK(strstr(last_message, "ykp=2.000") != NULL);
            CHECK(strstr(host_messages, "PARAM kp=") != NULL && strstr(last_message, "source=PROFILE") != NULL);
            before = host_precise_calls;
            host_yaw = 3.0f; tick(); expected_w = -2.0f * 3.0f * 0.0174533f;
            CHECK(host_precise_calls > before && host_integer_calls == 0u);
            CHECK(fabsf(last_w - expected_w) < 0.000001f);
            /* Global gain changes must not alter this already-started round. */
            host_heading_kp = 4.0f; tick();
            CHECK(fabsf(last_w - expected_w) < 0.000001f && s_dist_heading_kp == 2.0f);
            host_yaw = -3.0f; tick(); CHECK(last_w > 0.0f);
            host_yaw = 3.0f; host_tick += T_DIST_TRACE_MS; tick();
            CHECK(strstr(last_message, "TRC type=DIST") != NULL && strstr(last_message, "ykp=2.000") != NULL);
            if (mode <= 16) host_fore = mode == 15 ? 50.0f : -50.0f;
            else host_lateral = mode == 17 ? -50.0f : 50.0f;
            run_cmd("g");
            CHECK(s_round == R_BRAKE && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
            before = host_precise_calls;
            run_cmd("g"); CHECK(s_round == R_BRAKE && host_precise_calls == before);
            host_tick += T_DIST_STILL_MS; tick();
            CHECK(s_round == R_READY && strstr(last_message, "REC type=DIST") != NULL);
            CHECK(strstr(last_message, "status=STOP") != NULL && strstr(last_message, "ykp=2.000") != NULL);
            CHECK(host_precise_calls == before && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        }
    }
    reset_fixture(); select_speed(15, 150); run_cmd("d1000"); run_cmd("g");
    CHECK(s_dist_heading_kp == 0.3f && s_dist_heading_profile == 0);
    CHECK(strstr(last_message, "source=GLOBAL") != NULL && strstr(last_message, "ykp=0.300") != NULL);
    puts("speed ykp: 12 per-run snapshots, negative feedback, precise dispatch, automatic START/TRC/REC actual gain, g stop passed");
    return 0;
}

static int check_turn_and_route_isolation(void)
{
    static const int turn_modes[] = { 20, 22, 30 };
    for (int k = 0; k < 3; ++k) {
        reset_fixture(); select_speed(17, 100); set_gain(5.0f);
        select_speed(turn_modes[k], 100); run_cmd("g"); tick();
        CHECK(s_round == R_RUN && host_precise_calls == 0u);
        CHECK(last_w == (turn_modes[k] == 30 ? -T_TURN_MAX_W : T_TURN_MAX_W));
        CHECK(last_x == 0.0f && last_y == 0.0f && host_heading_kp == 0.3f);
    }
    reset_fixture(); select_speed(17, 100); set_gain(5.0f);
    host_messages[0] = '\0';
    run_cmd("31"); run_cmd("g"); CHECK(sequence_start_stage() == 0);
    CHECK(s_msel == 17 && s_v == 250.0f && s_dist_heading_profile == 0);
    CHECK(s_dist_heading_kp == 0.3f && host_precise_calls > 0u && host_integer_calls == 0u);
    CHECK(strstr(host_messages, "ykp=0.300 source=ROUTE31") != NULL);
    host_yaw = 3.0f; tick();
    CHECK(fabsf(last_w + 0.3f * 3.0f * 0.0174533f) < 0.000001f);
    set_gain(4.0f); CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
    CHECK(host_heading_kp == 0.3f && s_dist_heading_kp == 0.3f);
    run_cmd("g"); select_speed(17, 100); CHECK(gain_is(5.0f, 1) == 0);
    puts("speed ykp: 20/22/30 turn profiles unchanged; mode31 local gain0.3 stays isolated from global/manual table passed");
    return 0;
}

int main(void)
{
    CHECK(check_direction_speed_profiles() == 0);
    CHECK(check_validation_capacity_and_locks() == 0);
    CHECK(check_run_snapshot_and_stop_reports() == 0);
    CHECK(check_turn_and_route_isolation() == 0);
    return 0;
}
