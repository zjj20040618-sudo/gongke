/* Actual Bluetooth router/state machine with the existing inert peripheral
 * fixture. These checks pin parameter ownership and command semantics only;
 * they do not simulate wheel traction, camera feedback or physical routes. */
#include <math.h>
#define main old_g_command_stop_main
#include "g_command_stop_test.c"
#undef main

#undef CHECK
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "route31 tuning scope line %d: %s\n", __LINE__, #expr); \
    return 1; \
} } while (0)

static int ff_equal(float a, float b)
{ return fabsf(a - b) < 0.000001f; }

static int check_report(const char *value, int local)
{
    char expected[40];
    host_messages[0] = '\0';
    run_cmd("param");
    snprintf(expected, sizeof expected, "fff=%s", value);
    CHECK(strstr(host_messages, expected) != NULL);
    CHECK((strstr(host_messages, "ROUTE_PROFILE mode=31 source=LOCAL") != NULL) == local);
    if (local) {
        const char *route_profile = strstr(host_messages, "ROUTE_PROFILE mode=31 source=LOCAL");
        const char *local_fff = strstr(route_profile, expected);
        const char *newline = strchr(route_profile, '\n');
        CHECK(local_fff != NULL && newline != NULL && local_fff < newline);
        snprintf(expected, sizeof expected, "ykp=%.3f", s_route_heading_kp);
        CHECK(strstr(route_profile, expected) != NULL && strstr(route_profile, expected) < newline);
        CHECK(strstr(host_messages, "source=ROUTE31 global_ykp=0.300") != NULL);
        CHECK(strstr(host_messages, expected) != NULL);
    } else {
        CHECK(strstr(host_messages, "source=ROUTE31") == NULL);
    }
    return 0;
}

static int check_defaults_and_direct_values(void)
{
    reset_fixture();
    CHECK(ROUTE_TEST_FORWARD_FF_SEED == 0.00625f);
    CHECK(s_forward_ff_ratio == 0.0125f && test_forward_ff_ratio() == 0.0125f);
    CHECK(s_route_forward_ff_ratio == 0.00625f);
    CHECK(ROUTE_TEST_HEADING_KP_SEED == 0.3f && s_route_heading_kp == 0.3f);
    CHECK(step_heading_kp_deg() == 0.3f);
    CHECK(check_report("0.01250", 0) == 0);
    run_cmd("31");
    CHECK(s_seq_state == SQ_READY && s_route_forward_ff_ratio == 0.00625f);
    CHECK(check_report("0.00625", 1) == 0);

    static const char *const inputs[] = { "fff0.01000", "fff0", "fff-0.02000", "fff0.04999", "fff-0.04999" };
    static const float expected[] = { 0.01f, 0.0f, -0.02f, 0.04999f, -0.04999f };
    for (unsigned i = 0; i < sizeof inputs / sizeof inputs[0]; ++i) {
        run_cmd(inputs[i]);
        if (!ff_equal(s_route_forward_ff_ratio, expected[i]))
            fprintf(stderr, "input=%s actual=%.9f expected=%.9f last=%s\n",
                    inputs[i], s_route_forward_ff_ratio, expected[i], last_message);
        CHECK(ff_equal(s_route_forward_ff_ratio, expected[i]));
        CHECK(s_forward_ff_ratio == 0.0125f && test_forward_ff_ratio() == 0.0125f);
    }
    run_cmd("fff0.00625"); CHECK(check_report("0.00625", 1) == 0);
    run_cmd("15"); run_cmd("v100"); run_cmd("d500"); run_cmd("g"); tick();
    CHECK(s_dist_ff_ratio == 0.0125f && last_x == 100.0f && last_y == -1.25f);
    run_cmd("g"); host_tick += T_DIST_STILL_MS; test_poll();
    run_cmd("32");
    CHECK(s_msel == 32 && s_seq_state == SQ_OFF && test_forward_ff_ratio() == 0.0125f);
    CHECK(check_report("0.01250", 0) == 0);
    return 0;
}

static int check_all_route_legs(void)
{
    static const int modes[12] = {17,16,30,15,16,15,17,15,20,15,20,15};
    static const int commands[12] = {-530,-650,-92,750,-170,210,-730,780,90,2450,90,2125};
    static const float speeds[12] = {100,100,100,300,20,100,100,100,100,100,100,100};
    const float direct_ff = 0.01f;
    unsigned forward_legs = 0u, translation_legs = 0u, ff_legs = 0u;
    reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("g");
    CHECK(ROUTE_TEST_STAGES == 12u);
    for (unsigned stage = 0; stage < 12u; ++stage) {
        CHECK(s_seq_stage == stage && sequence_start_stage() == 0);
        CHECK(s_msel == modes[stage]);
        CHECK(ff_equal(s_route_forward_ff_ratio, direct_ff) && test_forward_ff_ratio() == 0.0125f);
        if (dist_mode()) {
            ++translation_legs;
            CHECK(s_dist_target == (float)commands[stage] && s_v == speeds[stage]);
            CHECK(s_dist_heading_kp == 0.3f && s_route_heading_kp == 0.3f);
            CHECK(step_heading_kp_deg() == 0.3f);
            CHECK(ff_equal(s_dist_ff_ratio, s_msel == 15 && stage != 3u ? direct_ff : 0.0f));
            if (s_dist_ff_ratio != 0.0f) ++ff_legs;
            if (s_msel == 15) {
                ++forward_legs;
                CHECK(last_x == (stage == 3u ? 300.0f : 100.0f) &&
                      ff_equal(last_y, stage == 3u ? 0.0f : -1.0f));
            } else if (s_msel == 16) {
                CHECK(last_x == -speeds[stage] && last_y == 0.0f);
            } else {
                CHECK(last_x == 0.0f && last_y == -100.0f);
            }
            host_yaw = 3.0f; tick();
            CHECK(ff_equal(last_w, -0.3f * 3.0f * 0.0174533f));
            host_yaw = -3.0f; tick();
            CHECK(ff_equal(last_w, 0.3f * 3.0f * 0.0174533f));
            host_yaw = 0.0f;
        } else {
            CHECK(turn_target_deg() == (float)commands[stage]);
        }
        CHECK(sequence_finish_stage() == 0);
    }
    CHECK(forward_legs == 5u && ff_legs == 4u && translation_legs == 9u && s_seq_state == SQ_DONE);
    CHECK(s_route_heading_kp == 0.3f && step_heading_kp_deg() == 0.3f);
    CHECK(ff_equal(s_route_forward_ff_ratio, direct_ff) && test_forward_ff_ratio() == 0.0125f);
    CHECK(check_report("0.01000", 1) == 0);
    run_cmd("31");
    CHECK(s_seq_state == SQ_READY && ff_equal(s_route_forward_ff_ratio, direct_ff));
    CHECK(check_report("0.01000", 1) == 0);
    run_cmd("22"); CHECK(turn_target_deg() == 180.0f);
    run_cmd("30"); CHECK(turn_target_deg() == -92.0f);
    run_cmd("20"); CHECK(turn_target_deg() == 90.0f);
    CHECK(pulse_calls == 0 && servo_calls == 0 && !laser_state && !s_go);
    return 0;
}

static int check_ram_retention_and_separation(void)
{
    static const char *const stop_keys[] = { "g", "a", "0" };
    for (unsigned key = 0; key < sizeof stop_keys / sizeof stop_keys[0]; ++key) {
        reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("ykp2"); run_cmd("g");
        CHECK(sequence_start_stage() == 0);
        CHECK(s_dist_heading_kp == 2.0f);
        run_cmd(stop_keys[key]);
        CHECK(s_seq_state == SQ_STOPPED && ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
        CHECK(test_forward_ff_ratio() == 0.0125f);
        CHECK(check_report("0.01000", 1) == 0);
        run_cmd("31"); CHECK(s_seq_state == SQ_READY && ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(s_route_heading_kp == 2.0f);
        run_cmd("15"); run_cmd("fff0.02000");
        CHECK(ff_equal(test_forward_ff_ratio(), 0.02f) && ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(check_report("0.02000", 0) == 0);
        run_cmd("32");
        CHECK(ff_equal(test_forward_ff_ratio(), 0.02f) && ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
        CHECK(check_report("0.02000", 0) == 0);
        run_cmd("31"); CHECK(check_report("0.01000", 1) == 0);
        run_cmd("0"); test_init();
        CHECK(s_seq_state == SQ_OFF && s_route_forward_ff_ratio == 0.00625f);
        CHECK(s_forward_ff_ratio == 0.0125f && test_forward_ff_ratio() == 0.0125f);
        CHECK(s_route_heading_kp == 0.3f && step_heading_kp_deg() == 0.3f);
    }
    return 0;
}

static int check_invalid_inputs_and_phase_locks(void)
{
    static const char *const invalid[] = {
        "fff0.05001", "fff-0.05001", "fff", "fffnan", "fffinf", "fff1e-3", "fff0.01x", "fff--0.01"
    };
    reset_fixture(); run_cmd("31"); run_cmd("fff0.01000");
    for (unsigned i = 0; i < sizeof invalid / sizeof invalid[0]; ++i) {
        run_cmd(invalid[i]);
        CHECK(strstr(last_message, "ERR ") != NULL);
        CHECK(ff_equal(s_route_forward_ff_ratio, 0.01f) && test_forward_ff_ratio() == 0.0125f);
    }
    static const char *const invalid_ykp[] = {
        "ykp5.001", "ykp-0.001", "ykp", "ykpnan", "ykpinf", "ykp1e-3", "ykp2x", "ykp--2"
    };
    run_cmd("ykp2");
    for (unsigned i = 0; i < sizeof invalid_ykp / sizeof invalid_ykp[0]; ++i) {
        run_cmd(invalid_ykp[i]);
        CHECK(strstr(last_message, "ERR ") != NULL);
        CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
    }
    /* All twelve stages: STILL / WAIT / RUN / BRAKE reject parameter writes. */
    for (unsigned stage = 0; stage < 12u; ++stage) {
        for (unsigned phase = 0; phase < 4u; ++phase) {
            reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("ykp2"); run_cmd("g");
            s_seq_stage = (uint8_t)stage; route_seq_prepare();
            if (phase == 1u) {
                host_tick += T_DIST_STILL_MS; test_poll(); CHECK(s_seq_state == SQ_WAIT);
            }
            if (phase >= 2u) CHECK(sequence_start_stage() == 0);
            if (phase == 3u) {
                if (dist_mode()) {
                    if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
                    else host_fore = s_dist_odo0 + s_dist_target;
                } else host_yaw = turn_target_deg();
                test_poll(); CHECK(s_round == R_BRAKE);
            }
            run_cmd("fff0.00400");
            CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
            CHECK(ff_equal(s_route_forward_ff_ratio, 0.01f) && test_forward_ff_ratio() == 0.0125f);
            run_cmd("ykp4");
            CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
            CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
            if (dist_mode() && phase >= 2u) CHECK(s_dist_heading_kp == 2.0f);
            CHECK(s_seq_stage == stage && route_seq_active());
            run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED);
        }
    }
    return 0;
}

static int check_direction_exclusion(void)
{
    /* Mode31 has no right-strafe leg. Inject its distance submode only in this
     * fixture to pin that route-owned FFF cannot become a sideways FF. */
    reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("g");
    CHECK(sequence_start_stage() == 0);
    s_msel = 18; s_seq_prepared = 1u; mode_start(); s_seq_prepared = 0u; tick();
    CHECK(s_dist_ff_ratio == 0.0f && last_x == 0.0f && last_y == 100.0f);
    CHECK(ff_equal(s_route_forward_ff_ratio, 0.01f) && test_forward_ff_ratio() == 0.0125f);
    return 0;
}

static int check_heading_slot_direct_values_and_isolation(void)
{
    int manual_profile;
    reset_fixture(); run_cmd("17"); run_cmd("v100"); run_cmd("ykp5");
    CHECK(dist_heading_kp_get(&manual_profile) == 5.0f && manual_profile == 1);
    CHECK(step_heading_kp_deg() == 0.3f && s_route_heading_kp == 0.3f);
    /* Equal default numbers must not hide accidental ownership coupling.
     * Set a distinct global gain through the real command router, then prove
     * route31 still selects and dispatches its independent 0.3 RAM slot. */
    run_cmd("20"); run_cmd("ykp1");
    CHECK(step_heading_kp_deg() == 1.0f && s_route_heading_kp == 0.3f);
    run_cmd("31");
    CHECK(dist_heading_kp_get(&manual_profile) == 0.3f && manual_profile == 0);
    run_cmd("param");
    CHECK(strstr(last_message, "ykp=0.300 source=ROUTE31 global_ykp=1.000") != NULL);
    run_cmd("g"); CHECK(sequence_start_stage() == 0);
    host_yaw = 3.0f; tick();
    CHECK(s_dist_heading_kp == 0.3f && ff_equal(last_w, -0.3f * 3.0f * 0.0174533f));
    run_cmd("g"); run_cmd("20"); run_cmd("ykp0.3");
    CHECK(step_heading_kp_deg() == 0.3f && s_route_heading_kp == 0.3f);
    run_cmd("31"); run_cmd("ykp2");
    CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
    CHECK(check_report("0.00625", 1) == 0);
    run_cmd("g"); CHECK(sequence_start_stage() == 0);
    CHECK(s_dist_heading_kp == 2.0f);
    host_yaw = 3.0f; tick(); CHECK(ff_equal(last_w, -2.0f * 3.0f * 0.0174533f));
    host_yaw = -3.0f; tick(); CHECK(ff_equal(last_w, 2.0f * 3.0f * 0.0174533f));
    CHECK(host_precise_calls > 0u && host_integer_calls == 0u);
    run_cmd("g"); run_cmd("31"); run_cmd("ykp0");
    CHECK(s_route_heading_kp == 0.0f && step_heading_kp_deg() == 0.3f);
    run_cmd("g"); CHECK(sequence_start_stage() == 0);
    host_yaw = 3.0f; tick(); CHECK(s_dist_heading_kp == 0.0f && last_w == 0.0f);
    run_cmd("g"); run_cmd("31"); run_cmd("ykp5");
    CHECK(s_route_heading_kp == 5.0f && step_heading_kp_deg() == 0.3f);
    run_cmd("17"); run_cmd("v100");
    CHECK(dist_heading_kp_get(&manual_profile) == 5.0f && manual_profile == 1);
    run_cmd("param"); CHECK(strstr(last_message, "source=PROFILE global_ykp=0.300") != NULL);
    run_cmd("32"); run_cmd("param");
    CHECK(strstr(last_message, "ykp=0.300 source=GLOBAL global_ykp=0.300") != NULL);
    CHECK(step_heading_kp_deg() == 0.3f && s_route_heading_kp == 5.0f);
    run_cmd("31"); CHECK(s_route_heading_kp == 5.0f);
    CHECK(check_report("0.00625", 1) == 0);
    run_cmd("0"); test_init();
    CHECK(s_route_heading_kp == 0.3f && step_heading_kp_deg() == 0.3f);
    run_cmd("17"); run_cmd("v100");
    CHECK(dist_heading_kp_get(&manual_profile) == 0.3f && manual_profile == 0);
    return 0;
}

int main(void)
{
    CHECK(check_defaults_and_direct_values() == 0);
    CHECK(check_all_route_legs() == 0);
    CHECK(check_ram_retention_and_separation() == 0);
    CHECK(check_invalid_inputs_and_phase_locks() == 0);
    CHECK(check_direction_exclusion() == 0);
    CHECK(check_heading_slot_direct_values_and_isolation() == 0);
    puts("route31 tuning scope: local fff00625/ykp0.3, nine heading-feedback/four FFF legs, cross300 without unmeasured FFF, back20/-170 then forward100/210, direct overrides, global/manual+32 isolation, RAM retention/reset, exact reports, invalid/48phase locks and 780/2450/right90/left92/180 preservation passed");
    return 0;
}
