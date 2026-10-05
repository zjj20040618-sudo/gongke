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
    CHECK(ROUTE_TEST_FORWARD_FF_SEED == -0.00625f);
    CHECK(s_forward_ff_ratio == -0.00625f && test_forward_ff_ratio() == 0.0125f);
    CHECK(s_route_forward_ff_ratio == -0.00625f);
    CHECK(ROUTE_TEST_HEADING_KP_SEED == 0.3f && s_route_heading_kp == 0.3f);
    CHECK(step_heading_kp_deg() == 0.3f);
    CHECK(check_report("0.01250", 0) == 0);
    run_cmd("31");
    CHECK(s_seq_state == SQ_READY && s_route_forward_ff_ratio == -0.00625f);
    CHECK(check_report("-0.00625", 1) == 0);

    static const char *const inputs[] = { "fff0.01000", "fff0", "fff-0.02000", "fff0.04999", "fff-0.04999" };
    static const float expected[] = { 0.01f, 0.0f, -0.02f, 0.04999f, -0.04999f };
    for (unsigned i = 0; i < sizeof inputs / sizeof inputs[0]; ++i) {
        run_cmd(inputs[i]);
        if (!ff_equal(s_route_forward_ff_ratio, expected[i]))
            fprintf(stderr, "input=%s actual=%.9f expected=%.9f last=%s\n",
                    inputs[i], s_route_forward_ff_ratio, expected[i], last_message);
        CHECK(ff_equal(s_route_forward_ff_ratio, expected[i]));
        CHECK(s_forward_ff_ratio == -0.00625f && test_forward_ff_ratio() == 0.0125f);
    }
    run_cmd("fff0.00625"); CHECK(check_report("0.00625", 1) == 0);
    run_cmd("15"); run_cmd("v100"); run_cmd("d500"); run_cmd("g"); tick();
    CHECK(s_dist_ff_ratio == -0.00625f && last_x == 100.0f && last_y == 0.625f);
    run_cmd("g"); host_tick += T_DIST_STILL_MS; test_poll();
    run_cmd("32");
    CHECK(s_msel == 32 && s_seq_state == SQ_OFF && test_forward_ff_ratio() == 0.0125f);
    CHECK(check_report("0.01250", 0) == 0);
    return 0;
}

static int check_all_route_legs(void)
{
    static const int modes[15] = {17,16,20,16,15,16,30,0,16,20,15,20,15,20,15};
    static const int commands[15] = {-530,-650,90,-650,80,-190,-92,0,-730,90,780,90,2450,90,2125};
    static const float speeds[15] = {100,100,100,300,20,100,100,50,100,100,100,100,100,100,100};
    const float direct_ff = 0.01f;
    unsigned forward_legs = 0u, translation_legs = 0u, ff_legs = 0u, heading_legs = 0u;
    reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("g");
    CHECK(ROUTE_TEST_STAGES == 15u && ROUTE_TEST_ALIGN_STAGE == 7u && ROUTE_TEST_BACK_STAGE == 8u);
    for (unsigned stage = 0; stage < 15u; ++stage) {
        if (stage == 7u) {
            CHECK(s_seq_state == SQ_BUCKET_ALIGN && s_msel == 31 && host_target_calls == 1);
            CHECK(sequence_finish_bucket_stub() == 0);
            CHECK(ff_equal(s_route_forward_ff_ratio, direct_ff) && test_forward_ff_ratio() == 0.0125f);
            continue;
        }
        if (stage == 8u) {
            CHECK(s_seq_state == SQ_MANUAL_D_WAIT && s_bucket36_back_mm == 0u);
            run_cmd("d730");
        }
        CHECK(s_seq_stage == stage && sequence_start_stage() == 0);
        CHECK(s_msel == modes[stage]);
        CHECK(s_route_test_plan[stage].heading_hold == (stage == 3u || stage == 4u ? 0u : 1u));
        CHECK(ff_equal(s_route_forward_ff_ratio, direct_ff) && test_forward_ff_ratio() == 0.0125f);
        if (dist_mode()) {
            ++translation_legs;
            CHECK(s_dist_target == (float)commands[stage] && s_v == speeds[stage]);
            const float expected_kp = stage == 3u || stage == 4u ? 0.0f : 0.3f;
            CHECK(s_dist_heading_kp == expected_kp && s_route_heading_kp == 0.3f);
            if (expected_kp != 0.0f) ++heading_legs;
            CHECK(step_heading_kp_deg() == 0.3f);
            CHECK(ff_equal(s_dist_ff_ratio, stage == 10u || stage == 12u || stage == 14u ? direct_ff :
                  stage == 1u || stage == 5u || stage == 8u ? 0.00625f : 0.0f));
            if (s_dist_ff_ratio != 0.0f) ++ff_legs;
            if (s_msel == 15) {
                ++forward_legs;
                CHECK(last_x == speeds[stage] && ff_equal(last_y, stage == 4u ? 0.0f : -1.0f));
            } else if (s_msel == 16) {
                CHECK(last_x == -speeds[stage] && last_y == -s_dist_ff_ratio * speeds[stage]);
            } else {
                CHECK(last_x == 0.0f && last_y == -100.0f);
            }
            host_yaw = 3.0f; tick();
            CHECK(ff_equal(last_w, -expected_kp * 3.0f * 0.0174533f));
            host_yaw = -3.0f; tick();
            CHECK(ff_equal(last_w, expected_kp * 3.0f * 0.0174533f));
            host_yaw = 0.0f;
        } else {
            CHECK(turn_target_deg() == (float)commands[stage]);
        }
        CHECK(sequence_finish_stage() == 0);
    }
    CHECK(forward_legs == 4u && ff_legs == 6u && translation_legs == 9u && heading_legs == 7u && s_seq_state == SQ_DONE);
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

static int check_route_ff_sign_dispatch(void)
{
    static const char *const selections[] = { "31", "34" };
    static const char *const commands[] = { "", "fff0", "fff-0.02000", "fff0.02000" };
    static const float ratios[] = { -0.00625f, 0.0f, -0.02f, 0.02f };
    static const float lateral[] = { 0.625f, 0.0f, 2.0f, -2.0f };
    for (unsigned mode = 0u; mode < 2u; ++mode) {
        for (unsigned input = 0u; input < 4u; ++input) {
            reset_fixture(); run_cmd(selections[mode]);
            if (commands[input][0]) run_cmd(commands[input]);
            CHECK(ff_equal(s_route_forward_ff_ratio, ratios[input]) && test_forward_ff_ratio() == 0.0125f);
            run_cmd("g"); s_seq_stage = 10u; route_seq_prepare();
            CHECK(sequence_start_stage() == 0);
            CHECK(s_msel == 15 && s_dist_target == 780.0f && s_v == 100.0f);
            CHECK(ff_equal(s_dist_ff_ratio, ratios[input]) && last_x == 100.0f && ff_equal(last_y, lateral[input]));
            CHECK(step_heading_kp_deg() == 0.3f && s_forward_ff_ratio == -0.00625f);
            run_cmd("g"); run_cmd("0"); test_init();
            CHECK(s_route_forward_ff_ratio == -0.00625f && test_forward_ff_ratio() == 0.0125f);
        }
    }
    puts("route31/34 FFF: default reverse, explicit zero/negative/positive delivered lateral sign, reset default; manual/32 value unchanged passed");
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
        CHECK(ff_equal(s_forward_ff_ratio, 0.02f) && test_forward_ff_ratio() == 0.0125f &&
              ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(check_report("0.02000", 0) == 0);
        run_cmd("32");
        CHECK(test_forward_ff_ratio() == 0.0125f && ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
        CHECK(check_report("0.01250", 0) == 0);
        run_cmd("31"); CHECK(check_report("0.01000", 1) == 0);
        run_cmd("0"); test_init();
        CHECK(s_seq_state == SQ_OFF && s_route_forward_ff_ratio == -0.00625f);
        CHECK(s_forward_ff_ratio == -0.00625f && test_forward_ff_ratio() == 0.0125f);
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
    /* Four ordinary phases at 14 motion nodes and four bucket phases. */
    for (unsigned stage = 0; stage < 15u; ++stage) {
        for (unsigned phase = 0; phase < 4u; ++phase) {
            reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("ykp2"); run_cmd("g");
            s_seq_stage = (uint8_t)stage; route_seq_prepare();
            if (stage == 7u) CHECK(sequence_bucket_stub_phase(phase) == 0);
            else {
                if (stage == 8u) {
                    CHECK(s_seq_state == SQ_MANUAL_D_WAIT);
                    run_cmd("d730");
                }
                if (phase == 1u) {
                    host_tick += T_DIST_STILL_MS; test_poll(); CHECK(s_seq_state == SQ_WAIT);
                }
                if (phase >= 2u) CHECK(sequence_start_stage() == 0);
            }
            if (phase == 3u && stage != 7u) {
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
            if (stage != 7u && dist_mode() && phase >= 2u)
                CHECK(s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : 2.0f));
            CHECK(s_seq_stage == stage && route_seq_active());
            run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED);
        }
    }
    return 0;
}

static int check_direction_exclusion(void)
{
    static const int modes[] = {16,15,16};
    static const float targets[] = {-650.0f,80.0f,-190.0f};
    static const float velocities[] = {-300.0f,20.0f,-100.0f};
    /* Crossing/contact have no FF. Clearance uses the independent BFF,
     * never the caller's forward FFF. */
    for (unsigned i = 0u; i < 3u; ++i) {
        reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("g");
        s_seq_stage = (uint8_t)(3u + i); route_seq_prepare();
        CHECK(sequence_start_stage() == 0);
        CHECK(s_msel == modes[i] && s_dist_target == targets[i] && last_x == velocities[i]);
        CHECK(s_dist_ff_ratio == (i == 2u ? 0.00625f : 0.0f) &&
              s_dist_heading_kp == (i == 2u ? 0.3f : 0.0f) &&
              last_y == (i == 2u ? -0.625f : 0.0f));
        CHECK(ff_equal(s_route_forward_ff_ratio, 0.01f) && test_forward_ff_ratio() == 0.0125f);
    }
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
    host_messages[0] = '\0'; run_cmd("param");
    CHECK(strstr(host_messages, "ykp=0.300 source=ROUTE31 global_ykp=1.000") != NULL);
    run_cmd("g"); CHECK(sequence_start_stage() == 0);
    host_yaw = 3.0f; tick();
    CHECK(s_dist_heading_kp == 0.3f && ff_equal(last_w, -0.3f * 3.0f * 0.0174533f));
    run_cmd("g"); run_cmd("20"); run_cmd("ykp0.3");
    CHECK(step_heading_kp_deg() == 0.3f && s_route_heading_kp == 0.3f);
    run_cmd("31"); run_cmd("ykp2");
    CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
    CHECK(check_report("-0.00625", 1) == 0);
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
    CHECK(check_report("-0.00625", 1) == 0);
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
    CHECK(check_route_ff_sign_dispatch() == 0);
    CHECK(check_ram_retention_and_separation() == 0);
    CHECK(check_invalid_inputs_and_phase_locks() == 0);
    CHECK(check_direction_exclusion() == 0);
    CHECK(check_heading_slot_direct_values_and_isolation() == 0);
    CHECK(check_route_cross_heading_restart(31u) == 0);
    puts("route31 tuning scope:15 nodes inclbucket/NEW d730, fff-.00625/bff+.00625/ykp.3 trialseeds; seven heading-feedback/six FFlegs; crossing/contact yaw/FF disabled; freshback190 restoresykp+BFF; overrides/mission32 isolation, RAM reset, reports, invalid/60phase locks and distances/90/180 preserved passed");
    return 0;
}
