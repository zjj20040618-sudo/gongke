/* Real Bluetooth dispatcher -> captured body velocity -> real precise IK and
 * four-wheel target dispatch. Peripheral stubs are inert; no vehicle acceptance. */
#define main retained_g_fixture_main
#include "g_command_stop_test.c"
#undef main

static float ff_delivered[4];
static unsigned ff_delivered_mask;
void ctrl_stop_all(void) { }
void ctrl_set_speed_precise(int motor, float rpm)
{
    ff_delivered[motor] = rpm;
    ff_delivered_mask |= 1u << (unsigned)motor;
}

/* Keep the dispatcher's observable capture stubs. Namespacing the real motion
 * implementation provides a second boundary check without editing the fixture. */
#define motion_init ff_real_motion_init
#define motion_ik ff_real_motion_ik
#define motion_vel_set ff_real_motion_vel_set
#define motion_vel_set_precise ff_real_motion_vel_set_precise
#define motion_brake ff_real_motion_brake
#define motion_pose ff_real_motion_pose
#define motion_pose_update ff_real_motion_pose_update
#define motion_odo_mm ff_real_motion_odo_mm
#define motion_lateral_odo_mm ff_real_motion_lateral_odo_mm
#define motion_ramp_init ff_real_motion_ramp_init
#define motion_ramp_step ff_real_motion_ramp_step
#define motion_profile_get ff_real_motion_profile_get
#define motion_profile_set ff_real_motion_profile_set
#define motion_profile_config_missing ff_real_motion_profile_config_missing
#define motion_linear_ramp_init ff_real_motion_linear_ramp_init
#define motion_linear_profile_step ff_real_motion_linear_profile_step
#define motion_linear_ramp_step ff_real_motion_linear_ramp_step
#include "../App/motion.c"
#undef motion_init
#undef motion_ik
#undef motion_vel_set
#undef motion_vel_set_precise
#undef motion_brake
#undef motion_pose
#undef motion_pose_update
#undef motion_odo_mm
#undef motion_lateral_odo_mm
#undef motion_ramp_init
#undef motion_ramp_step
#undef motion_profile_get
#undef motion_profile_set
#undef motion_profile_config_missing
#undef motion_linear_ramp_init
#undef motion_linear_profile_step
#undef motion_linear_ramp_step

static int ff_near(float a, float b)
{
    return fabsf(a - b) < 0.00001f;
}

static void ff_select(int mode, unsigned speed)
{
    char command[20];
    snprintf(command, sizeof command, "%d", mode); run_cmd(command);
    snprintf(command, sizeof command, "v%u", speed); run_cmd(command);
}

static int ff_check_real_ik(void)
{
    const float velocity_per_rpm = M_WHEEL_R_MM * 2.0f * 3.14159f / 60.0f;
    float expected[4];
    motion_ik_precise(last_x, last_y, last_w, expected);
    ff_delivered_mask = 0u;
    ff_real_motion_vel_set_precise(last_x, last_y, last_w);
    CHECK(ff_delivered_mask == 15u);
    for (int m = 0; m < 4; ++m) CHECK(ff_near(ff_delivered[m], expected[m]));
    float recovered_x = (ff_delivered[0] + ff_delivered[1] + ff_delivered[2] + ff_delivered[3]) * 0.25f * velocity_per_rpm;
    float recovered_y = (-ff_delivered[0] + ff_delivered[1] - ff_delivered[2] + ff_delivered[3]) * 0.25f * velocity_per_rpm;
    CHECK(fabsf(recovered_x - last_x) < 0.0002f);
    CHECK(fabsf(recovered_y - last_y) < 0.0002f);
    return 0;
}

static int check_seed_directions_and_all_speeds(void)
{
    static const unsigned speeds[] = {1u,20u,100u,200u,300u,350u,600u};
    for (int mode = 15; mode <= 16; ++mode)
        for (unsigned n = 0u; n < sizeof speeds / sizeof speeds[0]; ++n) {
            reset_fixture(); ff_select(mode, speeds[n]); run_cmd("d1500"); run_cmd("g"); tick();
            float ratio = mode == 15 ? -0.00625f : 0.00625f;
            CHECK(s_round == R_RUN && s_dist_precise == 1u && ff_near(s_dist_ff_ratio, ratio));
            CHECK(ff_near(last_x, mode == 15 ? (float)speeds[n] : -(float)speeds[n]));
            CHECK(ff_near(last_y, -ratio * (float)speeds[n]) && last_w == 0.0f);
            CHECK(host_precise_calls > 0u && host_integer_calls == 0u);
            CHECK(ff_check_real_ik() == 0);
            /* Signed lateral term survives sub-RPM precision at low speeds. */
            CHECK(mode == 15 ? ff_delivered[1] > ff_delivered[0] : ff_delivered[1] < ff_delivered[0]);
            run_cmd("g"); CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        }
    puts("FFF: all normal speeds1/20/100/200/300/350/600; forward-left-drift gets body-right, backward-body-right gets body-left; precise IK dispatch/roundtrip passed");
    return 0;
}

static int check_independent_commands_validation_and_snapshot(void)
{
    static const char *const invalid[] = {"fff0.051","fff-0.051","fff","fff0.02junk","fffnan",
                                         "bff0.051","bff-0.051","bff","bff0.02junk","bffnan"};
    reset_fixture(); ff_select(15,100);
    run_cmd("fff-0.02"); run_cmd("bff0.03");
    CHECK(ff_near(s_forward_ff_ratio, -0.02f) && ff_near(s_backward_ff_ratio, 0.03f));
    for (unsigned n = 0u; n < sizeof invalid / sizeof invalid[0]; ++n) {
        run_cmd(invalid[n]);
        CHECK(strstr(last_message, "ERR") != NULL);
        CHECK(ff_near(s_forward_ff_ratio, -0.02f) && ff_near(s_backward_ff_ratio, 0.03f));
    }
    for (unsigned speed = 100u; speed <= 300u; speed += 100u) {
        ff_select(15, speed); run_cmd("d1000"); run_cmd("g"); tick();
        CHECK(ff_near(s_dist_ff_ratio, -0.02f) && ff_near(last_y, (float)speed * 0.02f));
        run_cmd("fff0.01"); CHECK(strstr(last_message, "ERR") != NULL);
        tick(); CHECK(ff_near(s_dist_ff_ratio, -0.02f));
        run_cmd("g"); host_tick += T_DIST_STILL_MS; tick();
        ff_select(16, speed); run_cmd("d1000"); run_cmd("g"); tick();
        CHECK(ff_near(s_dist_ff_ratio, 0.03f) && ff_near(last_y, -(float)speed * 0.03f));
        run_cmd("bff0.01"); CHECK(strstr(last_message, "ERR") != NULL);
        tick(); CHECK(ff_near(s_dist_ff_ratio, 0.03f));
        run_cmd("g"); host_tick += T_DIST_STILL_MS; tick();
    }
    ff_select(15,300); run_cmd("fff0"); run_cmd("bff0"); run_cmd("d1000"); run_cmd("g"); tick();
    CHECK(s_dist_ff_ratio == 0.0f && last_x == 300.0f && last_y == 0.0f);
    run_cmd("g"); host_tick += T_DIST_STILL_MS; tick();
    ff_select(16,300); run_cmd("d1000"); run_cmd("g"); tick();
    CHECK(s_dist_ff_ratio == 0.0f && last_x == -300.0f && last_y == 0.0f);
    run_cmd("g"); host_tick += T_DIST_STILL_MS; tick();
    test_init();
    CHECK(ff_near(s_forward_ff_ratio,-0.00625f) && ff_near(s_backward_ff_ratio,0.00625f));
    puts("FFF: fff/bff independent RAM ratios, both signs/zero/range/malformed rejection, speed switches, run snapshot/write locks and test_init restoration passed");
    return 0;
}

static int check_body_relative_sign_contract_and_mission_isolation(void)
{
    static const float ratios[] = {-0.05f,-0.01f,0.0f,0.01f,0.05f};
    for (int mode = 15; mode <= 16; ++mode)
        for (unsigned n = 0u; n < sizeof ratios / sizeof ratios[0]; ++n) {
            char command[24];
            reset_fixture(); ff_select(mode,300);
            snprintf(command,sizeof command,"%s%.3f",mode == 15 ? "fff" : "bff",ratios[n]);
            run_cmd(command); CHECK(strstr(last_message,"ERR") == NULL);
            run_cmd("d1500"); run_cmd("g"); tick();
            CHECK(ff_near(s_dist_ff_ratio,ratios[n]));
            CHECK(ff_near(last_y,-ratios[n]*300.0f));
            CHECK(ff_check_real_ik() == 0);
        }
    reset_fixture();
    float mission_before = test_forward_ff_ratio();
    ff_select(15,300);
    run_cmd("fff-0.03"); run_cmd("bff0.03");
    CHECK(ff_near(test_forward_ff_ratio(),mission_before));
    run_cmd("32");
    CHECK(ff_near(test_forward_ff_ratio(),mission_before));
    CHECK(!host_laser_on_calls && !pulse_calls && !servo_calls);
    reset_fixture(); ff_select(15,300); run_cmd("fff-0.03"); run_cmd("bff0.03"); run_cmd("35");
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !host_laser_on_calls);
    puts("FFF: +/- sign always refers to vehicle-body left/right independent of forward/backward travel; candidate manual defaults do not leak to formal32 or activate35 passed");
    return 0;
}

static int check_strafe_threshold_and_turn_isolation(void)
{
    for (int mode = 17; mode <= 18; ++mode)
        for (unsigned speed = 200u; speed <= 300u; speed += 100u) {
            reset_fixture(); ff_select(mode,speed);
            run_cmd("fff0.04"); run_cmd("bff-0.04"); run_cmd("lff0.02"); run_cmd("rff0.03");
            run_cmd("d1500"); run_cmd("g"); tick();
            float ratio = speed == 300u ? (mode == 17 ? 0.02f : 0.03f) : 0.0f;
            float signed_speed = mode == 17 ? -(float)speed : (float)speed;
            CHECK(ff_near(s_dist_ff_ratio,ratio) && last_y == signed_speed);
            CHECK(ff_near(last_x,ratio*signed_speed) && last_w == 0.0f);
            CHECK(ff_check_real_ik() == 0);
        }
    static const int turns[] = {20,22,30};
    for (unsigned n = 0u; n < sizeof turns / sizeof turns[0]; ++n) {
        reset_fixture(); ff_select(15,300); run_cmd("fff-0.04"); run_cmd("bff0.04"); ff_select(turns[n],300);
        run_cmd("g"); tick();
        CHECK(last_x == 0.0f && last_y == 0.0f && host_precise_calls == 0u);
        CHECK(last_w == (turns[n] == 30 ? -T_TURN_MAX_W : T_TURN_MAX_W));
    }
    puts("FFF: lff/rff originalv300 gate and signed orthogonal axis unchanged;90/180/left92 rotation behavior isolated passed");
    return 0;
}

static int check_route37_hold_disabled_is_exempt(void)
{
    for (unsigned stage = 0u; stage < 3u; ++stage) {
        reset_fixture(); ff_select(15,300); run_cmd("fff-0.04"); run_cmd("bff0.04");
        run_cmd("37"); run_cmd("fff-0.02"); run_cmd("bff0.03"); run_cmd("g");
        s_seq_stage = (uint8_t)stage; route_seq_prepare();
        CHECK(sequence_start_stage() == 0);
        CHECK(ff_near(s_forward_ff_ratio,-0.04f) && ff_near(s_backward_ff_ratio,0.04f));
        CHECK(s_dist_ff_ratio == (stage == 2u ? 0.03f : 0.0f));
        CHECK(last_y == (stage == 2u ? -3.0f : 0.0f));
        CHECK(stage == 0u ? last_x == -300.0f : stage == 1u ? last_x == 20.0f : last_x == -100.0f);
        CHECK(ff_check_real_ik() == 0);
        run_cmd("bff0"); CHECK(strstr(last_message,"ERR ROUTE_SEQ_ACTIVE") != NULL);
        CHECK(ff_near(s_route_backward_ff_ratio,0.03f));
        run_cmd("g"); CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    }
    puts("FFF:37 crossing650 and board80 keep zero correction despite nonzero RAM values; normal back190 uses route-only bff, global slots untouched passed");
    return 0;
}

static void ff_select_route(unsigned owner)
{
    char command[16];
    snprintf(command,sizeof command,"%u",owner); run_cmd(command);
}

static int check_route_strafe_seed_report_and_manual_isolation(void)
{
    reset_fixture(); ff_select(17,300); run_cmd("lff0.04"); run_cmd("rff-0.04");
    ff_select_route(37u);
    CHECK(s_route_left_ff_ratio == 0.0f && s_route_right_ff_ratio == 0.0f);
    host_messages[0] = '\0'; run_cmd("param");
    CHECK(strstr(host_messages,"lff=0.0000") != NULL && strstr(host_messages,"rff=0.0000") != NULL);
    run_cmd("lff0.02"); run_cmd("rff-0.03");
    CHECK(ff_near(s_route_left_ff_ratio,0.02f) && ff_near(s_route_right_ff_ratio,-0.03f));
    CHECK(ff_near(s_left_ff_ratio,0.04f) && ff_near(s_right_ff_ratio,-0.04f));
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    ff_select_route(36u);
    CHECK(ff_near(s_route_left_ff_ratio,0.02f) && ff_near(s_route_right_ff_ratio,-0.03f));
    run_cmd("g"); s_seq_stage = 6u; route_seq_prepare();
    CHECK(sequence_start_stage() == 0 && s_msel == 18 && s_v == 100.0f);
    CHECK(ff_near(s_dist_ff_ratio,-0.03f) && ff_near(last_x,-3.0f) && last_y == 100.0f);
    CHECK(ff_check_real_ik() == 0);
    run_cmd("g"); ff_select(17,300); run_cmd("d1500"); run_cmd("g"); tick();
    CHECK(ff_near(s_dist_ff_ratio,0.04f) && ff_near(last_x,-12.0f) && last_y == -300.0f);
    run_cmd("g"); host_tick += T_DIST_STILL_MS; tick();
    ff_select(18,300); run_cmd("d1500"); run_cmd("g"); tick();
    CHECK(ff_near(s_dist_ff_ratio,-0.04f) && ff_near(last_x,-12.0f) && last_y == 300.0f);
    run_cmd("g"); host_tick += T_DIST_STILL_MS; tick();
    ff_select_route(31u);
    CHECK(ff_near(s_route_left_ff_ratio,0.02f) && ff_near(s_route_right_ff_ratio,-0.03f));
    test_init();
    CHECK(s_route_left_ff_ratio == 0.0f && s_route_right_ff_ratio == 0.0f);
    puts("routeFFF: default unmeasured lateral slots0 reported,37 accepts/preserves inert LFF/RFF slots then36 applies them atv100; route/manual values never leak; test_init restores0 passed");
    return 0;
}

static int check_all_route_owners_precise_ff_at_all_speeds(void)
{
    static const struct {unsigned owner,stage; int mode;} cases[] = {
        {31u,0u,17},{31u,1u,16},{31u,10u,15},
        {34u,0u,17},{34u,1u,16},{34u,10u,15},
        {36u,0u,17},{36u,1u,16},{36u,6u,18},{36u,9u,15},
        {37u,2u,16}
    };
    static const unsigned speeds[] = {1u,20u,100u,200u,300u,350u,600u};
    for (unsigned n = 0u; n < sizeof cases / sizeof cases[0]; ++n)
        for (unsigned k = 0u; k < sizeof speeds / sizeof speeds[0]; ++k) {
            reset_fixture(); ff_select_route(cases[n].owner);
            run_cmd("fff-0.02"); run_cmd("bff0.03"); run_cmd("lff0.02"); run_cmd("rff-0.03");
            run_cmd("g"); s_seq_stage = (uint8_t)cases[n].stage; route_seq_prepare();
            CHECK(sequence_start_stage() == 0 && s_msel == cases[n].mode && route_seq_leg()->heading_hold);
            /* Caller-owned speed input tests the existing real start boundary;
             * the const route recipe is NOT edited and BLE cannot bypass its
             * active write lock to make this speed change on a real route. */
            motion_brake(); s_round = R_READY; s_v = (float)speeds[k]; mode_start(); tick();
            CHECK(s_round == R_RUN && s_seq_state == SQ_RUN && s_seq_mode == cases[n].owner);
            float speed = (float)speeds[k];
            float ratio = cases[n].mode == 15 ? -0.02f : cases[n].mode == 16 ? 0.03f :
                          cases[n].mode == 17 ? 0.02f : -0.03f;
            CHECK(ff_near(s_dist_ff_ratio,ratio));
            if (cases[n].mode <= 16) {
                CHECK(last_x == (cases[n].mode == 15 ? speed : -speed));
                CHECK(ff_near(last_y,-ratio*speed));
            } else {
                CHECK(last_y == (cases[n].mode == 17 ? -speed : speed));
                CHECK(ff_near(last_x,ratio*last_y));
            }
            CHECK(last_w == 0.0f && ff_check_real_ik() == 0);
            float snapshot = s_dist_ff_ratio;
            run_cmd("lff0"); CHECK(strstr(last_message,"ERR ROUTE_SEQ_ACTIVE") != NULL);
            run_cmd("rff0"); CHECK(strstr(last_message,"ERR ROUTE_SEQ_ACTIVE") != NULL);
            CHECK(s_dist_ff_ratio == snapshot);
            run_cmd("g"); CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        }
    puts("routeFFF: all4 route owners and their existing ordinary directions,77 direction/speed input cases includingv100/v300; FFF/BFF/LFF/RFF reach real precise4wheel IK, active snapshots locked passed");
    return 0;
}

int main(void)
{
    CHECK(check_seed_directions_and_all_speeds() == 0);
    CHECK(check_independent_commands_validation_and_snapshot() == 0);
    CHECK(check_strafe_threshold_and_turn_isolation() == 0);
    CHECK(check_route37_hold_disabled_is_exempt() == 0);
    CHECK(check_body_relative_sign_contract_and_mission_isolation() == 0);
    CHECK(check_route_strafe_seed_report_and_manual_isolation() == 0);
    CHECK(check_all_route_owners_precise_ff_at_all_speeds() == 0);
    puts("translation_feedforward_test: all host checks passed; drift ratio/sign with installed load still needs ruler measurement");
    return 0;
}
