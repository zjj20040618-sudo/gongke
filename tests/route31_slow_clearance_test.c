/* Real router, turn scheduler and request-bound parser; injected time/IMU.
 * Verifies software scheduling only, NOT physical clearance or traction. */
#define ROUTE31_TARGET_MID_FIXTURE_MAIN retained_slow_target_mid_main
#include "route31_target_mid_yaw_test.c"
#undef ROUTE31_TARGET_MID_FIXTURE_MAIN

static int slow_defaults_and_scope(void)
{
    const unsigned road[] = {0u,1u,3u,5u,6u,7u,12u,15u};
    CHECK(ROUTE31_MID_YAW_LIMIT_DEG == 1.5f);
    CHECK(s_route31_plan[0].distance_mm == 545u);
    for (unsigned n = 0u; n < sizeof road / sizeof road[0]; ++n) {
        unsigned stage = road[n];
        CHECK(fast_prepare(31u,stage) == 0);
        if (stage == ROUTE31_HOSTAGE_EXIT_STAGE) {
            s_route31_hostage_rank = 1u; route_seq_prepare();
        }
        CHECK(sequence_start_stage() == 0);
        float expected = stage == 3u ? 300.0f : stage >= 9u ? 50.0f : 100.0f;
        CHECK(s_v == expected && route_seq_speed_mms() == expected);
        CHECK(s_route31_straight_v == 100.0f && s_route31_lateral_v == 100.0f);
        CHECK(stage != 0u || (s_dist_target == -565.0f && last_y == -100.0f));
        CHECK(stage != 7u || s_dist_target == -760.0f);
        CHECK(stage != 3u || (s_dist_target == -(float)ROUTE31_CROSS_BACK_MM &&
                             s_dist_heading_kp == 0.0f && s_dist_ff_ratio == 0.0f));
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !laser_state);
    }
    CHECK(fast_start_road(31u,4u) == 0);
    CHECK(s_v == 40.0f && s_dist_heading_kp == 0.0f);
    run_cmd("0");
    for (unsigned stage = 2u; stage <= 8u; stage += 6u) {
        if (stage == ROUTE31_PREDEPLOY_STAGE) {
            CHECK(fast_prepare(31u,7u) == 0);
            s_route31_rack_deployed = 1u;
            s_seq_stage = (uint8_t)stage; route_seq_prepare();
            CHECK(sequence_start_stage() == 0);
        } else CHECK(fast_start_road(31u,stage) == 0);
        CHECK(turn_speed_scale(90.0f) == 1.0f && turn_speed_scale(20.0f) == 1.0f);
        CHECK(fabsf(turn_velocity(90.0f) - T_TURN_MAX_W) < .00001f);
        CHECK(fabsf(turn_velocity(-90.0f) + T_TURN_MAX_W) < .00001f);
        CHECK(fabsf(turn_velocity(.5f) - ROUTE31_TURN_MIN_W) < .00001f);
        CHECK(turn_target_deg() == (stage == 2u ? 90.0f : -90.0f));
        run_cmd("0");
    }
    CHECK(fast_prepare(43u,2u) == 0);
    CHECK(s_route43_tune.straight_v == 100.0f && s_route43_tune.lateral_v == 100.0f);
    CHECK(s_route43_tune.road_mm[7] == 780u && s_route43_tune.road_mm[0] == 545u);
    CHECK(turn_speed_scale(90.0f) == 1.0f && turn_speed_scale(20.0f) == 1.0f);
    run_cmd("0"); run_cmd("20");
    CHECK(turn_speed_scale(90.0f) == 1.0f && turn_target_deg() == 90.0f);
    run_cmd("0");
    run_cmd("30");
    CHECK(T_TURN90_LEFT_COMP_DEG == 0.0f && turn_target_deg() == -90.0f);
    CHECK(turn_speed_scale(-90.0f) == 1.0f);
    run_cmd("0"); run_cmd("22");
    CHECK(turn_target_deg() == 180.0f);
    run_cmd("0");
    return 0;
}

static int slow_mid_threshold_and_original_heading(void)
{
    for (unsigned side = 0u; side < 2u; ++side) {
        float sign = side ? -1.0f : 1.0f;
        CHECK(fast_start_road(31u,1u) == 0);
        float original = s_dist_heading0;
        unsigned zeros = (unsigned)zero_calls;
        host_yaw = original + sign * 2.5f;
        host_tick += 20u; wire_poll();
        CHECK(s_round == R_RUN && !s_dist_mid_pause);
        host_yaw = original + sign * 2.501f;
        host_tick += 20u; wire_poll();
        CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && fast_stopped());
        CHECK((unsigned)zero_calls == zeros && s_dist_heading0 == original);
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());

        CHECK(fast_start_road(31u,5u) == 0);
        original = s_dist_heading0;
        host_yaw = original + sign * 1.5f;
        host_tick += 20u; wire_poll();
        CHECK(s_round == R_RUN && !s_dist_mid_pause);
        host_yaw = original + sign * 1.501f;
        host_tick += 20u; wire_poll();
        CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && fast_stopped());
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());

        CHECK(fast_target_seek(31u) == 0);
        host_yaw = sign * 1.5f; target_mid_elapsed(20u);
        CHECK(!s_target31_mid_phase && last_x == 50.0f);
        CHECK(target_mid_begin(sign * 1.501f) == 0);
        CHECK(last_w * sign < 0.0f && !laser_state);
        zeros = (unsigned)zero_calls;
        host_yaw = sign * .2f; target_mid_elapsed(20u);
        CHECK(s_target31_mid_phase == 3u && fast_stopped());
        target_mid_elapsed(399u); CHECK(s_target31_mid_phase == 3u);
        target_mid_elapsed(1u);
        CHECK(!s_target31_mid_phase && s_target31_mid_need_new && last_x == 50.0f && last_y == 0.0f);
        CHECK((unsigned)zero_calls == zeros && s_target35_heading == 0.0f && !laser_state);
        run_cmd("0");
    }
    CHECK(fast_start_road(31u,3u) == 0);
    host_yaw = s_dist_heading0 + 4.0f; host_tick += 20u; wire_poll();
    CHECK(s_round == R_RUN && !s_dist_mid_pause && s_v == 300.0f);
    run_cmd("0");
    CHECK(fast_target_seek(35u) == 0);
    host_yaw = 4.0f; target_mid_elapsed(20u);
    CHECK(!s_target31_mid_phase && last_x == 50.0f && !laser_state);
    run_cmd("0");
    return 0;
}

int main(void)
{
    CHECK(slow_defaults_and_scope() == 0);
    CHECK(slow_mid_threshold_and_original_heading() == 0);
    puts("31 revisedLEFT565/BACK760 vs43 retained545/780; ordinary100/tail50FF0; cross300/contact40; preCross2.5/postCross1.5/end0.4 originalheading/new-image barrier;35 isolated (host only)");
    return 0;
}
