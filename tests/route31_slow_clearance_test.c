/* Real router, turn scheduler and request-bound parser; injected time/IMU.
 * Verifies software scheduling only, NOT physical clearance or traction. */
#define ROUTE31_TARGET_MID_FIXTURE_MAIN retained_slow_target_mid_main
#include "route31_target_mid_yaw_test.c"
#undef ROUTE31_TARGET_MID_FIXTURE_MAIN

/* Isolate the ordinary executor after its separately replayed internal RIGHT90.
 * The complete31 chain never seeds these flags or skips those turns. */
static int slow_start_ordinary_road(unsigned stage)
{
    CHECK(fast_prepare(31u,stage) == 0);
    if (stage == 6u || stage == 7u) {
        s_route31_exit_turn_done = 1u;
        if (stage == 7u) s_route31_entry_turn_done = 1u;
        route_seq_prepare();
    }
    CHECK(s_seq_state == SQ_STILL && sequence_start_stage() == 0 && s_seq_state == SQ_RUN);
    return 0;
}

static int slow_defaults_and_scope(void)
{
    const unsigned road[] = {0u,1u,3u,5u,6u,7u,12u,15u};
    CHECK(ROUTE31_MID_YAW_LIMIT_DEG == 1.0f && ROUTE31_POST_YAW_TOL_DEG == .3f &&
          ROUTE31_TURN_TOL_DEG == .3f && ROUTE31_YAW_RETRY_RUN_MS == 400u);
    CHECK(s_route31_plan[0].distance_mm == 545u);
    for (unsigned n = 0u; n < sizeof road / sizeof road[0]; ++n) {
        unsigned stage = road[n];
        CHECK(fast_prepare(31u,stage == ROUTE31_TARGET_CORNER_STAGE ? ROUTE31_TARGET_STAGE : stage) == 0);
        if (stage == ROUTE31_TARGET_CORNER_STAGE) {
            /* This is an isolated road-speed case, not a completed aim/fire.
             * Seed the road via a real current-request ACK/54 and capture it
             * BEFORE receive_end; the dedicated lifecycle test covers fire. */
            uint8_t order[]={0x62u,(uint8_t)wire_request,(uint8_t)(wire_request>>8),
                             9u,0u,0x54u,1u,0u,8u,2u,2u,6u,8u,255u};
            wire_ack(wire_request,2u,0u); wire_feed(order,sizeof order,0);
            route31_target_rank_capture(); CHECK(s_route31_target_order.rank == 2u);
            step_vision_receive_end(); s_target35_route=0u; s_target35_phase=TA_OFF;
            s_seq_stage = (uint8_t)stage; route_seq_prepare();
        }
        if (stage == ROUTE31_HOSTAGE_EXIT_STAGE) {
            s_route31_hostage_rank = 1u; route_seq_prepare();
        }
        if (stage == 6u || stage == 7u) {
            s_route31_exit_turn_done = 1u;
            if (stage == 7u) s_route31_entry_turn_done = 1u;
            route_seq_prepare();
            if (stage == 6u) {
                /* Distinct retained strafe knobs must not leak into FORWARD. */
                s_route31_exit_right_ff_ratio = .02f;
                s_route31_exit_yaw_kp = 9.0f;
            }
        }
        CHECK(sequence_start_stage() == 0);
        float expected = stage == 3u ? 300.0f : stage >= 9u ? ROUTE31_TASK_V_MMS :
                         stage == 0u ? 250.0f : 150.0f;
        CHECK(s_v == expected && route_seq_speed_mms() == expected);
        CHECK(s_route31_straight_v == 150.0f && s_route31_lateral_v == 250.0f);
        CHECK(stage != 0u ||
              (s_dist_heading_kp == 4.5f && s_dist_ff_ratio == .010f));
        CHECK(stage != 0u || (s_dist_target == -(float)ROUTE31_START_LEFT_MM && last_y == -250.0f));
        CHECK(stage != 6u || (s_msel == 15 && s_dist_target == (float)ROUTE31_EXIT_FORWARD_MM &&
                              s_dist_heading_kp == 10.0f && s_dist_ff_ratio == s_route_forward_ff_ratio &&
                              fabsf(last_y+s_route_forward_ff_ratio*s_v)<.0001f));
        CHECK(stage != 7u || s_dist_target == (float)ROUTE31_TASK_ENTRY_FORWARD_MM);
        CHECK(stage != 7u || (s_msel == 15 && s_dist_heading_kp == 10.0f &&
                              s_dist_ff_ratio == s_route_forward_ff_ratio &&
                              fabsf(last_y+s_route_forward_ff_ratio*s_v)<.0001f));
        CHECK(stage != 3u || (s_dist_target == -(float)ROUTE31_CORNER_CROSS_MM &&
                             s_dist_heading_kp == 0.0f && s_dist_ff_ratio == 0.0f));
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !laser_state);
    }
    CHECK(fast_start_road(31u,4u) == 0);
    CHECK(s_v == 40.0f && s_dist_heading_kp == 0.0f);
    run_cmd("0");
    const unsigned turn_stages[] = {2u,6u,7u,8u};
    for (unsigned turn = 0u; turn < sizeof turn_stages / sizeof turn_stages[0]; ++turn) {
        unsigned stage = turn_stages[turn];
        CHECK(fast_start_road(31u,stage) == 0);
        CHECK(turn_speed_scale(90.0f) == 1.0f && turn_speed_scale(20.0f) == 1.0f);
        CHECK(fabsf(turn_velocity(90.0f) - T_TURN_MAX_W) < .00001f);
        CHECK(fabsf(turn_velocity(-90.0f) + T_TURN_MAX_W) < .00001f);
        CHECK(fabsf(turn_velocity(.5f) - .40f) < .00001f &&
              fabsf(turn_velocity(-.5f) + .40f) < .00001f);
        CHECK(s_msel == 20 && turn_target_deg() == 90.0f && !host_timer_active);
        run_cmd("0");
    }
    CHECK(fast_prepare(43u,2u) == 0);
    CHECK(s_route43_tune.straight_v == 100.0f && s_route43_tune.lateral_v == 100.0f);
    CHECK(s_route43_tune.road_mm[7] == 780u && s_route43_tune.road_mm[0] == 545u);
    CHECK(turn_speed_scale(90.0f) == 1.0f && turn_speed_scale(20.0f) == 1.0f);
    CHECK(s_route43_tune.left_yaw_kp == 3.0f && s_route43_tune.exit_yaw_kp == 3.0f);
    CHECK(fabsf(turn_velocity(.5f) - .30f) < .00001f &&
          fabsf(turn_velocity(-.5f) + .30f) < .00001f);
    run_cmd("0"); run_cmd("20");
    CHECK(turn_speed_scale(90.0f) == 1.0f && turn_target_deg() == 90.0f);
    CHECK(fabsf(turn_velocity(.5f) - .18f) < .00001f &&
          fabsf(turn_velocity(-.5f) + .18f) < .00001f);
    run_cmd("0");
    run_cmd("30");
    CHECK(T_TURN90_LEFT_COMP_DEG == 0.0f && turn_target_deg() == -90.0f);
    CHECK(turn_speed_scale(-90.0f) == 1.0f);
    run_cmd("0"); run_cmd("22");
    CHECK(turn_target_deg() == 180.0f);
    run_cmd("0");
    return 0;
}

/* Isolated real correction executor: signed outputs are commands, not proof
 * of chassis torque. Test inactive retained31 too, so it cannot leak .40
 * into a later manual session. No distance or stage geometry is altered. */
static int slow_signed_yaw_floor_scope(void)
{
    const unsigned owner[] = {31u,43u,44u,0u,31u};
    const float floor[] = {.40f,.30f,.30f,.08f,.08f};
    for (unsigned n = 0u; n < sizeof owner / sizeof owner[0]; ++n)
        for (int side = -1; side <= 1; side += 2) {
            CHECK(wire_boot() == 0);
            s_seq_mode = (uint8_t)owner[n];
            s_seq_state = n < 3u ? SQ_RUN : SQ_OFF;
            s_seq_stage = 0u; s_route44_index = 0u; s_msel = 17;
            s_round = R_ALIGN; s_dist_heading0 = 0.0f;
            s_dist_align_t0 = host_tick; s_dist_align_hold = 0u;
            host_yaw = (float)side * .5f;
            dist_align_poll();
            CHECK(last_x == 0.0f && last_y == 0.0f && !laser_state &&
                  fabsf(last_w + (float)side * floor[n]) < .00001f);
            if (n < 3u) {
                CHECK(dist_align_max_w() == .60f);
                CHECK(dist_align_tolerance_deg() == (owner[n] == 31u ? .3f : .4f));
                CHECK(turn_tolerance_deg() == (owner[n] == 31u ? .3f : .4f));
                CHECK(dist_heading_kp_get(NULL) == (owner[n] == 31u ? 4.5f : 3.0f));
                CHECK(fabsf(turn_velocity(.5f) - (owner[n] == 31u ? .40f : owner[n] == 43u ? .30f : .18f)) < .00001f);
                CHECK(fabsf(turn_velocity(-.5f) + (owner[n] == 31u ? .40f : owner[n] == 43u ? .30f : .18f)) < .00001f);
            } else {
                CHECK(fabsf(turn_velocity(.5f) - .18f) < .00001f &&
                      fabsf(turn_velocity(-.5f) + .18f) < .00001f);
            }
        }
    return 0;
}

static int slow_strict31_end_tolerance_and_legacy_scope(void)
{
    const unsigned owners[] = {31u,43u,44u};
    for (unsigned n = 0u; n < sizeof owners / sizeof owners[0]; ++n)
        for (int side = -1; side <= 1; side += 2) {
            CHECK(wire_boot() == 0);
            s_seq_mode = (uint8_t)owners[n]; s_seq_state = SQ_RUN;
            s_seq_stage = 0u; s_route44_index = 0u; s_msel = 17;
            s_round = R_ALIGN; s_dist_heading0 = 0.0f; s_dist_align_t0 = host_tick;
            host_yaw = (float)side * .3f; dist_align_poll();
            if (owners[n] == 31u) {
                CHECK(!s_dist_align_hold && fabsf(last_w + (float)side * .40f) < .00001f);
                host_yaw = (float)side * .299f; dist_align_poll();
                CHECK(s_dist_align_hold && fast_stopped());
            } else CHECK(s_dist_align_hold && fast_stopped());
            CHECK(turn_error_accepted((float)side * .3f));
            CHECK(turn_error_accepted((float)side * .4f) == (owners[n] != 31u));
        }
    return 0;
}

static int slow_mid_threshold_and_original_heading(void)
{
    const unsigned stages[] = {1u,5u,6u,7u};
    const float limits[] = {2.0f,2.0f,2.0f,1.0f};
    CHECK(ROUTE31_EARLY_MID_YAW_LIMIT_DEG == 2.0f &&
          ROUTE31_EARLY_MID_YAW_LAST_STAGE == 6u &&
          ROUTE31_PRE_CROSS_MID_YAW_LIMIT_DEG == 2.0f &&
          ROUTE31_MID_YAW_LIMIT_DEG == 1.0f && ROUTE43_MID_YAW_LIMIT_DEG == 1.5f);
    for (unsigned side = 0u; side < 2u; ++side) {
        float sign = side ? -1.0f : 1.0f;
        for (unsigned i = 0u; i < sizeof stages / sizeof stages[0]; ++i) {
            CHECK(slow_start_ordinary_road(stages[i]) == 0);
            const float original = s_dist_heading0;
            const unsigned zeros = (unsigned)zero_calls;
            CHECK(dist_mid_yaw_enabled() && route_mid_yaw_limit_deg() == limits[i]);
            host_yaw = original + sign * limits[i];
            host_tick += 20u; wire_poll();
            CHECK(s_round == R_RUN && !s_dist_mid_pause);
            host_yaw = original + sign * (limits[i] + .001f);
            host_tick += 20u; wire_poll();
            CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && fast_stopped());
            CHECK((unsigned)zero_calls == zeros && s_dist_heading0 == original);
            run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
        }

        CHECK(fast_target_seek(31u) == 0);
        host_yaw = sign * 1.0f; target_mid_elapsed(20u);
        CHECK(!s_target31_mid_phase && last_x == ROUTE31_TASK_V_MMS);
        CHECK(target_mid_begin(sign * 1.001f) == 0);
        CHECK(fabsf(last_w + sign * .40f) < .00001f && !laser_state);
        unsigned zeros = (unsigned)zero_calls;
        host_yaw = sign * .2f; target_mid_elapsed(20u);
        CHECK(s_target31_mid_phase == 3u && fast_stopped());
        target_mid_elapsed(399u); CHECK(s_target31_mid_phase == 3u);
        target_mid_elapsed(1u);
        CHECK(!s_target31_mid_phase && s_target31_mid_need_new && last_x == ROUTE31_TASK_V_MMS && last_y == 0.0f);
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
    /* Only31's distance stage policy changed. Both legacy owners retain1.5
     * throughout early/late stage numbers; their complete runners stay gated. */
    const unsigned owners[] = {43u,44u};
    for (unsigned owner = 0u; owner < sizeof owners / sizeof owners[0]; ++owner)
        for (unsigned stage = 0u; stage < sizeof stages / sizeof stages[0]; ++stage) {
            CHECK(fast_prepare(31u,stages[stage]) == 0);
            s_seq_mode = (uint8_t)owners[owner];
            CHECK(route_mid_yaw_limit_deg() == 1.5f &&
                  route_search_mid_yaw_limit_deg() == 1.5f);
            run_cmd("0");
        }
    return 0;
}

int main(void)
{
    CHECK(slow_defaults_and_scope() == 0);
    CHECK(slow_signed_yaw_floor_scope() == 0);
    CHECK(slow_strict31_end_tolerance_and_legacy_scope() == 0);
    CHECK(slow_mid_threshold_and_original_heading() == 0);
    CHECK(ROUTE31_TARGET_RANK1_MM == 635u && ROUTE31_TARGET_RANK2_MM == 465u &&
          ROUTE31_TARGET_RANK3_MM == 390u && ROUTE31_BALL_PREGRAB_LEFT_MM == 0u &&
          ROUTE31_HOSTAGE_PREGRAB_LEFT_MM == 0u);
    printf("31 revisedLEFT585/configuredCROSS550+diagonal100 then BACK190/RIGHT90/FWD775/RIGHT90/FWD775/RIGHT90 vs43 retained545/780/620; targetrank table635/465/390; ordinary150/pv250/ykp10/FFF_retained_no_bff/rff/xkp/tail%.0fFF0/pregrabLEFT0; signed31yaw/turnfloor0.40/43yaw0.30/44yaw0.30-turn0.18/manualyaw0.08-turn0.18 isolated; cross300/contact40;stages1/5/6 +/-2 exactmoves2.001brakes,stage7/target +/-1 exactmoves1.001brakes/end<0.3;43/44 trigger1.5 retained (host only)\n",ROUTE31_TASK_V_MMS);
    return 0;
}
