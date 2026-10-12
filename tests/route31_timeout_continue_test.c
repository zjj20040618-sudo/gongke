/* Real BLE route scheduler; synthetic time/IMU/counts are not physical proof. */
#define ROUTE31_FAST_TIMING_FIXTURE_MAIN retained_timeout_fast_main
#include "route31_fast_timing_test.c"
#undef ROUTE31_FAST_TIMING_FIXTURE_MAIN

static void timeout_elapsed(uint32_t ms) { host_tick += ms; wire_poll(); }

/* A synthetic executor entry follows an already-completed internal RIGHT90;
 * the full automatic chain validates those real turn completions separately. */
static int timeout_start_ordinary_road(unsigned stage)
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

static int check_defaults_gain_and_stopped_commands(void)
{
    CHECK(timeout_start_ordinary_road(7u) == 0);
    CHECK(s_v == 150.0f && s_route31_lateral_v == 250.0f && s_dist_heading_kp == 10.0f);
    host_yaw = s_dist_heading0 + 1.0f; timeout_elapsed(20u);
    CHECK(s_round == R_RUN && !s_dist_mid_pause && last_w < 0.0f);
    CHECK(fabsf(last_w + 10.0f * 0.0174533f) < .0001f);
    run_cmd("0"); run_cmd("31"); run_cmd("ykp8");
    CHECK(route_heading_kp() == 8.0f && s_route_heading_kp == .3f);
    run_cmd("ykp11"); CHECK(route_heading_kp() == 8.0f);
    run_cmd("ykp10"); CHECK(route_heading_kp() == 10.0f);
    CHECK(s_route31_left_yaw_kp == 4.5f && s_route31_exit_yaw_kp == 4.5f);
    run_cmd("43"); CHECK(route_heading_kp() == .3f && s_route43_tune.lateral_v == 100.0f);
    CHECK(s_route43_tune.left_yaw_kp == 3.0f && s_route43_tune.exit_yaw_kp == 3.0f);
    run_cmd("ykp10"); CHECK(route_heading_kp() == .3f);
    run_cmd("0"); run_cmd("15"); run_cmd("v100"); run_cmd("ykp10");
    CHECK(dist_heading_kp_get(NULL) == step_heading_kp_deg());
    CHECK(fast_prepare(31u,3u) == 0 && sequence_start_stage() == 0);
    CHECK(s_dist_heading_kp == 0.0f && s_v == 300.0f); run_cmd("0");
    return 0;
}

static int check_distance_mid_timeout_resume_and_fault_priority(void)
{
    for (unsigned fault = 0u; fault < 6u; ++fault) {
        CHECK(timeout_start_ordinary_road(7u) == 0);
        const float original_heading = s_dist_heading0, original_target = s_dist_target;
        const uint32_t original_test = s_active_test;
        const unsigned zeros = (unsigned)zero_calls;
        host_fore = s_dist_odo0 + 20.0f;
        host_yaw = s_dist_heading0 + 3.0f; timeout_elapsed(20u);
        CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u);
        timeout_elapsed(T_DIST_STILL_MS);
        CHECK(s_round == R_ALIGN && s_dist_mid_pause == 2u);
        host_tick = s_dist_align_t0 + ROUTE31_POST_YAW_MAX_MS - 1u; wire_poll();
        CHECK(s_round == R_ALIGN && s_seq_stage == 7u);
        if (fault == 1u) run_cmd("g");
        if (fault == 2u) host_imu_valid = 0;
        if (fault == 3u) host_yaw = NAN;
        if (fault == 4u) run_cmd("a");
        if (fault == 5u) run_cmd("0");
        timeout_elapsed(1u);
        if (!fault) {
            CHECK(s_round == R_BRAKE && s_dist_mid_pause == 3u && fast_stopped());
            timeout_elapsed(T_DIST_STILL_MS);
            CHECK(s_seq_state == SQ_RUN && s_seq_stage == 7u && s_round == R_RUN &&
                  !s_dist_mid_pause && s_dist_heading0 == original_heading &&
                  s_dist_target == original_target && s_active_test == original_test &&
                  s_dist_progress_mm == 20.0f && host_yaw == original_heading + 3.0f &&
                  (unsigned)zero_calls == zeros);
            CHECK(strstr(host_messages,"result=RESUME") &&
                  strstr(host_messages,"timeout_accept=1") &&
                  !strstr(host_messages,"status=YAW_TIMEOUT action=NEXT"));
            timeout_elapsed(399u);
            CHECK(s_round == R_RUN && !s_dist_mid_pause && last_x > 0.0f);
            timeout_elapsed(1u);
            CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u &&
                  s_seq_stage == 7u && s_active_test == original_test &&
                  s_dist_heading0 == original_heading && fast_stopped());
        } else CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
        run_cmd("0");
    }
    /* Other route owners retain their rejection of excessive timed-out yaw. */
    CHECK(fast_start_road(43u,7u) == 0);
    host_yaw = s_dist_heading0 + 3.0f; timeout_elapsed(20u);
    timeout_elapsed(T_DIST_STILL_MS);
    host_tick = s_dist_align_t0 + ROUTE31_POST_YAW_MAX_MS; wire_poll();
    CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 7u && fast_stopped());
    return 0;
}

static int check_forward_timeout_keeps_remaining_and_rotation_excluded(void)
{
    for (int side = -1; side <= 1; side += 2) {
        CHECK(fast_prepare(31u,1u) == 0);
        /* Isolate the final FORWARD executor using an already-accepted rank;
         * rank admission and whole-chain lifecycle are separate replay cases. */
        s_seq_stage = ROUTE31_HOSTAGE_EXIT_STAGE; s_route31_hostage_rank = 1u;
        route_seq_prepare(); CHECK(sequence_start_stage() == 0 && s_msel == 15);
        const float goal = s_dist_heading0, target = s_dist_target;
        const uint32_t test = s_active_test;
        const unsigned zeros = (unsigned)zero_calls;
        host_fore = s_dist_odo0 + 20.0f;
        host_yaw = goal + (float)side * 3.0f; timeout_elapsed(20u);
        CHECK(s_dist_mid_pause == 1u && s_round == R_BRAKE);
        timeout_elapsed(T_DIST_STILL_MS);
        CHECK(s_dist_mid_pause == 2u && s_round == R_ALIGN && s_dist_progress_mm == 20.0f);
        /* Rotation-induced encoder odometry must not consume translation. */
        host_fore += 50.0f; host_counts[0] += 100;
        host_tick = s_dist_align_t0 + ROUTE31_POST_YAW_MAX_MS; wire_poll();
        CHECK(s_round == R_BRAKE && s_dist_mid_pause == 3u);
        timeout_elapsed(T_DIST_STILL_MS);
        CHECK(s_round == R_RUN && s_seq_stage == ROUTE31_HOSTAGE_EXIT_STAGE &&
              s_dist_target == target && s_dist_heading0 == goal && s_active_test == test &&
              s_dist_progress_mm == 20.0f && dist_travel_mm() == 20.0f &&
              (unsigned)zero_calls == zeros && !laser_state);
        timeout_elapsed(399u); CHECK(s_round == R_RUN && !s_dist_mid_pause && last_x > 0.0f);
        timeout_elapsed(1u); CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && fast_stopped());
        run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
    }
    return 0;
}

static int check_postcross_forward_mid_timeout_and_scope(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
        for (int yaw_side = -1; yaw_side <= 1; yaw_side += 2) {
            CHECK(timeout_start_ordinary_road(6u) == 0);
            CHECK(s_route31_exit_turn_done && !s_route31_entry_turn_done &&
                  s_msel == 15 && s_dist_target == (float)ROUTE31_EXIT_FORWARD_MM &&
                  s_dist_heading_kp == 10.0f && s_dist_ff_ratio == s_route_forward_ff_ratio && dist_mid_yaw_enabled());
            /* Stage6 is actual FORWARD775. The negative-axis replay only
             * isolates the same executor's signed BACK775 accounting;
             * it does not change this route or claim physical reverse travel. */
            if (direction < 0) { s_msel = 16; s_dist_target = -(float)ROUTE31_EXIT_FORWARD_MM; }
            CHECK(!dist_lateral() && dist_mid_yaw_enabled());
            const float goal = s_dist_heading0, target = s_dist_target;
            const uint32_t test = s_active_test;
            const unsigned zeros = (unsigned)zero_calls;
            host_fore = s_dist_odo0 + (float)direction * 123.0f;
            host_lateral = s_dist_orth0 + 7.0f;
            host_counts[0] = 13;
            host_yaw = goal + (float)yaw_side * 2.0f; timeout_elapsed(20u);
            CHECK(s_round == R_RUN && !s_dist_mid_pause &&
                  last_x * (float)direction > 0.0f &&
                  last_w * (float)yaw_side < 0.0f);
            host_yaw = goal + (float)yaw_side * 2.001f; timeout_elapsed(20u);
            CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && fast_stopped());
            timeout_elapsed(T_DIST_STILL_MS);
            CHECK(s_round == R_ALIGN && s_dist_mid_pause == 2u &&
                  s_dist_progress_mm == (float)direction * 123.0f &&
                  s_dist_progress_counts[0] == 13);
            timeout_elapsed(20u);
            CHECK(last_w * (float)yaw_side < 0.0f);
            /* The yaw repair's encoder/orthogonal odometry must not consume
             * the remaining652mm or create a longitudinal path measurement. */
            host_fore += 50.0f; host_lateral += 40.0f; host_counts[0] += 100;
            host_tick = s_dist_align_t0 + ROUTE31_POST_YAW_MAX_MS; wire_poll();
            CHECK(s_round == R_BRAKE && s_dist_mid_pause == 3u && fast_stopped());
            timeout_elapsed(T_DIST_STILL_MS);
            CHECK(s_round == R_RUN && !s_dist_mid_pause && s_seq_stage == 6u &&
                  s_dist_target == target && s_dist_heading0 == goal &&
                  s_active_test == test && (unsigned)zero_calls == zeros &&
                  dist_travel_mm() == (float)direction * 123.0f &&
                  s_dist_target - dist_travel_mm() == (float)direction * ((float)ROUTE31_EXIT_FORWARD_MM - 123.0f) &&
                  dist_orth_odo() - s_dist_orth0 == 7.0f &&
                  s_dist_progress_counts[0] == 13 &&
                  s_dist_count_base[0] == host_counts[0] && !laser_state);
            timeout_elapsed(399u);
            CHECK(s_round == R_RUN && !s_dist_mid_pause &&
                  last_x * (float)direction > 0.0f);
            timeout_elapsed(1u);
            CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && fast_stopped() &&
                  s_seq_stage == 6u && s_active_test == test &&
                  s_dist_heading0 == goal && s_dist_target == target);
            run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
        }
    /* Preserve the departure leg and short arm offsets: no newly introduced
     * moving-yaw stop; their endpoint policy is tested by the chain fixtures. */
    CHECK(fast_start_road(31u,0u) == 0 && !dist_mid_yaw_enabled());
    host_yaw = s_dist_heading0 + 4.0f; timeout_elapsed(20u);
    CHECK(s_round == R_RUN && !s_dist_mid_pause); run_cmd("0");
    CHECK(timeout_start_ordinary_road(6u) == 0);
    /* Long lateral guards remain a resolver capability, not the new route:
     * changing mode here isolates its100mm predicate without claiming travel. */
    s_msel = 18;
    s_dist_target = 30.0f; CHECK(!dist_mid_yaw_enabled());
    s_dist_target = 99.999f; CHECK(!dist_mid_yaw_enabled());
    s_dist_target = 100.0f; CHECK(dist_mid_yaw_enabled()); run_cmd("0");
    CHECK(fast_start_road(43u,6u) == 0 && !dist_mid_yaw_enabled());
    host_yaw = s_dist_heading0 + 3.0f; timeout_elapsed(20u);
    CHECK(s_round == R_RUN && !s_dist_mid_pause); run_cmd("0");
    CHECK(timeout_start_ordinary_road(6u) == 0);
    /* Predicate owner isolation; the complete44 route fixture checks its
     * native physical/logical schedule separately.44 already guards straight
     * BACK; only its long lateral exclusion must remain unchanged here. */
    s_msel = 18;
    s_seq_mode = ROUTE44_MODE; CHECK(!dist_mid_yaw_enabled()); run_cmd("0");
    return 0;
}

static int check_target_mid_timeout_retry_same_request(void)
{
    const char *keys[] = {"g","a","0"};
    for (int side = -1; side <= 1; side += 2)
        for (unsigned key = 0u; key < sizeof keys / sizeof keys[0]; ++key) {
            CHECK(fast_target_seek(31u) == 0);
            const uint16_t request = wire_request;
            const float goal = s_target35_heading;
            const unsigned zeros = (unsigned)zero_calls;
            host_yaw = goal + (float)side * 1.001f; timeout_elapsed(20u);
            CHECK(s_target31_mid_phase == 1u && fast_stopped());
            timeout_elapsed(T_DIST_STILL_MS);
            CHECK(s_target31_mid_phase == 2u && last_w * (float)side < 0.0f);
            timeout_elapsed(ROUTE31_POST_YAW_MAX_MS);
            CHECK(s_target31_mid_phase == 4u && fast_stopped() && !s_target35_good);
            timeout_elapsed(T_DIST_STILL_MS);
            CHECK(!s_target31_mid_phase && s_target31_mid_need_new &&
                  s_target35_phase == TA_SEEK && last_x == ROUTE31_TASK_V_MMS &&
                  s_target35_heading == goal && wire_request == request && !laser_state &&
                  (unsigned)zero_calls == zeros && strstr(host_messages,"timeout_accept=1"));
            timeout_elapsed(399u);
            CHECK(!s_target31_mid_phase && last_x == ROUTE31_TASK_V_MMS && !s_target35_good);
            timeout_elapsed(1u);
            CHECK(s_target31_mid_phase == 1u && fast_stopped() &&
                  wire_request == request && s_target35_heading == goal);
            run_cmd(keys[key]); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
            timeout_elapsed(5000u);
            CHECK(s_seq_state == SQ_STOPPED && fast_stopped() && !host_laser_on_calls);
        }
    return 0;
}

static int check_turn_timeout_next_and_other_owner(void)
{
    for (unsigned owner = 31u; owner <= 43u; owner += 12u) {
        CHECK(fast_start_road(owner,2u) == 0);
        host_yaw = 10.0f; host_tick = s_meas_t0 + turn_max_ms(); wire_poll();
        CHECK(s_round == R_BRAKE && !strcmp(s_turn_result,"TIMEOUT"));
        timeout_elapsed(turn_settle_ms());
        if (owner == 31u) {
            CHECK(s_seq_state != SQ_STOPPED && s_seq_stage == 3u);
            CHECK(strstr(host_messages,"status=TURN_TIMEOUT action=NEXT reached=0"));
            CHECK(strstr(host_messages,"status=TIMEOUT cmd_deg=90.00"));
        } else CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 2u);
        run_cmd("0");
    }
    CHECK(fast_start_road(31u,2u) == 0);
    turn_begin_settle("TIMEOUT"); host_yaw = NAN; timeout_elapsed(turn_settle_ms());
    CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
    return 0;
}

static int check_internal_turn_timeout_keeps_its_forward_leg(void)
{
    for (unsigned stage = 6u; stage <= 8u; ++stage) {
        CHECK(fast_start_road(31u,stage) == 0 && s_msel == 20 &&
              turn_target_deg() == 90.0f);
        host_yaw = 10.0f; host_tick = s_meas_t0 + turn_max_ms(); wire_poll();
        CHECK(s_round == R_BRAKE && !strcmp(s_turn_result,"TIMEOUT"));
        timeout_elapsed(turn_settle_ms());
        CHECK(s_seq_state != SQ_STOPPED && !laser_state && !host_timer_active);
        if (stage < 8u) {
            CHECK(s_seq_state == SQ_STILL && s_seq_stage == stage &&
                  (stage == 6u ? s_route31_exit_turn_done : s_route31_entry_turn_done) &&
                  route_seq_leg()->mode == 15u && route_seq_leg()->distance_mm ==
                  (stage == 6u ? ROUTE31_EXIT_FORWARD_MM : ROUTE31_TASK_ENTRY_FORWARD_MM));
            CHECK(sequence_start_stage() == 0 && s_msel == 15 && s_dist_target ==
                  (float)(stage == 6u ? ROUTE31_EXIT_FORWARD_MM : ROUTE31_TASK_ENTRY_FORWARD_MM));
        } else {
            wire_sync();
            CHECK(s_seq_stage == ROUTE31_PAIR_STAGE && s_seq_state == SQ_TASK &&
                  wire_task == 1u && s_receiving && !pulse_calls);
        }
        CHECK(strstr(host_messages,"status=TURN_TIMEOUT action=NEXT reached=0"));
        run_cmd("0");
        CHECK(s_seq_state == SQ_STOPPED && fast_stopped() &&
              !s_route31_exit_turn_done && !s_route31_entry_turn_done);
    }
    return 0;
}

static int check_contact_timeout_next_without_fake_contact(void)
{
    CHECK(fast_start_road(31u,4u) == 0 && s_board_contact.active);
    host_tick = s_meas_t0 + ROUTE31_BOARD_CONTACT_MAX_MS; wire_poll();
    CHECK(s_round == R_BRAKE && s_dist_reason == 5u);
    timeout_elapsed(T_DIST_STILL_MS);
    CHECK(s_seq_state != SQ_STOPPED && s_seq_stage == 5u && !s_board_contact.active);
    CHECK(strstr(host_messages,"result=CONTACT_TIMEOUT") &&
          strstr(host_messages,"status=CONTACT_TIMEOUT action=NEXT reached=0") &&
          !strstr(host_messages,"result=TILT_REACHED"));
    run_cmd("0");
    return 0;
}

int main(void)
{
    CHECK(check_defaults_gain_and_stopped_commands() == 0);
    CHECK(check_distance_mid_timeout_resume_and_fault_priority() == 0);
    CHECK(check_forward_timeout_keeps_remaining_and_rotation_excluded() == 0);
    CHECK(check_postcross_forward_mid_timeout_and_scope() == 0);
    CHECK(check_target_mid_timeout_retry_same_request() == 0);
    CHECK(check_turn_timeout_next_and_other_owner() == 0);
    CHECK(check_internal_turn_timeout_keeps_its_forward_leg() == 0);
    CHECK(check_contact_timeout_next_without_fake_contact() == 0);
    printf("31 kp10/pv250/task-coarse%g; postcross actualFORWARD775 ordinaryFFF retained and signedFORWARD/BACK652 remainder excluding yaw odometry,mid-yaw2s restores same leg/test/goal,399ms moves400ms retries;R1/shortoffset/43/44 lateral unchanged;only turn/contact timeout NEXT reached0;g/a/0/invalidIMU priority preserved (host only)\n", (double)ROUTE31_TASK_V_MMS);
    return 0;
}
