/* Real Bluetooth/distance/VAT state machines and real motion ramp arithmetic.
 * Only time, IMU, encoders and actuator plumbing are host replacements.
 * These checks prove software sequencing, not physical route accuracy. */
#define HOST_REAL_MOTION_PROFILE
#define main retained_mid_g_fixture_main
#include "g_command_stop_test.c"
#undef main

#define motion_init mid_real_motion_init
#define motion_ik mid_real_motion_ik
#define motion_vel_set mid_real_motion_vel_set
#define motion_ik_precise mid_real_motion_ik_precise
#define motion_vel_set_precise mid_real_motion_vel_set_precise
#define motion_vel_set_creep mid_real_motion_vel_set_creep
#define motion_brake mid_real_motion_brake
#define motion_pose mid_real_motion_pose
#define motion_odo_mm mid_real_motion_odo_mm
#define motion_lateral_odo_mm mid_real_motion_lateral_odo_mm
#define motion_pose_update mid_real_motion_pose_update
void mid_real_motion_brake(void);
#include "../App/motion.c"
#undef motion_init
#undef motion_ik
#undef motion_vel_set
#undef motion_ik_precise
#undef motion_vel_set_precise
#undef motion_vel_set_creep
#undef motion_brake
#undef motion_pose
#undef motion_odo_mm
#undef motion_lateral_odo_mm
#undef motion_pose_update

void ctrl_stop_all(void) { }
void ctrl_set_speed_precise(int motor, float rpm) { (void)motor; (void)rpm; }
void ctrl_set_speed_creep(int motor, float rpm) { (void)motor; (void)rpm; }

#undef CHECK
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "route31 mid-yaw line %d: %s\n", __LINE__, #expr); \
    return 1; \
} } while (0)

static int mid_near(float a, float b)
{ return fabsf(a - b) < 0.0001f; }
static int mid_stopped(void)
{ return !last_x && !last_y && !last_w; }
static void mid_elapsed(unsigned milliseconds)
{ host_tick += milliseconds; test_poll(); }

/* Set up a real distance executor at a selected route node. Other fixture
 * suites separately exercise the complete QR/arm/task chain leading here. */
static int mid_start(unsigned owner, unsigned stage, unsigned mode, float goal)
{
    char command[16];
    reset_fixture();
    snprintf(command, sizeof command, "%u", owner ? owner : mode);
    run_cmd(command);
    if (owner) {
        s_seq_stage = (uint8_t)stage; s_seq_state = SQ_RUN; s_seq_run = 1u;
        s_seq_prepared = 1u; s_msel = (int)mode;
        s_seq_qr[0] = s_seq_qr[1] = s_seq_qr[2] = 1;
        s_route31_hostage_rank = 1u;
    }
    s_d = 100.0f; s_v = 200.0f;
    host_absolute_yaw = host_yaw = goal;
    mode_start(); s_seq_prepared = 0u;
    CHECK(s_round == R_RUN && s_dist_heading0 == goal);
    CHECK(s_dist_target == ((mode == 16u || mode == 17u) ? -100.0f : 100.0f));
    CHECK(!s_dist_mid_pause && s_dist_progress_mm == 0.0f);
    mid_elapsed(20u);
    CHECK(s_round == R_RUN && !s_dist_mid_pause);
    return 0;
}

static int mid_brake_and_coast(int direction, int yaw_sign)
{
    unsigned stage = s_seq_stage, zeros = (unsigned)zero_calls;
    unsigned record = s_active_test;
    host_fore = (float)direction * 40.0f; host_lateral = 3.0f;
    for (int i = 0; i < 4; ++i) host_counts[i] = direction * 100;
    host_yaw = s_dist_heading0 + (float)yaw_sign * 2.0f;
    mid_elapsed(20u);
    CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && mid_stopped());
    CHECK(s_seq_state == SQ_RUN && s_seq_stage == stage && s_active_test == record);
    CHECK((unsigned)zero_calls == zeros && !s_dist_end_saved);
    /* Real braking coast belongs to this straight leg, not to the rotation. */
    host_fore = (float)direction * 47.0f; host_lateral = 4.0f;
    for (int i = 0; i < 4; ++i) host_counts[i] = direction * (117 + i);
    mid_elapsed(20u);
    mid_elapsed(T_DIST_STILL_MS - 1u);
    CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && mid_stopped());
    mid_elapsed(1u);
    CHECK(s_round == R_ALIGN && s_dist_mid_pause == 2u && mid_stopped());
    CHECK(mid_near(s_dist_progress_mm, (float)direction * 47.0f));
    CHECK(mid_near(s_dist_end_mm, (float)direction * 47.0f));
    for (int i = 0; i < 4; ++i)
        CHECK(s_dist_progress_counts[i] == direction * (117 + i));
    CHECK((unsigned)zero_calls == zeros && s_seq_stage == stage && s_active_test == record);
    CHECK(strstr(host_messages, "status=DONE") == NULL);
    mid_elapsed(20u);
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w * (float)yaw_sign < 0.0f);
    CHECK(fabsf(last_w) >= 0.30f && fabsf(last_w) <= 0.60f);
    return 0;
}

static int check_threshold_and_wrapping(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
        for (int side = -1; side <= 1; side += 2) {
            CHECK(mid_start(31u, direction < 0 ? 7u : 12u,
                            direction < 0 ? 16u : 15u, 0.0f) == 0);
            unsigned stage = s_seq_stage, zeros = (unsigned)zero_calls;
            host_fore = (float)direction * 40.0f;
            host_yaw = (float)side * 1.5f; mid_elapsed(20u);
            CHECK(s_round == R_RUN && !s_dist_mid_pause && s_seq_stage == stage);
            CHECK(last_x * (float)direction > 0.0f && (unsigned)zero_calls == zeros);
            host_yaw = (float)side * 1.501f; mid_elapsed(20u);
            CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && mid_stopped());
            CHECK((unsigned)zero_calls == zeros && s_seq_stage == stage);
        }
    CHECK(mid_start(31u, 7u, 16u, 179.0f) == 0);
    host_fore = -40.0f; host_yaw = -179.0f; mid_elapsed(20u);
    CHECK(s_dist_mid_pause == 1u && s_round == R_BRAKE);
    mid_elapsed(T_DIST_STILL_MS); mid_elapsed(20u);
    CHECK(s_round == R_ALIGN && last_w < 0.0f && mid_near(dist_yaw_error(host_yaw), -2.0f));
    puts("mid-yaw: strict +/-1.5 boundary, both signed directions and wrapped shortest error passed");
    return 0;
}

static int check_resume_distance_counts_and_ramp(void)
{
    for (int direction = -1; direction <= 1; direction += 2)
        for (int side = -1; side <= 1; side += 2) {
            CHECK(mid_start(31u, direction < 0 ? 7u : 12u,
                            direction < 0 ? 16u : 15u, 7.0f) == 0);
            unsigned stage = s_seq_stage, record = s_active_test, zeros = (unsigned)zero_calls;
            CHECK(mid_brake_and_coast(direction, side) == 0);
            float goal = s_dist_heading0, target = s_dist_target;
            /* Rotation can change odometry/counts in either direction. None
             * of it may consume the outstanding53mm of the straight leg. */
            host_fore += 60.0f; host_lateral += 40.0f;
            for (int i = 0; i < 4; ++i) host_counts[i] += i & 1 ? -31 : 37;
            host_yaw = goal + (float)side * 0.2f; mid_elapsed(20u);
            CHECK(s_round == R_ALIGN && s_dist_align_hold && mid_stopped());
            mid_elapsed(200u);
            host_counts[3]++; mid_elapsed(20u); /* Restarts stable-wheel hold. */
            int32_t resumed_base[4]; memcpy(resumed_base, host_counts, sizeof resumed_base);
            float resumed_fore = host_fore;
            mid_elapsed(399u);
            CHECK(s_round == R_ALIGN && s_dist_mid_pause == 2u && mid_stopped());
            mid_elapsed(1u);
            CHECK(s_round == R_RUN && !s_dist_mid_pause && !s_dist_end_saved && mid_stopped());
            CHECK(s_seq_state == SQ_RUN && s_seq_stage == stage && s_active_test == record);
            CHECK(s_dist_heading0 == goal && s_dist_target == target && (unsigned)zero_calls == zeros);
            CHECK(mid_near(dist_travel_mm(), (float)direction * 47.0f));
            CHECK(mid_near(s_dist_orth0, 40.0f) && mid_near(s_dist_odo0, resumed_fore));
            CHECK(s_dist_ramp.cur == 0.0f && !s_dist_align_timeout_accepted);
            CHECK(strstr(host_messages, "result=RESUME same_leg=1") &&
                  strstr(host_messages, "zeroed=0") && !strstr(host_messages, "status=DONE"));
            mid_elapsed(20u);
            CHECK(mid_near(last_x, (float)direction * 14.0f)); /* Real700*0.020 reacceleration. */
            CHECK(s_dist_heading0 == goal && (unsigned)zero_calls == zeros);
            host_fore = resumed_fore + (float)direction * 52.99f;
            for (int i = 0; i < 4; ++i) host_counts[i] = resumed_base[i] + direction * 53;
            mid_elapsed(20u); CHECK(s_round == R_RUN && s_seq_stage == stage);
            host_fore = resumed_fore + (float)direction * 53.0f;
            mid_elapsed(20u);
            CHECK(s_round == R_BRAKE && !s_dist_mid_pause && s_dist_reason == 1u);
            host_yaw = goal; mid_elapsed(T_DIST_STILL_MS); mid_elapsed(20u);
            CHECK(s_round == R_ALIGN && mid_stopped());
            mid_elapsed(400u);
            CHECK(s_seq_stage == stage + 1u && (unsigned)zero_calls == zeros + 1u);
            CHECK(mid_near(s_dist_end_mm, target));
            for (int i = 0; i < 4; ++i)
                CHECK(s_dist_end_counts[i] == direction * (170 + i));
            CHECK(strstr(host_messages, "status=DONE") != NULL);
            CHECK(!pulse_calls && !servo_calls && !laser_state);
        }
    puts("mid-yaw: coast included, correction odo/counts excluded, original signed goal/stage retained,400ms wheel hold and real ramp restart passed");
    return 0;
}

static int check_timeout_residual_is_not_fake_alignment(void)
{
    for (int side = -1; side <= 1; side += 2)
        for (unsigned accepted = 0u; accepted < 2u; ++accepted) {
            CHECK(mid_start(31u, 7u, 16u, 0.0f) == 0);
            CHECK(mid_brake_and_coast(-1, side) == 0);
            unsigned stage = s_seq_stage, zeros = (unsigned)zero_calls;
            host_yaw = (float)side * (accepted ? 1.5f : 1.501f);
            host_tick = s_dist_align_t0 + 1999u; test_poll();
            CHECK(s_round == R_ALIGN && s_dist_mid_pause == 2u && s_seq_stage == stage);
            ++host_tick; test_poll();
            CHECK(mid_stopped() && (unsigned)zero_calls == zeros && s_seq_stage == stage);
            if (accepted) {
                CHECK(s_seq_state == SQ_RUN && s_round == R_BRAKE && s_dist_mid_pause == 3u);
                /* Angular coast after a timed-out correction also must stop
                 * before the translation baseline is rebased/resumed. */
                host_fore += 60.0f; host_lateral += 40.0f;
                host_counts[0] += 37; host_counts[1] -= 31;
                mid_elapsed(20u); mid_elapsed(T_DIST_STILL_MS - 1u);
                CHECK(s_round == R_BRAKE && s_dist_mid_pause == 3u && mid_stopped());
                CHECK(!strstr(host_messages, "result=RESUME") && (unsigned)zero_calls == zeros);
                mid_elapsed(1u);
                CHECK(s_seq_state == SQ_RUN && s_round == R_RUN && !s_dist_mid_pause);
                CHECK(s_dist_heading0 == 0.0f && host_yaw == (float)side * 1.5f);
                CHECK(mid_near(dist_travel_mm(), -47.0f));
                CHECK(strstr(host_messages, "result=RESUME") && strstr(host_messages, "timeout_accept=1"));
                CHECK(!strstr(host_messages, "status=DONE"));
            } else {
                CHECK(s_seq_state == SQ_STOPPED && !s_dist_mid_pause && s_dist_reason == 3u);
                CHECK(!s_dist_align_timeout_accepted);
                CHECK(strstr(host_messages, "timeout_residual_gt1.5") &&
                      strstr(host_messages, "status=YAW_TIMEOUT") && !strstr(host_messages, "result=RESUME"));
            }
        }
    puts("mid-yaw:1999ms no resume;2000ms +/-1.5 residual accepted after wheel-still250 without zero,larger residual fails stopped without false success passed");
    return 0;
}

static int check_cancellation_and_fault_priority(void)
{
    const char *const keys[] = {"g", "a", "0"};
    for (unsigned phase = 0u; phase < 4u; ++phase)
        for (unsigned key = 0u; key < 3u; ++key) {
            CHECK(mid_start(31u, 7u, 16u, 0.0f) == 0);
            if (!phase) {
                host_fore = -40.0f; host_yaw = 2.0f; mid_elapsed(20u);
                CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u);
            } else {
                CHECK(mid_brake_and_coast(-1, 1) == 0);
                if (phase == 2u) { host_yaw = 0.1f; mid_elapsed(20u); CHECK(s_dist_align_hold); }
                if (phase == 3u) {
                    host_yaw = 1.0f; host_tick = s_dist_align_t0 + 2000u; test_poll();
                    CHECK(s_round == R_BRAKE && s_dist_mid_pause == 3u);
                }
            }
            unsigned zeros = (unsigned)zero_calls, stage = s_seq_stage;
            run_cmd(keys[key]);
            CHECK(s_seq_state == SQ_STOPPED && !s_dist_mid_pause && mid_stopped());
            unsigned dispatches = host_precise_calls;
            for (unsigned poll = 0u; poll < 130u; ++poll) {
                host_yaw = 0.0f; mid_elapsed(20u);
                CHECK(mid_stopped() && host_precise_calls == dispatches && s_seq_state == SQ_STOPPED);
            }
            CHECK((unsigned)zero_calls == zeros && s_seq_stage == stage && !pulse_calls && !servo_calls && !laser_state);
            CHECK(strstr(host_messages, "result=RESUME") == NULL);
        }
    for (unsigned cause = 0u; cause < 3u; ++cause) {
        CHECK(mid_start(31u, 7u, 16u, 0.0f) == 0);
        CHECK(mid_brake_and_coast(-1, 1) == 0);
        unsigned zeros = (unsigned)zero_calls;
        if (!cause) host_imu_valid = 0;
        else if (cause == 1u) host_yaw = NAN;
        else host_abort = 1;
        host_tick = s_dist_align_t0 + 2000u; test_poll();
        CHECK(s_seq_state == SQ_STOPPED && !s_dist_mid_pause && mid_stopped() && (unsigned)zero_calls == zeros);
        CHECK(strstr(host_messages, "result=RESUME") == NULL);
        CHECK(strstr(host_messages, cause == 2u ? "status=ABORT" : "status=IMUERR") != NULL);
    }
    puts("mid-yaw: g/a/0 cancel brake/correction/hold/post-timeout still;IMU/nonfinite/abort beat timeout and never resume passed");
    return 0;
}

static int check_owner_and_crossing_scope(void)
{
    /* R1 stays excluded even when a legal QR has already been seen; changing
     * the motion family cannot accidentally enable the stage0 gate. */
    CHECK(mid_start(31u, 0u, 15u, 0.0f) == 0);
    CHECK(!dist_mid_yaw_enabled() && dist_ordinary_leg());
    host_fore = 40.0f; host_yaw = 4.0f; mid_elapsed(20u);
    CHECK(s_round == R_RUN && !s_dist_mid_pause && last_x > 0.0f);
    CHECK(strstr(host_messages, "DIST_MID_YAW") == NULL);
    /* R2 is now the first eligible straight leg, after QR_VALID starts stage1. */
    for (int side = -1; side <= 1; side += 2) {
        CHECK(mid_start(31u, 1u, 16u, 0.0f) == 0);
        CHECK(dist_mid_yaw_enabled() && dist_ordinary_leg());
        host_fore = -40.0f; host_yaw = (float)side * 1.5f; mid_elapsed(20u);
        CHECK(s_round == R_RUN && !s_dist_mid_pause && last_x < 0.0f);
        CHECK(last_w * (float)side < 0.0f);
        host_yaw = (float)side * 1.501f; mid_elapsed(20u);
        CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && mid_stopped());
    }
    /* The post-contact road leg remains eligible, with the same strict limit. */
    for (int side = -1; side <= 1; side += 2) {
        CHECK(mid_start(31u, 5u, 16u, 0.0f) == 0);
        CHECK(dist_mid_yaw_enabled());
        host_fore = -40.0f; host_yaw = (float)side * 1.5f; mid_elapsed(20u);
        CHECK(s_round == R_RUN && !s_dist_mid_pause && last_x < 0.0f);
        host_yaw = (float)side * 1.501f; mid_elapsed(20u);
        CHECK(s_round == R_BRAKE && s_dist_mid_pause == 1u && mid_stopped());
    }
    const unsigned owners[] = {0u, 34u, 36u, 37u, 43u};
    for (unsigned i = 0u; i < sizeof owners / sizeof owners[0]; ++i) {
        CHECK(mid_start(owners[i], owners[i] == 37u ? 0u : 1u, 16u, 0.0f) == 0);
        host_fore = -40.0f; host_yaw = 4.0f; mid_elapsed(20u);
        CHECK(s_round == R_RUN && !s_dist_mid_pause && last_x < 0.0f);
        CHECK(strstr(host_messages, "DIST_MID_YAW") == NULL);
    }
    for (unsigned mode = 17u; mode <= 18u; ++mode) {
        CHECK(mid_start(31u, mode == 17u ? 0u : 6u, mode, 0.0f) == 0);
        host_lateral = mode == 17u ? -40.0f : 40.0f;
        host_yaw = 4.0f; mid_elapsed(20u);
        CHECK(s_round == R_RUN && !s_dist_mid_pause && fabsf(last_y) > 0.0f);
    }
    CHECK(mid_start(31u, 3u, 16u, 0.0f) == 0);
    CHECK(!dist_ordinary_leg() && s_route31_cross_heading_valid);
    host_fore = -40.0f; host_yaw = 4.0f; mid_elapsed(20u);
    CHECK(s_round == R_RUN && !s_dist_mid_pause && last_x < 0.0f && last_w == 0.0f);
    reset_fixture(); run_cmd("31"); run_cmd("g");
    CHECK(fixture_prepare_route31_board() == 0);
    CHECK(sequence_start_stage() == 0 && s_board_contact.active && s_msel == 15);
    host_yaw = 4.0f; mid_elapsed(20u);
    CHECK(s_round == R_RUN && !s_dist_mid_pause && last_x > 0.0f && last_w == 0.0f);
    CHECK(strstr(host_messages, "DIST_MID_YAW") == NULL);
    puts("mid-yaw:31 QR-accepted R2 and post-contact BACK190 strict threshold enabled; R1/standalone/34/36/37/43/lateral/cross/contact exclusions retained passed");
    return 0;
}

static int check_real_qr_wait_then_r2_pause_resume(void)
{
    reset_fixture(); run_cmd("31"); run_cmd("g");
    CHECK(sequence_start_stage() == 0);
    CHECK(s_seq_stage == 0u && s_seq_state == SQ_RUN && s_msel == 17 &&
          !dist_mid_yaw_enabled() && !proto_qr_get(NULL));
    host_lateral = -40.0f; host_yaw = 2.0f; mid_elapsed(20u);
    CHECK(s_round == R_RUN && !s_dist_mid_pause && last_y < 0.0f &&
          strstr(host_messages, "DIST_MID_YAW") == NULL);
    host_lateral = s_dist_odo0 + s_dist_target; host_yaw = 0.0f;
    mid_elapsed(20u); CHECK(s_round == R_BRAKE);
    mid_elapsed(T_DIST_STILL_MS); fixture_complete_distance_alignment();
    CHECK(s_seq_state == SQ_QR_WAIT && s_seq_stage == 0u && mid_stopped());
    CHECK(!dist_mid_yaw_enabled() && !s_seq_qr[0] && !s_seq_qr[1] && !s_seq_qr[2]);
    unsigned zeros = (unsigned)zero_calls, record = s_active_test;
    /* An ACK alone, failed link, or illegal tuple cannot start R2. */
    for (unsigned sample = 0u; sample < 3u; ++sample) {
        host_scene_status = sample == 0u ? 0 : sample == 1u ? 1 : -1;
        if (sample == 1u) host_qr_accept(4, 2, 3);
        host_yaw = 4.0f; mid_elapsed(1000u);
        CHECK(s_seq_state == SQ_QR_WAIT && s_seq_stage == 0u && mid_stopped() &&
              !s_dist_mid_pause && !dist_mid_yaw_enabled() && !proto_qr_get(NULL));
        CHECK((unsigned)zero_calls == zeros && s_active_test == record &&
              !pulse_calls && !servo_calls && !laser_state);
    }
    host_scene_status = 1; host_qr_accept(1, 2, 3); mid_elapsed(20u);
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == 1u && mid_stopped());
    CHECK(s_seq_qr[0] == 1 && s_seq_qr[1] == 2 && s_seq_qr[2] == 3 &&
          host_receive_closed && !proto_qr_get(NULL));
    CHECK(!dist_mid_yaw_enabled()); /* Preparing still is not moving. */
    CHECK(sequence_start_stage() == 0);
    CHECK(s_seq_state == SQ_RUN && s_seq_stage == 1u && s_msel == 16 &&
          dist_mid_yaw_enabled() && s_dist_target == -610.0f && s_dist_heading0 == 0.0f);
    /* Eligibility uses the accepted route stage, not the closed QR cache,
     * which later scene requests may also clear. Exercise actual R2's goal. */
    CHECK(!proto_qr_get(NULL));
    zeros = (unsigned)zero_calls; record = s_active_test;
    host_messages[0] = '\0'; /* Isolate R2 records from R1's valid DONE. */
    CHECK(mid_brake_and_coast(-1, 1) == 0);
    float original_goal = s_dist_target, original_heading = s_dist_heading0;
    host_fore += 60.0f; host_lateral += 40.0f;
    for (int i = 0; i < 4; ++i) host_counts[i] += i & 1 ? -31 : 37;
    host_yaw = 0.2f; mid_elapsed(20u);
    CHECK(s_round == R_ALIGN && s_dist_align_hold && mid_stopped());
    mid_elapsed(400u);
    CHECK(s_round == R_RUN && !s_dist_mid_pause && mid_stopped() &&
          s_seq_state == SQ_RUN && s_seq_stage == 1u && s_active_test == record);
    CHECK(s_dist_target == original_goal && s_dist_heading0 == original_heading &&
          (unsigned)zero_calls == zeros && mid_near(dist_travel_mm(), -47.0f));
    CHECK(strstr(host_messages, "result=RESUME same_leg=1") &&
          !strstr(host_messages, "timeout_accept=1"));
    mid_elapsed(20u);
    CHECK(mid_near(last_x, -14.0f) && mid_near(dist_travel_mm(), -47.0f) &&
          s_seq_stage == 1u && dist_mid_yaw_enabled() && !pulse_calls && !servo_calls);
    puts("mid-yaw: real R1 no pause; QR_WAIT missing/ACK-only/illegal/failed link stays stopped; legal QR closes receive then R2 pauses,excludes correction travel and resumes same610mm goal/heading with ramp14 without re-zero passed");
    return 0;
}

static int check_current_parameter_isolation(void)
{
    reset_fixture();
    CHECK(ROUTE31_ENTRY_BACK_MM == 805u && ROUTE31_EXIT_RIGHT_MM == 780u);
    CHECK(ROUTE31_RED_TO_CORNER_MM == 525u && ROUTE31_GREEN_TO_CORNER_MM == 445u && ROUTE31_BLUE_TO_CORNER_MM == 365u);
    CHECK(ROUTE31_RIGHT_TARGET_DEG == 90.0f && ROUTE31_HOSTAGE_RIGHT_TARGET_DEG == 93.0f &&
          ROUTE43_RIGHT_TARGET_DEG == 90.0f);
    CHECK(ROUTE31_LEFT_TARGET_DEG == -90.0f && ROUTE31_RETURN_TARGET_DEG == 185.0f);
    CHECK(mid_near(ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO, 0.045f));
    CHECK(mid_near(ROUTE43_TARGET_SEARCH_RIGHT_FF_RATIO, 0.065f));
    CHECK(s_route43_tune.road_mm[7] == 760u && s_route43_tune.corner_mm[0] == 520u &&
          s_route43_tune.corner_mm[1] == 420u && s_route43_tune.corner_mm[2] == 320u);
    CHECK(ROUTE31_BALL_TO_BUCKET_LEFT_MM == 15u && ROUTE31_RETURN_RIGHT_MM == 0u);
    CHECK(s_route31_ball_to_bucket_offset_leg.mode == 17u && s_route31_ball_to_bucket_offset_leg.distance_mm == 15u &&
          s_route31_return_right_offset_leg.mode == 18u && s_route31_return_right_offset_leg.distance_mm == 0u);
    CHECK(s_route43_tune.pair_left_mm == 40u && s_route43_tune.return_left_mm == 20u);
    s_seq_mode = 31u; s_seq_stage = ROUTE31_PAIR_STAGE; s_seq_pair_offset = 1u;
    s_route31_lateral_v = 250.0f; CHECK(route_seq_speed_mms() == 80.0f);
    s_route31_lateral_v = 40.0f; CHECK(route_seq_speed_mms() == 40.0f);
    s_seq_mode = 43u; s_route43_tune.lateral_v = 250.0f;
    CHECK(route_seq_speed_mms() == 250.0f);
    puts("mid-yaw: current31 ENTRY805/cross-right90/hostage-right93/colors525445365/targetFF4.5%,firstLEFT15cap80/no-return-offset and deferred43 isolation passed");
    return 0;
}

int main(void)
{
    CHECK(check_threshold_and_wrapping() == 0);
    CHECK(check_resume_distance_counts_and_ramp() == 0);
    CHECK(check_timeout_residual_is_not_fake_alignment() == 0);
    CHECK(check_cancellation_and_fault_priority() == 0);
    CHECK(check_owner_and_crossing_scope() == 0);
    CHECK(check_real_qr_wait_then_r2_pause_resume() == 0);
    CHECK(check_current_parameter_isolation() == 0);
    puts("route31 mid-yaw software regression PASS; no hardware/flash/push evidence");
    return 0;
}
