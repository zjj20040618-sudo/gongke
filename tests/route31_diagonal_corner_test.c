/* Real31 BLE/VAT/CRC/STEP runner plus namespaced real motion.c.
 * HOST encoder/IMU inputs prove sequencing and safety gates only; no traction,
 * swept-body clearance, diagonal scale or physical route acceptance implied. */
#define ROUTE44_CURRENT_FIXTURE_MAIN diagonal31_retained_route44_main
#include "route44_current_test.c"
#undef ROUTE44_CURRENT_FIXTURE_MAIN

static int diagonal31_near(float actual, float expected)
{ return fabsf(actual - expected) < 0.002f; }

/* Enter through the actual first LEFT565 and legal QR gate, then complete
 * BACK490. No direct stage jump or extra g authorizes the internal diagonal. */
static int diagonal31_begin(void)
{
    CHECK(wire_boot() == 0);
    host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
    run_cmd("yfix1"); run_cmd("31"); wire_sync();
    wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123");
    run_cmd("g");
    CHECK(r44_start_road() == 0 && s_seq_stage == 0u && !route31_diagonal_active());
    CHECK(r44_finish_road() == 0);
    if (s_seq_state == SQ_QR_WAIT) {
        wire_sync(); wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123");
    }
    CHECK(r44_start_road() == 0 && s_seq_stage == 1u && !s_route31_corner_diag);
    CHECK(s_msel == 16 && s_dist_target == -490.0f && !route31_diagonal_active());
    CHECK(r44_finish_road() == 0);
    CHECK(s_seq_stage == 1u && s_route31_corner_diag && s_seq_state == SQ_STILL);
    CHECK(strstr(host_messages, "same_stage=1 no_extra_g=1") != NULL);
    CHECK(r44_start_road() == 0);
    CHECK(s_seq_mode == 31u && s_seq_stage == 1u && s_msel == 16 &&
          s_round == R_RUN && route31_diagonal_active() && !turn_closed_loop_mode());
    CHECK(diagonal31_near(s_dist_target, -sqrtf(20000.0f)) && s_v == 50.0f &&
          s_dist_ff_ratio == 0.0f && s_dist_heading_kp > 0.0f);
    CHECK(!pulse_calls && !host_timer_active && !s_route31_rack_deployed);
    host_messages[0] = '\0';
    return 0;
}

/* Inverse p/q transformation. p is TOTAL signed translation including any
 * preserved pre-correction progress; q is offset from the rebased orth origin. */
static void diagonal31_position(float p, float q)
{
    float projected = s_dist_odo0 + p - s_dist_progress_mm;
    float orthogonal = s_dist_orth0 + q;
    host_fore = s_route31_diag_ux * projected - s_route31_diag_uy * orthogonal;
    host_lateral = s_route31_diag_uy * projected + s_route31_diag_ux * orthogonal;
}

static int diagonal31_stopped_error(const char *reason)
{
    CHECK(r44_service(1200u) == 0);
    CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 1u && r44_stopped() &&
          !route31_diagonal_active() && !s_route31_corner_diag);
    CHECK(strstr(host_messages, reason) != NULL);
    host_imu_valid = 1; host_yaw = host_absolute_yaw = 0.0f;
    host_fore = host_lateral = 0.0f;
    CHECK(r44_service(7000u) == 0 && s_seq_state == SQ_STOPPED && r44_stopped());
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && r44_stopped());
    return 0;
}

static int check_diagonal31_defaults_direction_speed_and_sequence(void)
{
    CHECK(ROUTE31_CORNER_BACK_MM == 490u && ROUTE31_CORNER_DIAG_BACK_MM == 100u &&
          ROUTE31_CORNER_DIAG_LEFT_MM == 100u && ROUTE31_CORNER_CROSS_MM == 520u &&
          ROUTE31_CORNER_DIAG_V_MMS == 50.0f && ROUTE31_CORNER_DIAG_ORTH_KP == 0.5f &&
          ROUTE31_CORNER_DIAG_ORTH_TOL_MM == 12.0f && ROUTE31_CORNER_DIAG_MAX_MS == 6000u);
    CHECK(diagonal31_begin() == 0);
    float heading = s_dist_heading0;
    unsigned zeros = (unsigned)zero_calls, test = s_active_test;
    for (unsigned tick = 0u; tick < 15u; ++tick) {
        CHECK(r44_service(20u) == 0);
        CHECK(last_x < 0.0f && last_y < 0.0f && last_w == 0.0f);
        CHECK(sqrtf(last_x * last_x + last_y * last_y) <= 50.001f &&
              diagonal31_near(last_x, last_y) && s_dist_heading0 == heading &&
              (unsigned)zero_calls == zeros && s_active_test == test);
    }
    /* Private q hold must work even when the global orth gain defaults to0. */
    for (int side = -1; side <= 1; side += 2) {
        diagonal31_position(-20.0f, (float)side * 8.0f);
        host_yaw = heading + (float)side * 0.2f;
        CHECK(r44_service(20u) == 0 && s_round == R_RUN && !s_dist_mid_pause);
        CHECK((-s_route31_diag_uy * last_x + s_route31_diag_ux * last_y) * side < 0.0f);
        CHECK(last_w * side < 0.0f && sqrtf(last_x * last_x + last_y * last_y) <= 50.001f);
    }
    host_yaw = heading;
    diagonal31_position(s_dist_target - 0.1f, 0.0f);
    CHECK(r44_service(20u) == 0 && s_round == R_BRAKE && s_dist_reason == 1u);
    CHECK(r44_service(1200u) == 0 && s_seq_stage == 2u && !s_route31_corner_diag &&
          s_msel == 20 && turn_target_deg() == 90.0f && !route31_diagonal_active());
    CHECK(strstr(host_messages, "DIAGONAL31 result=DONE") != NULL &&
          strstr(host_messages, "next_original_RIGHT90=1") != NULL);
    CHECK(r44_start_road() == 0 && r44_finish_road() == 0 && r44_start_road() == 0);
    CHECK(s_seq_stage == 3u && s_msel == 16 && s_dist_target == -520.0f &&
          route_seq_leg()->distance_mm == 520u && !route31_diagonal_active());
    run_cmd("0"); CHECK(r44_stopped());
    puts("31 actualLEFT565/QR/BACK490 -> same-stage BACK100+LEFT100 noextraG/no45turn -> originalRIGHT90 -> privateCROSS520;negativevxvy/yawhold/qhold/vectorcap50 passed");
    return 0;
}

static int check_diagonal31_endpoint_and_braking_coast(void)
{
    /* Reaching a body-axis distance alone is not a diagonal endpoint. */
    CHECK(diagonal31_begin() == 0);
    host_fore = -100.0f; host_lateral = 0.0f;
    CHECK(r44_service(20u) == 0 && s_round == R_RUN && s_seq_stage == 1u);
    CHECK(dist_travel_mm() > s_dist_target && !s_dist_end_saved);
    run_cmd("0"); CHECK(r44_stopped());
    /* One axis can fake the full p projection, but q must reject it. */
    CHECK(diagonal31_begin() == 0);
    host_fore = (s_dist_target - 0.1f) / s_route31_diag_ux; host_lateral = 0.0f;
    CHECK(r44_service(20u) == 0 && s_round == R_BRAKE && s_dist_reason == 6u);
    CHECK(diagonal31_stopped_error("DIAGONAL_ERROR") == 0);
    for (unsigned fault = 0u; fault < 4u; ++fault) {
        CHECK(diagonal31_begin() == 0);
        diagonal31_position(s_dist_target - 0.1f,
                            fault == 0u ? 12.5f : fault == 1u ? -12.5f : 0.0f);
        CHECK(r44_service(20u) == 0 && s_round == R_BRAKE);
        if (fault < 2u) CHECK(s_dist_reason == 6u);
        else {
            CHECK(s_dist_reason == 1u);
            /* Initially valid arrival; a measured coast invalidates it. */
            diagonal31_position(s_dist_target - (fault == 2u ? 13.0f : 0.1f),
                                fault == 3u ? 13.0f : 0.0f);
            ++host_counts[0];
        }
        CHECK(diagonal31_stopped_error("DIAGONAL_ERROR") == 0);
    }
    puts("31 diagonal body-axis-alone notDONE;fakep/qbothsigns rejected;postbrake along/orth overshoot rechecked beforeyaw and stopsDIAGONAL_ERROR passed");
    return 0;
}

static int check_diagonal31_timeout_sensor_failures_and_stops(void)
{
    CHECK(diagonal31_begin() == 0);
    CHECK(r44_service(5999u) == 0 && s_round == R_RUN && s_seq_state == SQ_RUN);
    CHECK(r44_service(1u) == 0 && s_round == R_BRAKE && s_dist_reason == 6u);
    CHECK(diagonal31_stopped_error("DIAGONAL_ERROR") == 0);
    for (unsigned pause = 0u; pause < 2u; ++pause) {
        CHECK(diagonal31_begin() == 0);
        diagonal31_position(-30.0f, 2.0f); host_yaw = s_dist_heading0 + 3.0f;
        CHECK(r44_service(20u) == 0 && s_round == R_BRAKE && s_dist_mid_pause == 1u);
        if (pause) CHECK(r44_service(T_DIST_STILL_MS + 20u) == 0 && s_round == R_ALIGN);
        /* A delayed host poll models a scheduling gap, not physical motion.
         * The whole-leg deadline must precede either pause's normal executor. */
        host_tick = s_meas_t0 + ROUTE31_CORNER_DIAG_MAX_MS;
        wire_poll();
        CHECK(s_round == R_BRAKE && s_dist_reason == 6u && !s_dist_mid_pause &&
              !last_x && !last_y && !last_w);
        CHECK(diagonal31_stopped_error("DIAGONAL_ERROR") == 0);
    }
    for (unsigned phase = 0u; phase < 2u; ++phase) {
        CHECK(diagonal31_begin() == 0);
        /*5990ms endpoint cannot buy extra settling time. The second case
         * enters real endpoint ALIGN earlier, then expires while holding yaw. */
        host_tick = s_meas_t0 + ROUTE31_CORNER_DIAG_MAX_MS -
                    (phase ? T_DIST_STILL_MS + 60u : 10u);
        diagonal31_position(s_dist_target - 0.1f, 0.0f); wire_poll();
        CHECK(s_round == R_BRAKE && s_dist_reason == 1u && !s_dist_mid_pause);
        if (phase) {
            CHECK(r44_service(T_DIST_STILL_MS + 20u) == 0 && s_round == R_ALIGN);
            CHECK(r44_service(20u) == 0 && s_dist_align_hold && !s_dist_mid_pause);
        }
        host_tick = s_meas_t0 + ROUTE31_CORNER_DIAG_MAX_MS; wire_poll();
        CHECK(s_round == R_BRAKE && s_dist_reason == 6u &&
              !last_x && !last_y && !last_w && !s_dist_align_enabled);
        CHECK(diagonal31_stopped_error("DIAGONAL_ERROR") == 0);
        CHECK(strstr(host_messages, "DIAGONAL31 result=DONE") == NULL);
    }
    CHECK(diagonal31_begin() == 0);
    host_tick = s_meas_t0 + ROUTE31_CORNER_DIAG_MAX_MS - 10u;
    diagonal31_position(s_dist_target - 0.1f, 0.0f); wire_poll();
    CHECK(s_round == R_BRAKE && s_dist_reason == 1u);
    /*Invoke the actual completion callback after a scheduler delay, without
     * running tick's deadline first. Its final success gate must also reject. */
    host_tick = s_meas_t0 + ROUTE31_CORNER_DIAG_MAX_MS + T_DIST_STILL_MS;
    dist_finish_report();
    CHECK(s_seq_state == SQ_STOPPED && s_dist_reason == 6u && r44_stopped());
    CHECK(diagonal31_stopped_error("DIAGONAL_ERROR") == 0 &&
          strstr(host_messages, "DIAGONAL31 result=DONE") == NULL);
    for (unsigned fault = 0u; fault < 4u; ++fault) {
        CHECK(diagonal31_begin() == 0);
        if (fault == 0u) host_imu_valid = 0;
        else if (fault == 1u) host_yaw = NAN;
        else if (fault == 2u) host_fore = NAN;
        else host_lateral = NAN;
        CHECK(r44_service(20u) == 0 &&
              (s_round == R_BRAKE || s_seq_state == SQ_STOPPED) &&
              !last_x && !last_y && !last_w && !laser_state && !host_timer_active);
        CHECK(diagonal31_stopped_error(fault < 2u ? "IMUERR" : "DIAGONAL_ERROR") == 0);
    }
    static const char *const keys[] = {"g", "a", "0"};
    for (unsigned phase = 0u; phase < 4u; ++phase)
        for (unsigned key = 0u; key < 3u; ++key) {
            CHECK(diagonal31_begin() == 0);
            if (phase == 1u || phase == 2u) {
                diagonal31_position(-30.0f, 2.0f);
                host_yaw = s_dist_heading0 + 3.0f;
                CHECK(r44_service(20u) == 0 && s_dist_mid_pause == 1u);
                if (phase == 2u) CHECK(r44_service(T_DIST_STILL_MS + 20u) == 0 &&
                                      s_round == R_ALIGN && s_dist_mid_pause == 2u);
            } else if (phase == 3u) {
                diagonal31_position(s_dist_target - 0.1f, 0.0f);
                CHECK(r44_service(20u) == 0 && s_round == R_BRAKE);
            }
            run_cmd(keys[key]);
            CHECK(s_seq_state == SQ_STOPPED && r44_stopped() &&
                  !route31_diagonal_active() && !s_route31_corner_diag);
            host_imu_valid = 1; host_yaw = host_absolute_yaw = 0.0f;
            CHECK(r44_service(7000u) == 0 && s_seq_state == SQ_STOPPED && r44_stopped());
            run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && r44_stopped());
        }
    puts("31 diagonal 5999/6000ms wholelegdeadline inRUN/midBRAKE/midALIGN/endBRAKE/endALIGN;5990endpoint and delayedfinish cannotDONE late;invalidIMU/nonfiniteyaw/fore/lat stop;g/a/0 atRUN,midBRAKE,midALIGN,endBRAKE neverresume passed");
    return 0;
}

static int check_diagonal31_mid_yaw_pq_restore_and_end_snapshot(void)
{
    for (int side = -1; side <= 1; side += 2) {
        CHECK(diagonal31_begin() == 0);
        float heading = s_dist_heading0, target = s_dist_target;
        unsigned zeros = (unsigned)zero_calls, test = s_active_test;
        diagonal31_position(-35.0f, 3.0f);
        host_yaw = heading + (float)side * 3.0f;
        CHECK(r44_service(20u) == 0 && s_dist_mid_pause == 1u && s_round == R_BRAKE);
        diagonal31_position(-40.0f, 4.0f); ++host_counts[0];
        CHECK(r44_service(T_DIST_STILL_MS + 20u) == 0 && s_round == R_ALIGN &&
              s_dist_mid_pause == 2u && diagonal31_near(s_dist_progress_mm, -40.0f));
        host_fore += 60.0f; host_lateral -= 40.0f;
        for (int motor = 0; motor < 4; ++motor) host_counts[motor] += motor & 1 ? -31 : 37;
        host_yaw = heading + (float)side * 0.2f;
        CHECK(r44_service(20u) == 0 && s_dist_align_hold && r44_stopped());
        CHECK(r44_service(399u) == 0 && s_round == R_ALIGN && s_dist_mid_pause == 2u);
        CHECK(r44_service(1u) == 0 && s_round == R_RUN && !s_dist_mid_pause);
        CHECK(diagonal31_near(dist_travel_mm(), -40.0f) &&
              diagonal31_near(dist_orth_odo() - s_dist_orth0, 4.0f));
        CHECK(s_seq_stage == 1u && route31_diagonal_active() && s_active_test == test &&
              s_dist_heading0 == heading && s_dist_target == target && (unsigned)zero_calls == zeros);
        CHECK(strstr(host_messages, "same_leg=1") && strstr(host_messages, "zeroed=0"));
        host_yaw = heading;
        diagonal31_position(target + 1.0f, 0.0f);
        CHECK(r44_service(20u) == 0 && s_round == R_RUN);
        diagonal31_position(target - 0.1f, 4.0f);
        CHECK(r44_service(20u) == 0 && s_round == R_BRAKE && s_dist_reason == 1u);
        CHECK(r44_service(T_DIST_STILL_MS) == 0 && s_round == R_ALIGN &&
              diagonal31_near(s_route31_diag_end_orth, 4.0f));
        /* Terminal yaw wheel travel must not rewrite the saved p/q report. */
        host_fore += 33.0f; host_lateral -= 25.0f;
        for (int motor = 0; motor < 4; ++motor) ++host_counts[motor];
        host_yaw = heading;
        CHECK(r44_service(20u) == 0 && s_dist_align_hold);
        CHECK(r44_service(400u) == 0 && s_seq_stage == 2u && !s_route31_corner_diag);
        CHECK(strstr(host_messages, "DIAGONAL31 result=DONE") && strstr(host_messages, "q_mm=4.0"));
        run_cmd("0"); CHECK(r44_stopped());
    }
    puts("31 diagonal midyaw bothsigns:brakingcoast retained,p/qrotationexcluded,remainingtarget/originalheading/samestage/test/nozero preserved;terminalreport qpreyaw snapshot passed");
    return 0;
}

static int check_diagonal31_other_owners_and_manual16_isolated(void)
{
    CHECK(wire_boot() == 0); run_cmd("43");
    CHECK(s_route43_tune.road_mm[1] == 610u && s_route43_tune.road_mm[3] == 620u &&
          !route31_diagonal_active() && !s_route31_corner_diag);
    run_cmd("g");
    CHECK(r44_start_road() == 0 && r44_finish_road() == 0);
    if (s_seq_state == SQ_QR_WAIT) {
        wire_sync(); wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123");
    }
    if (s_seq_state == SQ_STEP_WAIT) run_cmd("g"); /*43 does not use44 frozen snapshots. */
    CHECK(r44_start_road() == 0 && s_seq_stage == 1u && s_dist_target == -610.0f &&
          !route31_diagonal_active() && !s_route31_corner_diag);
    CHECK(r44_finish_road() == 0);
    if (s_seq_state == SQ_STEP_WAIT) run_cmd("g");
    CHECK(r44_start_road() == 0 && s_seq_stage == 2u &&
          s_msel == 20 && !route31_diagonal_active());
    run_cmd("0"); CHECK(r44_stopped());
    CHECK(r44_boot() == 0); run_cmd("g");
    CHECK(r44_start_road() == 0 && r44_finish_road() == 0 && r44_start_road() == 0 &&
          s_msel == 20 && !route31_diagonal_active());
    CHECK(r44_finish_road() == 0 && s_seq_state == SQ_QR_WAIT);
    wire_sync(); wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123");
    CHECK(r44_start_road() == 0 && s_route44_index == 2u && s_seq_stage == 1u &&
          s_dist_target == 600.0f && !route31_diagonal_active());
    run_cmd("0"); CHECK(r44_stopped());
    CHECK(diagonal31_begin() == 0); run_cmd("0");
    run_cmd("16"); run_cmd("v100"); run_cmd("d100"); run_cmd("g");
    CHECK(r44_service(20u) == 0 && s_seq_state == SQ_OFF && s_round == R_RUN &&
          !route31_diagonal_active() && !s_route31_corner_diag && s_dist_target == -100.0f);
    CHECK(dist_odo() == host_fore && dist_orth_odo() == host_lateral && last_x < 0.0f);
    run_cmd("0"); CHECK(r44_stopped());
    puts("31 diagonal state resets;43 retainedBACK610/CROSS620 andRIGHT90,44 fixedphysicalnode,manual16 ordinaryaxis do notinherit privatecorner passed");
    return 0;
}

int main(void)
{
    CHECK(check_diagonal31_defaults_direction_speed_and_sequence() == 0);
    CHECK(check_diagonal31_endpoint_and_braking_coast() == 0);
    CHECK(check_diagonal31_timeout_sensor_failures_and_stops() == 0);
    CHECK(check_diagonal31_mid_yaw_pq_restore_and_end_snapshot() == 0);
    CHECK(check_diagonal31_other_owners_and_manual16_isolated() == 0);
    puts("route31_diagonal_corner_test: host-only proof; no physical diagonal, corner clearance or IMU/encoder calibration acceptance");
    return 0;
}
