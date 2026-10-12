/* Actual BLE/CRC/rank/fire route and production ramp/motion implementation.
 * Camera, yaw and encoder inputs are injected: no physical clearance proof. */
#define ROUTE44_CURRENT_FIXTURE_MAIN restored_corner_retained_route44_main
#define HOST_REAL_MOTION_PROFILE 1
#define motion_vel_set_arc_pair restored_real_motion_vel_set_arc_pair
#include "route44_current_test.c"
#undef motion_vel_set_arc_pair
#undef ROUTE44_CURRENT_FIXTURE_MAIN

void motion_profile_get(MotionProfileTune *out) { r44_real_motion_profile_get(out); }
int motion_profile_set(const MotionProfileTune *in) { return r44_real_motion_profile_set(in); }
void motion_ramp_init(MotionRamp *r,float a,float d) { r44_real_motion_ramp_init(r,a,d); }
void motion_linear_ramp_init(MotionRamp *r) { r44_real_motion_linear_ramp_init(r); }
float motion_linear_profile_step(MotionRamp *r,float v,float remain,float dt)
{ return r44_real_motion_linear_profile_step(r,v,remain,dt); }
float motion_linear_ramp_step(MotionRamp *r,float v,float remain,float dt)
{ return r44_real_motion_linear_ramp_step(r,v,remain,dt); }

static uint16_t restored_distance(unsigned rank)
{
    const uint16_t values[] = {ROUTE31_TARGET_RANK1_MM,
        ROUTE31_TARGET_RANK2_MM,ROUTE31_TARGET_RANK3_MM};
    return values[rank - 1u];
}

static unsigned restored_explicit_v;

static int restored_approach(unsigned rank)
{
    CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
    run_cmd("yfix1"); run_cmd("31");
    if (restored_explicit_v) {
        char command[16]; snprintf(command,sizeof command,"v%u",restored_explicit_v); run_cmd(command);
    }
    run_cmd("g");
    s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
    /* Isolate the stopped target entry; subsequent rank/fire/road/turn and
     * hostage entry are the real dispatcher, not direct stage callbacks. */
    s_seq_stage = ROUTE31_TARGET_STAGE; route_seq_prepare(); wire_sync();
    CHECK(s_seq_state == SQ_TASK && s_target35_route && wire_task == 2u &&
          wire_digit == 2u && s_receiving && !s_ack && r44_stopped());
    wire_ack(wire_request,2u,0u); r44_rank(8u,(uint8_t)rank);
    for (unsigned n = 0u; n < 500u && !laser_state; ++n) {
        xy_obj_poll(wire_request,8,target35_point(),100,0);
        CHECK(r44_service(VAT_POLL_MS) == 0);
    }
    CHECK(laser_state && s_target35_phase == TA_FIRE);
    CHECK(r44_service(ROUTE31_LASER_MS) == 0 && !laser_state &&
          s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
          route_seq_leg()->distance_mm == restored_distance(rank) &&
          s_route31_target_order.rank == rank && !s_receiving);
    CHECK(r44_start_road() == 0 && s_msel == 15 && s_round == R_RUN &&
          s_dist_target == (float)restored_distance(rank) &&
          s_dist_heading_kp == 10.0f && s_dist_align_enabled);
    return 0;
}

static int restored_turn(unsigned rank)
{
    CHECK(restored_approach(rank) == 0 && r44_service(1200u) == 0);
    float origin = s_dist_odo0;
    host_fore = origin + s_dist_target - 1.0f;
    CHECK(r44_service(20u) == 0 && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
          s_round == R_RUN && last_x > 0.0f && last_x < s_v);
    int brakes = brake_calls;
    host_fore = origin + s_dist_target;
    CHECK(r44_service(20u) == 0 && s_round == R_BRAKE &&
          s_seq_stage == ROUTE31_TARGET_CORNER_STAGE && brake_calls > brakes && r44_stopped());
    for (unsigned n = 0u; n < 180u && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE; ++n) {
        host_yaw = s_dist_heading0;
        CHECK(r44_service(20u) == 0 && s_seq_state != SQ_STOPPED);
    }
    CHECK(s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE && !s_route31_hostage_corner_diag);
    CHECK(r44_start_road() == 0 && s_msel == 20 && turn_closed_loop_mode() &&
          turn_target_deg() == ROUTE31_HOSTAGE_RIGHT_TARGET_DEG && !route31_diagonal_active() && !s_route31_arc_running);
    return 0;
}

static int restored_diagonal(unsigned rank)
{
    CHECK(restored_turn(rank) == 0);
    uint16_t previous = wire_request; unsigned commands = wire_commands;
    host_absolute_yaw += turn_target_deg() - host_yaw; host_yaw = turn_target_deg();
    for (unsigned n = 0u; n < 180u && !s_route31_hostage_corner_diag; ++n)
        CHECK(r44_service(20u) == 0 && s_seq_state != SQ_STOPPED);
    CHECK(s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE && s_route31_hostage_corner_diag &&
          wire_request == previous && wire_commands == commands && r44_stopped());
    CHECK(r44_start_road() == 0 && s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE &&
          s_msel == 15 && route31_diagonal_active() && !turn_closed_loop_mode() &&
          s_dist_target == (float)ROUTE31_HOSTAGE_CORNER_DIAG_MM &&
          s_v == (s_route31_straight_v_set
              ? fminf(route31_straight_speed(),ROUTE31_HOSTAGE_CORNER_DIAG_V_MMS)
              : ROUTE31_HOSTAGE_CORNER_DIAG_V_MMS) &&
          s_dist_heading_kp > 0.0f && s_dist_ff_ratio == 0.0f && s_dist_align_enabled);
    CHECK(fabsf(s_route31_diag_ux - 0.70710678f) < 0.0001f &&
          fabsf(s_route31_diag_uy + 0.70710678f) < 0.0001f);
    return 0;
}

static void restored_position(float p,float q)
{
    float along = s_dist_odo0 + p - s_dist_progress_mm;
    float orth = s_dist_orth0 + q;
    host_fore = s_route31_diag_ux * along - s_route31_diag_uy * orth;
    host_lateral = s_route31_diag_uy * along + s_route31_diag_ux * orth;
}

static int check_rank_stopped_turn_diagonal_and_fresh_hostage(void)
{
    CHECK(!ROUTE31_ARC_ENABLE && ROUTE31_HOSTAGE_CORNER_DIAG_MM == 110u &&
          ROUTE31_HOSTAGE_CORNER_DIAG_V_MMS == 200.0f &&
          ROUTE31_HOSTAGE_CORNER_DIAG_ORTH_TOL_MM == 5.0f &&
          ROUTE31_HOSTAGE_CORNER_DIAG_ALONG_TOL_MM == 20.0f);
    CHECK(ROUTE31_TARGET_RANK1_MM > ROUTE31_TARGET_RANK2_MM &&
          ROUTE31_TARGET_RANK2_MM > ROUTE31_TARGET_RANK3_MM && ROUTE31_TARGET_RANK3_MM > 0u);
    for (unsigned rank = 1u; rank <= 3u; ++rank) {
        CHECK(restored_diagonal(rank) == 0 && r44_service(100u) == 0);
        CHECK(last_x > 0.0f && last_y < 0.0f && last_w == 0.0f &&
              fabsf(last_x + last_y) < 0.001f &&
              hypotf(last_x,last_y) <= ROUTE31_HOSTAGE_CORNER_DIAG_V_MMS + 0.001f &&
              !host_timer_active && !pulse_calls);
        /* Exercise actual IK/publication for the emitted front-left command. */
        float expected[4]; r44_real_motion_ik_precise(last_x,last_y,last_w,expected);
        r44_real_motion_vel_set_precise(last_x,last_y,last_w);
        for (unsigned motor = 0u; motor < 4u; ++motor)
            CHECK(fabsf(r44_motion_rpm[motor] - expected[motor]) < 0.0001f);
        restored_position(10.0f,2.0f); host_yaw = s_dist_heading0 + 0.2f;
        CHECK(r44_service(20u) == 0 && last_w < 0.0f &&
              (-s_route31_diag_uy * last_x + s_route31_diag_ux * last_y) < 0.0f);
        host_yaw = s_dist_heading0;
        restored_position((float)ROUTE31_HOSTAGE_CORNER_DIAG_MM - 0.1f,0.0f);
        CHECK(r44_service(20u) == 0 && s_round == R_RUN);
        restored_position((float)ROUTE31_HOSTAGE_CORNER_DIAG_MM,0.0f);
        float component = (float)ROUTE31_HOSTAGE_CORNER_DIAG_MM * 0.70710678f;
        CHECK(fabsf(host_fore - component) < 0.01f && fabsf(host_lateral + component) < 0.01f);
        uint16_t previous = wire_request; unsigned commands = wire_commands;
        CHECK(r44_service(20u) == 0 && s_round == R_BRAKE &&
              s_dist_reason == 1u && s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE && r44_stopped());
        host_messages[0] = '\0';
        for (unsigned n = 0u; n < 180u && s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE; ++n) {
            host_yaw = s_dist_heading0;
            CHECK(r44_service(20u) == 0 && s_seq_state != SQ_STOPPED);
        }
        CHECK(s_seq_stage == ROUTE31_HOSTAGE_STAGE && s_seq_state == SQ_TASK &&
              !s_route31_hostage_corner_diag && !route31_diagonal_active() &&
              !s_route31_arc_running && !host_timer_active && !pulse_calls &&
              s_receiving && !s_ack && wire_opcode == 0x63u && wire_task == 3u &&
              wire_digit == 3u && wire_request > previous && wire_commands == commands + 1u);
        wire_ack(previous,2u,0u);
        CHECK(r44_service(300u) == 0 && !s_ack && r44_stopped());
        wire_ack(wire_request,2u,0u);
        CHECK(r44_service(400u) == 0 && s_ack && last_x > 0.0f && !host_timer_active);
        run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && r44_stopped());
    }
    printf("31 realrank1/2/3:%u/%u/%u,endpoint decel/brake/yaw -> originalRIGHT%.2f -> same-stage forwardLEFT110 vector(77.78,-77.78)/default200/qhold/yawhold/brake/yaw -> fresh63task3ACK -> forwardsearch passed\n",
           (unsigned)ROUTE31_TARGET_RANK1_MM,(unsigned)ROUTE31_TARGET_RANK2_MM,
           (unsigned)ROUTE31_TARGET_RANK3_MM,(double)ROUTE31_HOSTAGE_RIGHT_TARGET_DEG);
    return 0;
}

static int check_corner_stop_and_geometry_guards(void)
{
    const char *const keys[] = {"g","a","0"};
    for (unsigned phase = 0u; phase < 2u; ++phase)
        for (unsigned key = 0u; key < 3u; ++key) {
            CHECK((phase ? restored_diagonal(2u) : restored_turn(2u)) == 0);
            unsigned commands = wire_commands;
            run_cmd(keys[key]);
            CHECK(s_seq_state == SQ_STOPPED && !s_route31_hostage_corner_diag &&
                  !route31_diagonal_active() && r44_stopped());
            host_yaw = host_absolute_yaw = 90.0f; host_fore = 1000.0f; host_lateral = -1000.0f;
            CHECK(r44_service(7000u) == 0 && s_seq_state == SQ_STOPPED &&
                  wire_commands == commands && !host_timer_active && !pulse_calls && r44_stopped());
        }
    CHECK(restored_diagonal(2u) == 0);
    /* A single body-axis length is not the total diagonal length. Forged p
     * with excessive q may reach brake but cannot authorize task3. */
    host_fore = (float)ROUTE31_HOSTAGE_CORNER_DIAG_MM; host_lateral = 0.0f;
    CHECK(r44_service(20u) == 0 && s_round == R_RUN && s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE);
    restored_position((float)ROUTE31_HOSTAGE_CORNER_DIAG_MM,
                      ROUTE31_HOSTAGE_CORNER_DIAG_ORTH_TOL_MM + 0.1f);
    unsigned commands = wire_commands;
    CHECK(r44_service(20u) == 0 && s_round == R_BRAKE && s_dist_reason == 6u);
    CHECK(r44_service(1200u) == 0 && s_seq_state == SQ_STOPPED &&
          !s_route31_hostage_corner_diag && r44_stopped() && wire_commands == commands);
    puts("31 turn/diag g/a/0 immediate stop+clearflag no delayed task3 orarm;bodyaxis110 notendpoint/q>5 stops passed");
    return 0;
}

static int check_corner_along_stop_tolerance(void)
{
    CHECK(restored_diagonal(2u) == 0);
    unsigned commands = wire_commands;
    /* Replay the log's +1.6mm at brake and +10.2mm after settling. The
     * private along-track stop allowance does not relax orthogonal q. */
    restored_position((float)ROUTE31_HOSTAGE_CORNER_DIAG_MM + 1.6f,0.0f);
    CHECK(r44_service(20u) == 0 && s_round == R_BRAKE && s_dist_reason == 1u);
    restored_position((float)ROUTE31_HOSTAGE_CORNER_DIAG_MM + 10.2f,0.0f);
    for (unsigned n = 0u; n < 180u && s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE; ++n) {
        host_yaw = s_dist_heading0;
        CHECK(r44_service(20u) == 0 && s_seq_state != SQ_STOPPED);
    }
    CHECK(s_seq_stage == ROUTE31_HOSTAGE_STAGE && s_seq_state == SQ_TASK &&
          wire_commands == commands + 1u && wire_opcode == 0x63u &&
          wire_task == 3u && !s_ack && r44_stopped());
    run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && r44_stopped());

    /* Excessive transverse error still stops even though the same 10.2mm
     * along overshoot is allowed. */
    CHECK(restored_diagonal(2u) == 0); commands = wire_commands;
    restored_position((float)ROUTE31_HOSTAGE_CORNER_DIAG_MM + 10.2f,
                      ROUTE31_HOSTAGE_CORNER_DIAG_ORTH_TOL_MM + 0.1f);
    CHECK(r44_service(20u) == 0 && s_round == R_BRAKE && s_dist_reason == 6u);
    CHECK(r44_service(1200u) == 0 && s_seq_state == SQ_STOPPED &&
          wire_commands == commands && r44_stopped());

    /* A final settled p beyond the named 20mm allowance remains an error. */
    CHECK(restored_diagonal(2u) == 0); commands = wire_commands;
    restored_position((float)ROUTE31_HOSTAGE_CORNER_DIAG_MM + 1.6f,0.0f);
    CHECK(r44_service(20u) == 0 && s_round == R_BRAKE && s_dist_reason == 1u);
    restored_position((float)ROUTE31_HOSTAGE_CORNER_DIAG_MM +
                      ROUTE31_HOSTAGE_CORNER_DIAG_ALONG_TOL_MM + 0.1f,0.0f);
    CHECK(r44_service(1200u) == 0 && s_seq_state == SQ_STOPPED &&
          wire_commands == commands && r44_stopped());
    puts("31 left-front110 log overshoot+10.2/q0 entersfresh hostage; q>5 and settledalong>20 stillstop passed");
    return 0;
}

static int check_other_modes_keep_original_turn(void)
{
    const unsigned modes[] = {43u,44u};
    for (unsigned i = 0u; i < 2u; ++i) {
        CHECK(wire_boot() == 0); char command[8];
        snprintf(command,sizeof command,"%u",modes[i]); run_cmd(command); run_cmd("g");
        s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
        s_seq_stage = ROUTE31_HOSTAGE_TURN_STAGE;
        if (modes[i] == 44u) {
            unsigned index = 0u;
            while (index < ROUTE44_STAGES && s_route44_plan[index].role != ROUTE31_HOSTAGE_TURN_STAGE) ++index;
            CHECK(index < ROUTE44_STAGES); s_route44_index = (uint8_t)index;
        }
        route_seq_prepare(); CHECK(sequence_start_stage() == 0 && s_seq_state == SQ_RUN && s_msel == 20);
        host_absolute_yaw += turn_target_deg() - host_yaw; host_yaw = turn_target_deg();
        for (unsigned n = 0u; n < 180u && s_seq_state == SQ_RUN; ++n)
            CHECK(r44_service(20u) == 0 && !s_route31_hostage_corner_diag && !route31_diagonal_active());
        CHECK(s_seq_stage == ROUTE31_HOSTAGE_STAGE && s_seq_state == SQ_STEP_WAIT && r44_stopped());
        run_cmd("0"); CHECK(r44_stopped());
    }
    puts("43/44 original turn reachesg-gated hostage stage withoutnew diagonal passed");
    return 0;
}

static int check_diagonal_speed_scope(void)
{
    CHECK(restored_approach(2u) == 0 && !s_route31_straight_v_set && s_v == 150.0f);
    run_cmd("0"); CHECK(r44_stopped());
    const unsigned values[] = {80u,300u};
    for (unsigned i = 0u; i < 2u; ++i) {
        restored_explicit_v = values[i];
        CHECK(restored_diagonal(2u) == 0 && s_route31_straight_v_set &&
              s_v == (i ? 200.0f : 80.0f));
        run_cmd("0"); CHECK(r44_stopped());
    }
    restored_explicit_v = 0u;
    CHECK(restored_diagonal(2u) == 0 && !s_route31_straight_v_set && s_v == 200.0f);
    run_cmd("0"); CHECK(r44_stopped());
    puts("31 ordinary default150 unchanged;private diagonal default200,explicitv80->80/v300->cap200 passed");
    return 0;
}

int main(void)
{
    CHECK(check_rank_stopped_turn_diagonal_and_fresh_hostage() == 0);
    CHECK(check_corner_stop_and_geometry_guards() == 0);
    CHECK(check_corner_along_stop_tolerance() == 0);
    CHECK(check_other_modes_keep_original_turn() == 0);
    CHECK(check_diagonal_speed_scope() == 0);
    puts("route31_restored_corner_test: host software replay only;physical110mm scale/turn/clearance unverified");
    return 0;
}
