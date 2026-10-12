/* Real BLE/CRC/rank/route dispatcher plus actual motion.c wheel publication.
 * Camera, yaw and odometry are HOST inputs, not a traction/clearance model. */
#define ROUTE44_CURRENT_FIXTURE_MAIN retained_arc_route44_main
#define HOST_REAL_MOTION_PROFILE 1
#define motion_vel_set_arc_pair arc_real_motion_vel_set_arc_pair
#include "route44_current_test.c"
#undef motion_vel_set_arc_pair
#undef ROUTE44_CURRENT_FIXTURE_MAIN

/* Use the production acceleration/deceleration planner, not the inherited
 * fixture's constant-speed stand-in: the final1mm check must expose braking. */
void motion_profile_get(MotionProfileTune *out) { r44_real_motion_profile_get(out); }
int motion_profile_set(const MotionProfileTune *in) { return r44_real_motion_profile_set(in); }
void motion_ramp_init(MotionRamp *r,float a,float d) { r44_real_motion_ramp_init(r,a,d); }
void motion_linear_ramp_init(MotionRamp *r) { r44_real_motion_linear_ramp_init(r); }
float motion_linear_profile_step(MotionRamp *r,float v,float remain,float dt)
{ return r44_real_motion_linear_profile_step(r,v,remain,dt); }
float motion_linear_ramp_step(MotionRamp *r,float v,float remain,float dt)
{ return r44_real_motion_linear_ramp_step(r,v,remain,dt); }

static uint16_t arc_rank_distance(unsigned rank)
{
    const uint16_t distances[] = {ROUTE31_TARGET_RANK1_MM,
        ROUTE31_TARGET_RANK2_MM,ROUTE31_TARGET_RANK3_MM};
    return distances[rank - 1u];
}

static int arc_enter_target(unsigned speed)
{
    char command[16]; const int32_t qr[3] = {1,2,3};
    CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
    run_cmd("31"); snprintf(command,sizeof command,"v%u",speed); run_cmd(command);
    CHECK(s_route31_straight_v_set && s_route31_straight_v == (float)speed);
    run_cmd("g"); memcpy(s_seq_qr,qr,sizeof qr);
    /* Isolate the established stopped target-task entry; everything from
     * this real63 request through rank/fire/approach/arc/task3 is production. */
    s_seq_stage = ROUTE31_TARGET_STAGE; route_seq_prepare(); wire_sync();
    CHECK(s_seq_state == SQ_TASK && s_target35_route && wire_task == 2u &&
          wire_digit == 2u && s_receiving && !s_ack && r44_stopped());
    wire_ack(wire_request,2u,0u);
    return 0;
}

static int arc_fire_and_rank(unsigned rank)
{
    r44_rank(8u,(uint8_t)rank);
    for (unsigned n = 0u; n < 500u && !laser_state; ++n) {
        xy_obj_poll(wire_request,8,target35_point(),100,0);
        CHECK(r44_service(VAT_POLL_MS) == 0);
    }
    CHECK(laser_state && s_target35_phase == TA_FIRE && host_laser_on_calls == 1u);
    CHECK(r44_service(ROUTE31_LASER_MS) == 0);
    CHECK(!laser_state && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
          route_seq_leg()->distance_mm == arc_rank_distance(rank) &&
          s_route31_target_order.rank == rank && s_route31_target_order.digit == 2u &&
          !s_receiving && !host_timer_active && !pulse_calls);
    return 0;
}

static int arc_start_approach(unsigned speed,unsigned rank)
{
    CHECK(arc_enter_target(speed) == 0 && arc_fire_and_rank(rank) == 0);
    CHECK(r44_start_road() == 0 && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
          s_msel == 15 && s_dist_target == (float)arc_rank_distance(rank) &&
          s_v == (float)speed && s_dist_heading_kp == 10.0f);
    CHECK(r44_service(1200u) == 0 && s_round == R_RUN && last_x == (float)speed);
    return 0;
}

static int arc_capture_pair(float left,float right)
{
    float conversion = 60.0f * M_GEAR_RATIO / (M_WHEEL_R_MM * 2.0f * 3.14159f);
    CHECK(fabsf(r44_motion_rpm[0] - left * conversion) < 0.0001f &&
          fabsf(r44_motion_rpm[3] - left * conversion) < 0.0001f &&
          fabsf(r44_motion_rpm[1] - right * conversion) < 0.0001f &&
          fabsf(r44_motion_rpm[2] - right * conversion) < 0.0001f);
    CHECK(r44_motion_rpm[0] > 0.0f && r44_motion_rpm[3] > 0.0f &&
          r44_motion_rpm[1] >= 0.0f && r44_motion_rpm[2] >= 0.0f &&
          !r44_motion_irq_mask);
    return 0;
}

static int check_arc_seamless_rank_speed_and_next_handshake(void)
{
    const unsigned speeds[] = {50u,150u,300u};
    for (unsigned rank = 1u; rank <= 3u; ++rank) {
        unsigned speed = speeds[rank - 1u];
        CHECK(arc_start_approach(speed,rank) == 0);
        int brakes = brake_calls, zeros = zero_calls;
        uint16_t previous = wire_request; unsigned commands = wire_commands;
        float origin = s_dist_odo0;
        host_yaw = host_absolute_yaw = 0.2f;
        host_fore = origin + (float)arc_rank_distance(rank) - 1.0f;
        CHECK(r44_service(20u) == 0 && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
              s_round == R_RUN && last_x == (float)speed && last_w < 0.0f &&
              brake_calls == brakes && zero_calls == zeros);
        host_yaw = host_absolute_yaw = 0.0f;
        host_fore = origin + (float)arc_rank_distance(rank);
        CHECK(r44_service(20u) == 0 && s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE &&
              s_seq_state == SQ_RUN && s_round == R_RUN && s_msel == 20 &&
              brake_calls == brakes && zero_calls == zeros &&
              wire_request == previous && wire_commands == commands);
        CHECK(s_route31_arc.phase == ARC_TRIAL_ARC &&
              s_route31_arc.config.before_mm == 0.0f && s_route31_arc.config.after_mm == 0.0f &&
              s_route31_arc.config.wheel_v_mms == (float)speed &&
              s_route31_arc.config.target_deg == 89.5f && s_route31_arc.yaw_goal == 89.5f);
        CHECK(arc_capture_pair((float)speed,(float)speed) == 0);
        CHECK(r44_service(20u) == 0 && s_route31_arc.right_mms < (float)speed &&
              s_route31_arc.right_mms > 0.0f && s_route31_arc.left_mms == (float)speed);
        CHECK(r44_service(1000u) == 0 && s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE &&
              s_route31_arc.right_mms == 0.0f && s_route31_arc.left_mms == (float)speed);
        CHECK(arc_capture_pair((float)speed,0.0f) == 0 && brake_calls == brakes && zero_calls == zeros);
        host_yaw = host_absolute_yaw = 80.0f;
        CHECK(r44_service(100u) == 0 && s_route31_arc.right_mms > 0.0f &&
              s_route31_arc.right_mms < (float)speed && s_route31_arc.yaw_goal == 89.5f);
        CHECK(arc_capture_pair(s_route31_arc.left_mms,s_route31_arc.right_mms) == 0);
        host_messages[0] = '\0'; /* Preserve the bounded fixture log for DONE. */
        host_yaw = host_absolute_yaw = 89.5f;
        for (unsigned n = 0u; n < 80u && !s_route31_arc_stable; ++n)
            CHECK(r44_service(20u) == 0);
        CHECK(s_route31_arc_stable && s_route31_arc_running &&
              s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE &&
              s_route31_arc.left_mms == (float)speed && s_route31_arc.right_mms == (float)speed &&
              brake_calls == brakes && zero_calls == zeros);
        CHECK(r44_service(ROUTE31_STABLE_MS - 1u) == 0 && s_route31_arc_running &&
              s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE && brake_calls == brakes &&
              zero_calls == zeros && arc_capture_pair((float)speed,(float)speed) == 0);
        CHECK(r44_service(1u) == 0);
        CHECK(s_seq_stage == ROUTE31_HOSTAGE_STAGE && s_seq_state == SQ_TASK &&
              !s_route31_arc_running && !arc_trial_active(&s_route31_arc) &&
              strstr(host_messages,"REC type=ARC31 status=DONE") != NULL &&
              s_receiving && !s_ack && wire_opcode == 0x63u && wire_task == 3u &&
              wire_digit == 3u && wire_request > previous && wire_commands == commands + 1u &&
              !host_timer_active && !pulse_calls && !laser_state);
        CHECK(zero_calls == zeros && !last_x && !last_y && !last_w);
        /* A real matching task3 ACK is still mandatory; old task2 does not
         * resume motion or authorize a mechanical action at the new stage. */
        wire_ack(previous,2u,0u); CHECK(r44_service(300u) == 0 && !s_ack && !host_timer_active);
        wire_ack(wire_request,2u,0u); CHECK(r44_service(400u) == 0 && s_ack && last_x > 0.0f);
        run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && r44_stopped());
    }
    puts("31 rank1/2/3 selected distances/V50/150/300:real ramp holdsV through last1mm and yaw hold;APPROACH->ARC no brake/zero;two equal left positive/right smoothly0 then restore;89.5 locked/rolling399 waits400 completes;DONE actual fresh63/task3 ACK gate passed");
    return 0;
}

static int check_arc_cancel_and_imu_priority(void)
{
    const char *const keys[] = {"g","a","0"};
    for (unsigned fault = 0u; fault < 5u; ++fault) {
        CHECK(arc_start_approach(150u,2u) == 0);
        host_fore = s_dist_odo0 + s_dist_target; CHECK(r44_service(100u) == 0);
        CHECK(s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE && arc_trial_active(&s_route31_arc));
        unsigned commands = wire_commands, starts = host_timer_start_calls;
        if (fault < 3u) run_cmd(keys[fault]);
        else if (fault == 3u) host_imu_valid = 0;
        else host_absolute_yaw = NAN;
        wire_poll();
        CHECK(s_seq_state == SQ_STOPPED && !arc_trial_active(&s_route31_arc) &&
              r44_stopped() && wire_commands == commands && host_timer_start_calls == starts);
        host_imu_valid = 1; host_absolute_yaw = host_yaw = 89.5f;
        CHECK(r44_service(1000u) == 0 && s_seq_state == SQ_STOPPED &&
              s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE && wire_commands == commands &&
              !host_timer_active && !pulse_calls && r44_stopped());
    }
    puts("31 ARC g/a/0/invalidIMU/nonfinite heading cancel before success;late89.5 cannot restart/complete/sendtask3/arm passed");
    return 0;
}

static int check_arc_other_modes_keep_stopped_turn(void)
{
    const unsigned modes[] = {43u,44u};
    for (unsigned i = 0u; i < 2u; ++i) {
        CHECK(wire_boot() == 0); char command[8];
        snprintf(command,sizeof command,"%u",modes[i]); run_cmd(command); run_cmd("g");
        s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
        s_seq_stage = ROUTE31_TARGET_CORNER_STAGE;
        if (modes[i] == 44u) {
            unsigned index = 0u;
            while (index < ROUTE44_STAGES && s_route44_plan[index].role != ROUTE31_TARGET_CORNER_STAGE)
                ++index;
            CHECK(index < ROUTE44_STAGES); s_route44_index = (uint8_t)index;
        }
        route_seq_prepare(); CHECK(sequence_start_stage() == 0 && s_seq_state == SQ_RUN && s_msel == 15);
        CHECK(!arc_trial_active(&s_route31_arc));
        host_fore = s_dist_odo0 + s_dist_target;
        for (unsigned n = 0u; n < 180u && s_seq_state == SQ_RUN; ++n) {
            host_yaw = s_dist_heading0; CHECK(r44_service(20u) == 0);
        }
        CHECK(s_seq_state == SQ_STEP_WAIT && r44_stopped() && !arc_trial_active(&s_route31_arc));
        run_cmd("g"); CHECK(sequence_start_stage() == 0 && s_seq_state == SQ_RUN &&
              s_msel == 20 && turn_closed_loop_mode() && !arc_trial_active(&s_route31_arc));
        run_cmd("0"); CHECK(r44_stopped());
    }
    puts("43/44 keep ordinary approach endpoint brake/yaw/g-gated in-place20 turn;no new ARC activation passed");
    return 0;
}

int main(void)
{
    CHECK(check_arc_seamless_rank_speed_and_next_handshake() == 0);
    CHECK(check_arc_cancel_and_imu_priority() == 0);
    CHECK(check_arc_other_modes_keep_stopped_turn() == 0);
    puts("route31_arc_transition_test:software commands/sequence only;physical turn radius/clearance/traction remain unverified");
    return 0;
}
