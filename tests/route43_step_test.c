/* Mode43 single-action g gate through the real Bluetooth/VAT/CRC parser.
 * Pixels, yaw and ISR events are host inputs, not physical acceptance. */
#define VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN route43_prior_xy_fixture_main
#include "vision_align_bluetooth_test.c"
#undef VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN

static VisionAlignTestStatus r43_status;

static int r43_stopped(void)
{
    return last_x == 0.0f && last_y == 0.0f && last_w == 0.0f &&
           !laser_state && !host_timer_active && !s_go;
}

static void r43_snapshot(void) { vision_align_test_status(&r43_status); }

static void r43_rank(uint16_t request, uint8_t model, uint8_t rank)
{
    uint8_t p[14] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8),
                     9u, 0u, 0x54u, 0u, 0u, model, rank, 0u, 255u, 255u, 255u};
    route_wire_le16(p + 6u, xy_seq++);
    if (rank >= 1u && rank <= 3u) {
        uint8_t first = model >= 3u ? 3u : 0u, next = 0u;
        p[10] = 3u; p[10u + rank] = model;
        for (uint8_t i = 0u; i < 3u; ++i) {
            if (i == rank - 1u) continue;
            while (first + next == model) ++next;
            p[11u + i] = first + next++;
        }
    }
    wire_feed(p, sizeof p, 0);
}

static int r43_service(unsigned ms)
{
    while (ms) {
        unsigned slice = ms < 20u ? ms : 20u;
        host_timer_sync_hal();
        uint64_t until = host_timer_us + (uint64_t)slice * 1000u;
        while (host_timer_active && host_timer_next_us <= until)
            CHECK(jog_host_timer_event() == 0);
        host_timer_us = until; host_tick = (uint32_t)(until / 1000u);
        wire_poll(); ms -= slice;
    }
    r43_snapshot();
    return 0;
}

static int r43_emit(unsigned count)
{
    int goal = pulse_calls + (int)count;
    /* Initial DIR hold is 2ms: at maximum PPS up to40 timer events may
     * precede the first STEP, including when testing only two pulses. */
    unsigned budget = 2u * count + T_JOG_MAX_PPS / 500u + 2u;
    CHECK(host_timer_active);
    while (pulse_calls < goal && budget--)
        CHECK(jog_host_timer_event() == 0);
    CHECK(pulse_calls == goal && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    return 0;
}

static int r43_boot(const char *qr)
{
    CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
    run_cmd("42"); run_cmd("e3"); run_cmd("h4"); run_cmd("u2200"); run_cmd("f777");
    run_cmd("27"); run_cmd("f333"); run_cmd("nr7");
    run_cmd("31"); run_cmd("v219"); run_cmd("pv321");
    run_cmd("43"); wire_sync();
    CHECK(s_seq_mode == 43u && s_msel == 43 && s_seq_state == SQ_READY && r43_stopped());
    CHECK(s_route43_tune.straight_v == 200.0f && s_route43_tune.lateral_v == 250.0f &&
          s_route43_tune.rack_steps == 3700u && s_route43_tune.rack_pps == 5000u);
    CHECK(s_route43_tune.road_mm[7] == 760u && s_route43_tune.pair_left_mm == 40u && s_route43_tune.return_left_mm == 20u &&
          s_route43_tune.corner_mm[0] == 520u && s_route43_tune.corner_mm[1] == 420u &&
          s_route43_tune.corner_mm[2] == 320u);
    CHECK(s_route43_tune.road_mm[4] == 0u && s_route31_plan[4].distance_mm == 0u &&
          s_route31_plan[4].speed_mms == 40.0f);
    CHECK(s_route43_tune.rank_mm[0] == 1415u && s_route43_tune.rank_mm[1] == 1315u &&
          s_route43_tune.rank_mm[2] == 1215u);
    CHECK(s_route31_straight_v == 219.0f && s_route31_lateral_v == 321.0f);
    CHECK(s_grab42_extend == 3u && s_grab42_down == 4u && s_grab42_u == 2200u &&
          s_grab42_pps == 777u && s_jog_pps == 333u);
    if (qr) {
        wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, qr);
        CHECK(proto_qr_get(NULL) && notice_calls == 1u && r43_stopped());
    }
    return 0;
}

/* WAIT_G owns the stopped vehicle; polling and late camera images must not
 * run the pending action. Public read-only reports are allowed. */
static int r43_wait(unsigned unit)
{
    if (s_seq_state != SQ_STEP_WAIT || s_route43_unit != unit)
        fprintf(stderr,"43 expectedWAIT unit=%u actualstate=%u unit=%u stage=%u pending=%u mode=%d round=%u msg=%s\n",
                unit,(unsigned)s_seq_state,(unsigned)s_route43_unit,(unsigned)s_seq_stage,
                (unsigned)s_route43_pending,s_msel,(unsigned)s_round,last_message);
    CHECK(s_seq_state == SQ_STEP_WAIT && s_seq_mode == 43u &&
          s_route43_unit == unit && route_seq_active() && r43_stopped());
    unsigned stage = s_seq_stage, pending = s_route43_pending;
    unsigned turn = s_seq_pair_turn, offset = s_seq_pair_offset, return_offset = s_seq_return_offset;
    int pulses = pulse_calls, servos = servo_calls;
    uint16_t held_u = host_servo, request = wire_request;
    uint32_t rack = s_route43_rack_issued;
    /* The producer keeps servicing current sessions while the route owner
     * waits for g; a valid ACK must not itself advance the pending action. */
    if (s_receiving) wire_ack(request, wire_mode, 0u);
    for (unsigned frame = 0u; frame < 5u; ++frame) {
        host_tick += 20u; xy_obj(request, 9, 140, VAT_BUCKET_Y_PX);
    }
    CHECK(r43_service(5000u) == 0);
    const char *const reports[] = {"?", "diag", "param", "route"};
    for (unsigned k = 0u; k < sizeof reports / sizeof reports[0]; ++k) run_cmd(reports[k]);
    CHECK(s_seq_state == SQ_STEP_WAIT && s_seq_stage == stage && s_route43_unit == unit &&
          s_route43_pending == pending && s_seq_pair_turn == turn && s_seq_pair_offset == offset &&
          s_seq_return_offset == return_offset);
    CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == held_u &&
          wire_request == request && s_route43_rack_issued == rack && r43_stopped());
    return 0;
}

static int r43_motion_begin(unsigned unit, unsigned mode, unsigned mm, float speed)
{
    CHECK(s_seq_state == SQ_READY || s_seq_state == SQ_STEP_WAIT);
    run_cmd("g");
    CHECK(s_route43_unit == unit && s_seq_state == SQ_STILL && s_msel == (int)mode &&
          route_seq_leg()->mode == mode && s_v == speed && r43_stopped());
    CHECK(sequence_start_stage() == 0 && s_seq_state == SQ_RUN);
    if (mode >= 15u && mode <= 18u) {
        if (s_seq_stage == 4u) {
            CHECK(mode == 15u && mm == 0u && speed == 40.0f && s_d == -1.0f &&
                  s_board_contact.active && s_board_contact.hits == 0u &&
                  s_dist_target == 0.0f && s_dist_heading_kp == 0.0f &&
                  s_dist_ff_ratio == 0.0f && !s_dist_align_enabled &&
                  last_x == 40.0f && last_y == 0.0f && last_w == 0.0f);
            return 0;
        }
        float target = (mode == 16u || mode == 17u) ? -(float)mm : (float)mm;
        CHECK(s_dist_target == target);
        if (mode == 15u || mode == 16u) CHECK(last_x * target > 0.0f);
        else CHECK(last_y * target > 0.0f);
    } else CHECK(last_x == 0.0f && last_y == 0.0f &&
                 turn_target_deg() == (mode == 30u ? -90.0f : mode == 22u ? 180.0f : 90.0f));
    return 0;
}

static int r43_motion_finish(unsigned next_unit)
{
    CHECK(s_seq_state == SQ_RUN);
    int heading_hold = route_seq_leg()->heading_hold;
    if (dist_mode()) {
        if (s_seq_stage == 4u) {
            CHECK(s_board_contact.active && !heading_hold);
            host_fore = s_dist_odo0 + 10000.0f; wire_poll(); /* Distance cannot claim contact. */
            CHECK(s_round == R_RUN && s_seq_state == SQ_RUN && !s_board_contact.hits);
            host_pitch = s_board_contact.pitch0 + ROUTE31_BOARD_CONTACT_TILT_DEG + 0.1f;
            /* A startup inclination cannot claim contact during the guard.
             * Once the guard expires, two frames are necessary but still not
             * sufficient: the same inclination must persist for 100ms. */
            host_tick = s_meas_t0 + ROUTE31_BOARD_CONTACT_START_GUARD_MS - 1u; wire_poll();
            CHECK(s_round == R_RUN && s_seq_state == SQ_RUN && !s_board_contact.hits);
            host_tick += 1u; wire_poll();
            CHECK(s_round == R_RUN && s_seq_state == SQ_RUN && s_board_contact.hits == 1u);
            wire_poll(); /* The same IMU sample cannot become the second hit. */
            CHECK(s_round == R_RUN && s_board_contact.hits == 1u);
            for (unsigned ms = 20u; ms < ROUTE31_BOARD_CONTACT_CONFIRM_MS; ms += 20u) {
                host_tick += 20u; wire_poll();
                CHECK(s_round == R_RUN && s_seq_state == SQ_RUN);
            }
            host_tick += 20u; wire_poll();
        } else {
            if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
            else host_fore = s_dist_odo0 + s_dist_target;
            host_yaw = s_dist_heading0 + 1.0f; wire_poll();
        }
        CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN);
        host_tick += T_DIST_STILL_MS; wire_poll();
        if (heading_hold) {
            CHECK(s_round == R_ALIGN && s_seq_state == SQ_RUN);
            host_tick += 20u; wire_poll(); CHECK(last_w < 0.0f && fabsf(last_w) >= 0.18f);
            host_yaw = s_dist_heading0 + 0.39f; wire_poll();
            CHECK(s_dist_align_hold && last_w == 0.0f && s_seq_state == SQ_RUN);
            host_tick += 699u; wire_poll(); CHECK(s_seq_state == SQ_RUN && s_round == R_ALIGN);
            host_tick += 1u; wire_poll();
        }
    } else {
        host_absolute_yaw += turn_target_deg();
        host_yaw = turn_target_deg(); wire_poll();
        CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && last_w == 0.0f);
        host_tick += T_TURN_SETTLE_MS; wire_poll();
    }
    /* R1 enters QR_WAIT at the distance executor's tail. The next owner
     * service may consume an already valid tuple, but never starts R2. */
    if (s_seq_state == SQ_QR_WAIT && next_unit != 999u) wire_poll();
    if (next_unit == 999u) CHECK(s_seq_state == SQ_QR_WAIT && r43_stopped());
    else if (next_unit == 0u) CHECK(s_seq_state == SQ_DONE && r43_stopped());
    else CHECK(r43_wait(next_unit) == 0);
    return 0;
}

static int r43_prefix(void)
{
    const unsigned mode[] = {17u,16u,20u,16u,15u,16u,18u,16u};
    const unsigned mm[] = {535u,630u,0u,650u,0u,190u,800u,760u};
    const float speed[] = {250.0f,200.0f,100.0f,300.0f,40.0f,200.0f,250.0f,200.0f};
    for (unsigned k = 0u; k < 8u; ++k) {
        CHECK(r43_motion_begin(k + 1u, mode[k], mm[k], speed[k]) == 0);
        CHECK(r43_motion_finish(k + 2u) == 0);
    }
    CHECK(s_seq_stage == ROUTE31_PREDEPLOY_STAGE && !s_route31_rack_deployed &&
          !s_route43_rack_issued && s_route31_lift_phase == R31_LIFT_OFF && !pulse_calls);
    return 0;
}

static int r43_predeploy(unsigned count, unsigned pps)
{
    CHECK(s_seq_state == SQ_STEP_WAIT && s_route43_unit == 9u);
    run_cmd("g");
    CHECK(s_seq_state == SQ_ARM_PREP && s_route31_lift_phase == R31_RACK_EXTEND &&
          s_route43_unit == 9u && host_jog_axis == 0 && host_jog_dir == 0 &&
          host_timer_pps == pps && s_jog_clock.remaining == count && !servo_calls);
    CHECK(s_route43_rack_issued == count && s_route43_rack_pps_issued == pps);
    CHECK(r43_emit(count - 1u) == 0); wire_poll();
    CHECK(host_timer_active && !s_route31_rack_deployed && s_seq_state == SQ_ARM_PREP);
    CHECK(r43_emit(1u) == 0); wire_poll();
    CHECK(s_route31_lift_phase == R31_RACK_EXTEND_WAIT && !s_route31_rack_deployed);
    CHECK(r43_service(249u) == 0 && s_seq_state == SQ_ARM_PREP);
    CHECK(r43_service(1u) == 0 && s_route31_rack_deployed && s_route31_lift_phase == R31_LIFT_OFF);
    CHECK(r43_wait(10u) == 0 && s_seq_stage == ROUTE31_PREDEPLOY_STAGE && !servo_calls);
    return 0;
}

static int r43_aim(int model, int x, int y, VisionAlignTestState done)
{
    CHECK(s_seq_state == SQ_TASK && s_seq_mode == 43u);
    wire_ack(wire_request, 2u, 0u); CHECK(r43_service(300u) == 0);
    if (model <= 2) r43_rank(wire_request, (uint8_t)model, 2u);
    for (unsigned frame = 0u; frame < 120u; ++frame) {
        host_tick += 20u; xy_obj(wire_request, model, x, y); r43_snapshot();
        if (r43_status.state == done || s_seq_state != SQ_TASK) break;
    }
    r43_snapshot();
    if (r43_status.x_goal != x || r43_status.state != done)
        fprintf(stderr,"43 aim want=%d state=%u actualgoal=%d actualstate=%u mech=%u seq=%u unit=%u model=%d msg=%s\n",
                x,(unsigned)done,r43_status.x_goal,(unsigned)r43_status.state,
                (unsigned)s_route31_lift_phase,(unsigned)s_seq_state,
                (unsigned)s_route43_unit,model,r43_status.reason);
    CHECK(r43_status.x_goal == x && r43_status.state == done);
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !laser_state);
    return 0;
}

static int r43_ball(void)
{
    CHECK(r43_wait(11u) == 0);
    run_cmd("g"); wire_sync();
    CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_PAIR_STAGE && s_route43_unit == 11u);
    CHECK(wire_task == 1u && wire_digit == 1u && !host_timer_active);
    CHECK(r43_aim(4, 150, VAT_BALL_Y_PX, VAT_WAIT_BALL_ACTION) == 0);
    CHECK(s_route31_lift_phase == R31_LIFT_BALL_DOWN && host_timer_pps == 10000u &&
          host_jog_axis == 1 && host_jog_dir == 1 && s_jog_clock.remaining == 32000u);
    CHECK(r43_emit(32000u) == 0); wire_poll();
    CHECK(host_servo == 1900u && servo_calls == 1 && s_route31_lift_phase == R31_LIFT_BALL_WAIT);
    CHECK(r43_service(1999u) == 0 && s_seq_state == SQ_TASK && !host_timer_active);
    CHECK(r43_service(1u) == 0 && host_timer_active && host_jog_dir == 0 &&
          s_route31_lift_phase == R31_LIFT_BALL_UP && host_timer_pps == 10000u);
    CHECK(r43_emit(32000u) == 0); wire_poll();
    CHECK(r43_wait(12u) == 0 && s_seq_pair_turn && !s_seq_pair_offset &&
          s_route31_lift_phase == R31_LIFT_OFF && host_servo == 1900u);
    return 0;
}

static int r43_to_bucket_wait(void)
{
    CHECK(r43_boot("123") == 0 && r43_prefix() == 0 && r43_predeploy(3700u,5000u) == 0);
    CHECK(r43_motion_begin(10u,30u,0u,100.0f) == 0 && r43_motion_finish(11u) == 0);
    CHECK(r43_ball() == 0);
    uint16_t before = wire_request;
    CHECK(r43_motion_begin(12u,22u,0u,100.0f) == 0 && r43_motion_finish(13u) == 0);
    CHECK(s_seq_pair_offset && !s_seq_pair_turn && wire_request == before);
    CHECK(r43_motion_begin(13u,17u,40u,250.0f) == 0 && r43_motion_finish(14u) == 0);
    r43_snapshot();
    CHECK(!s_seq_pair_offset && !s_seq_pair_turn && s_route43_pending == R43_PENDING_BUCKET &&
          wire_request == before && r43_status.state == VAT_TURN_ACTIVE && host_servo == 1900u);
    return 0;
}

static int r43_bucket_action_complete(unsigned count, unsigned pps)
{
    CHECK(host_timer_active && host_jog_axis == 1 && host_jog_dir == 1 && host_timer_pps == 10000u);
    CHECK(r43_emit(21999u) == 0); wire_poll(); CHECK(host_servo == 1900u && servo_calls == 1);
    CHECK(r43_emit(1u) == 0); wire_poll();
    CHECK(host_servo == 1150u && servo_calls == 2 && s_route31_lift_phase == R31_CLAW_RELEASE_WAIT);
    uint32_t release = s_route31_lift_wait_t0;
    CHECK(r43_service(999u) == 0 && host_servo == 1150u && !host_timer_active);
    CHECK(r43_service(1u) == 0 && host_servo == 1500u && servo_calls == 3 &&
          s_route31_lift_phase == R31_CLAW_RESTAGED_WAIT && s_route31_lift_wait_t0 == release);
    CHECK(r43_service(999u) == 0 && !host_timer_active);
    CHECK(r43_service(1u) == 0 && s_route31_lift_phase == R31_LIFT_BUCKET_UP &&
          host_timer_active && host_jog_dir == 0 && host_timer_pps == 10000u);
    CHECK(r43_emit(22000u) == 0); wire_poll();
    CHECK(s_route31_lift_phase == R31_RACK_RETRACT && host_timer_active &&
          host_timer_pps == pps && host_jog_axis == 0 && host_jog_dir == 1 &&
          s_jog_clock.remaining == count); /* No extra post-lift 2s pause. */
    CHECK(r43_emit(count) == 0); wire_poll();
    CHECK(r43_service(249u) == 0 && s_seq_state == SQ_TASK && s_route31_rack_deployed);
    CHECK(r43_service(1u) == 0 && !s_route31_rack_deployed && s_route31_lift_phase == R31_LIFT_OFF);
    CHECK(r43_wait(15u) == 0 && s_seq_stage == ROUTE31_RETURN180_STAGE && host_servo == 1500u);
    return 0;
}

static int r43_bucket(unsigned count, unsigned pps)
{
    CHECK(r43_wait(14u) == 0); uint16_t old_request = wire_request;
    run_cmd("g"); wire_sync();
    CHECK(s_seq_state == SQ_TASK && s_route43_unit == 14u && wire_request > old_request &&
          wire_task == 4u && wire_digit == 0u && host_servo == 1900u && !host_timer_active);
    for (unsigned frame = 0u; frame < 5u; ++frame) {
        host_tick += 20u; xy_obj(old_request, 9, 140, VAT_BUCKET_Y_PX);
    }
    CHECK(!host_timer_active && host_servo == 1900u && servo_calls == 1);
    CHECK(r43_aim(9,140,VAT_BUCKET_Y_PX,VAT_WAIT_BUCKET_ACTION) == 0);
    CHECK(r43_bucket_action_complete(count,pps) == 0);
    return 0;
}

static int r43_target(void)
{
    CHECK(r43_wait(17u) == 0); run_cmd("g"); wire_sync();
    CHECK(s_seq_state == SQ_TASK && s_route43_unit == 17u && s_target35_route &&
          wire_task == 2u && wire_digit == 2u && target35_point() == 255 &&
          target35_low() == 252 && target35_high() == 258);
    wire_ack(wire_request,2u,0u); CHECK(r43_service(1100u) == 0);
    CHECK(last_x == s_route43_tune.straight_v && last_y == last_x * 0.03f);
    host_tick += 20u; xy_obj(wire_request,8,259,10);
    CHECK(last_x == 30.0f && !s_target35_good && !laser_state);
    host_tick += 20u; xy_obj(wire_request,8,251,10);
    CHECK(last_x == -30.0f && !s_target35_good && !laser_state);
    for (unsigned frame = 0u; frame < 80u && !laser_state; ++frame) {
        host_tick += 20u; xy_obj(wire_request,8,frame < 5u ? (frame & 1u ? 252 : 258) : 255,10);
    }
    CHECK(laser_state && s_target35_phase == TA_FIRE && s_seq_state == SQ_TASK);
    CHECK(r43_service(1980u) == 0 && laser_state && s_seq_state == SQ_TASK);
    CHECK(r43_service(20u) == 0 && !laser_state && r43_wait(18u) == 0);
    return 0;
}

static int r43_hostage_retract_after_grip(void)
{
    CHECK(s_route31_lift_phase == R31_HOSTAGE_GRIP_WAIT && host_servo == 1900u &&
          servo_calls == 5 && !host_timer_active && !s_route31_hostage_hold);
    r43_snapshot(); CHECK(r43_status.state == VAT_WAIT_HOSTAGE_ACTION && s_receiving);
    CHECK(r43_service(249u) == 0 && s_route31_lift_phase == R31_HOSTAGE_GRIP_WAIT &&
          !host_timer_active && r43_status.state == VAT_WAIT_HOSTAGE_ACTION);
    CHECK(r43_service(1u) == 0 && s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT &&
          host_timer_active && host_jog_axis == 0 && host_jog_dir == 1 &&
          host_timer_pps == s_route43_rack_pps_issued && s_jog_clock.remaining == 3000u &&
          r43_status.state == VAT_WAIT_HOSTAGE_ACTION && !s_route31_hostage_hold);
    CHECK(r43_emit(2999u) == 0); wire_poll(); r43_snapshot();
    CHECK(host_timer_active && host_servo == 1900u && servo_calls == 5 &&
          r43_status.state == VAT_WAIT_HOSTAGE_ACTION && !s_route31_hostage_hold);
    CHECK(r43_emit(1u) == 0); wire_poll();
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT_WAIT && !host_timer_active);
    CHECK(r43_service(249u) == 0 && r43_status.state == VAT_WAIT_HOSTAGE_ACTION &&
          s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT_WAIT && !s_route31_hostage_hold);
    CHECK(r43_service(1u) == 0 && s_route31_lift_phase == R31_LIFT_OFF &&
          host_servo == 1900u && servo_calls == 5 && !host_timer_active && s_route31_rack_deployed);
    return 0;
}

static int r43_hostage(void)
{
    CHECK(r43_wait(20u) == 0); run_cmd("g"); wire_sync();
    CHECK(s_seq_state == SQ_TASK && s_route43_unit == 20u && wire_task == 3u && wire_digit == 3u);
    CHECK(r43_aim(0,230,VAT_HOSTAGE_Y_PX,VAT_WAIT_HOSTAGE_ACTION) == 0);
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND && !s_route31_hostage_hold &&
          host_servo == 1150u && servo_calls == 4 && host_timer_active &&
          host_jog_axis == 0 && host_jog_dir == 0 &&
          host_timer_pps == s_route43_rack_pps_issued &&
          s_jog_clock.remaining == s_route43_rack_issued);
    CHECK(r43_emit(s_route43_rack_issued - 1u) == 0); wire_poll();
    CHECK(host_timer_active && host_servo == 1150u && servo_calls == 4);
    CHECK(r43_emit(1u) == 0); wire_poll();
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND_WAIT && !host_timer_active);
    CHECK(r43_service(249u) == 0 && host_servo == 1150u && servo_calls == 4);
    CHECK(r43_service(1u) == 0 && host_servo == 1900u && servo_calls == 5 &&
          s_route31_lift_phase == R31_HOSTAGE_GRIP_WAIT && !s_route31_hostage_hold && !host_timer_active);
    CHECK(r43_hostage_retract_after_grip() == 0 && s_route31_hostage_hold && s_route31_hostage_rank == 2u);
    r43_snapshot(); CHECK(r43_status.state == VAT_DONE && r43_status.target_rank == 2u);
    wire_poll(); CHECK(r43_wait(21u) == 0 && host_servo == 1900u); /* No observation hold. */
    return 0;
}

static int check_r43_full_twenty_one_actions(void)
{
    CHECK(T_MODE_MAX == 43 && ROUTE_STEP_MODE == 43);
    CHECK(r43_to_bucket_wait() == 0 && r43_bucket(3700u,5000u) == 0);
    CHECK(r43_motion_begin(15u,22u,0u,100.0f) == 0 && r43_motion_finish(16u) == 0);
    CHECK(s_seq_return_offset && route_seq_leg()->mode == 17u);
    CHECK(r43_motion_begin(16u,17u,20u,250.0f) == 0 && r43_motion_finish(17u) == 0);
    CHECK(!s_seq_return_offset);
    CHECK(r43_target() == 0);
    CHECK(r43_motion_begin(18u,15u,420u,200.0f) == 0 && r43_motion_finish(19u) == 0);
    CHECK(host_servo == 1500u && servo_calls == 3);
    CHECK(r43_motion_begin(19u,20u,0u,100.0f) == 0 && r43_motion_finish(20u) == 0);
    CHECK(host_servo == 1500u && servo_calls == 3 && r43_hostage() == 0);
    CHECK(r43_motion_begin(21u,15u,1315u,200.0f) == 0 && r43_motion_finish(0u) == 0);
    CHECK(pulse_calls == 122100 && servo_calls == 5 && host_servo == 1900u &&
          host_jog_pulses[0][0] == 7400u && host_jog_pulses[0][1] == 6700u &&
          host_jog_pulses[1][0] == 54000u && host_jog_pulses[1][1] == 54000u);
    unsigned commands = wire_commands; run_cmd("g"); CHECK(r43_service(10000u) == 0);
    CHECK(s_seq_state == SQ_DONE && wire_commands == commands && r43_stopped());
    CHECK(s_route31_straight_v == 219.0f && s_route31_lateral_v == 321.0f &&
          s_grab42_extend == 3u && s_grab42_u == 2200u && s_jog_pps == 333u);
    puts("43:21 separately gated actions;road/turn yaw-held DONE before WAIT_G;fresh ball150/bucket140/hostage230;122100 STEP/hostage retract3000/racknet700/fiveservo/grip1900 through endpoint;no periodic auto-next passed");
    return 0;
}

static int check_r43_late_qr_and_distance_slot(void)
{
    CHECK(r43_boot(NULL) == 0); wire_ack(wire_request,1u,0u);
    CHECK(r43_motion_begin(1u,17u,535u,250.0f) == 0 && r43_motion_finish(999u) == 0);
    CHECK(s_seq_stage == 0u && s_route43_unit == 1u && !s_seq_pair_turn && r43_stopped());
    uint16_t request = wire_request;
    CHECK(r43_service(60000u) == 0 && s_seq_state == SQ_QR_WAIT && r43_stopped());
    xy_qr((uint16_t)(request-1u),"123"); CHECK(s_seq_state == SQ_QR_WAIT && r43_stopped());
    xy_qr(request,"123");
    CHECK(r43_wait(2u) == 0 && s_seq_stage == 1u && !s_receiving && !proto_qr_get(NULL));
    run_cmd("d777");
    CHECK(s_route43_tune.road_mm[1] == 777u && s_route43_tune.road_mm[0] == 535u &&
          s_route31_plan[1].distance_mm == 630u && r43_wait(2u) == 0);
    CHECK(r43_motion_begin(2u,16u,777u,200.0f) == 0);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && r43_stopped());
    CHECK(r43_service(5000u) == 0); run_cmd("g");
    CHECK(s_seq_state == SQ_STOPPED && r43_stopped());
    puts("43 QR:R1 headingDONE -> QR_WAIT;60000ms/oldQR inert,late validtuple -> WAIT_G2 not R2;d777 edits only next backward slot without starting passed");
    return 0;
}

static int r43_bad_write(const char *command)
{
    RouteStepTune before = s_route43_tune;
    unsigned stage = s_seq_stage, state = s_seq_state, unit = s_route43_unit;
    int pulses = pulse_calls, servos = servo_calls;
    uint16_t u = host_servo;
    host_messages[0] = '\0'; run_cmd(command);
    CHECK(memcmp(&before,&s_route43_tune,sizeof before) == 0 && s_seq_stage == stage &&
          s_seq_state == state && s_route43_unit == unit && pulse_calls == pulses &&
          servo_calls == servos && host_servo == u && !laser_state);
    return 0;
}

static int check_r43_tuning_ranges_and_isolation(void)
{
    CHECK(r43_boot("123") == 0);
    run_cmd("d1"); CHECK(s_route43_tune.road_mm[0] == 1u && s_seq_state == SQ_READY && r43_stopped());
    run_cmd("d20000"); CHECK(s_route43_tune.road_mm[0] == 20000u && r43_stopped());
    run_cmd("e1"); CHECK(s_route43_tune.rack_steps == 1u && !host_timer_active);
    run_cmd("nl100000"); CHECK(s_route43_tune.rack_steps == 100000u && !host_timer_active);
    run_cmd("f1"); CHECK(s_route43_tune.rack_pps == 1u && s_jog_pps == 333u);
    run_cmd("f20000"); CHECK(s_route43_tune.rack_pps == 20000u && s_jog_pps == 333u);
    run_cmd("v1"); run_cmd("pv600");
    CHECK(s_route43_tune.straight_v == 1.0f && s_route43_tune.lateral_v == 600.0f &&
          s_route31_straight_v == 219.0f && s_route31_lateral_v == 321.0f);
    run_cmd("v600"); run_cmd("pv1");
    CHECK(s_route43_tune.straight_v == 600.0f && s_route43_tune.lateral_v == 1.0f);
    const char *const bad[] = {
        "d0","d20001","d-1","d-0","d1.0","d+1","d1x","d2147483648",
        "e0","e100001","e-1","e-0","nl0","nl-1","nl100001","nr2500","e1x",
        "f0","f20001","f-1","f-0","f+1","f1.0","f2147483648",
        "v0","v601","v-1","v1x","pv0","pv601","pv-1","pv1.0","pv2147483648",
        "u1800","su1500","cc","co","h32000","a nonsense"
    };
    for (unsigned k = 0u; k < sizeof bad / sizeof bad[0]; ++k) CHECK(r43_bad_write(bad[k]) == 0);
    CHECK(s_seq_state == SQ_READY && r43_stopped() && !pulse_calls && !servo_calls);
    run_cmd("43"); wire_sync();
    CHECK(s_route43_tune.road_mm[0] == 20000u && s_route43_tune.rack_steps == 100000u &&
          s_route43_tune.rack_pps == 20000u && s_route43_tune.straight_v == 600.0f &&
          s_route43_tune.lateral_v == 1.0f && s_seq_state == SQ_READY && r43_stopped());
    run_cmd("d535"); run_cmd("e3700"); run_cmd("f1000"); run_cmd("v100"); run_cmd("pv100");
    /* Exactly one next-distance slot is mutable in WAIT. A right-angle turn
     * is not a d slot, and route v/pv never change turn/cross/contact speed. */
    wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
    CHECK(r43_motion_begin(1u,17u,535u,100.0f) == 0 && r43_motion_finish(2u) == 0);
    run_cmd("v210"); run_cmd("pv310"); run_cmd("d631");
    CHECK(s_route43_tune.road_mm[1] == 631u && r43_wait(2u) == 0);
    CHECK(r43_motion_begin(2u,16u,631u,210.0f) == 0 && r43_motion_finish(3u) == 0);
    CHECK(r43_bad_write("d555") == 0);
    CHECK(r43_motion_begin(3u,20u,0u,100.0f) == 0 && r43_motion_finish(4u) == 0);
    CHECK(r43_motion_begin(4u,16u,650u,300.0f) == 0 && r43_motion_finish(5u) == 0);
    CHECK(r43_bad_write("d777") == 0);
    CHECK(r43_motion_begin(5u,15u,0u,40.0f) == 0 && r43_motion_finish(6u) == 0);
    CHECK(r43_motion_begin(6u,16u,190u,210.0f) == 0 && r43_motion_finish(7u) == 0);
    CHECK(r43_motion_begin(7u,18u,800u,310.0f) == 0 && r43_motion_finish(8u) == 0);
    run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && r43_stopped());
    run_cmd("31"); CHECK(s_seq_state == SQ_READY && s_seq_mode == 31u);
    wire_sync(); wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
    run_cmd("g"); CHECK(s_seq_state == SQ_STILL && s_seq_stage == 0u && s_msel == 17);
    CHECK(sequence_start_stage() == 0);
    host_lateral = s_dist_odo0 + s_dist_target; wire_poll();
    CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN);
    host_tick += T_DIST_STILL_MS; wire_poll();
    fixture_complete_distance_alignment(); wire_poll();
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == 1u); /*31 still auto-next. */
    run_cmd("0");
    puts("43 tuning:d/e(nl)/f/v/pv inclusive bounds,no motor start;malformed/overflow rejected;next distance scoped,turn/contact no d;road/pv independent,cross300/tilt_contact40/turn100 private;31 still automatic passed");
    return 0;
}

static int check_r43_rack_snapshot_and_wait_locks(void)
{
    CHECK(r43_boot("123") == 0 && r43_prefix() == 0);
    run_cmd("e3333"); run_cmd("f1234");
    CHECK(r43_predeploy(3333u,1234u) == 0);
    CHECK(r43_bad_write("e4444") == 0 && r43_bad_write("nl4444") == 0 && r43_bad_write("f2000") == 0);
    CHECK(r43_motion_begin(10u,30u,0u,100.0f) == 0 && r43_motion_finish(11u) == 0);
    CHECK(r43_ball() == 0);
    CHECK(r43_motion_begin(12u,22u,0u,100.0f) == 0 && r43_motion_finish(13u) == 0);
    run_cmd("d73"); CHECK(s_route43_tune.pair_left_mm == 73u && r43_wait(13u) == 0);
    CHECK(r43_motion_begin(13u,17u,73u,250.0f) == 0 && r43_motion_finish(14u) == 0);
    CHECK(r43_bad_write("d999") == 0 && r43_bad_write("e4444") == 0 && r43_bad_write("f2000") == 0);
    CHECK(r43_bucket(3333u,1234u) == 0);
    CHECK(host_jog_pulses[0][0] == 3333u && host_jog_pulses[0][1] == 3333u &&
          s_route43_rack_issued == 3333u && s_route43_rack_pps_issued == 1234u && pulse_calls == 114666);
    CHECK(r43_motion_begin(15u,22u,0u,100.0f) == 0 && r43_motion_finish(16u) == 0);
    CHECK(s_seq_return_offset && route_seq_leg()->mode == 17u);
    CHECK(r43_motion_begin(16u,17u,20u,250.0f) == 0 && r43_motion_finish(17u) == 0);
    CHECK(!s_seq_return_offset);
    CHECK(r43_target() == 0);
    CHECK(r43_motion_begin(18u,15u,420u,200.0f) == 0 && r43_motion_finish(19u) == 0);
    CHECK(r43_motion_begin(19u,20u,0u,100.0f) == 0 && r43_motion_finish(20u) == 0);
    CHECK(r43_hostage() == 0 && s_route43_rack_issued == 3333u && s_route43_rack_pps_issued == 1234u);
    CHECK(r43_motion_begin(21u,15u,1315u,200.0f) == 0 && r43_motion_finish(0u) == 0);
    CHECK(pulse_calls == 120999 && host_jog_pulses[0][0] == 6666u &&
          host_jog_pulses[0][1] == 6333u && host_jog_pulses[1][0] == 54000u &&
          host_jog_pulses[1][1] == 54000u && servo_calls == 5 && host_servo == 1900u);
    run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && r43_stopped() && host_servo == 1900u);
    puts("43 rack:WAIT edits3333/f1234,issued snapshot retained;left d73 own slot;bucket exact3333 return and hostage extend3333/retract3000 bothat1234;120999 STEP/racknet333 and endpointgrip1900 passed");
    return 0;
}

/* Traverse preceding actions with public g and real packet/IRQ inputs; do
 * not manufacture an advanced stage or an undeclared rack position. */
static int r43_execute(unsigned unit)
{
    const unsigned mode[] = {17u,16u,20u,16u,15u,16u,18u,16u};
    const unsigned mm[] = {535u,630u,0u,650u,0u,190u,800u,760u};
    const float speed[] = {250.0f,200.0f,100.0f,300.0f,40.0f,200.0f,250.0f,200.0f};
    if (unit <= 8u) {
        CHECK(r43_motion_begin(unit,mode[unit-1u],mm[unit-1u],speed[unit-1u]) == 0);
        CHECK(r43_motion_finish(unit+1u) == 0);
        return 0;
    }
    switch (unit) {
    case 9u: CHECK(r43_predeploy(3700u,5000u) == 0); break;
    case 10u: CHECK(r43_motion_begin(unit,30u,0u,100.0f) == 0 && r43_motion_finish(unit+1u) == 0); break;
    case 11u: CHECK(r43_ball() == 0); break;
    case 12u: CHECK(r43_motion_begin(unit,22u,0u,100.0f) == 0 && r43_motion_finish(unit+1u) == 0); break;
    case 13u: CHECK(r43_motion_begin(unit,17u,40u,250.0f) == 0 && r43_motion_finish(unit+1u) == 0); break;
    case 14u: CHECK(r43_bucket(3700u,5000u) == 0); break;
    case 15u: CHECK(r43_motion_begin(unit,22u,0u,100.0f) == 0 && r43_motion_finish(unit+1u) == 0); break;
    case 16u: CHECK(r43_motion_begin(unit,17u,20u,250.0f) == 0 && r43_motion_finish(unit+1u) == 0); break;
    case 17u: CHECK(r43_target() == 0); break;
    case 18u: CHECK(r43_motion_begin(unit,15u,420u,200.0f) == 0 && r43_motion_finish(unit+1u) == 0); break;
    case 19u: CHECK(r43_motion_begin(unit,20u,0u,100.0f) == 0 && r43_motion_finish(unit+1u) == 0); break;
    case 20u: CHECK(r43_hostage() == 0); break;
    case 21u: CHECK(r43_motion_begin(unit,15u,1315u,200.0f) == 0 && r43_motion_finish(0u) == 0); break;
    default: CHECK(0);
    }
    return 0;
}

static int r43_at(unsigned unit)
{
    CHECK(unit >= 1u && unit <= 21u && r43_boot("123") == 0);
    for (unsigned prior = 1u; prior < unit; ++prior) CHECK(r43_execute(prior) == 0);
    if (unit > 1u) CHECK(r43_wait(unit) == 0);
    else CHECK(s_seq_state == SQ_READY && r43_stopped());
    return 0;
}

static int r43_cancel(const char *key)
{
    RouteStepTune before = s_route43_tune;
    unsigned unit = s_route43_unit;
    int pulses = pulse_calls, servos = servo_calls;
    uint16_t u = host_servo, request = wire_request;
    run_cmd(key);
    CHECK(s_seq_state == SQ_STOPPED && s_route43_unit == unit && r43_stopped() &&
          s_route31_lift_phase == R31_LIFT_OFF && !s_receiving && !s_due &&
          !s_seq_pair_turn && !s_seq_pair_offset && !s_seq_return_offset && !s_route31_hostage_hold && !s_route31_hostage_opened);
    test_stepper_timer_irq(); wire_ack(request,2u,0u); xy_qr(request,"123");
    xy_obj(request,4,150,VAT_BALL_Y_PX); xy_obj(request,9,140,VAT_BUCKET_Y_PX);
    xy_obj(request,0,230,VAT_HOSTAGE_Y_PX); r43_rank(request,0u,3u);
    CHECK(r43_service(10000u) == 0); run_cmd("g"); CHECK(r43_service(1000u) == 0);
    CHECK(s_seq_state == SQ_STOPPED && r43_stopped() && !s_receiving && !s_due &&
          pulse_calls == pulses && servo_calls == servos && host_servo == u &&
          memcmp(&before,&s_route43_tune,sizeof before) == 0);
    return 0;
}

static int check_r43_bucket_loss_release_gate(void)
{
    CHECK(r43_to_bucket_wait() == 0);
    run_cmd("g"); wire_sync(); wire_ack(wire_request,2u,0u);
    CHECK(r43_service(VAT_ROUTE_BUCKET_MISSING_MS + 600u) == 0 &&
          r43_status.state == VAT_ROUTE_BUCKET_SEARCH && last_x == -200.0f &&
          !r43_status.bucket_seen && !r43_status.bucket_fallback && !host_timer_active);
    int pulses = pulse_calls, servos = servo_calls; uint16_t request = wire_request;
    CHECK(r43_service(5000u) == 0 && !r43_status.bucket_seen &&
          !host_timer_active && pulse_calls == pulses && servo_calls == servos);
    xy_obj(request,9,500,VAT_BUCKET_Y_PX);
    CHECK(r43_service(1999u) == 0 && r43_status.bucket_age_ms == 1999u && !host_timer_active);
    CHECK(r43_service(1u) == 0 && !strcmp(r43_status.reason,"BUCKET_LOST2S_BRAKE") &&
          !host_timer_active && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    CHECK(r43_service(249u) == 0 && !host_timer_active && servo_calls == servos);
    CHECK(r43_service(1u) == 0 && s_route31_lift_phase == R31_LIFT_BUCKET_DOWN &&
          r43_status.bucket_fallback && !r43_status.alignment_confirmed && !r43_status.good &&
          host_servo == 1900u && servo_calls == servos);
    CHECK(r43_bucket_action_complete(3700u,5000u) == 0 && r43_wait(15u) == 0);
    CHECK(r43_status.bucket_fallback && !r43_status.alignment_confirmed && !r43_status.good);
    CHECK(r43_motion_begin(15u,22u,0u,100.0f) == 0 && r43_motion_finish(16u) == 0);
    CHECK(s_seq_return_offset && !s_target35_route && r43_wait(16u) == 0);
    CHECK(r43_motion_begin(16u,17u,20u,250.0f) == 0 && r43_motion_finish(17u) == 0);
    CHECK(!s_seq_return_offset && !s_target35_route && r43_wait(17u) == 0);
    CHECK(r43_cancel("0") == 0);
    puts("43 bucket loss:no-first never releases;NEW loss2s+250 commits real down/release2/up immediate rack;return180 WAIT_G16/LEFT20+yawfix WAIT_G17 before task2,not fakealignment passed");
    return 0;
}

static int check_r43_hostage_retract_range_gate(void)
{
    CHECK(R31_HOSTAGE_GRIP_WAIT == 15u && R31_HOSTAGE_RACK_RETRACT == 16u &&
          R31_HOSTAGE_RACK_RETRACT_WAIT == 17u);
    CHECK(r43_boot("123") == 0 && r43_prefix() == 0);
    run_cmd("e2999"); run_cmd("f1234");
    CHECK(s_route43_tune.rack_steps == 2999u && !host_timer_active);
    CHECK(r43_predeploy(2999u,1234u) == 0);
    for (unsigned unit = 10u; unit <= 13u; ++unit) CHECK(r43_execute(unit) == 0);
    CHECK(r43_bucket(2999u,1234u) == 0);
    for (unsigned unit = 15u; unit <= 19u; ++unit) CHECK(r43_execute(unit) == 0);
    CHECK(r43_wait(20u) == 0 && !s_route31_rack_deployed &&
          s_route43_rack_issued == 2999u && s_route43_rack_pps_issued == 1234u);
    int pulses = pulse_calls, servos = servo_calls;
    uint16_t u = host_servo;
    run_cmd("g"); wire_sync(); wire_ack(wire_request,2u,0u);
    CHECK(r43_service(300u) == 0);
    r43_rank(wire_request,0u,2u);
    host_messages[0] = '\0';
    for (unsigned frame = 0u; frame < 120u && s_seq_state != SQ_STOPPED; ++frame) {
        host_tick += 20u; xy_obj(wire_request,0,230,VAT_HOSTAGE_Y_PX);
    }
    CHECK(s_seq_state == SQ_STOPPED && s_route31_lift_phase == R31_LIFT_OFF &&
          strstr(host_messages,"HOSTAGE_RACK_STEPS_RANGE") && r43_stopped() &&
          pulse_calls == pulses && servo_calls == servos && host_servo == u && u == 1500u &&
          !s_route31_hostage_opened && !s_route31_hostage_hold &&
          s_route43_rack_issued == 2999u && s_route43_rack_pps_issued == 1234u);
    CHECK(r43_cancel("0") == 0);
    puts("43 e2999:accepted/predeployed/ball-bucket exact return;hostage3000 retract range rejects before1150/STEP/grip with no clamp;late inputs/g inert passed");
    return 0;
}

static int r43_hostage_search(void)
{
    CHECK(r43_at(20u) == 0); run_cmd("g"); wire_sync();
    CHECK(s_seq_state == SQ_TASK && wire_task == 3u && wire_digit == 3u);
    wire_ack(wire_request,2u,0u); CHECK(r43_service(300u) == 0);
    CHECK(r43_status.state == VAT_ROUTE_SEARCH && !r43_status.hostage_seen &&
          !r43_status.hostage_fallback && !r43_status.alignment_confirmed &&
          !host_timer_active && host_servo == 1500u && servo_calls == 3);
    return 0;
}

/* No helper injects a fabricated rank here. The dropout grab is explicitly
 * different evidence from five-frame pixel alignment, and54 is not01. */
static int check_r43_hostage_loss_and_unknown_rank(void)
{
    CHECK(r43_at(20u) == 0); run_cmd("g"); wire_sync();
    xy_obj(wire_request,0,500,VAT_HOSTAGE_Y_PX); r43_snapshot();
    CHECK(!r43_status.hostage_seen && !host_timer_active); /* Before matchingACK. */
    wire_ack(wire_request,2u,0u); CHECK(r43_service(300u) == 0);
    xy_obj(wire_request,1,500,VAT_HOSTAGE_Y_PX);
    xy_obj((uint16_t)(wire_request-1u),0,500,VAT_HOSTAGE_Y_PX);
    CHECK(r43_service(5000u) == 0 && !r43_status.hostage_seen && !host_timer_active &&
          servo_calls == 3 && !s_route31_hostage_hold && s_route43_unit == 20u);
    CHECK(r43_cancel("0") == 0);

    CHECK(r43_hostage_search() == 0);
    unsigned before_pulses = (unsigned)pulse_calls;
    unsigned coordinate_seq = xy_seq;
    xy_obj_poll(wire_request,0,500,VAT_HOSTAGE_Y_PX,0);
    xy_obj_poll(wire_request,-1,0,0,0); wire_poll(); r43_snapshot();
    CHECK(r43_status.hostage_seen && r43_status.hostage_age_ms == 0u &&
          !r43_status.alignment_confirmed && !r43_status.hostage_fallback);
    CHECK(r43_service(1000u) == 0);
    unsigned current_seq = xy_seq;
    xy_seq = coordinate_seq; xy_obj(wire_request,0,500,VAT_HOSTAGE_Y_PX);
    xy_seq = current_seq;
    r43_snapshot(); CHECK(r43_status.hostage_age_ms == 1000u);
    r43_rank(wire_request,0u,0u); /* Real unknown54 must not refresh coordinate age. */
    xy_obj(wire_request,1,500,VAT_HOSTAGE_Y_PX);
    xy_obj((uint16_t)(wire_request-1u),0,500,VAT_HOSTAGE_Y_PX);
    CHECK(r43_service(999u) == 0 && r43_status.hostage_age_ms == 1999u &&
          !host_timer_active && servo_calls == 3 && (unsigned)pulse_calls == before_pulses);
    CHECK(r43_service(1u) == 0 && r43_status.hostage_age_ms == 2000u &&
          strcmp(r43_status.reason,"HOSTAGE_LOST2S_BRAKE") == 0 &&
          !r43_status.alignment_confirmed && !r43_status.hostage_fallback &&
          !host_timer_active && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    CHECK(r43_service(100u) == 0); ++host_counts[0]; wire_poll();
    CHECK(r43_service(249u) == 0 && !host_timer_active && servo_calls == 3);
    CHECK(r43_service(1u) == 0 && s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND &&
          r43_status.state == VAT_WAIT_HOSTAGE_ACTION && r43_status.hostage_fallback &&
          !r43_status.alignment_confirmed && r43_status.good == 0u &&
          host_jog_axis == 0 && host_jog_dir == 0 && host_timer_pps == 5000u &&
          host_servo == 1150u && servo_calls == 4 && s_jog_clock.remaining == 3700u);
    CHECK(r43_emit(3700u) == 0); wire_poll();
    CHECK(r43_service(250u) == 0 && host_servo == 1900u && servo_calls == 5 &&
          s_route31_lift_phase == R31_HOSTAGE_GRIP_WAIT &&
          r43_status.state == VAT_WAIT_HOSTAGE_ACTION && !s_route31_hostage_hold);
    CHECK(r43_hostage_retract_after_grip() == 0 &&
          r43_status.state == VAT_WAIT_HOSTAGE_RANK && s_receiving &&
          !s_route31_hostage_hold && s_route31_hostage_rank == 0u && !host_timer_active);
    CHECK(r43_service(5000u) == 0 && s_seq_state == SQ_TASK && s_route43_unit == 20u &&
          r43_status.hostage_fallback && !r43_status.alignment_confirmed && r43_status.good == 0u &&
          (unsigned)pulse_calls == before_pulses + 6700u && servo_calls == 5 && host_servo == 1900u);
    xy_obj(wire_request,0,230,VAT_HOSTAGE_Y_PX);
    r43_rank((uint16_t)(wire_request-1u),0u,2u); r43_rank(wire_request,1u,2u);
    unsigned unknown_sequence = xy_seq;
    r43_rank(wire_request,0u,0u);
    unsigned following_sequence = xy_seq;
    xy_seq = unknown_sequence; r43_rank(wire_request,0u,2u); /* Duplicate conflict. */
    xy_seq = unknown_sequence - 1u; r43_rank(wire_request,0u,2u); /* Stale54. */
    xy_seq = following_sequence;
    CHECK(r43_service(1000u) == 0 && r43_status.state == VAT_WAIT_HOSTAGE_RANK &&
          r43_status.target_rank == 0u && !s_route31_hostage_hold);
    r43_rank(wire_request,0u,2u); r43_snapshot();
    CHECK(r43_status.state == VAT_DONE && r43_status.target_rank == 2u &&
          r43_status.hostage_fallback && !r43_status.alignment_confirmed &&
          r43_status.good == 0u && s_route31_hostage_hold && !s_receiving);
    wire_poll(); CHECK(r43_wait(21u) == 0);
    CHECK((unsigned)pulse_calls == before_pulses + 6700u && servo_calls == 5 && host_servo == 1900u);
    CHECK(r43_motion_begin(21u,15u,1315u,200.0f) == 0 && r43_cancel("0") == 0);
    puts("43 hostage:preACK/wrongtarget/oldrequest never arm;sticky01 survives empty;1999 no grab,2000 BRAKE,encoder change restarts250;real grab onlyonce/no fakealignment;rank0 waitsRX,old/wrong54 inert,realrank2 noextraHold -> WAIT_G21/d1315 passed");
    return 0;
}

static int check_r43_hostage_fresh_recovery_and_stop_priority(void)
{
    CHECK(r43_hostage_search() == 0); xy_obj(wire_request,0,500,VAT_HOSTAGE_Y_PX);
    CHECK(r43_service(1999u) == 0 && !host_timer_active);
    xy_obj(wire_request,0,500,VAT_HOSTAGE_Y_PX); r43_snapshot();
    CHECK(r43_status.hostage_age_ms == 0u && !r43_status.hostage_fallback);
    CHECK(r43_service(1999u) == 0 && !host_timer_active && servo_calls == 3);
    CHECK(r43_service(1u) == 0 && strcmp(r43_status.reason,"HOSTAGE_LOST2S_BRAKE") == 0);
    CHECK(r43_service(100u) == 0);
    xy_obj(wire_request,0,500,VAT_HOSTAGE_Y_PX); r43_snapshot();
    CHECK(r43_status.hostage_age_ms == 0u && !r43_status.hostage_fallback &&
          !r43_status.alignment_confirmed && !host_timer_active && servo_calls == 3);
    CHECK(r43_service(1999u) == 0 && !host_timer_active && servo_calls == 3);
    CHECK(r43_service(1u) == 0 && strcmp(r43_status.reason,"HOSTAGE_LOST2S_BRAKE") == 0);
    CHECK(r43_cancel("0") == 0);

    const char *const keys[] = {"g","a","0"};
    for (unsigned fault = 0u; fault < 6u; ++fault) {
        CHECK(r43_hostage_search() == 0); xy_obj(wire_request,0,500,VAT_HOSTAGE_Y_PX);
        CHECK(r43_service(1999u) == 0);
        int pulses = pulse_calls, servos = servo_calls; uint16_t u = host_servo;
        host_tick += 1u;
        if (fault < 3u) CHECK(r43_cancel(keys[fault]) == 0);
        else {
            if (fault == 3u) { host_imu_valid = 0; wire_poll(); }
            else if (fault == 4u) { host_abort = 1; wire_poll(); }
            else wire_ack(wire_request,2u,1u);
            CHECK(s_seq_state == SQ_STOPPED && r43_stopped() &&
                  pulse_calls == pulses && servo_calls == servos && host_servo == u &&
                  s_route31_lift_phase == R31_LIFT_OFF && !s_receiving);
            CHECK(r43_cancel("0") == 0);
        }
    }
    puts("43 hostage:newvalid01 resets1999 timer and cancels pending brake before action;g/a/0,IMU,abort,NACK win at2000 before STEP/open/grip passed");
    return 0;
}

static int check_r43_all_wait_and_running_cancellations(void)
{
    const char *const keys[] = {"g","a","0"};
    for (unsigned unit = 2u; unit <= 21u; ++unit) for (unsigned k = 1u; k < 3u; ++k) {
        CHECK(r43_at(unit) == 0 && r43_cancel(keys[k]) == 0);
    }
    for (unsigned unit = 1u; unit <= 21u; ++unit) for (unsigned k = 0u; k < 3u; ++k) {
        CHECK(r43_at(unit) == 0); run_cmd("g");
        CHECK(s_seq_state == SQ_STILL || s_seq_state == SQ_ARM_PREP || s_seq_state == SQ_TASK);
        if (s_seq_state == SQ_STILL) CHECK(sequence_start_stage() == 0);
        CHECK(r43_cancel(keys[k]) == 0);
    }
    /* QR_WAIT is a stopped gate: mistimed g is ignored, never queued.
     * Only a fresh tuple can expose WAIT_G2; a/0 cancel the whole route. */
    for (unsigned k = 0u; k < 3u; ++k) {
        CHECK(r43_boot(NULL) == 0); wire_ack(wire_request,1u,0u);
        CHECK(r43_motion_begin(1u,17u,535u,250.0f) == 0 && r43_motion_finish(999u) == 0);
        if (k == 0u) {
            int pulses = pulse_calls, servos = servo_calls;
            run_cmd("g"); CHECK(r43_service(5000u) == 0);
            CHECK(s_seq_state == SQ_QR_WAIT && s_route43_unit == 1u && r43_stopped() &&
                  pulse_calls == pulses && servo_calls == servos);
            xy_qr(wire_request,"123"); CHECK(r43_wait(2u) == 0);
            CHECK(r43_service(5000u) == 0 && s_seq_state == SQ_STEP_WAIT && r43_stopped());
            CHECK(r43_cancel("0") == 0);
        } else CHECK(r43_cancel(keys[k]) == 0);
    }
    puts("43 cancels:40 a/0 at all20 WAIT_G boundaries,63 g/a/0 during each21 action;QR_WAIT g ignored/notqueued,a/0 cancel;lateACK/QR/01/54/IRQ and repeatedg never reopen RX/STEP/claw/next;held claw unchanged passed");
    return 0;
}

static int r43_mechanical_at(unsigned phase)
{
    int hostage = phase == R31_HOSTAGE_RACK_EXTEND || phase == R31_HOSTAGE_RACK_EXTEND_WAIT ||
                  phase == R31_HOSTAGE_GRIP_WAIT || phase == R31_HOSTAGE_RACK_RETRACT ||
                  phase == R31_HOSTAGE_RACK_RETRACT_WAIT;
    CHECK(r43_at(phase == R31_RACK_EXTEND || phase == R31_RACK_EXTEND_WAIT ? 9u :
                 phase == R31_LIFT_BALL_DOWN || phase == R31_LIFT_BALL_WAIT || phase == R31_LIFT_BALL_UP ? 11u :
                 hostage ? 20u : 14u) == 0);
    run_cmd("g"); wire_sync();
    if (phase == R31_RACK_EXTEND) return 0;
    if (phase == R31_RACK_EXTEND_WAIT) {
        CHECK(r43_emit(3700u) == 0); wire_poll();
        CHECK(s_route31_lift_phase == phase); return 0;
    }
    if (hostage) {
        CHECK(r43_aim(0,230,VAT_HOSTAGE_Y_PX,VAT_WAIT_HOSTAGE_ACTION) == 0);
        if (phase == R31_HOSTAGE_RACK_EXTEND) return 0;
        CHECK(r43_emit(3700u) == 0); wire_poll();
        CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND_WAIT && !host_timer_active);
        if (phase == R31_HOSTAGE_RACK_EXTEND_WAIT) return 0;
        CHECK(r43_service(250u) == 0 && s_route31_lift_phase == R31_HOSTAGE_GRIP_WAIT &&
              host_servo == 1900u && !s_route31_hostage_hold);
        if (phase == R31_HOSTAGE_GRIP_WAIT) return 0;
        CHECK(r43_service(250u) == 0 && s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT &&
              host_timer_active && host_jog_axis == 0 && host_jog_dir == 1 &&
              host_timer_pps == 5000u && s_jog_clock.remaining == 3000u);
        if (phase == R31_HOSTAGE_RACK_RETRACT) return 0;
        CHECK(r43_emit(3000u) == 0); wire_poll();
        CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT_WAIT &&
              phase == R31_HOSTAGE_RACK_RETRACT_WAIT && !host_timer_active);
        return 0;
    }
    int ball = phase == R31_LIFT_BALL_DOWN || phase == R31_LIFT_BALL_WAIT || phase == R31_LIFT_BALL_UP;
    CHECK(r43_aim(ball ? 4 : 9,ball ? 150 : 140,ball ? VAT_BALL_Y_PX : VAT_BUCKET_Y_PX,
                  ball ? VAT_WAIT_BALL_ACTION : VAT_WAIT_BUCKET_ACTION) == 0);
    if (phase == R31_LIFT_BALL_DOWN || phase == R31_LIFT_BUCKET_DOWN) return 0;
    CHECK(r43_emit(ball ? 32000u : 22000u) == 0); wire_poll();
    if (phase == R31_LIFT_BALL_WAIT || phase == R31_CLAW_RELEASE_WAIT) return 0;
    CHECK(r43_service(ball ? 2000u : 1000u) == 0);
    if (phase == R31_LIFT_BALL_UP || phase == R31_CLAW_RESTAGED_WAIT) return 0;
    CHECK(r43_service(1000u) == 0);
    if (phase == R31_LIFT_BUCKET_UP) return 0;
    CHECK(r43_emit(22000u) == 0); wire_poll();
    /* Completed bucket lift starts return rack in this same service pass. */
    if (phase == R31_RACK_RETRACT) return 0;
    CHECK(r43_emit(3700u) == 0); wire_poll();
    CHECK(s_route31_lift_phase == R31_RACK_RETRACT_WAIT && phase == R31_RACK_RETRACT_WAIT);
    return 0;
}

static int check_r43_mechanical_owner_and_faults(void)
{
    const unsigned phases[] = {
        R31_RACK_EXTEND,R31_RACK_EXTEND_WAIT,R31_LIFT_BALL_DOWN,R31_LIFT_BALL_WAIT,
        R31_LIFT_BALL_UP,R31_LIFT_BUCKET_DOWN,R31_CLAW_RELEASE_WAIT,R31_CLAW_RESTAGED_WAIT,
        R31_LIFT_BUCKET_UP,R31_RACK_RETRACT,R31_RACK_RETRACT_WAIT,
        R31_HOSTAGE_RACK_EXTEND,R31_HOSTAGE_RACK_EXTEND_WAIT,R31_HOSTAGE_GRIP_WAIT,
        R31_HOSTAGE_RACK_RETRACT,R31_HOSTAGE_RACK_RETRACT_WAIT
    };
    const char *const keys[] = {"g","a","0"};
    const char *const writes[] = {
        "43","31","42","27","35","38","d777","v210","pv310","e3500","nl3500",
        "f1200","u1500","su1500","co","cc","ykp10","fff0","yfix0"
    };
    for (unsigned p = 0u; p < sizeof phases / sizeof phases[0]; ++p) {
        for (unsigned k = 0u; k < 3u; ++k) {
            CHECK(r43_mechanical_at(phases[p]) == 0 && s_route31_lift_phase == phases[p]);
            if (k == 0u) {
                for (unsigned w = 0u; w < sizeof writes / sizeof writes[0]; ++w)
                    CHECK(r43_bad_write(writes[w]) == 0);
            }
            CHECK(r43_cancel(keys[k]) == 0);
        }
    }
    const unsigned moving[] = {
        R31_RACK_EXTEND,R31_LIFT_BALL_DOWN,R31_LIFT_BALL_UP,
        R31_LIFT_BUCKET_DOWN,R31_LIFT_BUCKET_UP,R31_RACK_RETRACT,R31_HOSTAGE_RACK_EXTEND,
        R31_HOSTAGE_RACK_RETRACT
    };
    for (unsigned p = 0u; p < sizeof moving / sizeof moving[0]; ++p) {
        CHECK(r43_mechanical_at(moving[p]) == 0 && r43_emit(2u) == 0);
        JogClock stopped_job;
        int pulses = pulse_calls, servos = servo_calls; uint16_t u = host_servo;
        CHECK(step_clock_snapshot(&stopped_job,1) == 0u && stopped_job.completed == 2u);
        wire_poll();
        CHECK(s_seq_state == SQ_STOPPED && r43_stopped() && host_servo == u &&
              pulse_calls == pulses && servo_calls == servos);
        run_cmd("g"); CHECK(r43_service(10000u) == 0 && s_seq_state == SQ_STOPPED && r43_stopped());
    }
    for (unsigned k = 0u; k < 3u; ++k) {
        CHECK(r43_mechanical_at(R31_HOSTAGE_RACK_RETRACT_WAIT) == 0);
        CHECK(r43_service(250u) == 0 && s_route31_hostage_hold && host_servo == 1900u);
        CHECK(r43_cancel(keys[k]) == 0 && host_servo == 1900u);
    }
    puts("43 mechanics:16phases x3 g/a/0 stops and19 writes locked;hostage grip/retract/pending-next cancel retains1900;all8 partial-count failures STOP without late phase/action/g resume passed");
    return 0;
}

static int check_r43_clock_start_failures(void)
{
    for (unsigned job = 0u; job < 8u; ++job) {
        if (job == 0u) CHECK(r43_at(9u) == 0);
        else if (job == 1u || job == 3u || job == 6u) {
            CHECK(r43_at(job == 1u ? 11u : job == 3u ? 14u : 20u) == 0);
            run_cmd("g"); wire_sync();
        } else CHECK(r43_mechanical_at(job == 2u ? R31_LIFT_BALL_WAIT :
                                      job == 4u ? R31_CLAW_RESTAGED_WAIT :
                                      job == 7u ? R31_HOSTAGE_GRIP_WAIT : R31_LIFT_BUCKET_UP) == 0);
        if (job == 5u) CHECK(r43_emit(22000u) == 0);
        int pulses = pulse_calls, servos = servo_calls;
        uint16_t u = host_servo;
        host_messages[0] = '\0'; host_timer_start_fail = 1;
        if (job == 0u) run_cmd("g");
        else if (job == 1u || job == 3u || job == 6u) {
            wire_ack(wire_request,2u,0u); CHECK(r43_service(300u) == 0);
            if (job == 6u) r43_rank(wire_request,0u,2u);
            for (unsigned frame = 0u; frame < 120u && s_seq_state != SQ_STOPPED; ++frame) {
                host_tick += 20u;
                xy_obj(wire_request,job == 1u ? 4 : job == 3u ? 9 : 0,
                       job == 1u ? 150 : job == 3u ? 140 : 230,
                       job == 1u ? VAT_BALL_Y_PX : job == 3u ? VAT_BUCKET_Y_PX : VAT_HOSTAGE_Y_PX);
            }
        } else if (job == 5u) wire_poll();
        else CHECK(r43_service(job == 4u ? 1000u : job == 7u ? 250u : 2000u) == 0);
        CHECK(s_seq_state == SQ_STOPPED && s_route31_lift_phase == R31_LIFT_OFF &&
              r43_stopped() && strstr(host_messages,"LIFT_START_ERROR") &&
              pulse_calls == pulses && (job == 6u ? servo_calls == servos + 1 && host_servo == 1150u :
                                      servo_calls == servos && host_servo == u));
        CHECK(r43_cancel("0") == 0);
    }
    puts("43 STEP:all eight clock-start failures stop before any new pulse/return/turn;hostage open1150 may precede failed extend but never grip;failed retract keeps1900;late inputs and g cannot resume passed");
    return 0;
}

static int check_r43_every_next_distance_slot(void)
{
    const unsigned units[] = {1u,2u,4u,6u,7u,8u,13u,16u,18u,21u};
    const unsigned stages[] = {0u,1u,3u,5u,6u,7u,9u,10u,12u,15u};
    const unsigned modes[] = {17u,16u,16u,16u,18u,16u,17u,17u,15u,15u};
    const float speeds[] = {250.0f,200.0f,300.0f,200.0f,250.0f,200.0f,250.0f,250.0f,200.0f,200.0f};
    for (unsigned index = 0u; index < sizeof units / sizeof units[0]; ++index) {
        CHECK(r43_at(units[index]) == 0);
        RouteStepTune expected = s_route43_tune;
        unsigned mm = 901u + index;
        if (units[index] == 13u) expected.pair_left_mm = (uint16_t)mm;
        else if (units[index] == 16u) expected.return_left_mm = (uint16_t)mm;
        else if (units[index] == 18u) expected.corner_mm[1] = (uint16_t)mm;
        else if (units[index] == 21u) expected.rank_mm[1] = (uint16_t)mm;
        else expected.road_mm[stages[index]] = (uint16_t)mm;
        char command[12]; snprintf(command,sizeof command,"d%u",mm);
        int pulses = pulse_calls, servos = servo_calls;
        run_cmd(command);
        CHECK(memcmp(&expected,&s_route43_tune,sizeof expected) == 0 &&
              s_route43_unit == units[index] && r43_stopped() &&
              pulse_calls == pulses && servo_calls == servos);
        CHECK(r43_motion_begin(units[index],modes[index],mm,speeds[index]) == 0);
        CHECK(r43_cancel("0") == 0);
    }
    const unsigned no_distance[] = {3u,5u,9u,10u,11u,12u,14u,15u,17u,19u,20u};
    for (unsigned index = 0u; index < sizeof no_distance / sizeof no_distance[0]; ++index) {
        CHECK(r43_at(no_distance[index]) == 0 && r43_bad_write("d900") == 0);
        CHECK(r43_cancel("0") == 0);
    }
    puts("43 d:all ten road/offset/selected-color/selected-rank next slots independently edited,each next g uses its signed leg only;eleven tiltcontact/turn/predeploy/task nodes reject d without motion passed");
    return 0;
}

static float route_turn_old_w(float error)
{
    float w = T_TURN_KP * error;
    if (w > T_TURN_MAX_W) w = T_TURN_MAX_W;
    if (w < -T_TURN_MAX_W) w = -T_TURN_MAX_W;
    if (w > 0.0f && w < T_TURN_MIN_W) w = T_TURN_MIN_W;
    if (w < 0.0f && w > -T_TURN_MIN_W) w = -T_TURN_MIN_W;
    return w;
}

static int check_route_turn_profile_scope(void)
{
    CHECK(ROUTE31_TURN_SPEED_SCALE == 2.0f && ROUTE31_TURN_SLOW_DEG == 15.0f &&
          ROUTE31_TURN_FAST_DEG == 25.0f);
    CHECK(T_TURN_KP == 0.15f && T_TURN_MAX_W == 2.0f && T_TURN_MIN_W == 0.18f &&
          T_TURN_TOL_DEG == 0.3f && T_TURN_SETTLE_MS == 700u && TURN90_STILL_DEG == 0.2f);
    CHECK(T_TURN_MAX_MS == 12000u && T_TURN180_MAX_MS == 12000u);
    const float errors[] = {0.0f,0.01f,1.0f,5.0f,7.5f,15.0f,15.01f,20.0f,24.99f,25.0f,30.0f,180.0f};
    for (unsigned owner = 31u; owner <= 43u; owner += 12u) {
        reset_fixture(); s_seq_mode = (uint8_t)owner; s_seq_state = SQ_RUN;
        for (unsigned k = 0u; k < sizeof errors / sizeof errors[0]; ++k) {
            float factor = errors[k] <= 15.0f ? 1.0f : errors[k] >= 25.0f ?
                           2.0f : 1.0f + (errors[k] - 15.0f) / 10.0f;
            for (int dir = -1; dir <= 1; dir += 2) {
                float error = (float)dir * errors[k];
                CHECK(fabsf(turn_speed_scale(error) - factor) < 0.00001f);
                CHECK(fabsf(turn_velocity(error) - route_turn_old_w(error) * factor) < 0.00001f);
                CHECK(fabsf(turn_velocity(error)) <= 4.0f);
            }
        }
        CHECK(turn_velocity(30.0f) == 4.0f && turn_velocity(-30.0f) == -4.0f);
        s_seq_state = SQ_OFF;
        CHECK(turn_speed_scale(30.0f) == 1.0f && turn_velocity(30.0f) == 2.0f &&
              turn_velocity(-30.0f) == -2.0f); /* Retained owner never leaks. */
    }
    const unsigned bystanders[] = {32u,34u,35u,36u,37u,38u,41u,42u};
    for (unsigned k = 0u; k < sizeof bystanders / sizeof bystanders[0]; ++k) {
        reset_fixture(); s_seq_mode = (uint8_t)bystanders[k]; s_seq_state = SQ_RUN;
        CHECK(turn_speed_scale(30.0f) == 1.0f && turn_velocity(30.0f) == 2.0f &&
              turn_velocity(-30.0f) == -2.0f && turn_velocity(1.0f) == 0.18f);
    }
    const char *const standalone[] = {"20","22","30"};
    for (unsigned k = 0u; k < sizeof standalone / sizeof standalone[0]; ++k) {
        reset_fixture(); s_seq_mode = 43u; run_cmd(standalone[k]); run_cmd("g"); tick();
        float dir = k == 2u ? -1.0f : 1.0f;
        CHECK(s_seq_state == SQ_OFF && s_round == R_RUN && last_w == dir * 2.0f);
        CHECK(turn_target_deg() == (k == 2u ? -92.0f : k == 1u ? 180.0f : 90.0f));
    }
    puts("31/43 turn profile:signed cap4 only active route,<=15deg old P/min,20deg x1.5,>=25deg x2;SQ_OFF/legacy/manual isolated;tol/hold/12s unchanged passed");
    return 0;
}

static int check_r43_turn_actual_tick_and_settle(void)
{
    const unsigned units[] = {3u,10u,12u,15u,19u};
    const unsigned modes[] = {20u,30u,22u,22u,20u};
    const float remaining[] = {20.0f,15.0f,7.5f,1.0f,-0.5f};
    const float commands[] = {3.0f,2.0f,1.125f,0.18f,-0.18f};
    for (unsigned k = 0u; k < sizeof units / sizeof units[0]; ++k) {
        CHECK(r43_at(units[k]) == 0 && r43_motion_begin(units[k],modes[k],0u,100.0f) == 0);
        float target = turn_target_deg(), dir = target < 0.0f ? -1.0f : 1.0f;
        CHECK(last_w == dir * 4.0f && s_round == R_RUN);
        for (unsigned p = 0u; p < sizeof remaining / sizeof remaining[0]; ++p) {
            host_yaw = target - dir * remaining[p]; host_tick += 20u; wire_poll();
            CHECK(s_round == R_RUN && s_seq_state == SQ_RUN &&
                  fabsf(last_w - dir * commands[p]) < 0.00001f);
            CHECK(last_x == 0.0f && last_y == 0.0f && !laser_state);
        }
        host_yaw = target; host_tick += 20u; wire_poll();
        CHECK(s_round == R_BRAKE && !last_w && s_seq_state == SQ_RUN);
        /* Coarse scaling does not weaken inertia reopening or stable hold. */
        host_yaw = target + dir * 0.31f; host_tick += 350u; wire_poll();
        CHECK(s_round == R_RUN && s_seq_state == SQ_RUN);
        host_tick += 20u; wire_poll();
        CHECK(fabsf(last_w + dir * 0.18f) < 0.00001f && s_round == R_RUN);
        host_yaw = target + dir * 0.29f; host_tick += 20u; wire_poll();
        CHECK(s_round == R_BRAKE && !last_w);
        host_tick += 350u; host_yaw = target - dir * 0.01f; wire_poll();
        CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && !last_w &&
              s_turn_settle_t0 == host_tick); /* Within tolerance, but moving >0.2deg. */
        host_tick += 699u; wire_poll();
        CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && !last_w);
        host_absolute_yaw += target; host_tick += 1u; wire_poll();
        CHECK(s_seq_state == SQ_STEP_WAIT && s_route43_unit == units[k] + 1u && r43_stopped());
    }
    puts("43 actual +/-90 and both+180 ticks:coarse signed4 ->smooth3 ->old2/1.125/min0.18;overshoot reverses,>0.3 reopens,>0.2 resets700ms,not699/no auto-next passed");
    return 0;
}

int main(void)
{
    CHECK(check_r43_full_twenty_one_actions() == 0);
    CHECK(check_r43_late_qr_and_distance_slot() == 0);
    CHECK(check_r43_tuning_ranges_and_isolation() == 0);
    CHECK(check_r43_rack_snapshot_and_wait_locks() == 0);
    CHECK(check_r43_hostage_retract_range_gate() == 0);
    CHECK(check_r43_bucket_loss_release_gate() == 0);
    CHECK(check_r43_hostage_loss_and_unknown_rank() == 0);
    CHECK(check_r43_hostage_fresh_recovery_and_stop_priority() == 0);
    CHECK(check_r43_all_wait_and_running_cancellations() == 0);
    CHECK(check_r43_mechanical_owner_and_faults() == 0);
    CHECK(check_r43_clock_start_failures() == 0);
    CHECK(check_r43_every_next_distance_slot() == 0);
    CHECK(check_route_turn_profile_scope() == 0);
    CHECK(check_r43_turn_actual_tick_and_settle() == 0);
    puts("route43_step_test: host software only; no vehicle/camera/laser/arm physical acceptance");
    return 0;
}
