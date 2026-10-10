/* Real31 router + real XY engine + CRC/ACK/request/QR parser. A host fixture
 * injects camera pixels, encoder endpoints and yaw: NOT physical acceptance. */
/* Historical LEFT20 coverage is an explicit fixture-only build. Ordinary31
 * uses LEFT15 after first180 and goes directly to target after return182;
 * no return lateral or zero-distance job; hostage pregrab body
 * offset remains disabled. No production setting is changed by this override. */
#include "route_test_plan.h"
#if defined(ROUTE31_HOSTAGE_OFFSET_LEGACY_TEST)
#undef ROUTE31_HOSTAGE_PREGRAB_LEFT_MM
#define ROUTE31_HOSTAGE_PREGRAB_LEFT_MM 20u
#endif
#if defined(ROUTE31_BALL_OFFSET_LEGACY_TEST)
#undef ROUTE31_BALL_TO_BUCKET_LEFT_MM
#define ROUTE31_BALL_TO_BUCKET_LEFT_MM 20u
#endif
#define VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN task31_prior_xy_fixture_main
#include "vision_align_bluetooth_test.c"
#undef VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN

static VisionAlignTestStatus task31_status;
static uint32_t task31_racing_last_pulse_ms;
static unsigned task31_pv_override; /* Test-only: dispatch pv while31 is READY, never write an active owner. */
/* Independent test expectation, not the production grip selector:31 uses
 *2100 for both grabs, whereas deferred43 must retain1900. */
static uint16_t task31_grip_expected(void)
{ return s_seq_mode == ROUTE_STEP_MODE ? 1900u : 2100u; }
static unsigned task31_rack_expected(void)
{ return s_seq_mode == ROUTE_STEP_MODE ? 3200u : 3400u; }
static VisionAlignTestState task31_fine_x_state(void)
{ return VAT_Y_ALIGN_ENABLE ? VAT_STEP_MOVE : VAT_FINE_CONTINUOUS; }

/* The inherited road-only stopped() asserts no pulse has EVER occurred.
 * This route now intentionally moves two axes and commands the claw: require
 * no chassis/laser/timer output, not a lifetime-zero servo action count. */
static int task31_stopped(void)
{
    return last_x == 0.0f && last_y == 0.0f && last_w == 0.0f &&
           !laser_state && !s_go && !host_timer_active;
}
#define stopped task31_stopped

static int task31_drive_stopped(void)
{ return last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !laser_state; }

static void task31_snapshot(void)
{ vision_align_test_status(&task31_status); }

static int task31_goal(int model)
{
    return model == 9 ? (s_seq_mode == ROUTE_STEP_MODE ? VAT_ROUTE43_BUCKET_X_PX : VAT_ROUTE_BUCKET_X_PX)
                     : model <= 2 ? VAT_ROUTE_HOSTAGE_X_PX : VAT_ROUTE_BALL_X_PX;
}

static int task31_search_speed_exact(float cruise)
{
    float expected = 700.0f * (float)(uint32_t)(host_tick - s_route_search_from) * 0.001f;
    if (expected > cruise) expected = cruise;
    CHECK(s_route_search_started && fabsf(last_x - expected) < 0.0001f &&
          fabsf(last_y - expected * s_route_search_ff) < 0.0001f);
    return 0;
}

/* Virtual requested-pps events only, not a claim about physical TIM7 timing.
 * Report-task service cannot create steps: every pulse comes from its ISR. */
static int task31_service_ms(unsigned ms)
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
    task31_snapshot();
    return 0;
}
/* A live camera, not synthetic arrival: used only when the test intentionally
 * exercises a longer heading correction rather than the dropout policy. */
static void task31_fresh_images_ms(unsigned ms, int model, int x, int y)
{
    while (ms) {
        unsigned slice = ms < 20u ? ms : 20u;
        host_tick += slice; xy_obj(wire_request, model, x, y); ms -= slice;
    }
    task31_snapshot();
}

static int task31_emit(unsigned count)
{
    int goal = pulse_calls + (int)count;
    unsigned budget = count + 50u;
    CHECK(host_timer_active);
    if (s_route31_lift_phase == R31_RACK_EXTEND || s_route31_lift_phase == R31_RACK_RETRACT ||
        s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND || s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT) {
        CHECK(host_timer_pps == 5000u && host_jog_axis == 0);
        CHECK(host_jog_dir == (s_route31_lift_phase == R31_RACK_RETRACT ||
                              s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT ? 1 : 0));
    }
    else CHECK(host_timer_pps == 10000u && host_jog_axis == 1);
    while (pulse_calls < goal && budget--) CHECK(jog_host_timer_event() == 0);
    CHECK(pulse_calls == goal && task31_drive_stopped());
    return 0;
}

static int task31_ball_down_done(void)
{
    /* Rack was already deployed before LEFT90; aligning a ball cannot emit
     * another extension or invent a mechanical origin. */
    CHECK(s_route31_rack_deployed && host_jog_pulses[0][0] == 3400u && !host_jog_pulses[0][1]);
    CHECK(s_route31_lift_phase == R31_LIFT_BALL_DOWN && host_jog_dir == 1);
    CHECK(task31_emit(32000u) == 0);
    CHECK(!host_timer_active && host_jog_pulses[1][1] == 32000u && !host_jog_pulses[1][0]);
    int prior_servo_calls = servo_calls;
    wire_poll();
    CHECK(s_route31_lift_phase == R31_LIFT_BALL_WAIT && s_route31_lift_wait_t0 == host_tick);
    CHECK(host_servo == ROUTE31_BALL_GRIP_US && servo_calls == prior_servo_calls + 1);
    CHECK(s_seq_state == SQ_TASK && !s_seq_pair_turn && stopped());
    return 0;
}

static int task31_ball_up_begin(void)
{
    int before = pulse_calls;
    CHECK(task31_service_ms(1999u) == 0);
    CHECK(s_route31_lift_phase == R31_LIFT_BALL_WAIT && pulse_calls == before && stopped() && host_servo == ROUTE31_BALL_GRIP_US);
    CHECK(task31_service_ms(1u) == 0);
    CHECK(s_route31_lift_phase == R31_LIFT_BALL_UP && host_timer_active &&
          host_timer_pps == 10000u && host_jog_axis == 1 && host_jog_dir == 0 && pulse_calls == before);
    CHECK(s_jog_pps == 333u && !s_seq_pair_turn && s_seq_state == SQ_TASK);
    return 0;
}

static int task31_ball_up_done(void)
{
    CHECK(task31_emit(32000u) == 0 && !host_timer_active);
    CHECK(host_jog_pulses[1][1] == 32000u && host_jog_pulses[1][0] == 32000u);
    wire_poll();
    CHECK(s_route31_lift_phase == R31_LIFT_OFF && s_seq_pair_turn && s_seq_state == SQ_STILL);
    CHECK(s_seq_stage == ROUTE31_PAIR_STAGE && s_jog_pps == 333u);
    CHECK(host_servo == ROUTE31_BALL_GRIP_US && host_jog_pulses[0][0] == 3400u && !host_jog_pulses[0][1]);
    return 0;
}

static int task31_bucket_down_done(void)
{
    CHECK(s_route31_lift_phase == R31_LIFT_BUCKET_DOWN && host_jog_axis == 1 && host_jog_dir == 1);
    CHECK(host_servo == task31_grip_expected() && host_timer_active && s_route31_rack_deployed);
    int servos = servo_calls;
    CHECK(task31_emit(19999u) == 0 && host_timer_active && s_jog_clock.remaining == 1u);
    wire_poll(); task31_snapshot();
    CHECK(s_route31_lift_phase == R31_LIFT_BUCKET_DOWN && host_servo == task31_grip_expected() && servo_calls == servos);
    CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_PAIR_STAGE && !s_seq_pair_turn &&
          task31_status.state == VAT_WAIT_BUCKET_ACTION && task31_drive_stopped());
    CHECK(task31_emit(1u) == 0 && !host_timer_active && host_servo == task31_grip_expected() && servo_calls == servos);
    uint32_t release_ms = host_tick;
    wire_poll(); task31_snapshot();
    CHECK(s_route31_lift_phase == R31_CLAW_RELEASE_WAIT && s_route31_lift_wait_t0 == release_ms);
    CHECK(host_servo == 1150u && servo_calls == servos + 1 && stopped());
    CHECK(host_jog_pulses[1][1] == 52000u && host_jog_pulses[1][0] == 32000u);
    CHECK(task31_status.state == VAT_WAIT_BUCKET_ACTION && s_seq_stage == ROUTE31_PAIR_STAGE);
    return 0;
}

static int task31_bucket_up_done(void)
{
    if (s_route31_lift_phase == R31_LIFT_BUCKET_DOWN) CHECK(task31_bucket_down_done() == 0);
    if (s_route31_lift_phase == R31_CLAW_RELEASE_WAIT) {
        CHECK(host_servo == ARM_SERVO_START_US && stopped());
        uint32_t release_t0 = s_route31_lift_wait_t0;
        int before_servo = servo_calls, before_steps = pulse_calls;
        CHECK(task31_service_ms(999u) == 0 && s_route31_lift_phase == R31_CLAW_RELEASE_WAIT && stopped());
        CHECK(host_servo == 1150u && servo_calls == before_servo && pulse_calls == before_steps);
        CHECK(task31_service_ms(1u) == 0 && s_route31_lift_phase == R31_CLAW_RESTAGED_WAIT && stopped());
        CHECK(host_servo == 1500u && servo_calls == before_servo + 1 &&
              s_route31_lift_wait_t0 == release_t0 && pulse_calls == before_steps);
    }
    if (s_route31_lift_phase == R31_CLAW_RESTAGED_WAIT) {
        CHECK(host_servo == 1500u && stopped());
        CHECK(task31_service_ms(999u) == 0 && s_route31_lift_phase == R31_CLAW_RESTAGED_WAIT && stopped());
        CHECK(task31_service_ms(1u) == 0 && s_route31_lift_phase == R31_LIFT_BUCKET_UP && host_timer_active);
    }
    CHECK(s_route31_lift_phase == R31_LIFT_BUCKET_UP && host_jog_dir == 0);
    CHECK(task31_emit(20000u) == 0 && !host_timer_active);
    CHECK(host_jog_pulses[1][1] == 52000u && host_jog_pulses[1][0] == 52000u);
    int before = pulse_calls;
    wire_poll(); /* No observation dwell after the last upward pulse. */
    CHECK(s_route31_lift_phase == R31_RACK_RETRACT && host_timer_active && host_jog_dir == 1 && host_servo == 1500u);
    CHECK(task31_emit(3400u) == 0 && !host_timer_active);
    uint32_t rack_end = host_tick;
    wire_poll();
    CHECK(s_route31_lift_phase == R31_RACK_RETRACT_WAIT && s_route31_lift_wait_t0 == rack_end);
    CHECK(task31_service_ms(249u) == 0 && s_route31_lift_phase == R31_RACK_RETRACT_WAIT && stopped());
    CHECK(task31_service_ms(1u) == 0);
    CHECK(s_route31_lift_phase == R31_LIFT_OFF && s_seq_state == SQ_STILL &&
          s_seq_stage == ROUTE31_RETURN180_STAGE && s_msel == 22 && pulse_calls == before + 3400 && stopped());
    CHECK(s_jog_pps == 333u && host_jog_pulses[0][0] == 3400u && host_jog_pulses[0][1] == 3400u);
    CHECK(host_servo == 1500u && servo_calls == 3);
    return 0;
}

static int task31_boot_gain(const char *qr, float gain)
{
    char command[24];
    CHECK(wire_boot() == 0);
    host_wire_diag_hook = real_proto_wire_diag_get;
    xy_seq = 1u;
    /* Deliberately different independent42 slots cannot affect31's fixed
     * rack/lift/grip sequence or become its return origin. */
    run_cmd("42"); run_cmd("e3"); run_cmd("h4"); run_cmd("u2200"); run_cmd("f777");
    run_cmd("27"); run_cmd("f333"); run_cmd("nr7");
    run_cmd("31"); wire_sync();
    if (task31_pv_override) {
        snprintf(command,sizeof command,"pv%u",task31_pv_override);run_cmd(command);
        CHECK(s_route31_lateral_v==(float)task31_pv_override);
    }
    snprintf(command,sizeof command,"ykp%.3f",gain);run_cmd(command);
    CHECK(s_seq_state == SQ_READY && stopped());
    wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, qr);
    CHECK(proto_qr_get(NULL) && notice_calls == 1u && stopped());
    run_cmd("g"); CHECK(s_seq_state == SQ_STILL && stopped());
    CHECK(s_grab42_extend == 3u && s_grab42_down == 4u && s_grab42_u == 2200u && s_grab42_pps == 777u);
    return 0;
}
static int task31_boot(const char *qr) { return task31_boot_gain(qr,0.3f); }

static int task31_finish_motion(void)
{
    int pair_turn = s_seq_pair_turn;
    int return_turn = route31_owner() && s_seq_stage == ROUTE31_RETURN180_STAGE && !s_seq_return_offset;
    int board_contact = route31_owner() && s_seq_stage == 4u;
    uint16_t prior_request = wire_request;
    int pulses = pulse_calls, servos = servo_calls;
    CHECK(sequence_start_stage() == 0);
    float prior_dist_target = s_dist_target;
    unsigned prior_precise = host_precise_calls, prior_integer = host_integer_calls;
    if (board_contact) {
        CHECK(s_board_contact.active && !s_board_contact.hits && s_msel == 15 &&
              route_seq_leg()->distance_mm == 0u && s_d == -1.0f && s_dist_target == 0.0f &&
              s_v == 40.0f && last_x == 40.0f && last_y == 0.0f && last_w == 0.0f &&
              s_dist_heading_kp == 0.0f && s_dist_ff_ratio == 0.0f);
        host_messages[0] = '\0'; /* Capture this sensor outcome, not a full earlier route log. */
    }
    if (!dist_mode()) host_absolute_yaw += turn_target_deg();
    CHECK(sequence_finish_stage() == 0);
    if (board_contact)
        CHECK(!s_board_contact.active && !s_board_contact.zeroed && !s_board_contact.hits &&
              s_dist_reason == 1u && strstr(host_messages, "result=TILT_REACHED") &&
              strstr(host_messages, s_seq_mode == ROUTE_STEP_MODE ? "threshold=1.50 hits=2" : "threshold=1.00 hits=2") && !host_timer_active);
    if (board_contact && s_seq_mode == ROUTE_TEST_MODE)
        CHECK(s_seq_stage == 5u && !s_seq_contact_post && s_seq_state == SQ_STILL && s_msel == 16);
    wire_sync();
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !laser_state);
    if (pair_turn) {
        task31_snapshot();
        unsigned offset_mm = s_seq_mode == ROUTE_STEP_MODE ? 40u : ROUTE31_BALL_TO_BUCKET_LEFT_MM;
        if (offset_mm == 0u) {
            CHECK(s_seq_mode == ROUTE_TEST_MODE && !s_seq_pair_offset && !s_seq_pair_turn &&
                  s_seq_state == SQ_TASK && s_msel == 31 && s_round == R_RUN &&
                  wire_request > prior_request && wire_task == 4u &&
                  task31_status.state == VAT_BRAKE && !task31_status.alignment_confirmed &&
                  !task31_status.good && !task31_status.latest && !host_timer_active);
        } else {
        CHECK(!s_seq_pair_turn && s_seq_pair_offset && s_seq_stage == ROUTE31_PAIR_STAGE &&
              route_seq_leg()->mode == 17u && route_seq_leg()->distance_mm == offset_mm &&
              route_seq_leg()->heading_hold == 1u);
        CHECK(task31_status.state == VAT_TURN_ACTIVE && wire_request == prior_request &&
              pulse_calls == pulses && servo_calls == servos && host_servo == task31_grip_expected());
        if (s_seq_mode == ROUTE_STEP_MODE) {
            /*43 retains its legacy separately gated 40mm offset. */
            CHECK(s_seq_state == SQ_STEP_WAIT);
            run_cmd("g");
        } else CHECK(s_seq_state == SQ_STILL);
        CHECK(sequence_start_stage() == 0 && s_msel == 17 && s_dist_target == -(float)offset_mm &&
              s_dist_heading_kp == 0.3f && s_dist_align_enabled && last_y < 0.0f && s_seq_pair_offset);
        CHECK(sequence_finish_stage() == 0);
        if (s_seq_mode == ROUTE_STEP_MODE) run_cmd("g");
        wire_sync(); task31_snapshot();
        CHECK(!s_seq_pair_offset && s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_PAIR_STAGE &&
              !s_seq_pair_turn && wire_request > prior_request && wire_task == 4u &&
              task31_status.state == VAT_BRAKE && !task31_status.alignment_confirmed);
        CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == task31_grip_expected() && stopped());
        }
        CHECK(s_seq_stage == ROUTE31_PAIR_STAGE && !s_seq_pair_turn && !s_seq_pair_offset &&
              pulse_calls == pulses && servo_calls == servos && host_servo == task31_grip_expected() && stopped());
    }
    if (return_turn) {
        const int deferred43 = s_seq_mode == ROUTE_STEP_MODE;
        if (deferred43) {
        CHECK(s_seq_return_offset && s_seq_stage == ROUTE31_RETURN180_STAGE &&
              s_seq_state == SQ_STEP_WAIT && route_seq_leg()->mode == 17u &&
              route_seq_leg()->distance_mm == 20u && wire_request == prior_request);
        run_cmd("g"); /*43 retains both fresh-g gates and its LEFT20. */
        CHECK(sequence_start_stage() == 0 && s_msel == 17 &&
              s_dist_target == -20.0f && s_dist_align_enabled &&
              last_y < 0.0f && s_seq_return_offset);
        CHECK(sequence_finish_stage() == 0);
        run_cmd("g");
        wire_sync();
        } else {
            CHECK(ROUTE31_RETURN_RIGHT_MM == 0u && !s_seq_return_offset &&
                  s_msel == 31 && s_round == R_RUN && !dist_mode() &&
                  s_dist_target == prior_dist_target &&
                  host_precise_calls == prior_precise && host_integer_calls == prior_integer);
        }
        CHECK(!s_seq_return_offset && s_seq_stage == ROUTE31_TARGET_STAGE &&
              s_seq_state == SQ_TASK && s_target35_route && wire_task == 2u &&
              wire_request > prior_request && stopped());
        CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == 1500u);
    }
    return 0;
}

static void task31_latch_and_prepare(unsigned stage, const char *qr)
{
    for (unsigned i = 0u; i < 3u; ++i) s_seq_qr[i] = qr[i] - '0';
    step_vision_receive_end();
    s_seq_stage = (uint8_t)stage; route_seq_prepare(); wire_sync();
}

static int task31_predeploy_start(void)
{
    CHECK(task31_boot("123") == 0);
    task31_latch_and_prepare(ROUTE31_PREDEPLOY_STAGE, "123");
    CHECK(s_seq_stage == ROUTE31_PREDEPLOY_STAGE && s_seq_state == SQ_ARM_PREP && s_msel == 31);
    CHECK(s_route31_lift_phase == R31_RACK_EXTEND && !s_route31_rack_deployed && host_timer_active);
    CHECK(host_jog_axis == 0 && host_jog_dir == 0 && host_timer_pps == 5000u && !pulse_calls);
    CHECK(task31_drive_stopped() && !servo_calls && host_servo == ARM_SERVO_START_US);
    return 0;
}

static int task31_predeploy_done(void)
{
    CHECK(s_seq_state == SQ_ARM_PREP && s_route31_lift_phase == R31_RACK_EXTEND);
    unsigned commands = wire_commands;
    CHECK(task31_emit(3400u) == 0 && !host_timer_active);
    uint32_t end = host_tick;
    wire_poll();
    CHECK(s_seq_state == SQ_ARM_PREP && s_route31_lift_phase == R31_RACK_EXTEND_WAIT);
    CHECK(s_route31_lift_wait_t0 == end && !s_route31_rack_deployed);
    CHECK(task31_service_ms(249u) == 0 && s_route31_lift_phase == R31_RACK_EXTEND_WAIT && stopped());
    CHECK(task31_service_ms(1u) == 0);
    CHECK(s_route31_rack_deployed && s_route31_lift_phase == R31_LIFT_OFF);
    CHECK(s_seq_stage == ROUTE31_PREDEPLOY_STAGE && s_seq_state == SQ_STILL && s_msel == 30);
    CHECK(host_jog_pulses[0][0] == 3400u && !host_jog_pulses[0][1]);
    CHECK(stopped() && !servo_calls && host_servo == ARM_SERVO_START_US && wire_commands == commands);
    return 0;
}

/* Start a task directly only for cancellation/mapping tests. Full traversal
 * below reaches the same state through all real road actions and R1 gate. */
static int task31_at_gain(unsigned stage, const char *qr, float gain)
{
    CHECK(task31_boot_gain(qr,gain) == 0);
    if (stage == ROUTE31_PAIR_STAGE) {
        /* Synthetic PAIR entry still performs real predeployment before its
         * LEFT90. Otherwise the fixture would conceal RACK_NOT_DEPLOYED. */
        task31_latch_and_prepare(ROUTE31_PREDEPLOY_STAGE, qr);
        CHECK(task31_predeploy_done() == 0);
        CHECK(task31_finish_motion() == 0);
    } else task31_latch_and_prepare(stage, qr);
    if (stage == ROUTE31_HOSTAGE_STAGE) {
        /* Synthetic direct task entry models the preceding bucket's already
         * issued1500 command. Production MUST NOT add1500 at RIGHT90. */
        host_servo = 1500u;
    }
    CHECK(!proto_qr_get(NULL));
    CHECK(s_seq_state == SQ_TASK && s_msel == 31 && stopped());
    return 0;
}
static int task31_at(unsigned stage, const char *qr) { return task31_at_gain(stage,qr,0.3f); }

/* Synthetic first-seen order, independent of QR shape/color and cx/cy.
 * Use the real request-bound54 parser, not a direct route cache assignment. */
static void task31_rank(uint16_t request, uint8_t model, uint8_t rank)
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

/* A pre-grab offset is a road action, never an arm job or an action ACK.
 * Complete the real executor; VAT must discard the old arrival frames and
 * preserve the request/rank before the caller supplies NEW coordinates. */
static int task31_hostage_offset_resume(void)
{
    uint16_t request = wire_request;
    unsigned commands = wire_commands;
    int pulses = pulse_calls, servos = servo_calls, zeros = zero_calls;
    uint16_t held = host_servo;
    CHECK(ROUTE31_HOSTAGE_PREGRAB_LEFT_MM == 20u &&
          ROUTE31_BALL_TO_BUCKET_LEFT_MM ==
#if defined(ROUTE31_BALL_OFFSET_LEGACY_TEST)
              20u &&
#else
              15u &&
#endif
          ROUTE31_PAIR_LEFT_MM == 40u);
    CHECK(s_seq_hostage_offset && s_route31_hostage_offset_done &&
          s_seq_state == SQ_STILL && s_seq_stage == ROUTE31_HOSTAGE_STAGE &&
          route_seq_leg()->mode == 17u && route_seq_leg()->distance_mm == 20u &&
          route_seq_leg()->heading_hold && !host_timer_active);
    CHECK(sequence_start_stage() == 0 && s_msel == 17 && s_dist_target == -20.0f &&
          last_y < 0.0f && s_dist_align_enabled && zero_calls == zeros);
    CHECK(sequence_finish_stage() == 0); wire_sync(); task31_snapshot();
    CHECK(s_seq_state == SQ_TASK && !s_seq_hostage_offset && s_route31_hostage_offset_done &&
          s_seq_stage == ROUTE31_HOSTAGE_STAGE && !host_timer_active &&
          s_route31_lift_phase == R31_LIFT_OFF && !task31_status.alignment_confirmed &&
          !task31_status.good && !task31_status.latest && task31_drive_stopped());
    CHECK(wire_request == request && wire_commands == commands &&
          pulse_calls == pulses && servo_calls == servos && host_servo == held && zero_calls == zeros + 1);
    return 0;
}

static int task31_hostage_reconfirm(int model, int y)
{
    if (ROUTE31_HOSTAGE_PREGRAB_LEFT_MM == 0u) {
        (void)model; (void)y;
        task31_snapshot();
        CHECK(!s_seq_hostage_offset && !s_route31_hostage_offset_done &&
              s_seq_state == SQ_TASK && s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND &&
              host_timer_active && task31_status.alignment_confirmed && task31_status.good == 5u &&
              task31_drive_stopped());
        return 0;
    }
    CHECK(task31_hostage_offset_resume() == 0);
    for (unsigned frame = 0u; frame < 80u && s_route31_lift_phase == R31_LIFT_OFF; ++frame) {
        host_tick += 20u; xy_obj(wire_request, model, VAT_ROUTE_HOSTAGE_X_PX, y);
    }
    task31_snapshot();
    CHECK(!s_seq_hostage_offset && s_route31_hostage_offset_done &&
          s_seq_state == SQ_TASK && s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND &&
          host_timer_active && task31_status.alignment_confirmed && task31_status.good == 5u);
    return 0;
}

static int task31_hostage_start(uint8_t rank)
{
    CHECK(task31_at(ROUTE31_HOSTAGE_STAGE, "123") == 0);
    wire_ack(wire_request, 2u, 0u); xy_advance(300u);
    task31_rank(wire_request, 0u, rank);
    for (unsigned i = 0u; i < 120u && s_route31_lift_phase == R31_LIFT_OFF; ++i) {
        host_tick += 20u; xy_obj(wire_request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
        if (s_seq_hostage_offset) CHECK(task31_hostage_reconfirm(0, VAT_HOSTAGE_Y_PX) == 0);
    }
    task31_snapshot();
    CHECK(task31_status.state == VAT_WAIT_HOSTAGE_ACTION && task31_status.alignment_confirmed &&
          !task31_status.hostage_fallback && task31_status.good == 5u);
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND && host_timer_active &&
          host_jog_axis == 0 && host_jog_dir == 0 && host_timer_pps == 5000u && !pulse_calls);
    CHECK(host_servo == 1150u && servo_calls == 1 && s_route31_hostage_opened && task31_drive_stopped());
    return 0;
}

static int task31_hostage_extend_done(void)
{
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND && host_timer_active);
    CHECK(host_servo == 1150u && s_route31_hostage_opened && task31_drive_stopped());
    int before = pulse_calls, servos = servo_calls;
    unsigned down = host_jog_pulses[1][1], up = host_jog_pulses[1][0];
    CHECK(task31_emit(task31_rack_expected() - 1u) == 0 && host_timer_active && s_jog_clock.remaining == 1u);
    wire_poll();
    CHECK(host_servo == 1150u && servo_calls == servos && s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND);
    CHECK(task31_emit(1u) == 0 && !host_timer_active); wire_poll();
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND_WAIT && host_servo == 1150u);
    CHECK(task31_service_ms(249u) == 0 && s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND_WAIT && servo_calls == servos);
    CHECK(task31_service_ms(1u) == 0 && s_route31_lift_phase == R31_HOSTAGE_GRIP_WAIT);
    CHECK(host_servo == task31_grip_expected() && servo_calls == servos + 1 &&
          pulse_calls == before + (int)task31_rack_expected() && stopped());
    CHECK(host_jog_pulses[1][1] == down && host_jog_pulses[1][0] == up);
    CHECK(task31_status.state == VAT_WAIT_HOSTAGE_ACTION && !s_route31_hostage_hold && s_receiving);
    CHECK(s_route31_lift_wait_t0 == host_tick);
    return 0;
}

static int task31_hostage_retract_begin(void)
{
    CHECK(s_route31_lift_phase == R31_HOSTAGE_GRIP_WAIT && host_servo == task31_grip_expected());
    int pulses = pulse_calls, servos = servo_calls;
    CHECK(task31_service_ms(249u) == 0 && s_route31_lift_phase == R31_HOSTAGE_GRIP_WAIT && stopped());
    CHECK(task31_status.state == VAT_WAIT_HOSTAGE_ACTION && pulse_calls == pulses && servo_calls == servos);
    CHECK(task31_service_ms(1u) == 0 && s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT && host_timer_active);
    CHECK(host_jog_axis == 0 && host_jog_dir == 1 && host_timer_pps == 5000u && s_jog_clock.remaining == 3000u);
    CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == task31_grip_expected() && task31_drive_stopped());
    return 0;
}

static int task31_hostage_retract_done(void)
{
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT && host_timer_active && host_servo == task31_grip_expected());
    int pulses = pulse_calls, servos = servo_calls;
    CHECK(task31_emit(2999u) == 0 && host_timer_active && s_jog_clock.remaining == 1u);
    wire_poll(); task31_snapshot();
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT && task31_status.state == VAT_WAIT_HOSTAGE_ACTION &&
          !s_route31_hostage_hold && s_receiving && servo_calls == servos);
    CHECK(task31_emit(1u) == 0 && !host_timer_active); wire_poll();
    uint32_t last_pulse_ms = host_tick;
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT_WAIT && s_route31_lift_wait_t0 == last_pulse_ms);
    CHECK(task31_service_ms(249u) == 0 && s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT_WAIT &&
          task31_status.state == VAT_WAIT_HOSTAGE_ACTION && !s_route31_hostage_hold && s_receiving);
    CHECK(task31_service_ms(1u) == 0 && s_route31_lift_phase == R31_LIFT_OFF && s_route31_rack_deployed);
    CHECK(pulse_calls == pulses + 3000 && servo_calls == servos && host_servo == task31_grip_expected() && stopped());
    CHECK(task31_status.state == VAT_DONE || task31_status.state == VAT_WAIT_HOSTAGE_RANK);
    return 0;
}

static int task31_hostage_grab_done(void)
{
    unsigned down = host_jog_pulses[1][1], up = host_jog_pulses[1][0];
    CHECK(task31_hostage_extend_done() == 0 && task31_hostage_retract_begin() == 0 && task31_hostage_retract_done() == 0);
    CHECK(host_jog_pulses[1][1] == down && host_jog_pulses[1][0] == up); /* No hostage down/up. */
    return 0;
}

static int task31_aim(int model, int y, VisionAlignTestState expected)
{
    int before = pulse_calls;
    int before_servo = servo_calls;
    uint16_t before_u = host_servo;
    const int goal = task31_goal(model);
    task31_snapshot(); CHECK(task31_status.x_goal == goal);
    wire_ack(wire_request, 2u, 0u); xy_advance(260u);
    if (model <= 2) {
        ProtoTargetRank rank;
        if (!proto_target_rank_get(&rank) || !rank.rank)
            task31_rank(wire_request, (uint8_t)model, 2u);
    } else if (model >= 3 && model <= 5) {
        ProtoTargetRank rank;
        if (!proto_target_rank_get(&rank) || !rank.rank)
            task31_rank(wire_request, (uint8_t)model, 1u);
    }
    for (unsigned i = 0u; i < 120u; ++i) {
        host_tick += 20u; xy_obj(wire_request, model, goal, y);
        if (model <= 2 && s_seq_hostage_offset) CHECK(task31_hostage_reconfirm(model, y) == 0);
        task31_snapshot();
        CHECK(task31_status.x_goal == goal);
        CHECK(last_y == 0.0f); /* No lateral command on these deliberately X-only arrival samples. */
#if !VAT_Y_ALIGN_ENABLE
        CHECK(task31_status.state != VAT_YAW_FIX); /* No injected search-heading error in these arrival loops. */
#endif
        if (task31_status.state == expected || s_seq_state == SQ_DONE ||
            (model <= 2 && s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND)) break;
    }
    if (model <= 2 && expected == VAT_DONE) CHECK(task31_hostage_grab_done() == 0);
    task31_snapshot();
    CHECK(task31_status.state == expected && task31_drive_stopped());
    if (expected == VAT_DONE)
        CHECK(s_seq_state == SQ_TASK && s_route31_hostage_hold && s_route31_hostage_rank == task31_status.target_rank);
    CHECK(s_receiving == (model >= 3 && model <= 5) && !s_due && pulse_calls == before + (model <= 2 ? 6400 : 0));
    if (expected == VAT_WAIT_BUCKET_ACTION) {
        CHECK(s_route31_lift_phase == R31_LIFT_BUCKET_DOWN && host_timer_active && task31_drive_stopped());
        CHECK(host_jog_axis == 1 && host_jog_dir == 1 && host_timer_pps == 10000u);
        CHECK(host_servo == task31_grip_expected() && servo_calls == before_servo && host_servo == before_u);
    } else if (expected == VAT_DONE) {
        CHECK(task31_status.alignment_confirmed && !task31_status.hostage_fallback && s_route31_hostage_opened && host_servo == task31_grip_expected());
        CHECK(servo_calls == before_servo + 2);
    } else CHECK(servo_calls == before_servo && host_servo == before_u);
    return 0;
}

static int task31_target_fire(int model)
{
    CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_TARGET_STAGE);
    CHECK(s_target35_route && wire_task == 2u && wire_digit == s_seq_qr[1] &&
          target35_point() == 250 && target35_low() == 247 && target35_high() == 253 &&
          target35_fine_enter() == 360);
    wire_ack(wire_request, 2u, 0u); xy_advance(1100u);
    CHECK(s_target35_phase == TA_SEEK && last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO) < 0.0001f &&
          !laser_state && !s_target35_seen && !s_target31_fine);
    /* Legal but far selected frames must not turn coarse approach into fine.
     * This is a one-way X360 gate, not the previous first-sighting latch. */
    host_tick += 20u; xy_obj(wire_request, model, 400, 10);
    CHECK(last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO) < 0.0001f && s_target35_seen && !s_target31_fine);
    host_tick += 20u; xy_obj(wire_request, model, 361, 10);
    CHECK(last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO) < 0.0001f && !s_target31_fine);
    xy_advance(T_TARGET35_FRESH_MS + 20u);
    CHECK(last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO) < 0.0001f && !s_target31_fine);
    host_tick += 20u; xy_obj(wire_request, model, 360, 10);
    CHECK(last_x == 30.0f && last_y == 0.0f && !laser_state && s_target35_seen && s_target31_fine);
    for (unsigned i = 0u; i < 5u; ++i) {
        host_tick += 20u; xy_obj(wire_request, model, 360, 10);
        CHECK(last_x == 30.0f && last_y == 0.0f && s_target35_phase == TA_SEEK &&
              !s_target35_good && !laser_state); /* Speed gate360 is never the247..253 route aim gate. */
    }
    host_tick += 20u; xy_obj(wire_request, model, 400, 10);
    CHECK(last_x == 30.0f && last_y == 0.0f && s_target31_fine);
    xy_advance(T_TARGET35_FRESH_MS + 20u);
    CHECK(last_x == 30.0f && last_y == 0.0f && s_target35_seen && s_target31_fine && !laser_state);
    host_tick += 20u; xy_obj(wire_request, model, 254, 10);
    CHECK(last_x == 30.0f && last_y == 0.0f && !laser_state && !s_target35_good);
    host_tick += 20u; xy_obj(wire_request, model, 246, 10);
    CHECK(last_x == -30.0f && last_y == 0.0f && !laser_state);
    /* Only x matters for laser: deliberately non-grasping cy=10. */
    for (unsigned i = 0u; i < 5u; ++i) {
        host_tick += 20u; xy_obj(wire_request, model, i & 1u ? 247 : 253, 10);
        if (i < 4u) CHECK(s_target35_phase == TA_SEEK && !laser_state);
    }
    CHECK(s_target35_phase == TA_FIRE && laser_state == 1 && !s_go && !host_timer_active);
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !s_receiving && !s_due);
    return 0;
}

static int task31_target_finish_yaw(void)
{
#if ROUTE31_TARGET_STOP_YAW_ENABLE
    CHECK(s_seq_state == SQ_TASK_YAW && s_seq_stage == ROUTE31_TARGET_STAGE && !laser_state);
    int zeros = zero_calls;
    host_yaw = s_route31_task_yaw_goal;
    wire_poll();
    CHECK(s_seq_state == SQ_TASK_YAW && s_route31_task_yaw_stable && zero_calls == zeros);
    xy_advance(ROUTE31_STABLE_MS);
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
          zero_calls == zeros && stopped());
#else
    /* Current31 does not command a post-laser stationary yaw turn. Ordinary
     * next-road preparation retains its existing stopped baseline handoff. */
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
          !s_target35_route && !laser_state && stopped());
#endif
    return 0;
}

static int check_full_qr_ball_bucket_target_hostage_chain(void)
{
    const int ball_y = VAT_Y_ALIGN_ENABLE ? VAT_BALL_Y_PX : 80;
    const int bucket_y = VAT_Y_ALIGN_ENABLE ? VAT_BUCKET_Y_PX : 80;
    const int hostage_y = VAT_Y_ALIGN_ENABLE ? VAT_HOSTAGE_Y_PX : 80;
    CHECK(task31_boot("123") == 0);
    CHECK(ROUTE31_RACK_EXTEND_STEPS == 3400u && ROUTE31_HOSTAGE_RACK_EXTEND_STEPS == 3400u);
    CHECK(ROUTE31_LIFT_DOWN_STEPS == 32000u && ROUTE31_LIFT_BALL_UP_STEPS == 32000u);
    CHECK(ROUTE31_LIFT_BUCKET_DOWN_STEPS == 20000u && ROUTE31_LIFT_BUCKET_UP_STEPS == 20000u);
    CHECK(ROUTE31_BALL_GRIP_WAIT_MS == 2000u && ROUTE31_BUCKET_RELEASE_WAIT_MS == 2000u &&
          ROUTE31_BUCKET_LIFT_WAIT_MS == 0u && ROUTE31_HOSTAGE_HOLD_MS == 0u &&
          ROUTE31_RETURN_LEFT_MM == 20u);
    CHECK(ROUTE31_BALL_GRIP_US == 2100u && ROUTE31_HOSTAGE_GRIP_US == 2100u &&
          ROUTE43_BALL_GRIP_US == 1900u && ROUTE43_HOSTAGE_GRIP_US == 1900u &&
          ARM_SERVO_GRIP_US == 1700u && ARM_SERVO_START_US == 1150u);
    CHECK(ROUTE31_ENTRY_BACK_MM == 810u && s_route31_plan[7].distance_mm == 810u &&
          s_route31_plan[6].distance_mm == 780u &&
          ROUTE31_PAIR_LEFT_MM == 40u && ROUTE31_BALL_TO_BUCKET_LEFT_MM ==
#if defined(ROUTE31_BALL_OFFSET_LEGACY_TEST)
              20u &&
#else
              15u &&
#endif
          ROUTE31_CLAW_READY_US == 1500u && ROUTE31_BUCKET_RELEASE_READY_MS == 1000u);
    CHECK(R31_LIFT_BALL_DOWN == 1 && R31_LIFT_BALL_WAIT == 2 && R31_LIFT_BALL_UP == 3 &&
          R31_LIFT_BUCKET_UP == 4 && R31_LIFT_BUCKET_WAIT == 5 && R31_RACK_EXTEND == 6 &&
          R31_RACK_EXTEND_WAIT == 7 && R31_CLAW_RELEASE_WAIT == 8 && R31_RACK_RETRACT == 9 &&
          R31_RACK_RETRACT_WAIT == 10 && R31_LIFT_BUCKET_DOWN == 11 && R31_CLAW_RESTAGED_WAIT == 12 &&
          R31_HOSTAGE_RACK_EXTEND == 13 && R31_HOSTAGE_RACK_EXTEND_WAIT == 14 &&
          R31_HOSTAGE_GRIP_WAIT == 15 && R31_HOSTAGE_RACK_RETRACT == 16 && R31_HOSTAGE_RACK_RETRACT_WAIT == 17);
    CHECK(ROUTE31_HOSTAGE_RACK_RETRACT_STEPS == 3000u && ROUTE31_HOSTAGE_GRIP_SETTLE_MS == 250u);
    CHECK(ROUTE31_RACK_EXTEND_DIR == 0 && ROUTE31_RACK_RETRACT_DIR == 1);
    CHECK(route_seq_leg()->distance_mm == 575u);
    CHECK(s_route31_plan[1].distance_mm == 610u && s_route31_plan[3].distance_mm == 620u &&
          ROUTE_CROSS_BACK_MM == 650u && s_route31_plan[4].distance_mm == 0u &&
          s_route31_plan[4].speed_mms == 40.0f && ROUTE31_BOARD_CONTACT_TILT_DEG == 1.0f &&
          ROUTE31_BOARD_CONTACT_FRAMES == 2u && ROUTE31_BOARD_CONTACT_ZERO_WAIT_MS == 1000u &&
          ROUTE31_BOARD_ZERO_WAIT_MS == 300u && ROUTE31_STABLE_MS == 400u &&
          ROUTE31_BOARD_CONTACT_START_GUARD_MS == 300u && ROUTE31_BOARD_CONTACT_CONFIRM_MS == 100u &&
          ROUTE31_BOARD_CONTACT_MAX_MS == 10000u);
    for (unsigned i = 0u; i < ROUTE31_PREDEPLOY_STAGE; ++i) {
        CHECK(s_seq_stage == i && s_seq_state == SQ_STILL);
        if (i == 2u) CHECK(s_msel == 20 && turn_target_deg() == 90.0f);
        CHECK(task31_finish_motion() == 0);
    }
    CHECK(s_seq_stage == ROUTE31_PREDEPLOY_STAGE && s_seq_state == SQ_ARM_PREP);
    CHECK(task31_predeploy_done() == 0);
    CHECK(task31_finish_motion() == 0);
    CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_PAIR_STAGE && s_msel == 31);
    CHECK(s_seq_qr[0] == 1 && s_seq_qr[1] == 2 && s_seq_qr[2] == 3 && !proto_qr_get(NULL));
    CHECK(wire_task == 1u && wire_digit == 1u && !laser_state);
    unsigned ball_request = wire_request;
    /* Coordinate before ACK and coordinates for a superseded request cannot
     * move or count. No blind ball step is permitted before its ACK. */
    xy_obj((uint16_t)ball_request, 4, VAT_ROUTE_BALL_X_PX, VAT_BALL_Y_PX); xy_advance(500u);
    task31_snapshot(); CHECK(task31_status.good == 0u && stopped());
    wire_ack((uint16_t)(ball_request - 1u), 2u, 0u);
    xy_obj((uint16_t)(ball_request - 1u), 4, VAT_ROUTE_BALL_X_PX, VAT_BALL_Y_PX); CHECK(stopped());
    CHECK(task31_aim(4, ball_y, VAT_WAIT_BALL_ACTION) == 0);
#if !VAT_Y_ALIGN_ENABLE
    CHECK(task31_status.cy == ball_y && !task31_status.step && !task31_status.yaw_dirty);
#endif
    CHECK(task31_ball_down_done() == 0);
    CHECK(task31_ball_up_begin() == 0);
    CHECK(task31_ball_up_done() == 0);
    CHECK(s_seq_stage == ROUTE31_PAIR_STAGE && s_seq_pair_turn && s_seq_state == SQ_STILL &&
          turn_target_deg() == 180.0f);
    CHECK(wire_task == 4u && wire_digit == 0u && wire_request > ball_request);
    uint16_t pre_turn_bucket = wire_request;
    wire_ack(pre_turn_bucket, 2u, 0u); xy_obj(pre_turn_bucket, 9, VAT_ROUTE_BUCKET_X_PX, VAT_BUCKET_Y_PX);
    CHECK(task31_finish_motion() == 0);
    CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_PAIR_STAGE && !s_seq_pair_turn && s_msel == 31);
    CHECK(wire_request > pre_turn_bucket && wire_task == 4u);
    task31_snapshot(); CHECK(task31_status.yaw_target == host_absolute_yaw && task31_status.x_goal == VAT_ROUTE_BUCKET_X_PX);
    xy_obj(pre_turn_bucket, 9, VAT_ROUTE_BUCKET_X_PX, VAT_BUCKET_Y_PX); xy_advance(1000u);
    CHECK(s_seq_state == SQ_TASK && stopped());
    wire_ack(wire_request, 2u, 0u); xy_advance(1000u);
    task31_snapshot();
    CHECK(s_seq_state == SQ_TASK && stopped() && task31_status.state == VAT_RECHECK); /* Initial2s bucket wait is stopped. */
    CHECK(task31_aim(9, bucket_y, VAT_WAIT_BUCKET_ACTION) == 0);
#if !VAT_Y_ALIGN_ENABLE
    CHECK(task31_status.cy == bucket_y && !task31_status.step && !task31_status.yaw_dirty);
#endif
    CHECK(task31_bucket_up_done() == 0);
    CHECK(s_seq_stage == ROUTE31_RETURN180_STAGE && s_seq_state == SQ_STILL && s_msel == 22 &&
          turn_target_deg() == 182.0f);
    CHECK(task31_finish_motion() == 0);
    CHECK(s_seq_stage == ROUTE31_TARGET_STAGE && s_seq_state == SQ_TASK && s_msel == 31);
    CHECK(task31_target_fire(8) == 0); /* QR second2 => green target model8 */
    xy_advance(1980u);
    CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_TARGET_STAGE && laser_state == 1);
    xy_advance(20u);
    CHECK(task31_target_finish_yaw() == 0);
    CHECK(s_seq_stage == ROUTE31_TARGET_CORNER_STAGE && s_seq_state == SQ_STILL && stopped());
    CHECK(route_seq_leg()->distance_mm == ROUTE31_GREEN_TO_CORNER_MM);
    CHECK(task31_finish_motion() == 0);
    CHECK(s_seq_stage == ROUTE31_HOSTAGE_TURN_STAGE && s_msel == 20 && turn_target_deg() == 93.0f);
    CHECK(host_servo == 1500u && servo_calls == 3 && !s_route31_hostage_opened);
    CHECK(task31_finish_motion() == 0);
    CHECK(host_servo == 1500u && servo_calls == 3 && !s_route31_hostage_opened);
    CHECK(s_seq_stage == ROUTE31_HOSTAGE_STAGE && s_seq_state == SQ_TASK && s_msel == 31);
    CHECK(wire_task == 3u && wire_digit == 3u && !proto_qr_get(NULL));
    CHECK(task31_aim(0, hostage_y, VAT_DONE) == 0); /* QR third3 => waist model0 */
#if !VAT_Y_ALIGN_ENABLE
    CHECK(task31_status.cy == hostage_y && !task31_status.step && !task31_status.yaw_dirty);
#endif
    CHECK(s_seq_state == SQ_TASK && s_route31_hostage_hold && s_route31_hostage_rank == 2u && stopped());
    wire_poll(); CHECK(s_seq_state == SQ_STILL && stopped()); /* Rank known: no extra2s hold. */
    CHECK(s_seq_stage == ROUTE31_HOSTAGE_EXIT_STAGE && route_seq_leg()->distance_mm == 1315u);
    CHECK(task31_finish_motion() == 0);
    CHECK(s_seq_state == SQ_DONE && s_msel == 31 && s_round == R_DONE && stopped());
    CHECK(!s_receiving && !s_due && !laser_state && pulse_calls == 117200 && servo_calls == 5 && !wire_bad);
    CHECK(host_jog_pulses[0][0] == 6800u && host_jog_pulses[0][1] == 6400u &&
          host_jog_pulses[1][0] == 52000u && host_jog_pulses[1][1] == 52000u && s_route31_rack_deployed);
    CHECK(host_servo == task31_grip_expected());
    CHECK(s_grab42_extend == 3u && s_grab42_down == 4u && s_grab42_u == 2200u && s_grab42_pps == 777u);
    unsigned commands = wire_commands;
    run_cmd("g"); xy_advance(10000u); xy_obj(wire_request, 0, 180, 220);
    CHECK(s_seq_state == SQ_DONE && stopped() && wire_commands == commands);
    printf("31 full hostchain: %s; entryBACK810/preCrossRIGHT90/hostageRIGHT93/predeploy3400/LEFT90/ballX135/down32000/grip2100hold2/up32000/+180/%s/newBucketX105/down20000/release1150/after1s-ready1500/total2s/up20000/rackback3400/+182/stable400/direct_target250_noReturnLateralOrZeroDistance;hostageX215/%s/open1150/rack3400/grip2100/retract3000/held+rankfinal;117200STEP/verticalnet0/racknet400/5servo commands passed\n",
           VAT_Y_ALIGN_ENABLE ? "legacyXY" : "defaultXonly,noYfix",
           ROUTE31_BALL_TO_BUCKET_LEFT_MM == 20u ? "EXPLICIT_FIXTURE_LEFT20_cap80_yawfix" : "CURRENT_LEFT15_cap80_yawfix",
           ROUTE31_HOSTAGE_PREGRAB_LEFT_MM ? "EXPLICIT_FIXTURE_LEFT20_yawfix_NEW_X" : "CURRENT_noBodyOffset");
    return 0;
}

static int check_three_target_distances_and_independent_laser_scope(void)
{
    const char *const qr[] = {"111", "222", "333"};
    const int models[] = {6, 8, 7};
    const unsigned mm[] = {510u, 430u, 350u};
    for (unsigned i = 0u; i < 3u; ++i) {
        CHECK(task31_at(ROUTE31_TARGET_STAGE, qr[i]) == 0);
        CHECK(task31_target_fire(models[i]) == 0); xy_advance(2000u);
        CHECK(task31_target_finish_yaw() == 0);
        CHECK(s_seq_state == SQ_STILL && stopped() && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE);
        CHECK(route_seq_leg()->distance_mm == mm[i]);
        CHECK(sequence_start_stage() == 0 && s_dist_target == mm[i] && s_v == 200.0f);
        CHECK(s_dist_ff_ratio == -ROUTE31_CORNER_RIGHT_FF_RATIO && last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_CORNER_RIGHT_FF_RATIO) < 0.0001f);
        CHECK(!laser_state && !s_receiving && !s_due); run_cmd("g");
    }
    /* Independent35 remains continuously ON, never inherits31's 2s timer. */
    CHECK(wire_boot() == 0); xy_seq = 1u; run_cmd("35"); wire_sync();
    wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "111"); run_cmd("g"); wire_poll();
    wire_ack(wire_request, 2u, 0u); xy_advance(1200u);
    CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && last_y == 0.0f && !s_target35_seen &&
          target35_point() == 255 && target35_low() == 250 && target35_high() == 260);
    host_tick += 20u; xy_obj(wire_request, 6, 400, 10);
    CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && last_y == 0.0f && !s_target31_fine);
    host_tick += 20u; xy_obj(wire_request, 6, 350, 10);
    CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && last_y == 0.0f && !s_target31_fine);
    host_tick += 20u; xy_obj(wire_request, 6, 190, 10);
    CHECK(s_target35_phase == TA_SEEK && last_x == -50.0f && last_y == 0.0f && !s_target35_good && !laser_state);
    for (unsigned i = 0u; i < 70u && !laser_state; ++i) { host_tick += 20u; xy_obj(wire_request, 6, 255, 10); }
    CHECK(laser_state == 1 && !s_target35_route && s_seq_state == SQ_OFF);
    xy_advance(5000u); CHECK(laser_state == 1 && s_target35_phase == TA_FIRE);
    run_cmd("g"); CHECK(stopped() && s_target35_phase == TA_STOPPED);
    return 0;
}

static int check_target31_gate_freshness_and_burst(void)
{
    for (unsigned burst = 0u; burst < 4u; ++burst) {
        CHECK(task31_at(ROUTE31_TARGET_STAGE, "123") == 0);
        wire_ack(wire_request, 2u, 0u); xy_advance(1100u);
        CHECK(s_target35_phase == TA_SEEK && last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO) < 0.0001f && !s_target31_fine);
        if (burst < 2u) {
            uint16_t far_seq = (uint16_t)xy_seq;
            host_tick += 20u; xy_obj(wire_request, 8, 400, 10);
            CHECK(s_target31_coarse_have && s_target31_coarse_sequence == far_seq);
            CHECK(!s_target31_fine && last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO) < 0.0001f);
            unsigned next_seq = xy_seq;
            xy_seq = far_seq; host_tick += 20u; xy_obj(wire_request, 8, 360, 10);
            CHECK(!s_target31_fine && last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO) < 0.0001f); /* Duplicate cannot cross the gate. */
            xy_seq = (uint16_t)(far_seq - 1u); host_tick += 20u; xy_obj(wire_request, 8, 359, 10);
            CHECK(!s_target31_fine && last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO) < 0.0001f); /* Backwards sequence cannot cross. */
            xy_seq = next_seq;
            host_tick += 20u; xy_obj((uint16_t)(wire_request - 1u), 8, 360, 10);
            host_tick += 20u; xy_obj(wire_request, 6, 360, 10);
            CHECK(!s_target31_fine && last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO) < 0.0001f); /* Old request / wrong QR color. */
        }
        host_tick += 20u;
        xy_obj_poll(wire_request, 8, 360, 10, 0);
        if (burst & 1u) xy_obj_poll(wire_request, 8, 400, 10, 0);
        else xy_obj_poll(wire_request, -1, 0, 0, 0);
        wire_poll();
        CHECK(s_target31_fine && s_target35_phase == TA_SEEK && last_x == 30.0f && last_y == 0.0f);
        xy_advance(T_TARGET35_FRESH_MS + 20u);
        CHECK(s_target31_fine && last_x == 30.0f && last_y == 0.0f);
        run_cmd("0"); CHECK(stopped() && s_seq_state == SQ_STOPPED);
    }
    puts("target31 X360gate:duplicate/backwardseq/oldrequest/wrongcolor cannot latch; valid360 followed by empty/far400 beforepoll still locks fine30/noFF permanently passed");
    return 0;
}

/* Every mechanical phase is reached through real QR/selected target frames,
 * actual owner polls and TIM7 IRQs, never by assigning the phase enum. */
static int task31_mechanical_phase(unsigned desired)
{
    if (desired >= R31_HOSTAGE_RACK_EXTEND && desired <= R31_HOSTAGE_RACK_RETRACT_WAIT) {
        CHECK(task31_hostage_start(2u) == 0);
        unsigned budget = 6u;
        while (s_route31_lift_phase != desired && budget--) {
            switch (s_route31_lift_phase) {
            case R31_HOSTAGE_RACK_EXTEND: CHECK(task31_emit(3400u) == 0); wire_poll(); break;
            case R31_HOSTAGE_RACK_EXTEND_WAIT: CHECK(task31_service_ms(250u) == 0); break;
            case R31_HOSTAGE_GRIP_WAIT: CHECK(task31_service_ms(250u) == 0); break;
            case R31_HOSTAGE_RACK_RETRACT: CHECK(task31_emit(3000u) == 0); wire_poll(); break;
            default: CHECK(0); break;
            }
        }
        CHECK(s_route31_lift_phase == desired && s_seq_state == SQ_TASK && task31_drive_stopped());
        return 0;
    }
    if (desired == R31_RACK_EXTEND || desired == R31_RACK_EXTEND_WAIT) {
        CHECK(task31_predeploy_start() == 0);
        if (desired == R31_RACK_EXTEND_WAIT) {
            CHECK(task31_emit(3400u) == 0); wire_poll();
        }
        CHECK(s_route31_lift_phase == desired && s_seq_state == SQ_ARM_PREP && task31_drive_stopped());
        return 0;
    }
    CHECK(task31_at(ROUTE31_PAIR_STAGE, "123") == 0);
    CHECK(task31_aim(4, VAT_BALL_Y_PX, VAT_WAIT_BALL_ACTION) == 0);
    unsigned budget = 20u;
    while (s_route31_lift_phase != desired && budget--) {
        switch (s_route31_lift_phase) {
        case R31_LIFT_BALL_DOWN: CHECK(task31_emit(32000u) == 0); wire_poll(); break;
        case R31_LIFT_BALL_WAIT: CHECK(task31_service_ms(2000u) == 0); break;
        case R31_LIFT_BALL_UP:
            CHECK(task31_ball_up_done() == 0);
            CHECK(task31_finish_motion() == 0);
            CHECK(host_servo == ROUTE31_BALL_GRIP_US && host_jog_pulses[0][0] == 3400u && !host_jog_pulses[0][1]);
            CHECK(task31_aim(9, VAT_BUCKET_Y_PX, VAT_WAIT_BUCKET_ACTION) == 0);
            break;
        case R31_LIFT_BUCKET_DOWN: CHECK(task31_bucket_down_done() == 0); break;
        case R31_CLAW_RELEASE_WAIT: CHECK(task31_service_ms(1000u) == 0); break;
        case R31_CLAW_RESTAGED_WAIT: CHECK(task31_service_ms(1000u) == 0); break;
        case R31_LIFT_BUCKET_UP: CHECK(task31_emit(20000u) == 0); wire_poll(); break;
        case R31_LIFT_BUCKET_WAIT: CHECK(0); break; /* Removed zero-length observation phase. */
        case R31_RACK_RETRACT: CHECK(task31_emit(3400u) == 0); wire_poll(); break;
        default: CHECK(0); break;
        }
    }
    CHECK(s_route31_lift_phase == desired && s_seq_state == SQ_TASK && task31_drive_stopped());
    return 0;
}

static int task31_cancel_and_verify(const char *key)
{
    uint16_t old_request = wire_request;
    uint32_t run = s_seq_run;
    int pulses_before = pulse_calls, servo_before = servo_calls;
    uint16_t held_u = host_servo;
    unsigned commands = wire_commands;
    run_cmd(key); wire_sync();
    CHECK(s_seq_state == SQ_STOPPED && s_msel == 31 && stopped() && !s_receiving && !s_due);
    CHECK(!vision_align_test_active() && !s_target35_route && !s_seq_pair_turn && !s_seq_pair_offset && !s_seq_return_offset &&
          !s_route31_hostage_opened && s_route31_lift_phase == R31_LIFT_OFF);
    CHECK(servo_calls == servo_before && host_servo == held_u);
    test_stepper_timer_irq(); CHECK(pulse_calls == pulses_before);
    xy_obj(old_request, 9, VAT_ROUTE_BUCKET_X_PX, VAT_BUCKET_Y_PX); xy_qr(old_request, "123");
    CHECK(task31_service_ms(10000u) == 0); run_cmd("g");
    CHECK(s_seq_state == SQ_STOPPED && s_seq_run == run && stopped() && pulse_calls == pulses_before && s_jog_pps == 333u);
    CHECK(servo_calls == servo_before && host_servo == held_u && wire_commands == commands);
    CHECK(s_grab42_extend == 3u && s_grab42_down == 4u && s_grab42_u == 2200u && s_grab42_pps == 777u);
    return 0;
}

static int check_task_cancellation_and_stale_frames(void)
{
    const char *const keys[] = {"g", "a", "0"};
    static const unsigned mechanical[] = {
        R31_RACK_EXTEND, R31_RACK_EXTEND_WAIT, R31_LIFT_BALL_DOWN, R31_LIFT_BALL_WAIT,
        R31_LIFT_BALL_UP, R31_LIFT_BUCKET_DOWN, R31_CLAW_RELEASE_WAIT, R31_CLAW_RESTAGED_WAIT, R31_LIFT_BUCKET_UP,
        R31_RACK_RETRACT, R31_RACK_RETRACT_WAIT, R31_HOSTAGE_RACK_EXTEND, R31_HOSTAGE_RACK_EXTEND_WAIT,
        R31_HOSTAGE_GRIP_WAIT, R31_HOSTAGE_RACK_RETRACT, R31_HOSTAGE_RACK_RETRACT_WAIT
    };
    for (unsigned phase = 0u; phase < sizeof mechanical / sizeof mechanical[0]; ++phase)
        for (unsigned k = 0u; k < 3u; ++k) {
            CHECK(task31_mechanical_phase(mechanical[phase]) == 0);
            if (host_timer_active) CHECK(task31_emit(100u) == 0);
            CHECK(task31_cancel_and_verify(keys[k]) == 0);
        }
    /* Preserve all old non-mechanical task/turn/laser cancellation coverage. */
    for (unsigned phase = 0u; phase < 6u; ++phase) for (unsigned k = 0u; k < 3u; ++k) {
        unsigned stage = phase >= 3u && phase <= 4u ? ROUTE31_TARGET_STAGE :
                         phase == 5u ? ROUTE31_HOSTAGE_STAGE : ROUTE31_PAIR_STAGE;
        CHECK(task31_at(stage, "123") == 0);
        if (phase == 1u || phase == 2u) {
            CHECK(task31_aim(4, VAT_BALL_Y_PX, VAT_WAIT_BALL_ACTION) == 0);
            CHECK(task31_ball_down_done() == 0 && task31_ball_up_begin() == 0 && task31_ball_up_done() == 0);
            wire_ack(wire_request, 2u, 0u);
            if (phase == 2u) CHECK(sequence_start_stage() == 0);
            CHECK(host_servo == ROUTE31_BALL_GRIP_US && !host_jog_pulses[0][1]);
        }
        if (phase == 3u) { wire_ack(wire_request, 2u, 0u); xy_advance(1100u); CHECK(last_x == 200.0f); }
        if (phase == 4u) CHECK(task31_target_fire(8) == 0);
        CHECK(task31_cancel_and_verify(keys[k]) == 0);
    }
    puts("31 tasks:48 mechanical +18 existing task/turn/fire g/a/0 cancellations;all8 partialSTEP jobs +settle/grip/release holds;TIM7stops,heldclaw/noautoopen-or-return,lateframes/noresume,42/f333 isolation passed");
    return 0;
}

static int check_lift_report_delay_and_clock_failure(void)
{
    /* Owner polling is500ms late for either lowering job. No claw command
     * may occur before a verified complete count; hold2 starts at its command,
     * not at the earlier physical pulse timestamp. */
    static const unsigned lowering[] = {R31_LIFT_BALL_DOWN, R31_LIFT_BUCKET_DOWN};
    for (unsigned kind = 0u; kind < 2u; ++kind) {
        CHECK(task31_mechanical_phase(lowering[kind]) == 0);
        int servos = servo_calls, pulses = pulse_calls;
        uint16_t held_u = host_servo;
        unsigned count = kind ? 20000u : 32000u;
        CHECK(task31_emit(count) == 0 && !host_timer_active);
        uint32_t last_pulse_ms = host_tick;
        CHECK(host_servo == held_u && servo_calls == servos);
        host_tick += 500u; wire_poll(); task31_snapshot();
        CHECK(s_route31_lift_phase == (kind ? R31_CLAW_RELEASE_WAIT : R31_LIFT_BALL_WAIT) &&
              s_route31_lift_wait_t0 == last_pulse_ms + 500u && pulse_calls == pulses + (int)count);
        CHECK(host_servo == (kind ? 1150u : task31_grip_expected()) && servo_calls == servos + 1);
        CHECK(task31_status.state == (kind ? VAT_WAIT_BUCKET_ACTION : VAT_WAIT_BALL_ACTION));
        CHECK(task31_service_ms(1999u) == 0 && stopped() && pulse_calls == pulses + (int)count);
        CHECK(s_route31_lift_phase == (kind ? R31_CLAW_RESTAGED_WAIT : R31_LIFT_BALL_WAIT));
        CHECK(host_servo == (kind ? 1500u : task31_grip_expected()) && servo_calls == servos + (kind ? 2 : 1));
        CHECK(task31_service_ms(1u) == 0 && host_timer_active &&
              s_route31_lift_phase == (kind ? R31_LIFT_BUCKET_UP : R31_LIFT_BALL_UP));
        CHECK(pulse_calls == pulses + (int)count);
        if (kind) CHECK(task31_bucket_up_done() == 0);
        else CHECK(task31_ball_up_done() == 0);
        run_cmd("0"); CHECK(stopped());
    }

    for (unsigned job = 0u; job < 6u; ++job) {
        if (job == 0u) CHECK(task31_boot("123") == 0);
        else if (job == 1u) CHECK(task31_at(ROUTE31_PAIR_STAGE, "123") == 0);
        else if (job == 2u) CHECK(task31_mechanical_phase(R31_LIFT_BALL_WAIT) == 0);
        else if (job == 3u) {
            CHECK(task31_mechanical_phase(R31_LIFT_BALL_UP) == 0);
            CHECK(task31_ball_up_done() == 0 && task31_finish_motion() == 0);
            CHECK(host_servo == task31_grip_expected() && !host_timer_active);
        } else if (job == 4u) CHECK(task31_mechanical_phase(R31_CLAW_RESTAGED_WAIT) == 0);
        else { CHECK(task31_mechanical_phase(R31_LIFT_BUCKET_UP) == 0); CHECK(task31_emit(20000u) == 0); }
        int pulses = pulse_calls, servos = servo_calls;
        uint16_t held_u = host_servo;
        host_messages[0] = '\0';
        host_timer_start_fail = 1;
        if (job == 5u) wire_poll();
        else if (job == 2u || job == 4u) CHECK(task31_service_ms(job == 4u ? 1000u : 2000u) == 0);
        else if (job) {
            wire_ack(wire_request, 2u, 0u); xy_advance(260u);
            for (unsigned frame = 0u; frame < 120u && s_seq_state != SQ_STOPPED; ++frame) {
                host_tick += 20u;
                xy_obj(wire_request, job == 3u ? 9 : 4, job == 3u ? VAT_ROUTE_BUCKET_X_PX : VAT_ROUTE_BALL_X_PX,
                       job == 3u ? VAT_BUCKET_Y_PX : VAT_BALL_Y_PX);
            }
        } else task31_latch_and_prepare(ROUTE31_PREDEPLOY_STAGE, "123");
        CHECK(s_seq_state == SQ_STOPPED && s_route31_lift_phase == R31_LIFT_OFF && stopped());
        CHECK(strstr(host_messages, "LIFT_START_ERROR"));
        CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == held_u && s_jog_pps == 333u && !s_seq_pair_turn);
        test_stepper_timer_irq(); run_cmd("g"); CHECK(task31_service_ms(10000u) == 0);
        CHECK(s_seq_state == SQ_STOPPED && pulse_calls == pulses && servo_calls == servos && host_servo == held_u && stopped());
    }
    static const unsigned moving[] = {
        R31_RACK_EXTEND, R31_LIFT_BALL_DOWN, R31_LIFT_BALL_UP,
        R31_LIFT_BUCKET_DOWN, R31_LIFT_BUCKET_UP, R31_RACK_RETRACT, R31_HOSTAGE_RACK_EXTEND, R31_HOSTAGE_RACK_RETRACT
    };
    for (unsigned job = 0u; job < sizeof moving / sizeof moving[0]; ++job) {
        CHECK(task31_mechanical_phase(moving[job]) == 0); CHECK(task31_emit(2u) == 0);
        int pulses = pulse_calls, servos = servo_calls;
        uint16_t held_u = host_servo;
        JogClock cancelled;
        host_messages[0] = '\0';
        CHECK(step_clock_snapshot(&cancelled, 1) == 0u && cancelled.completed == 2u);
        wire_poll();
        CHECK(s_seq_state == SQ_STOPPED && s_route31_lift_phase == R31_LIFT_OFF && stopped());
        CHECK(strstr(host_messages, job >= 6u ? "HOSTAGE_COUNT_ERROR" : "LIFT_COUNT_ERROR"));
        CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == held_u);
        test_stepper_timer_irq(); CHECK(task31_service_ms(10000u) == 0); run_cmd("g");
        CHECK(s_seq_state == SQ_STOPPED && pulse_calls == pulses && servo_calls == servos && host_servo == held_u && stopped());
    }
    puts("31 arm:both lowerings delayedowner500ms start full2s at actual grip2100/release1150;all6 ball/bucket STEP startfails and all8 partial-count faults STOP without laterclaw/return/turn,extraIRQ or autoresume passed");
    return 0;
}

static void task31_last_irq_during_brake(void)
{
    host_motion_brake_hook = NULL; /* A single interrupt at the chosen seam. */
    host_tick += 1u; host_timer_sync_hal();
    test_stepper_timer_irq();
    task31_racing_last_pulse_ms = host_tick;
}

static int check_last_pulse_during_lift_poll_wait_origin(void)
{
    static const unsigned lowering[] = {R31_LIFT_BALL_DOWN, R31_LIFT_BUCKET_DOWN};
    for (unsigned kind = 0u; kind < 2u; ++kind) {
        CHECK(task31_mechanical_phase(lowering[kind]) == 0);
        int pulses = pulse_calls, servos = servo_calls;
        unsigned count = kind ? 20000u : 32000u;
        CHECK(task31_emit(count - 1u) == 0 && host_timer_active && s_jog_clock.remaining == 1u);
        CHECK(host_servo == (kind ? task31_grip_expected() : 1150u) && servo_calls == servos);
        uint32_t entry_ms = host_tick;
        host_motion_brake_hook = task31_last_irq_during_brake;
        route31_lift_poll(); /* Actual production routine, ISR at brake/snapshot seam. */
        CHECK(task31_racing_last_pulse_ms == entry_ms + 1u &&
              pulse_calls == pulses + (int)count && !host_timer_active);
        CHECK(s_route31_lift_phase == (kind ? R31_CLAW_RELEASE_WAIT : R31_LIFT_BALL_WAIT) &&
              s_route31_lift_wait_t0 == task31_racing_last_pulse_ms && stopped() && !s_seq_pair_turn);
        CHECK(host_servo == (kind ? 1150u : task31_grip_expected()) && servo_calls == servos + 1);
        CHECK(task31_service_ms(1999u) == 0 && stopped() && pulse_calls == pulses + (int)count);
        CHECK(s_route31_lift_phase == (kind ? R31_CLAW_RESTAGED_WAIT : R31_LIFT_BALL_WAIT));
        CHECK(host_servo == (kind ? 1500u : task31_grip_expected()) && servo_calls == servos + (kind ? 2 : 1));
        CHECK(task31_service_ms(1u) == 0 && host_timer_active &&
              s_route31_lift_phase == (kind ? R31_LIFT_BUCKET_UP : R31_LIFT_BALL_UP) &&
              pulse_calls == pulses + (int)count);
        if (kind) CHECK(task31_bucket_up_done() == 0);
        else CHECK(task31_ball_up_done() == 0);
        run_cmd("0"); CHECK(stopped());
    }
    puts("31 arm:both lastIRQs complete1ms afterpollentry at brake/snapshot;grip2100/release1150 follow checked completion and cannot unsigned-underflow/skip full2s holds passed");
    return 0;
}

static int check_hostage_retract_hold_timing_and_faults(void)
{
    /* A late owner poll cannot backdate the new2100 grip hold to the
     * extension's final pulse, nor notify VAT before actual retraction. */
    CHECK(task31_mechanical_phase(R31_HOSTAGE_RACK_EXTEND) == 0);
    CHECK(task31_emit(3400u) == 0 && !host_timer_active);
    uint32_t extension_end = host_tick;
    host_tick += 500u; wire_poll(); task31_snapshot();
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND_WAIT &&
          s_route31_lift_wait_t0 == extension_end && host_servo == 1150u && servo_calls == 1);
    host_tick += 400u; wire_poll(); task31_snapshot();
    CHECK(s_route31_lift_phase == R31_HOSTAGE_GRIP_WAIT && host_servo == task31_grip_expected() &&
          s_route31_lift_wait_t0 == host_tick && servo_calls == 2 && pulse_calls == 3400);
    CHECK(task31_status.state == VAT_WAIT_HOSTAGE_ACTION && !s_route31_hostage_hold && s_receiving);
    CHECK(task31_hostage_retract_begin() == 0 && task31_hostage_retract_done() == 0);
    CHECK(pulse_calls == 6400 && !host_jog_pulses[1][0] && !host_jog_pulses[1][1]);
    CHECK(task31_cancel_and_verify("0") == 0);

    /* Complete the last requested return STEP at the brake/snapshot seam.
     * Its physical completion timestamp starts a full250ms settle. */
    CHECK(task31_mechanical_phase(R31_HOSTAGE_RACK_RETRACT) == 0);
    CHECK(task31_emit(2999u) == 0 && s_jog_clock.remaining == 1u && host_timer_active);
    uint32_t entry_ms = host_tick;
    host_motion_brake_hook = task31_last_irq_during_brake;
    route31_lift_poll(); task31_snapshot();
    CHECK(task31_racing_last_pulse_ms == entry_ms + 1u && pulse_calls == 6400 && !host_timer_active);
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT_WAIT &&
          s_route31_lift_wait_t0 == task31_racing_last_pulse_ms &&
          task31_status.state == VAT_WAIT_HOSTAGE_ACTION && !s_route31_hostage_hold);
    CHECK(task31_service_ms(249u) == 0 && task31_status.state == VAT_WAIT_HOSTAGE_ACTION &&
          s_route31_lift_phase == R31_HOSTAGE_RACK_RETRACT_WAIT && !s_route31_hostage_hold);
    CHECK(task31_service_ms(1u) == 0 && task31_status.state == VAT_DONE && s_route31_hostage_hold &&
          host_servo == task31_grip_expected() && servo_calls == 2 && s_route31_rack_deployed);
    CHECK(task31_cancel_and_verify("0") == 0);

    /* The eighth motor job starts only after gripping; a clock failure must
     * retain2100 and never perform an automatic release, retry or route leg. */
    CHECK(task31_mechanical_phase(R31_HOSTAGE_GRIP_WAIT) == 0);
    host_messages[0] = '\0'; host_timer_start_fail = 1;
    CHECK(task31_service_ms(250u) == 0 && s_seq_state == SQ_STOPPED && stopped());
    CHECK(strstr(host_messages, "LIFT_START_ERROR") && pulse_calls == 3400 &&
          servo_calls == 2 && host_servo == task31_grip_expected() && s_route31_lift_phase == R31_LIFT_OFF);
    test_stepper_timer_irq(); run_cmd("g"); CHECK(task31_service_ms(10000u) == 0);
    CHECK(s_seq_state == SQ_STOPPED && pulse_calls == 3400 && servo_calls == 2 && host_servo == task31_grip_expected() && stopped());

    /* Even3000 actual pulses on the wrong captured axis cannot authorize
     * owner success; matching count alone is insufficient. */
    CHECK(task31_mechanical_phase(R31_HOSTAGE_RACK_RETRACT) == 0);
    CHECK(task31_emit(3000u) == 0 && !host_timer_active);
    s_jog_clock.axis = 1;
    host_messages[0] = '\0'; wire_poll();
    CHECK(s_seq_state == SQ_STOPPED && strstr(host_messages, "HOSTAGE_COUNT_ERROR") &&
          pulse_calls == 6400 && servo_calls == 2 && host_servo == task31_grip_expected() && stopped());
    run_cmd("g"); CHECK(task31_service_ms(10000u) == 0 && pulse_calls == 6400 &&
          servo_calls == 2 && host_servo == task31_grip_expected() && stopped());
    puts("31 hostage return:latepoll anchors2100 command+full250;lastIRQ/2999+1 starts full250 settle before notify;8th clock-start/wrong-axis faults retain2100,no down/up/open/retry passed");
    return 0;
}

static int check_mechanical_ownership_and_qr_gate(void)
{
    static const unsigned phases[] = {
        R31_RACK_EXTEND,R31_RACK_EXTEND_WAIT,R31_LIFT_BALL_DOWN,R31_LIFT_BALL_WAIT,
        R31_LIFT_BALL_UP,R31_LIFT_BUCKET_DOWN,R31_CLAW_RELEASE_WAIT,R31_CLAW_RESTAGED_WAIT,R31_LIFT_BUCKET_UP,
        R31_RACK_RETRACT,R31_RACK_RETRACT_WAIT,R31_HOSTAGE_RACK_EXTEND,R31_HOSTAGE_RACK_EXTEND_WAIT,
        R31_HOSTAGE_GRIP_WAIT,R31_HOSTAGE_RACK_RETRACT,R31_HOSTAGE_RACK_RETRACT_WAIT
    };
    static const char *const writes[] = {
        "42","24","27","28","31","32","35","38","41","r1","e3","h4","u1500",
        "f2000","nl30","nr30","n30","su1500","co","cc","v300","d1000","ykp10","fff0"
    };
    static const char *const reads[] = {"?","diag","param","vision","route"};
    for (unsigned p = 0u; p < sizeof phases / sizeof phases[0]; ++p) {
        CHECK(task31_mechanical_phase(phases[p]) == 0);
        unsigned owner = s_seq_state;
        int pulses = pulse_calls, servos = servo_calls;
        uint16_t held = host_servo;
        uint32_t remaining = s_jog_clock.remaining;
        unsigned tx = wire_commands;
        for (unsigned c = 0u; c < sizeof writes / sizeof writes[0]; ++c) {
            run_cmd(writes[c]); wire_poll();
            CHECK(strstr(last_message,"ERR ROUTE_SEQ_ACTIVE"));
            CHECK(s_seq_state == owner && s_route31_lift_phase == phases[p] && s_msel == 31);
            CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == held && s_jog_clock.remaining == remaining);
            CHECK(wire_commands == tx && s_jog_pps == 333u);
            CHECK(s_grab42_extend == 3u && s_grab42_down == 4u && s_grab42_u == 2200u && s_grab42_pps == 777u);
        }
        for (unsigned c = 0u; c < sizeof reads / sizeof reads[0]; ++c) {
            run_cmd(reads[c]); wire_poll();
            CHECK(s_seq_state == owner && s_route31_lift_phase == phases[p]);
            CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == held && s_jog_clock.remaining == remaining && wire_commands == tx);
        }
        CHECK(task31_cancel_and_verify("0") == 0);
    }
    /* Real wire decoder rejects malformed/unknown QR while READY. The final
     * cached-QR gate also rejects an unset/invalid digit before any arm job. */
    const char *const bad_wire[] = {"", "103", "000", "444", "12"};
    for (unsigned bad = 0u; bad < sizeof bad_wire / sizeof bad_wire[0]; ++bad) {
        CHECK(wire_boot() == 0); xy_seq = 1u; run_cmd("31"); wire_sync();
        wire_ack(wire_request,1u,0u); xy_qr(wire_request,bad_wire[bad]);
        CHECK(!proto_qr_get(NULL) && !pulse_calls && !servo_calls && stopped());
        s_seq_stage = ROUTE31_PAIR_STAGE; route_seq_prepare(); wire_sync();
        CHECK(s_seq_state == SQ_STOPPED && strstr(host_messages,"LATCHED_QR_INVALID"));
        CHECK(!pulse_calls && !servo_calls && !host_timer_active && stopped());
    }
    const int32_t bad_values[] = {-1,0,4};
    for (unsigned digit = 0u; digit < 3u; ++digit) for (unsigned value = 0u; value < 3u; ++value) {
        CHECK(task31_boot("123") == 0);
        s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
        s_seq_qr[digit] = bad_values[value];
        s_seq_stage = ROUTE31_PAIR_STAGE; route_seq_prepare(); wire_sync();
        CHECK(s_seq_state == SQ_STOPPED && strstr(host_messages,"LATCHED_QR_INVALID"));
        CHECK(!pulse_calls && !servo_calls && !host_timer_active && stopped());
    }
    /* Deliberately skip the entry predeploy in this negative fixture. Legal
     * QR/pixels cannot authorize DOWN or invent another rack extension. */
    CHECK(task31_boot("123") == 0);
    task31_latch_and_prepare(ROUTE31_PAIR_STAGE, "123");
    CHECK(s_seq_state == SQ_TASK && !s_route31_rack_deployed && !pulse_calls && !servo_calls);
    wire_ack(wire_request, 2u, 0u); xy_advance(260u);
    for (unsigned frame = 0u; frame < 60u && s_seq_state != SQ_STOPPED; ++frame) {
        host_tick += 20u; xy_obj(wire_request, 4, VAT_ROUTE_BALL_X_PX, VAT_BALL_Y_PX);
    }
    CHECK(s_seq_state == SQ_STOPPED && strstr(host_messages,"RACK_NOT_DEPLOYED"));
    CHECK(!pulse_calls && !servo_calls && !host_timer_active && stopped());
    CHECK(task31_service_ms(10000u) == 0); run_cmd("g");
    CHECK(s_seq_state == SQ_STOPPED && !pulse_calls && !servo_calls && stopped());
    puts("31 arm ownership:24 mode/tune/actuatorwrites locked across16phases(includingSQ_ARM_PREP);reports read-only,42slots untouched;5 bad-wireQR/9cached-invaliddigits/no-rack prevent arm jobs passed");
    return 0;
}

static int task31_stop_step_for_new_image(void)
{
    const int ball_models[] = {4,3,5}, hostage_models[] = {1,2,0};
    int model = wire_task == PROTO_TASK_BUCKET ? 9 :
                wire_task == PROTO_TASK_HOSTAGE ? hostage_models[wire_digit - 1u] : ball_models[wire_digit - 1u];
    int y = wire_task == PROTO_TASK_BUCKET ? VAT_BUCKET_Y_PX :
            wire_task == PROTO_TASK_HOSTAGE ? VAT_HOSTAGE_Y_PX : VAT_BALL_Y_PX;
    task31_snapshot();
    if (!VAT_Y_ALIGN_ENABLE && s_route_owned && s_route_owner_mode == 31u) {
        /* Current31 runs X continuously. Encoder3mm is NOT its stopping rule;
         * a new in-band image brakes first, then needs stopped/new imagery. */
        CHECK(task31_status.state == VAT_FINE_CONTINUOUS && last_x != 0.0f && last_y == 0.0f);
        host_tick += 20u; xy_obj(wire_request, model, task31_status.x_goal, y); task31_snapshot();
        CHECK(task31_status.state == VAT_BRAKE && !task31_status.good && !task31_status.latest && stopped());
    } else {
    CHECK(task31_status.state == VAT_STEP_MOVE && last_w == 0.0f &&
          ((last_x != 0.0f) != (last_y != 0.0f)));
    if (last_x != 0.0f) host_fore += last_x > 0.0f ? 3.0f : -3.0f;
    else host_lateral += last_y > 0.0f ? 3.0f : -3.0f;
    xy_advance(20u); task31_snapshot();
    CHECK(task31_status.state == VAT_BRAKE && stopped());
    }
    xy_advance(260u); task31_snapshot();
    if (task31_status.state == VAT_YAW_FIX) {
        host_absolute_yaw = task31_status.yaw_target;
        xy_advance(20u);
        if (s_route_owned && s_route_owner_mode == 31u)
            task31_fresh_images_ms(400u, model, task31_status.x_goal, y);
        else xy_advance(400u);
        task31_snapshot();
    }
    CHECK(task31_status.state == VAT_RECHECK && stopped() && !task31_status.latest && !task31_status.good);
    return 0;
}

static int task31_workpoint_ready(unsigned kind)
{
    CHECK(task31_at(kind == 2u ? ROUTE31_HOSTAGE_STAGE : ROUTE31_PAIR_STAGE, "123") == 0);
    if (kind == 1u) {
        CHECK(task31_aim(4, VAT_BALL_Y_PX, VAT_WAIT_BALL_ACTION) == 0);
        CHECK(task31_ball_down_done() == 0 && task31_ball_up_begin() == 0 && task31_ball_up_done() == 0);
        CHECK(task31_finish_motion() == 0);
    }
    task31_snapshot(); CHECK(task31_status.x_goal == task31_goal(kind == 2u ? 0 : kind == 1u ? 9 : 4));
    wire_ack(wire_request, 2u, 0u); xy_advance(300u); task31_snapshot();
    if (kind == 2u) task31_rank(wire_request, 0u, 2u);
    CHECK(task31_status.state == (kind == 1u ? VAT_RECHECK : VAT_ROUTE_SEARCH));
    host_tick += 20u; xy_obj(wire_request, kind == 2u ? 0 : kind == 1u ? 9 : 4, task31_goal(kind == 2u ? 0 : kind == 1u ? 9 : 4),
                           kind == 2u ? VAT_HOSTAGE_Y_PX : kind == 1u ? VAT_BUCKET_Y_PX : VAT_BALL_Y_PX);
    xy_advance(260u); task31_snapshot();
    CHECK(task31_status.state == VAT_RECHECK && !task31_status.good && stopped());
    return 0;
}

static int check_route31_workpoint_boundaries_and_standalone_isolation(void)
{
    const int models[] = {4, 9, 0}, goals[] = {VAT_ROUTE_BALL_X_PX, VAT_ROUTE_BUCKET_X_PX, VAT_ROUTE_HOSTAGE_X_PX};
    const int ys[] = {VAT_BALL_Y_PX, VAT_BUCKET_Y_PX, VAT_HOSTAGE_Y_PX};
    const VisionAlignTestState done[] = {VAT_WAIT_BALL_ACTION, VAT_WAIT_BUCKET_ACTION, VAT_WAIT_HOSTAGE_ACTION};
    CHECK(VAT_X_PX == 190 && VAT_ROUTE_BALL_X_PX == 135 &&
          VAT_ROUTE_BUCKET_X_PX == 105 && VAT_ROUTE43_BUCKET_X_PX == 125 &&
          VAT_ROUTE_HOSTAGE_X_PX == 215 && VAT_TOL_PX == 10);
    CHECK(ROUTE31_TARGET_CX == 250 && ROUTE31_TARGET_LOW_CX == 247 && ROUTE31_TARGET_HIGH_CX == 253 &&
          ROUTE31_FINE_ENTER_CX == 360);
    CHECK(goals[0] - 10 == 125 && goals[0] + 10 == 145 &&
          goals[0] - 11 == 124 && goals[0] + 11 == 146);
    CHECK(goals[1] - 10 == 95 && goals[1] + 10 == 115 &&
          goals[1] - 11 == 94 && goals[1] + 11 == 116);
    CHECK(goals[2] - 10 == 205 && goals[2] + 10 == 225 &&
          goals[2] - 11 == 204 && goals[2] + 11 == 226);
    for (unsigned kind = 0u; kind < 3u; ++kind) for (int sign = -1; sign <= 1; sign += 2) {
        CHECK(task31_workpoint_ready(kind) == 0);
        host_tick += 20u; xy_obj(wire_request, models[kind], goals[kind] + sign * 11, ys[kind]);
        task31_snapshot();
        CHECK(task31_status.x_goal == goals[kind] && task31_status.state == task31_fine_x_state() &&
              last_x == sign * 20.0f && last_y == 0.0f && last_w == 0.0f && !task31_status.good);
        CHECK(task31_stop_step_for_new_image() == 0);
        /* Both inclusive edges are legal, but require five NEW stopped frames:
         * goal+/-11 was not an arrival and cannot contribute to this count. */
        for (unsigned fresh = 0u; fresh < 5u; ++fresh) {
            host_tick += 20u;
            xy_obj(wire_request, models[kind], goals[kind] + (fresh & 1u ? 10 : -10), ys[kind]);
            task31_snapshot();
            CHECK(task31_status.x_goal == goals[kind] &&
                  task31_status.good == fresh + 1u &&
                  last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !laser_state);
            if (fresh < 4u) CHECK(task31_status.state == VAT_ALIGN && !host_timer_active);
        }
        CHECK(task31_status.state == done[kind] && task31_drive_stopped());
        if (kind == 2u) {
            CHECK(task31_hostage_reconfirm(models[kind], ys[kind]) == 0);
            CHECK(s_seq_state == SQ_TASK && !s_route31_hostage_hold && host_timer_active);
            CHECK(task31_hostage_grab_done() == 0 && s_route31_hostage_hold);
        }
        else if (kind == 1u) CHECK(host_timer_active && s_route31_lift_phase == R31_LIFT_BUCKET_DOWN && host_servo == task31_grip_expected());
        else CHECK(host_timer_active && s_route31_lift_phase == R31_LIFT_BALL_DOWN && !vision_align_test_take_route_action());
        run_cmd("0"); CHECK(stopped() && !host_timer_active);
    }
    CHECK(task31_workpoint_ready(0u) == 0);
    host_tick += 20u; xy_obj(wire_request, 4, 190, VAT_BALL_Y_PX); task31_snapshot();
    CHECK(task31_status.x_goal == VAT_ROUTE_BALL_X_PX && task31_status.state == task31_fine_x_state() &&
          last_x == 20.0f && !task31_status.good && !host_timer_active);
    run_cmd("0"); CHECK(stopped()); /* Former ball190 is not an arrival. */
    /* Re-enter each independent mode AFTER31: route ball135/bucket105 must
     * not leak. Independent38..41 remain190, and41 retains its5s holds. */
    for (unsigned mode = 38u; mode <= 41u; ++mode) {
        int model = mode == 39u ? 9 : mode == 40u ? 0 : 4;
        int y = mode == 39u ? VAT_BUCKET_Y_PX : mode == 40u ? VAT_HOSTAGE_Y_PX : VAT_BALL_Y_PX;
        CHECK(xy_start(mode) == 0); task31_snapshot();
        CHECK(task31_status.x_goal == 190 && !host_timer_active && !pulse_calls);
        host_tick += 20u; xy_obj(wire_request, model, 179, y); task31_snapshot();
        CHECK(task31_status.state == VAT_STEP_MOVE && last_x == -20.0f && last_y == 0.0f &&
              !task31_status.good && task31_status.x_goal == 190);
        CHECK(task31_stop_step_for_new_image() == 0);
        for (unsigned fresh = 0u; fresh < 5u; ++fresh) {
            host_tick += 20u; xy_obj(wire_request, model, fresh & 1u ? 200 : 180, y); task31_snapshot();
            CHECK(task31_status.x_goal == 190 && task31_status.good == fresh + 1u && stopped());
        }
        CHECK(mode == 41u ? task31_status.state == VAT_HOLD_BALL : s_xy_state == XT_DONE);
        CHECK(!task31_status.alignment_confirmed && !s_route31_hostage_opened);
        CHECK(!vision_align_test_take_route_action() && !pulse_calls && !host_timer_active);
        run_cmd("0"); CHECK(stopped());
    }
    puts("31 per-task X: ball135/bucket105/hostage215;ball125/145 andbucket95/115 inclusive fivefresh,94/116 not-arrived;31 lift-owned waits;43 bucket125/independent38/39/40/41 remain190/noarm/41hold5s passed");
    return 0;
}

static int check_route_ball_hostage_initial_search100(void)
{
    const unsigned stages[] = { ROUTE31_PAIR_STAGE, ROUTE31_HOSTAGE_STAGE };
    const int models[] = {4,0}, ys[] = {VAT_BALL_Y_PX,VAT_HOSTAGE_Y_PX};
    for (unsigned kind = 0u; kind < 2u; ++kind) for (unsigned sticky = 0u; sticky < 2u; ++sticky) {
        const int goal = task31_goal(models[kind]);
        const int fine_cx = goal + VAT_ROUTE_FINE_ERROR_PX - 1;
        CHECK(task31_at(stages[kind], "123") == 0 && s_v == 200.0f);
        int pulses_before = pulse_calls;
        CHECK(s_route31_lift_phase == R31_LIFT_OFF && !host_timer_active);
        xy_obj(wire_request, models[kind], task31_goal(models[kind]), ys[kind]); xy_advance(1000u); task31_snapshot();
        CHECK(stopped() && !task31_status.good && task31_status.state == VAT_ALIGN); /* No matching task ACK. */
        wire_ack((uint16_t)(wire_request-1u), 2u, 0u); xy_advance(20u); CHECK(stopped());
        wire_ack(wire_request, 2u, 0u); xy_advance(20u); task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        uint32_t search_t0 = host_tick;
        xy_advance(20u); CHECK(fabsf(last_x - 14.0f) < 0.0001f && host_tick == search_t0 + 20u);
        xy_advance(20u); CHECK(fabsf(last_x - 28.0f) < 0.0001f && host_tick == search_t0 + 40u);
        CHECK(!task31_status.step && pulse_calls == pulses_before && !servo_calls && !laser_state);
        host_absolute_yaw = task31_status.yaw_target + 0.5f;
        xy_advance(1000u); task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && last_x == 200.0f && last_w < 0.0f && !task31_status.step);
        host_tick += 20u; xy_obj((uint16_t)(wire_request-1u), models[kind], task31_goal(models[kind]), ys[kind]);
        host_tick += 20u; xy_obj(wire_request, models[kind] == 4 ? 0 : 4, task31_goal(models[kind]), ys[kind]); task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && last_x == 200.0f && !task31_status.good);
        host_absolute_yaw = task31_status.yaw_target; xy_advance(20u);
        /* Exact15 remains coarse. At300ms the current request has not yet
         * entered its strict>300ms dropout brake. */
        host_tick += 20u; xy_obj(wire_request, models[kind], 400, ys[kind]); task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && last_x == 200.0f && !task31_status.good);
        host_tick += 20u; xy_obj(wire_request, models[kind], goal + VAT_ROUTE_FINE_ERROR_PX, ys[kind]); task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && last_x == 200.0f && !task31_status.good);
        CHECK(task31_service_ms(300u) == 0); task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && last_x == 200.0f && !task31_status.good);
        host_tick += 20u;
        uint16_t first_sequence = (uint16_t)xy_seq;
        if (sticky) {
            xy_obj_poll(wire_request, models[kind], fine_cx, ys[kind], 0);
            xy_obj_poll(wire_request, -1, 0, 0, 0); wire_poll();
        } else xy_obj(wire_request, models[kind], fine_cx, ys[kind]);
        task31_snapshot();
        CHECK(task31_status.state == VAT_BRAKE && stopped() && !task31_status.latest && !task31_status.good);
        xy_advance(240u); task31_snapshot(); CHECK(task31_status.state == VAT_BRAKE && stopped());
        xy_advance(20u); task31_snapshot();
        CHECK(task31_status.state == VAT_RECHECK && stopped() && !task31_status.latest && !task31_status.good);
        xy_advance(20u); task31_snapshot(); CHECK(task31_status.state == VAT_RECHECK && stopped());
        unsigned next_sequence = xy_seq;
        xy_seq = first_sequence; host_tick += 20u; xy_obj(wire_request, models[kind], fine_cx, ys[kind]);
        xy_seq = next_sequence; task31_snapshot();
        CHECK(task31_status.state == VAT_RECHECK && stopped() && !task31_status.latest && !task31_status.good);
        /* Receding outside the abs20 gate can only produce fine v20.
         * It must never recreate the coarse200/right-FF approach. */
        host_tick += 20u; xy_obj(wire_request, models[kind], 400, ys[kind]); task31_snapshot();
        CHECK(task31_status.state == task31_fine_x_state() && last_x == 20.0f && last_y == 0.0f && last_w == 0.0f);
        CHECK(task31_stop_step_for_new_image() == 0); xy_advance(20u); task31_snapshot();
        CHECK(task31_status.state == VAT_RECHECK && stopped()); /* Lost-after-seen never reopens blind100. */
        host_tick += 20u; xy_obj(wire_request, models[kind], task31_goal(models[kind]), ys[kind]-11); task31_snapshot();
#if VAT_Y_ALIGN_ENABLE
        CHECK(task31_status.state == VAT_STEP_MOVE && last_x == 0.0f && last_y == -30.0f && last_w == 0.0f);
        CHECK(task31_stop_step_for_new_image() == 0);
        CHECK(task31_aim(models[kind], ys[kind], kind ? VAT_DONE : VAT_WAIT_BALL_ACTION) == 0);
#else
        CHECK(task31_status.state == VAT_ALIGN && stopped() && task31_status.good == 1u && !task31_status.yaw_dirty);
        /* The diagnostic counter records the continuous command once; no
         * encoder-limited3mm job or time-cap completion was scheduled. */
        CHECK(task31_status.step == 1u && task31_status.step_mm == 0.0f &&
              !task31_status.step_capped);
        CHECK(task31_aim(models[kind], 80, kind ? VAT_DONE : VAT_WAIT_BALL_ACTION) == 0);
#endif
        if (!kind) CHECK(s_route31_lift_phase == R31_LIFT_BALL_DOWN && host_timer_active && pulse_calls == pulses_before);
        run_cmd("0");
    }
    const char *const keys[] = {"g","a","0"};
    for (unsigned kind = 0u; kind < 2u; ++kind) for (unsigned k = 0u; k < 3u; ++k) {
        CHECK(task31_at(stages[kind], "123") == 0);
        int pulses_before = pulse_calls;
        wire_ack(wire_request, 2u, 0u); xy_advance(300u); task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && task31_search_speed_exact(200.0f) == 0);
        unsigned commands = wire_commands;
        run_cmd(keys[k]); CHECK(stopped() && s_seq_state == SQ_STOPPED && !host_timer_active);
        host_tick += 20u; xy_obj(wire_request, models[kind], task31_goal(models[kind]), ys[kind]);
        CHECK(task31_service_ms(10000u) == 0); run_cmd("g");
        CHECK(stopped() && s_seq_state == SQ_STOPPED && wire_commands == commands && pulse_calls == pulses_before);
    }
    puts(VAT_Y_ALIGN_ENABLE
         ? "31 BALL/HOSTAGE:matchingACK cx400/goal+15 coarse200,firstgoal+14 brake/newXY/permanentfineX20Y30/lost-no-reblind/6 SEARCH g/a/0 cancellations passed"
         : "31 BALL/HOSTAGE:matchingACK cx400/goal+15 coarse200,firstgoal+14 brake/newX/permanentfineX20,cy error never createsYstep,lost-no-reblind/6 SEARCH g/a/0 cancellations passed");
    return 0;
}

static int check_current31_continuous_x_and_ball_loss_action(void)
{
#if !VAT_Y_ALIGN_ENABLE
    CHECK(task31_workpoint_ready(0u) == 0);
    int before_pulses = pulse_calls, before_servos = servo_calls;
    host_tick += 20u; xy_obj(wire_request, 4, VAT_ROUTE_BALL_X_PX + 11, VAT_BALL_Y_PX);
    task31_snapshot();
    CHECK(task31_status.state == VAT_FINE_CONTINUOUS && last_x == 20.0f && !task31_status.good);
    unsigned commands = host_precise_calls;
    host_fore += 20.0f; host_counts[0] += 100; xy_advance(20u); task31_snapshot();
    CHECK(task31_status.state == VAT_FINE_CONTINUOUS && last_x == 20.0f &&
          host_precise_calls > commands && !task31_status.step_capped &&
          !host_timer_active && pulse_calls == before_pulses && servo_calls == before_servos);
    /* A sign change cannot reverse in the same moving-image poll. */
    host_tick += 20u; xy_obj(wire_request, 4, VAT_ROUTE_BALL_X_PX - 11, VAT_BALL_Y_PX);
    task31_snapshot();
    CHECK(task31_status.state == VAT_BRAKE && stopped() &&
          !strcmp(task31_status.reason, "FINE_REVERSE_BRAKE") && !task31_status.good);
    xy_advance(260u); task31_snapshot();
    CHECK(task31_status.state == VAT_RECHECK && stopped() && !task31_status.latest);
    host_tick += 20u; xy_obj(wire_request, 4, VAT_ROUTE_BALL_X_PX - 11, VAT_BALL_Y_PX);
    task31_snapshot();
    CHECK(task31_status.state == VAT_FINE_CONTINUOUS && last_x == -20.0f && !task31_status.good);
    CHECK(task31_stop_step_for_new_image() == 0);
    for (unsigned frame = 0u; frame < 5u; ++frame) {
        host_tick += 20u; xy_obj(wire_request, 4, VAT_ROUTE_BALL_X_PX, VAT_BALL_Y_PX);
        task31_snapshot();
        CHECK(task31_status.good == frame + 1u && task31_drive_stopped());
        if (frame < 4u) CHECK(!host_timer_active && servo_calls == before_servos);
    }
    CHECK(task31_status.state == VAT_WAIT_BALL_ACTION &&
          task31_status.good == 5u && !task31_status.ball_fallback &&
          s_route31_lift_phase == R31_LIFT_BALL_DOWN && host_timer_active);
    run_cmd("0"); CHECK(stopped() && !host_timer_active);
#endif
    /* First selected01 at coarse500 is sufficient for the new dropout
     * contract: no fine-gate/arrival or fabricated aligned flag is needed. */
    CHECK(task31_at(ROUTE31_PAIR_STAGE, "123") == 0);
    wire_ack(wire_request, 2u, 0u); xy_advance(300u);
    host_tick += 20u; xy_obj(wire_request, 4, 500, VAT_BALL_Y_PX);
    task31_rank(wire_request, 4u, 2u);
    CHECK(task31_service_ms(300u) == 0 && task31_status.state == VAT_ROUTE_SEARCH &&
          last_x == 200.0f && !host_timer_active && !servo_calls);
    CHECK(task31_service_ms(1u) == 0 && task31_status.state == VAT_BRAKE &&
          !strcmp(task31_status.reason, "BALL_LOST300_BRAKE") && stopped());
    CHECK(task31_service_ms(249u) == 0 && !host_timer_active && !servo_calls);
    CHECK(task31_service_ms(1u) == 0 && task31_status.state == VAT_WAIT_BALL_ACTION &&
          task31_status.ball_fallback && !task31_status.alignment_confirmed &&
          !task31_status.good && s_route31_lift_phase == R31_LIFT_BALL_DOWN &&
          host_timer_active && host_jog_axis == 1 && host_jog_dir == 1 && host_timer_pps == 10000u);
    CHECK(task31_ball_down_done() == 0 && task31_ball_up_begin() == 0 &&
          task31_ball_up_done() == 0 && host_servo == 2100u);
    run_cmd("0"); CHECK(stopped() && !host_timer_active && host_servo == 2100u);
    puts(VAT_Y_ALIGN_ENABLE
         ? "31 legacyXY:coarse firstBALL01/strict300-301/wheelstill250 commits actual down32000/grip2100/up32000 without fakealignment passed"
         : "31 continuousX:encoder20mm never ends finev20;reverse brakes/newimage,arrival wheelstill250+fiveNEW;coarse BALL strict300-301 actual down32000/grip2100/up32000/no fakealignment passed");
    return 0;
}

static int check_route_ball_hostage_strict_coarse_boundaries(void)
{
    const unsigned stages[] = {ROUTE31_PAIR_STAGE, ROUTE31_HOSTAGE_STAGE};
    const int models[] = {4,0}, ys[] = {VAT_BALL_Y_PX,VAT_HOSTAGE_Y_PX};
    CHECK(VAT_ROUTE_FINE_ERROR_PX == 15 && VAT_TOL_PX == 10);
    for (unsigned kind = 0u; kind < 2u; ++kind) for (int sign = -1; sign <= 1; sign += 2) {
        CHECK(task31_at(stages[kind], "123") == 0);
        const int goal = task31_goal(models[kind]);
        const float ff = kind ? ROUTE31_HOSTAGE_SEARCH_RIGHT_FF_RATIO : ROUTE31_BALL_SEARCH_RIGHT_FF_RATIO;
        wire_ack(wire_request, 2u, 0u); xy_advance(300u); task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && s_route_search_ff == ff);
        const int before = pulse_calls, servos = servo_calls;
        host_tick += 20u; xy_obj(wire_request, models[kind], goal + sign * 15, ys[kind]);
        task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && s_direction == sign &&
              !task31_status.good && !task31_status.step && !host_timer_active);
        xy_advance(300u); task31_snapshot();
        CHECK(task31_status.state == VAT_ROUTE_SEARCH && last_x == sign * 200.0f &&
              fabsf(last_y - (sign > 0 ? 200.0f * ff : 0.0f)) < 0.0001f && last_w == 0.0f &&
              pulse_calls == before && servo_calls == servos);
        host_tick += 20u; xy_obj(wire_request, models[kind], goal + sign * 14, ys[kind]);
        task31_snapshot();
        CHECK(task31_status.state == VAT_BRAKE && !task31_status.good &&
              !task31_status.latest && stopped() && pulse_calls == before && servo_calls == servos);
        xy_advance(260u); task31_snapshot();
        CHECK(task31_status.state == VAT_RECHECK && !task31_status.good && !task31_status.latest && stopped());
        host_tick += 20u; xy_obj(wire_request, models[kind], goal + sign * 15, ys[kind]);
        task31_snapshot();
        CHECK(task31_status.state == task31_fine_x_state() && last_x == sign * 20.0f &&
              last_y == 0.0f && last_w == 0.0f && !task31_status.good && !host_timer_active);
        run_cmd("0"); CHECK(stopped());
    }
    puts("31 BALL/HOSTAGE:both exact goal+/-15 stay signed coarse200; +/-14 brake/fresh-recheck, permanentfine20; forward-only task-specific FF and unchanged+/-10 arrival passed");
    return 0;
}

static int check_route_search_private_gain_and_late_yaw(void)
{
    CHECK(task31_at_gain(ROUTE31_PAIR_STAGE,"123",3.0f)==0);
    CHECK(s_route_heading_kp==3.0f&&step_heading_kp_deg()==0.3f);
    wire_ack(wire_request,2u,0u);xy_advance(300u);task31_snapshot();
    CHECK(task31_status.state==VAT_ROUTE_SEARCH&&task31_search_speed_exact(200.0f)==0);
    xy_advance(300u); CHECK(last_x == 200.0f);
    float original_heading=task31_status.yaw_target;
    host_absolute_yaw=original_heading+0.5f;xy_advance(20u);
    CHECK(fabsf(last_w-(-0.5f*3.0f*0.0174533f))<0.00001f&&step_heading_kp_deg()==0.3f);
    CHECK(!vision_align_test_route_search_kp_set(1.0f));
    host_absolute_yaw=original_heading;host_tick+=20u;xy_obj(wire_request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX);task31_snapshot();
    CHECK(task31_status.state==VAT_BRAKE&&stopped()&&task31_status.yaw_ever);
    unsigned zero_before=(unsigned)zero_calls;
    host_absolute_yaw=original_heading+0.5f;xy_advance(260u);task31_snapshot();
#if VAT_ROUTE31_BALL_HOSTAGE_STOP_YAW_ENABLE
    CHECK(task31_status.state==VAT_YAW_FIX&&task31_status.yaw_dirty&&stopped());
    xy_advance(20u);CHECK(last_w<0.0f&&last_x==0.0f&&last_y==0.0f);
    host_absolute_yaw=original_heading;xy_advance(20u);
    task31_fresh_images_ms(400u,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX);task31_snapshot();
    CHECK(task31_status.state==VAT_RECHECK&&stopped()&&!task31_status.good&&!task31_status.latest&&
          (unsigned)zero_calls==zero_before&&step_heading_kp_deg()==0.3f);
    CHECK(task31_aim(4,VAT_BALL_Y_PX,VAT_WAIT_BALL_ACTION)==0);run_cmd("0");
#else
    int before_pulses=pulse_calls, before_servos=servo_calls;
    CHECK(task31_status.state==VAT_RECHECK && stopped() && !task31_status.good &&
          !task31_status.latest && (unsigned)zero_calls==zero_before &&
          host_absolute_yaw==original_heading+0.5f && task31_status.yaw_target==original_heading);
    for (unsigned frame=0u;frame<5u;++frame) {
        host_tick+=20u;xy_obj(wire_request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX);task31_snapshot();
        CHECK(task31_status.state!=VAT_YAW_FIX && !last_x && !last_y && !last_w &&
              (unsigned)zero_calls==zero_before && task31_status.good==frame+1u);
        if (frame<4u) CHECK(!host_timer_active && pulse_calls==before_pulses && servo_calls==before_servos);
    }
    CHECK(task31_status.state==VAT_WAIT_BALL_ACTION && !task31_status.ball_fallback &&
          host_timer_active && s_route31_lift_phase==R31_LIFT_BALL_DOWN);
    run_cmd("0");CHECK(stopped());

    CHECK(task31_at_gain(ROUTE31_HOSTAGE_STAGE,"123",3.0f)==0);
    wire_ack(wire_request,2u,0u);xy_advance(600u);task31_snapshot();
    original_heading=task31_status.yaw_target;
    task31_rank(wire_request,0u,2u);
    host_tick+=20u;xy_obj(wire_request,0,VAT_ROUTE_HOSTAGE_X_PX,VAT_HOSTAGE_Y_PX);
    task31_snapshot();CHECK(task31_status.state==VAT_BRAKE && stopped());
    zero_before=(unsigned)zero_calls;
    before_pulses=pulse_calls;before_servos=servo_calls;
    host_absolute_yaw=original_heading-0.5f;xy_advance(260u);task31_snapshot();
    CHECK(task31_status.state==VAT_RECHECK && !task31_status.good && !task31_status.latest &&
          stopped() && (unsigned)zero_calls==zero_before && servo_calls==before_servos && pulse_calls==before_pulses);
    for (unsigned frame=0u;frame<5u;++frame) {
        host_tick+=20u;xy_obj(wire_request,0,VAT_ROUTE_HOSTAGE_X_PX,VAT_HOSTAGE_Y_PX);task31_snapshot();
        CHECK(task31_status.state!=VAT_YAW_FIX && !last_x && !last_y && !last_w &&
              (unsigned)zero_calls==zero_before && task31_status.good==frame+1u);
        if (frame<4u) CHECK(!host_timer_active && pulse_calls==before_pulses && servo_calls==before_servos);
    }
    CHECK(task31_status.state==VAT_WAIT_HOSTAGE_ACTION && task31_status.alignment_confirmed &&
          !task31_status.hostage_fallback && task31_status.target_rank==2u &&
          host_timer_active && s_route31_lift_phase==R31_HOSTAGE_RACK_EXTEND && host_servo==1150u);
    run_cmd("0");CHECK(stopped());
#endif
    CHECK(task31_at_gain(ROUTE31_HOSTAGE_STAGE,"123",0.0f)==0);
    wire_ack(wire_request,2u,0u);xy_advance(300u);xy_advance(300u);task31_snapshot();
    host_absolute_yaw=task31_status.yaw_target+0.5f;xy_advance(20u);
    CHECK(task31_status.state==VAT_ROUTE_SEARCH&&last_x==200.0f&&last_w==0.0f&&
          s_route_heading_kp==0.0f&&step_heading_kp_deg()==0.3f);run_cmd("0");
#if VAT_ROUTE31_BALL_HOSTAGE_STOP_YAW_ENABLE
    puts("EXPLICIT STOP-YAW HISTORY31:SEARCH realykp3/ykp0 private(global0.3 untouched),late0.5deg corrected400ms/nozero/newXY beforelift passed");
#else
    puts("CURRENT31 SEARCH:real movingykp3/ykp0 private(global0.3 untouched);BALL/HOSTAGE stopped +/-0.5deg never commands stationaryyaw/zero,still250 and fiveNEW pixels commit normalarm;rank and g cancel retained passed");
#endif
    return 0;
}

static int task31_wait_rank(void)
{
    CHECK(task31_hostage_start(0u) == 0);
    CHECK(task31_hostage_grab_done() == 0);
    task31_snapshot();
    CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_HOSTAGE_STAGE &&
          !s_route31_hostage_hold && !s_route31_hostage_rank && s_receiving && stopped());
    CHECK(task31_status.state == VAT_WAIT_HOSTAGE_RANK && !task31_status.target_rank &&
          strcmp(task31_status.reason, "WAIT_HOSTAGE_RANK") == 0);
    CHECK(task31_status.x_goal == VAT_ROUTE_HOSTAGE_X_PX && task31_status.alignment_confirmed && s_route31_hostage_opened &&
          host_servo == task31_grip_expected() && servo_calls == 2 && pulse_calls == 6400);
    return 0;
}

static int task31_rank_hold(uint8_t rank)
{
    CHECK(task31_wait_rank() == 0);
    task31_rank((uint16_t)(wire_request - 1u), 0u, rank);
    xy_advance(20u); CHECK(!s_route31_hostage_hold && s_receiving && stopped());
    task31_rank(wire_request, 0u, rank);
    task31_snapshot(); /* Next owner poll can proceed without an observation wait. */
    CHECK(task31_status.state == VAT_DONE && s_route31_hostage_hold);
    ProtoTargetRank old;
    CHECK(s_route31_hostage_rank == rank && !proto_target_rank_get(&old) && !s_receiving && stopped());
    CHECK(host_servo == task31_grip_expected() && servo_calls == 2 && pulse_calls == 6400 && s_route31_hostage_opened);
    return 0;
}

static int check_hostage_real_rank_hold_and_final_route(void)
{
    const unsigned distances[] = {1415u, 1315u, 1215u};
    for (uint8_t rank = 1u; rank <= 3u; ++rank) {
        CHECK(task31_rank_hold(rank) == 0); /* Same QR123/shape0, three different first-seen ranks. */
        unsigned request = wire_request;
        CHECK(ROUTE31_HOSTAGE_HOLD_MS == 0u);
        wire_poll(); CHECK(s_seq_state == SQ_STILL && stopped());
        host_counts[0]++; wire_poll(); /* Final motion still has its usual wheel-still preparation. */
        CHECK(s_seq_state == SQ_STILL && stopped());
        CHECK(s_seq_stage == ROUTE31_HOSTAGE_EXIT_STAGE && route_seq_leg()->distance_mm == distances[rank-1u] &&
              route_seq_leg()->speed_mms == 200 && route_seq_leg()->mode == 15u && wire_request == request);
        CHECK(sequence_start_stage() == 0 && s_seq_state == SQ_RUN && s_v == 200.0f &&
              s_dist_ff_ratio == -ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO && last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO) < 0.0001f);
        CHECK(sequence_finish_stage() == 0 && s_seq_state == SQ_DONE && stopped());
        CHECK(!s_route31_hostage_hold && !laser_state && !s_receiving && pulse_calls == 6400 && servo_calls == 2 && host_servo == task31_grip_expected());
        run_cmd("g"); xy_advance(3000u); CHECK(s_seq_state == SQ_DONE && stopped());
    }
    const char *const keys[] = {"g", "a", "0"};
    for (unsigned phase = 0u; phase < 4u; ++phase) for (unsigned k = 0u; k < 3u; ++k) {
        if (phase == 0u) CHECK(task31_wait_rank() == 0);
        else {
            CHECK(task31_rank_hold(2u) == 0);
            if (phase >= 2u) { wire_poll(); CHECK(s_seq_state == SQ_STILL); }
            if (phase == 3u) CHECK(sequence_start_stage() == 0 && s_seq_state == SQ_RUN);
        }
        uint16_t old = wire_request; unsigned commands = wire_commands;
        run_cmd(keys[k]); CHECK(s_seq_state == SQ_STOPPED && !s_route31_hostage_hold && stopped());
        task31_rank(old, 0u, 2u); xy_obj(old, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
        CHECK(task31_service_ms(10000u) == 0); run_cmd("g");
        CHECK(s_seq_state == SQ_STOPPED && !s_receiving && wire_commands == commands && stopped());
    }
    puts("31 hostage:real54 rank0 waitsRX/noQR-as-rank,oldrequest refused;rank1/2/3 sameQR -> rankknown/no extra hold -> FWD1415/1315/1215v200/privateHOSTAGE_EXIT_RIGHT_FF/DONE;12 g/a/0 cancels atWAIT_RANK/PENDING_NEXT/finalPREP/RUN passed");
    return 0;
}

static int check_ball_needs_no_rank_before_bucket_request(void)
{
    CHECK(task31_at(ROUTE31_PAIR_STAGE, "123") == 0);
    wire_ack(wire_request, 2u, 0u); task31_rank(wire_request, 4u, 3u);
    CHECK(task31_aim(4, VAT_BALL_Y_PX, VAT_WAIT_BALL_ACTION) == 0);
    CHECK(task31_status.target_rank == 3u && task31_status.ball_rank == 3u && s_receiving);
    CHECK(task31_ball_down_done() == 0 && task31_ball_up_begin() == 0 && task31_ball_up_done() == 0);
    CHECK(wire_task == 4u && wire_digit == 0u);
    ProtoTargetRank old; CHECK(!proto_target_rank_get(&old));
    run_cmd("0"); CHECK(stopped());
    CHECK(task31_status.ball_rank == 3u);
    puts("31 ball:real task1/54 rank3 captured before newbucket request,notQR color;normal01 alignment/lift stays gated passed");
    return 0;
}

static int task31_wait_ack_report(uint16_t request, uint8_t task, uint8_t digit,
                                  const ProtoWireDiag *wire)
{
    char contract[160], counters[192];
    const char *report = strstr(host_messages, "reason=WAIT_TASK_ACK");
    CHECK(report != NULL);
    CHECK(strstr(report + strlen("reason=WAIT_TASK_ACK"), "reason=WAIT_TASK_ACK") == NULL);
    snprintf(contract, sizeof contract,
             "want=%u,%u,%u active=%u,%u,%u mode=%u ack=%u rx_on=%u failed=%u",
             (unsigned)request, (unsigned)task, (unsigned)digit,
             (unsigned)wire->request, (unsigned)wire->task, (unsigned)wire->selection,
             (unsigned)wire->mode, (unsigned)wire->ack, (unsigned)wire->receiving,
             (unsigned)wire->failed);
    snprintf(counters, sizeof counters,
             "txtry=%lu rxB=%lu ackN=%lu oldAck=%lu echoCmd=%lu",
             (unsigned long)wire->tx_attempts, (unsigned long)wire->rx_bytes,
             (unsigned long)wire->ack_packets, (unsigned long)wire->ack_mismatch,
             (unsigned long)wire->command_echo);
    CHECK(strstr(report, contract) != NULL && strstr(report, counters) != NULL);
    CHECK(!strstr(host_messages, "status=DONE") && !strstr(host_messages, "phase=DONE"));
    return 0;
}

static int check_route31_wait_task_ack_diagnostics(void)
{
    /* Real parser evidence remains cumulative across QR and task sessions.
     * Wrong ACK/echo evidence cannot replace the live task identity or let
     * the reporter close RX, start motion, or claim an aligned/DONE result. */
    const uint8_t tasks[] = {PROTO_TASK_BALL, PROTO_TASK_HOSTAGE, PROTO_TASK_BUCKET};
    const uint8_t digits[] = {1u, 3u, 0u};
    const int models[] = {4, 0, 9};
    for (unsigned kind = 0u; kind < 3u; ++kind) {
        CHECK(task31_at(kind == 1u ? ROUTE31_HOSTAGE_STAGE : ROUTE31_PAIR_STAGE, "123") == 0);
        if (kind == 2u) {
            CHECK(task31_aim(4, VAT_BALL_Y_PX, VAT_WAIT_BALL_ACTION) == 0);
            CHECK(task31_ball_down_done() == 0 && task31_ball_up_begin() == 0 && task31_ball_up_done() == 0);
            CHECK(task31_finish_motion() == 0);
        }
        task31_snapshot();
        uint16_t request = wire_request;
        unsigned stage = s_seq_stage;
        int pulses = pulse_calls, servos = servo_calls;
        ProtoWireDiag before, reported, rejected;
        real_proto_wire_diag_get(&before);
        CHECK(before.request == request && before.task == tasks[kind] && before.selection == digits[kind] &&
              before.mode == 2u && !before.ack && before.receiving && !before.failed);
        CHECK(before.rx_bytes > 0u && before.ack_packets > 0u && task31_status.request == request);
        CHECK(T_TARGET35_REPORT_MS == 500u && s_seq_state == SQ_TASK && stopped());
        host_messages[0] = '\0';
        host_tick = s_seq_task_report_t0 + 499u; wire_poll();
        CHECK(!strstr(host_messages, "WAIT_TASK_ACK") && stopped() && s_receiving);
        host_tick += 1u; wire_poll(); task31_snapshot();
        real_proto_wire_diag_get(&reported);
        CHECK(task31_wait_ack_report(request, tasks[kind], digits[kind], &reported) == 0);
        CHECK(reported.ack_packets == before.ack_packets && reported.rx_bytes == before.rx_bytes &&
              reported.tx_attempts > before.tx_attempts && s_seq_stage == stage && stopped());

        /* Old ACK and coordinate packets have real CRCs. A self-command echo
         * also increments its counter, but all three remain non-authorizing. */
        host_messages[0] = '\0';
        wire_ack((uint16_t)(request - 1u), 2u, 0u);
        xy_obj((uint16_t)(request - 1u), models[kind], task31_goal(models[kind]), 80);
        xy_obj(request, models[kind], task31_goal(models[kind]), 80);
        uint8_t echo[5] = {0x63u, (uint8_t)request, (uint8_t)(request >> 8), tasks[kind], digits[kind]};
        wire_feed_raw(echo, sizeof echo, 0); wire_poll();
        real_proto_wire_diag_get(&rejected);
        CHECK(!rejected.ack && rejected.receiving && !rejected.failed && rejected.request == request &&
              rejected.task == tasks[kind] && rejected.selection == digits[kind]);
        CHECK(rejected.ack_packets == reported.ack_packets + 1u &&
              rejected.ack_mismatch == reported.ack_mismatch + 1u &&
              rejected.command_echo == reported.command_echo + 1u && rejected.rx_bytes > reported.rx_bytes);
        CHECK(stopped() && s_seq_state == SQ_TASK && s_seq_stage == stage &&
              pulse_calls == pulses && servo_calls == servos && !host_timer_active);
        host_tick = s_seq_task_report_t0 + 499u; wire_poll();
        CHECK(!strstr(host_messages, "WAIT_TASK_ACK") && stopped());
        host_tick += 1u; wire_poll(); task31_snapshot();
        real_proto_wire_diag_get(&reported);
        CHECK(task31_wait_ack_report(request, tasks[kind], digits[kind], &reported) == 0);
        CHECK(reported.ack_packets == rejected.ack_packets && reported.rx_bytes == rejected.rx_bytes &&
              !task31_status.good && task31_status.state != VAT_DONE && s_receiving && !host_receive_closed);
        CHECK(stopped() && s_seq_state == SQ_TASK && s_seq_stage == stage &&
              pulse_calls == pulses && servo_calls == servos && !host_timer_active);

        wire_ack(request, 2u, 0u); xy_advance(20u); task31_snapshot();
        real_proto_wire_diag_get(&rejected);
        CHECK(rejected.ack && rejected.receiving && !rejected.failed &&
              rejected.request == request && rejected.ack_packets == reported.ack_packets + 1u);
        if (kind == 2u) {
            CHECK(task31_status.state == VAT_RECHECK && stopped());
            xy_advance(VAT_ROUTE_BUCKET_MISSING_MS + 20u); task31_snapshot();
            CHECK(task31_status.state == VAT_ROUTE_BUCKET_SEARCH && last_x < 0.0f && last_x >= -200.0f);
            xy_advance(300u); task31_snapshot(); CHECK(last_x == -200.0f);
        } else CHECK(task31_status.state == VAT_ROUTE_SEARCH && task31_search_speed_exact(200.0f) == 0);
        CHECK(last_y == 0.0f && last_w == 0.0f && s_seq_state == SQ_TASK && s_seq_stage == stage &&
              s_receiving && !host_receive_closed && !task31_status.good &&
              pulse_calls == pulses && servo_calls == servos && !host_timer_active && !laser_state);
        host_messages[0] = '\0';
        host_tick = s_seq_task_report_t0 + 500u; wire_poll();
        CHECK(strstr(host_messages, "XY31 run=") != NULL && !strstr(host_messages, "WAIT_TASK_ACK") &&
              !strstr(host_messages, "status=DONE") && !strstr(host_messages, "phase=DONE") && s_receiving);
        run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving);
    }
    puts("31 WAIT_TASK_ACK:auto500ms only,live want/active and cumulative QR+task rxB/ackN/oldAck/echoCmd; missingACK/oldACK/echo/preACKcoordinates cannot move or closeRX/DONE; matchingACK resumes ball/hostage200 or bucket wait2/continuous back200, diagnostics normalize without gate bypass passed");
    return 0;
}

/* Stop at PREP/RUN/first-brake/overshoot-correction/final-brake. Full400ms
 *turn hold gates fresh bucket/target RX, or an explicitly historical offset. */
static int task31_turn_phase(unsigned phase)
{
    CHECK(phase <= 4u && s_seq_mode == ROUTE_TEST_MODE && s_seq_state == SQ_STILL &&
          s_msel == 22 && turn_target_deg() ==
              (s_seq_stage == ROUTE31_RETURN180_STAGE && !s_seq_pair_turn ? 182.0f : 180.0f) &&
          !s_seq_pair_offset && !s_seq_return_offset);
    if (phase) CHECK(sequence_start_stage() == 0 && s_seq_state == SQ_RUN && s_round == R_RUN);
    if (phase >= 2u) {
        host_absolute_yaw += turn_target_deg();
        host_yaw = turn_target_deg(); wire_poll();
        CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && last_w == 0.0f &&
              !strcmp(s_turn_result, "DONE"));
    }
    if (phase >= 3u) {
        /* Inertia reopens the same rotation; no bucket/target task yet. */
        host_yaw = turn_target_deg() + 0.7f;
        host_tick += 20u; wire_poll(); CHECK(s_round == R_RUN);
        host_tick += 20u; wire_poll();
        CHECK(s_round == R_RUN && s_seq_state == SQ_RUN &&
              fabsf(last_w + 0.30f) < 0.000001f && last_x == 0.0f && last_y == 0.0f);
    }
    if (phase == 4u) {
        host_yaw = turn_target_deg(); host_tick += 20u; wire_poll();
        CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && last_w == 0.0f &&
              s_turn_settle_t0 == host_tick);
    }
    CHECK(!s_seq_pair_offset && !s_seq_return_offset && last_x == 0.0f && last_y == 0.0f);
    return 0;
}

static int task31_bucket_turn_phase(unsigned phase)
{
    CHECK(task31_mechanical_phase(R31_LIFT_BALL_UP) == 0 && task31_ball_up_done() == 0);
    CHECK(s_seq_pair_turn && host_servo == task31_grip_expected() && servo_calls == 1);
    uint16_t request = wire_request;
    wire_ack(request, 2u, 0u);
    CHECK(task31_turn_phase(phase) == 0);
    task31_snapshot();
    CHECK(s_seq_pair_turn && s_seq_stage == ROUTE31_PAIR_STAGE &&
          task31_status.state == VAT_TURN_ACTIVE && wire_request == request && host_servo == task31_grip_expected() &&
          servo_calls == 1 && host_jog_pulses[0][0] == 3400u && !host_jog_pulses[0][1] &&
          host_jog_pulses[1][0] == 32000u && host_jog_pulses[1][1] == 32000u && !host_timer_active);
    return 0;
}

/* Current LEFT15 and explicit historical LEFT20 stay under the route controller
 * at PREP/RUN/BRAKE/CORRECT/HOLD.
 * Camera packets cannot notify VAT or alter this finite offset's commands. */
static int task31_bucket_offset_phase(unsigned phase)
{
    CHECK(phase < 5u && task31_bucket_turn_phase(4u) == 0);
    uint16_t request = wire_request;
    host_tick += 400u; wire_poll(); task31_snapshot();
    CHECK(s_seq_state == SQ_STILL && s_seq_pair_offset && !s_seq_pair_turn &&
          route_seq_leg()->mode == 17u && route_seq_leg()->distance_mm == ROUTE31_BALL_TO_BUCKET_LEFT_MM &&
          route_seq_leg()->heading_hold && s_msel == 17 && s_v == 80.0f && stopped());
    if (phase) CHECK(sequence_start_stage() == 0 && s_dist_target == -(float)ROUTE31_BALL_TO_BUCKET_LEFT_MM &&
                     s_dist_align_enabled && s_dist_heading_kp == 0.3f && s_v == 80.0f && last_y == -80.0f);
    if (phase >= 2u) {
        host_yaw = s_dist_heading0 + 1.0f;
        host_lateral = s_dist_odo0 + s_dist_target; wire_poll();
        CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && stopped());
    }
    if (phase >= 3u) {
        host_tick += T_DIST_STILL_MS; wire_poll();
        CHECK(s_round == R_ALIGN && s_dist_align_started && !s_dist_align_hold);
        host_tick += 20u; wire_poll();
        CHECK(last_x == 0.0f && last_y == 0.0f && last_w < 0.0f);
    }
    if (phase == 4u) {
        host_yaw = s_dist_heading0; host_tick += 20u; wire_poll();
        CHECK(s_round == R_ALIGN && s_dist_align_hold && stopped());
    }
    task31_snapshot();
    CHECK(s_seq_pair_offset && !s_seq_pair_turn && s_seq_stage == ROUTE31_PAIR_STAGE &&
          wire_request == request && task31_status.state == VAT_TURN_ACTIVE &&
          host_servo == task31_grip_expected() && servo_calls == 1 && !host_timer_active);
    return 0;
}

static int check_first_bucket_left20_speed_cap_and_other_pv_scope(void)
{
    static const unsigned requested[] = { 0u, 600u, 40u };
    static const float expected[] = { 80.0f, 80.0f, 40.0f };
    CHECK(ROUTE31_BALL_TO_BUCKET_LEFT_MM ==
#if defined(ROUTE31_BALL_OFFSET_LEGACY_TEST)
          20u &&
#else
          15u &&
#endif
          ROUTE31_BALL_TO_BUCKET_LEFT_V_MMS == 80.0f);
    for (unsigned sample = 0u; sample < 3u; ++sample) {
        task31_pv_override = requested[sample];
        CHECK(task31_bucket_turn_phase(4u) == 0);
        float ram_pv = requested[sample] ? (float)requested[sample] : 250.0f;
        unsigned commands = wire_commands;
        uint16_t old_bucket = wire_request;
        int pulses = pulse_calls, servos = servo_calls;
        host_tick += 399u; wire_poll();
        CHECK(s_seq_pair_turn && !s_seq_pair_offset && s_round == R_BRAKE && stopped());
        ++host_tick; wire_poll(); task31_snapshot();
        CHECK(s_seq_state == SQ_STILL && !s_seq_pair_turn && s_seq_pair_offset &&
              s_v == expected[sample] && route_seq_speed_mms() == expected[sample] &&
              s_route31_lateral_v == ram_pv && route_seq_leg()->distance_mm == ROUTE31_BALL_TO_BUCKET_LEFT_MM &&
              task31_status.x_goal == 105 && wire_request == old_bucket && wire_commands == commands);

        /* Read the real speed dispatcher for unrelated LEFT575 and43 LEFT40;
         * the optional historical hostage LEFT20 has no cap either. */
        s_seq_pair_offset = 0u; s_seq_stage = 0u;
        CHECK(route_seq_speed_mms() == ram_pv);
#if ROUTE31_HOSTAGE_PREGRAB_LEFT_MM > 0u
        s_seq_stage = ROUTE31_HOSTAGE_STAGE; s_seq_hostage_offset = 1u;
        CHECK(route_seq_leg()->distance_mm == 20u && route_seq_speed_mms() == ram_pv);
#else
        CHECK(!s_seq_hostage_offset && ROUTE31_HOSTAGE_PREGRAB_LEFT_MM == 0u);
#endif
        s_seq_hostage_offset = 0u; s_seq_stage = ROUTE31_PAIR_STAGE; s_seq_pair_offset = 1u;
        s_seq_mode = ROUTE_STEP_MODE;
        float saved43 = s_route43_tune.lateral_v; s_route43_tune.lateral_v = 600.0f;
        CHECK(route_seq_leg()->distance_mm == 40u && route_seq_speed_mms() == 600.0f);
        s_route43_tune.lateral_v = saved43; s_seq_mode = ROUTE_TEST_MODE;

        CHECK(sequence_start_stage() == 0 && s_v == expected[sample] &&
              last_y == -expected[sample] && s_dist_target == -(float)ROUTE31_BALL_TO_BUCKET_LEFT_MM && s_dist_align_enabled);
        xy_obj(old_bucket, 9, 105, VAT_BUCKET_Y_PX); task31_snapshot();
        CHECK(task31_status.state == VAT_TURN_ACTIVE && !task31_status.good &&
              wire_commands == commands && pulse_calls == pulses && servo_calls == servos);
        host_lateral = s_dist_odo0 + s_dist_target; host_yaw = s_dist_heading0 + 0.7f; wire_poll();
        CHECK(s_round == R_BRAKE && stopped() && wire_request == old_bucket);
        host_tick += T_DIST_STILL_MS; wire_poll(); ++host_tick; wire_poll();
        CHECK(s_round == R_ALIGN && last_x == 0.0f && last_y == 0.0f && last_w < 0.0f);
        host_yaw = s_dist_heading0; host_tick += 20u; wire_poll();
        CHECK(s_dist_align_hold && stopped());
        host_tick += 399u; wire_poll();
        CHECK(s_seq_pair_offset && s_round == R_ALIGN && wire_request == old_bucket);
        ++host_tick; wire_poll(); wire_sync(); task31_snapshot();
        CHECK(!s_seq_pair_offset && s_seq_state == SQ_TASK && task31_status.state == VAT_BRAKE &&
              task31_status.x_goal == 105 && !task31_status.good && !task31_status.alignment_confirmed &&
              wire_request > old_bucket && wire_commands == commands + 1u &&
              pulse_calls == pulses && servo_calls == servos && s_route31_lateral_v == ram_pv && stopped());
        xy_obj(old_bucket, 9, 105, VAT_BUCKET_Y_PX); task31_snapshot();
        CHECK(!task31_status.good && !task31_status.latest && stopped());
        CHECK(task31_cancel_and_verify("0") == 0);
        task31_pv_override = 0u;
    }
    printf("31 first180 LEFT%u: defaultpv250->v80/pv600->v80/pv40->v40 realPREP/START;otherLEFT575 retainsRAMpv,43LEFT40 allows600;true brake+400ms yaw and NEWbucket105/oldframe isolation preserved passed\n",ROUTE31_BALL_TO_BUCKET_LEFT_MM);
    return 0;
}
#if !defined(ROUTE31_BALL_OFFSET_LEGACY_TEST)
static int check_first_bucket_current_left15_handoff(void)
{
    const char *const keys[] = {"g", "a", "0"};
    CHECK(ROUTE31_BALL_TO_BUCKET_LEFT_MM == 15u);
    CHECK(check_first_bucket_left20_speed_cap_and_other_pv_scope() == 0);
    /* Cover the entire current finite offset, not only the turn deadline. */
    for (unsigned phase = 0u; phase < 5u; ++phase) for (unsigned key = 0u; key < 3u; ++key) {
        CHECK(task31_bucket_offset_phase(phase) == 0);
        unsigned commands = wire_commands;
        uint16_t request = wire_request;
        int pulses = pulse_calls, servos = servo_calls;
        CHECK(task31_cancel_and_verify(keys[key]) == 0 && wire_commands == commands &&
              wire_request == request && pulse_calls == pulses && servo_calls == servos &&
              host_servo == 2100u && !s_seq_pair_offset);
    }
    for (unsigned key = 0u; key < 3u; ++key) {
        CHECK(task31_bucket_turn_phase(4u) == 0);
        unsigned commands = wire_commands;
        uint16_t request = wire_request;
        int pulses = pulse_calls, servos = servo_calls;
        host_tick += 400u; /* Cancel wins at the exact LEFT15 preparation deadline. */
        CHECK(task31_cancel_and_verify(keys[key]) == 0 && wire_commands == commands &&
              wire_request == request && pulse_calls == pulses && servo_calls == servos &&
              host_servo == 2100u && !s_seq_pair_offset);
    }
    puts("31 CURRENT first180 LEFT15: true overshoot reverse+399/400ms -> finite left15 capped80 -> original-yaw400 -> NEWbucket105; old/preACK pixels,15 offset-phase g/a/0 and turn-deadline isolation passed");
    return 0;
}
#endif
static int check_bucket_turn_order_frames_cancel_and_faults(void)
{
    const char *const keys[] = {"g", "a", "0"};
    for (unsigned phase = 0u; phase < 5u; ++phase)
        for (unsigned k = 0u; k < 3u; ++k) {
            CHECK(task31_bucket_turn_phase(phase) == 0);
            unsigned commands = wire_commands;
            int pulses = pulse_calls, servos = servo_calls;
            uint16_t request = wire_request;
            /* Even five aligned pixels from the pre-turn request cannot let
             * VAT countermand the180/settle controller or lower the arm. */
            for (unsigned frame = 0u; frame < 5u; ++frame) {
                host_tick += 20u; xy_obj(request, 9, VAT_ROUTE_BUCKET_X_PX, VAT_BUCKET_Y_PX); task31_snapshot();
                CHECK(task31_status.state == VAT_TURN_ACTIVE && !task31_status.good &&
                      s_seq_pair_turn && !s_seq_pair_offset && wire_request == request && wire_commands == commands);
                CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == task31_grip_expected() && !host_timer_active);
            }
            static const char *const writes[] = {"42", "v300", "d40", "u1000", "su1000"};
            for (unsigned w = 0u; w < sizeof writes / sizeof writes[0]; ++w) {
                run_cmd(writes[w]); wire_poll();
                CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") && s_seq_pair_turn && !s_seq_pair_offset &&
                      wire_request == request && servo_calls == servos && host_servo == task31_grip_expected());
            }
            CHECK(task31_cancel_and_verify(keys[k]) == 0);
        }
#if defined(ROUTE31_BALL_OFFSET_LEGACY_TEST)
    for (unsigned phase = 0u; phase < 5u; ++phase)
        for (unsigned k = 0u; k < 3u; ++k) {
            CHECK(task31_bucket_offset_phase(phase) == 0);
            unsigned commands = wire_commands;
            int pulses = pulse_calls, servos = servo_calls;
            uint16_t request = wire_request;
            float x = last_x, y = last_y, w = last_w;
            for (unsigned frame = 0u; frame < 5u; ++frame) {
                xy_obj(request, 9, VAT_ROUTE_BUCKET_X_PX, VAT_BUCKET_Y_PX); task31_snapshot();
                CHECK(s_seq_pair_offset && !s_seq_pair_turn && wire_request == request &&
                      wire_commands == commands && task31_status.state == VAT_TURN_ACTIVE &&
                      !task31_status.good && last_x == x && last_y == y && last_w == w &&
                      pulse_calls == pulses && servo_calls == servos && host_servo == task31_grip_expected() && !host_timer_active);
            }
            CHECK(task31_cancel_and_verify(keys[k]) == 0);
        }
    CHECK(task31_bucket_turn_phase(4u) == 0);
    uint16_t old_request = wire_request;
    int pulses = pulse_calls, servos = servo_calls;
    host_tick += 399u; wire_poll();
    CHECK(s_seq_pair_turn && !s_seq_pair_offset && s_seq_state == SQ_RUN &&
          s_round == R_BRAKE && wire_request == old_request && task31_status.state == VAT_TURN_ACTIVE);
    host_tick += 1u; wire_poll(); wire_sync(); task31_snapshot();
    CHECK(s_seq_stage == ROUTE31_PAIR_STAGE && s_seq_pair_offset && !s_seq_pair_turn &&
          s_seq_state == SQ_STILL && s_msel == 17 && route_seq_leg()->distance_mm == 20u &&
          task31_status.state == VAT_TURN_ACTIVE && wire_request == old_request && stopped());
    CHECK(sequence_start_stage() == 0 && s_dist_target == -20.0f && s_dist_align_enabled && last_y < 0.0f);
    host_yaw = s_dist_heading0 + 1.0f;
    host_lateral = s_dist_odo0 + s_dist_target; wire_poll();
    CHECK(s_round == R_BRAKE && s_seq_pair_offset && wire_request == old_request && stopped());
    host_tick += T_DIST_STILL_MS; wire_poll();
    CHECK(s_round == R_ALIGN && s_seq_pair_offset && wire_request == old_request);
    host_tick += 20u; wire_poll(); task31_snapshot();
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w < 0.0f &&
          task31_status.state == VAT_TURN_ACTIVE && wire_request == old_request);
    host_yaw = s_dist_heading0; host_tick += 20u; wire_poll();
    CHECK(s_dist_align_hold && stopped());
    host_tick += 399u; wire_poll(); task31_snapshot();
    CHECK(s_round == R_ALIGN && s_seq_pair_offset && wire_request == old_request &&
          task31_status.state == VAT_TURN_ACTIVE && stopped());
    host_tick += 1u; wire_poll(); wire_sync(); task31_snapshot();
    CHECK(s_seq_stage == ROUTE31_PAIR_STAGE && !s_seq_pair_offset && !s_seq_pair_turn &&
          s_seq_state == SQ_TASK && wire_request > old_request && wire_task == 4u &&
          task31_status.state == VAT_BRAKE && !task31_status.latest && !task31_status.good);
    CHECK(host_servo == task31_grip_expected() && servo_calls == servos && pulse_calls == pulses && stopped());
    /* Offset and yaw-fix are complete. Old turn-time frames and new preACK frames
     * cannot move the chassis, release the claw or arm the2s loss fallback. */
    xy_obj(old_request, 9, VAT_ROUTE_BUCKET_X_PX, VAT_BUCKET_Y_PX);
    xy_obj(wire_request, 9, VAT_ROUTE_BUCKET_X_PX, VAT_BUCKET_Y_PX); xy_advance(1000u); task31_snapshot();
    CHECK(stopped() && !task31_status.good && !task31_status.bucket_seen && !host_timer_active &&
          s_seq_stage == ROUTE31_PAIR_STAGE && !s_seq_pair_offset && !s_seq_return_offset);
    wire_ack(wire_request, 2u, 0u); xy_advance(260u); task31_snapshot();
    for (unsigned frame = 0u; frame < 5u; ++frame) {
        host_tick += 20u; xy_obj(old_request, 9, VAT_ROUTE_BUCKET_X_PX, VAT_BUCKET_Y_PX); task31_snapshot();
        CHECK(!task31_status.latest && !task31_status.good && !task31_status.bucket_seen &&
              pulse_calls == pulses && servo_calls == servos && stopped());
    }
    CHECK(task31_cancel_and_verify("0") == 0);
#endif
    for (unsigned cause = 0u; cause < 3u; ++cause) {
        CHECK(task31_bucket_turn_phase(cause == 2u ? 3u : 1u) == 0);
        int p = pulse_calls, s = servo_calls;
        if (cause == 0u) host_imu_valid = 0;
        else if (cause == 1u) host_abort = 1;
        else host_tick = s_meas_t0 + T_TURN180_MAX_MS;
        wire_poll();
        if (cause == 2u) {
            CHECK(s_seq_state == SQ_RUN && s_round == R_BRAKE &&
                  !strcmp(s_turn_result, "TIMEOUT") && s_seq_pair_turn && stopped());
            CHECK(task31_service_ms(ROUTE31_STABLE_MS) == 0);
        }
        CHECK(s_seq_state == SQ_STOPPED && !s_seq_pair_offset && !s_seq_pair_turn &&
              pulse_calls == p && servo_calls == s && host_servo == task31_grip_expected() && stopped());
        CHECK(task31_service_ms(10000u) == 0); run_cmd("g");
        CHECK(s_seq_state == SQ_STOPPED && pulse_calls == p && servo_calls == s && stopped());
    }
    puts("31 first180:VAT paused through PREP/RUN/BRAKE/overshoot/finalHOLD;15 g/a/0,5write locks and3IMU/abort/turn-timeout faults retain2100/rack; historicalLEFT20 additional phases isolated by compile flag passed");
    return 0;
}

static int check_release_ready_one_second_and_late_poll(void)
{
    static const unsigned delays[] = {999u, 1000u, 1500u, 2500u};
    for (unsigned k = 0u; k < sizeof delays / sizeof delays[0]; ++k) {
        CHECK(task31_mechanical_phase(R31_CLAW_RELEASE_WAIT) == 0);
        uint32_t release_t0 = s_route31_lift_wait_t0;
        int pulses = pulse_calls, servos = servo_calls;
        CHECK(host_servo == 1150u && !host_timer_active);
        host_tick = release_t0 + delays[k]; wire_poll();
        if (delays[k] < 1000u) {
            CHECK(s_route31_lift_phase == R31_CLAW_RELEASE_WAIT && host_servo == 1150u && servo_calls == servos);
            host_tick = release_t0 + 1000u; wire_poll();
        }
        CHECK(s_route31_lift_phase == R31_CLAW_RESTAGED_WAIT && s_route31_lift_wait_t0 == release_t0 &&
              host_servo == 1500u && servo_calls == servos + 1 && pulse_calls == pulses && stopped());
        if (delays[k] < 2000u) {
            host_tick = release_t0 + 1999u; wire_poll();
            CHECK(s_route31_lift_phase == R31_CLAW_RESTAGED_WAIT && !host_timer_active &&
                  servo_calls == servos + 1 && pulse_calls == pulses && stopped());
            host_tick = release_t0 + 2000u;
        }
        wire_poll();
        CHECK(s_route31_lift_phase == R31_LIFT_BUCKET_UP && host_timer_active &&
              host_jog_axis == 1 && host_jog_dir == 0 && host_servo == 1500u &&
              pulse_calls == pulses && servo_calls == servos + 1 && s_seq_stage == ROUTE31_PAIR_STAGE);
        CHECK(task31_cancel_and_verify("0") == 0);
    }
    puts("31 release:actual1150 t0+999 stays1150,1000 exactly1500 once/phase12,1999 no up/2000 up;1500/2500 latepoll retains original2s origin without duplicateclaw or earlySTEP passed");
    return 0;
}

static int check_hostage_alignment_opens_before_rank_once(void)
{
    CHECK(task31_wait_rank() == 0);
    uint16_t request = wire_request;
    CHECK(task31_status.alignment_confirmed && s_route31_hostage_opened && host_servo == task31_grip_expected() && servo_calls == 2);
    for (unsigned frame = 0u; frame < 25u; ++frame) {
        host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX); task31_snapshot();
        CHECK(task31_status.alignment_confirmed && !task31_status.target_rank && !s_route31_hostage_hold &&
              s_receiving && stopped() && host_servo == task31_grip_expected() && servo_calls == 2 && pulse_calls == 6400);
    }
    task31_rank((uint16_t)(request - 1u), 0u, 3u); xy_advance(20u);
    CHECK(!s_route31_hostage_hold && servo_calls == 2 && s_receiving);
    task31_rank(request, 0u, 3u);
    task31_snapshot();
    CHECK(task31_status.state == VAT_DONE && s_route31_hostage_rank == 3u && s_route31_hostage_hold &&
          servo_calls == 2 && pulse_calls == 6400 && host_servo == task31_grip_expected());
    run_cmd("0");
    const char *const keys[] = {"g", "a", "0"};
    for (unsigned k = 0u; k < 3u; ++k) {
        CHECK(task31_at(ROUTE31_HOSTAGE_STAGE, "123") == 0);
        task31_snapshot(); CHECK(!task31_status.alignment_confirmed && !s_route31_hostage_opened && host_servo == 1500u);
        wire_ack(wire_request, 2u, 0u); xy_advance(300u);
        task31_rank(wire_request, 0u, 0u);
        host_tick += 20u; xy_obj(wire_request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
        xy_advance(260u); task31_snapshot();
        CHECK(task31_status.state == VAT_RECHECK && !task31_status.good && !task31_status.alignment_confirmed);
        /* Former cx180 cannot open at the new215 goal; the four fresh correct frames must
         * still be below the five-frame confirmation threshold. */
        host_tick += 20u; xy_obj(wire_request, 0, 180, VAT_HOSTAGE_Y_PX); task31_snapshot();
        CHECK(task31_status.state == task31_fine_x_state() && last_x == -20.0f && !task31_status.alignment_confirmed &&
              !servo_calls && host_servo == 1500u);
        CHECK(task31_stop_step_for_new_image() == 0);
        for (unsigned frame = 0u; frame < 4u; ++frame) {
            host_tick += 20u; xy_obj(wire_request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX); task31_snapshot();
            CHECK(!task31_status.alignment_confirmed && !s_route31_hostage_opened &&
                  host_servo == 1500u && !servo_calls && task31_status.good == frame + 1u);
        }
        host_tick += 20u; xy_obj_poll(wire_request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX, 0);
        CHECK(task31_cancel_and_verify(keys[k]) == 0);
        CHECK(host_servo == 1500u && !servo_calls);
    }
    puts("31 hostage215:oldcx180/fourfresh cannot open;rank0 confirmedpixels opens1150 then rack3400/grip2100/retract3000 once before metadata;25frames/oldrank/latevalidrank never rewrite;3 fifthframe-pending g/a/0 stops retain1500;newrequest confirmation resets passed");
    return 0;
}

static int check_ball_recipe_private_grip_scope(void)
{
    CHECK(ROUTE31_RACK_PPS == 5000u && ROUTE31_LIFT_PPS == 10000u &&
          ROUTE31_RACK_EXTEND_STEPS == 3400u && ROUTE31_HOSTAGE_RACK_EXTEND_STEPS == 3400u &&
          ROUTE43_RACK_EXTEND_STEPS == 3200u &&
          T_GRAB_GRIP_DEFAULT_US == 1900u && ARM_SERVO_GRIP_US == 1700u);
    CHECK(wire_boot() == 0);
    run_cmd("42");
    CHECK(s_grab42_extend == 2500u && s_grab42_down == 0u && s_grab42_u == 1900u && s_grab42_pps == 500u);
    CHECK(host_servo == 1150u && !servo_calls && !pulse_calls && !host_timer_active && stopped());
    run_cmd("cc");
    CHECK(host_servo == 1700u && !pulse_calls && !host_timer_active && stopped());
    CHECK(ROUTE31_BALL_GRIP_US == 2100u && ROUTE31_HOSTAGE_GRIP_US == 2100u &&
          ROUTE43_BALL_GRIP_US == 1900u && ROUTE43_HOSTAGE_GRIP_US == 1900u && s_grab42_u == 1900u);
    run_cmd("28"); CHECK(s_servo_target_us == 0u && host_servo == 1700u && !host_timer_active);
    run_cmd("29"); CHECK(s_servo_target_us == 0u && host_servo == 1700u && !host_timer_active);
    puts("31 recipe private2100 and3400 leave43 rack3200/grip1900 and independent42 defaults2500/0/1900/f500, globalcc1700 and28/29 explicit-u contract unchanged passed");
    return 0;
}

static int task31_loss_search(void)
{
    CHECK(task31_at(ROUTE31_HOSTAGE_STAGE, "123") == 0);
    wire_ack(wire_request, 2u, 0u); CHECK(task31_service_ms(600u) == 0);
    CHECK(task31_status.state == VAT_ROUTE_SEARCH && last_x == 200.0f &&
          !task31_status.hostage_seen && !task31_status.hostage_fallback && !host_timer_active);
    return 0;
}

static int check_hostage_dropout_trigger_and_rank(void)
{
    CHECK(task31_at(ROUTE31_HOSTAGE_STAGE, "123") == 0);
    xy_obj(wire_request, 0, 500, VAT_HOSTAGE_Y_PX); /* PreACK cannot arm a timer. */
    wire_ack(wire_request, 2u, 0u); CHECK(task31_service_ms(600u) == 0);
    xy_obj(wire_request, 1, 500, VAT_HOSTAGE_Y_PX);
    xy_obj((uint16_t)(wire_request - 1u), 0, 500, VAT_HOSTAGE_Y_PX);
    CHECK(task31_service_ms(5000u) == 0 && !task31_status.hostage_seen &&
          !servo_calls && !pulse_calls && !host_timer_active);
    CHECK(task31_cancel_and_verify("0") == 0);

    CHECK(task31_loss_search() == 0);
    uint16_t selected_sequence = (uint16_t)xy_seq;
    xy_obj_poll(wire_request, 0, 500, VAT_HOSTAGE_Y_PX, 0);
    xy_obj_poll(wire_request, -1, 0, 0, 0); wire_poll(); task31_snapshot();
    CHECK(task31_status.hostage_seen && task31_status.hostage_age_ms == 0u);
    CHECK(task31_service_ms(100u) == 0);
    unsigned following_sequence = xy_seq;
    xy_seq = selected_sequence; xy_obj(wire_request, 0, 500, VAT_HOSTAGE_Y_PX);
    xy_seq = following_sequence;
    task31_rank(wire_request, 0u, 0u);
    xy_obj(wire_request, 1, 500, VAT_HOSTAGE_Y_PX);
    xy_obj((uint16_t)(wire_request - 1u), 0, 500, VAT_HOSTAGE_Y_PX);
    CHECK(task31_service_ms(200u) == 0 && task31_status.hostage_age_ms == 300u &&
          !host_timer_active && !servo_calls && !pulse_calls);
    CHECK(task31_service_ms(1u) == 0 && task31_status.hostage_age_ms == 301u &&
          !strcmp(task31_status.reason, "HOSTAGE_LOST300_BRAKE") && stopped());
#if ROUTE31_HOSTAGE_PREGRAB_LEFT_MM == 0u
    unsigned precise_after_loss = host_precise_calls, integer_after_loss = host_integer_calls;
    int zeros_after_loss = zero_calls;
#endif
    CHECK(task31_service_ms(100u) == 0); ++host_counts[0]; wire_poll();
    CHECK(task31_service_ms(249u) == 0 && !host_timer_active && !servo_calls);
    CHECK(task31_service_ms(1u) == 0);
#if ROUTE31_HOSTAGE_PREGRAB_LEFT_MM > 0u
    CHECK(s_seq_hostage_offset &&
          s_route31_lift_phase == R31_LIFT_OFF && !host_timer_active && !servo_calls && !pulse_calls);
    CHECK(task31_hostage_offset_resume() == 0);
    CHECK(task31_service_ms(270u) == 0);
#else
    CHECK(!s_seq_hostage_offset && !s_route31_hostage_offset_done &&
          s_seq_state == SQ_TASK && task31_drive_stopped() &&
          host_precise_calls == precise_after_loss && host_integer_calls == integer_after_loss &&
          zero_calls == zeros_after_loss);
#endif
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND &&
          task31_status.state == VAT_WAIT_HOSTAGE_ACTION && task31_status.hostage_fallback &&
          !task31_status.alignment_confirmed && !task31_status.good && host_servo == 1150u && servo_calls == 1);
    CHECK(task31_hostage_grab_done() == 0 && task31_status.state == VAT_WAIT_HOSTAGE_RANK && s_receiving);
#if ROUTE31_HOSTAGE_PREGRAB_LEFT_MM == 0u
    CHECK(!s_seq_hostage_offset && !s_route31_hostage_offset_done &&
          host_precise_calls == precise_after_loss && host_integer_calls == integer_after_loss &&
          zero_calls == zeros_after_loss && strstr(host_messages, "HOSTAGE31_PREGRAB_LEFT") == NULL &&
          strstr(host_messages, "HOSTAGE_OFFSET_RECHECK") == NULL);
#endif
    CHECK(task31_service_ms(5000u) == 0 && !s_route31_hostage_hold && !s_route31_hostage_rank &&
          pulse_calls == 6400 && servo_calls == 2 && host_servo == task31_grip_expected());
    /* Recovery after committed action cannot repeat or cancel the grip. */
    xy_obj(wire_request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
    task31_rank((uint16_t)(wire_request - 1u), 0u, 2u); task31_rank(wire_request, 1u, 2u);
    CHECK(task31_service_ms(500u) == 0 && task31_status.state == VAT_WAIT_HOSTAGE_RANK);
    task31_rank(wire_request, 0u, 2u); task31_snapshot();
    CHECK(task31_status.state == VAT_DONE && task31_status.target_rank == 2u &&
          task31_status.hostage_fallback && !task31_status.alignment_confirmed && !task31_status.good);
    wire_poll(); CHECK(s_seq_state == SQ_STILL &&
          route_seq_leg()->distance_mm == 1315u && pulse_calls == 6400 && servo_calls == 2);
    CHECK(task31_finish_motion() == 0 && s_seq_state == SQ_DONE && s_route31_rack_deployed && host_servo == task31_grip_expected());
    puts("31 hostage fallback: neverseen/preACK/wrongclass/oldrequest inert;sticky01 despite empty/54/replay;strict300/301+encoderstill250;actual3400/grip2100/retract3000 once,no fakegood/alignment;rank0 RX,real54rank2/noextraHold/final1315 heldgrip passed");
    return 0;
}

static int check_hostage_dropout_recovery_priority_and_fault(void)
{
    CHECK(task31_loss_search() == 0); xy_obj(wire_request, 0, 500, VAT_HOSTAGE_Y_PX);
    CHECK(task31_service_ms(300u) == 0); xy_obj(wire_request, 0, 500, VAT_HOSTAGE_Y_PX);
    task31_snapshot(); CHECK(task31_status.hostage_age_ms == 0u);
    CHECK(task31_service_ms(300u) == 0 && !servo_calls && !host_timer_active);
    CHECK(task31_service_ms(1u) == 0 && !strcmp(task31_status.reason, "HOSTAGE_LOST300_BRAKE"));
    CHECK(task31_service_ms(100u) == 0); xy_obj(wire_request, 0, 500, VAT_HOSTAGE_Y_PX);
    CHECK(task31_service_ms(300u) == 0 && !servo_calls && !host_timer_active);
    CHECK(task31_service_ms(1u) == 0 && !strcmp(task31_status.reason, "HOSTAGE_LOST300_BRAKE"));
    /* Explicit poll->take race: expose pending action without the App owner,
     * receive a new01, then take must atomically recheck it before committing. */
    host_tick += 250u; vision_align_test_poll(); task31_snapshot();
    CHECK(task31_status.state == VAT_WAIT_HOSTAGE_ACTION && task31_status.hostage_fallback && !host_timer_active);
    xy_obj_poll(wire_request, 0, 500, VAT_HOSTAGE_Y_PX, 0);
    CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
    task31_snapshot();
    CHECK(task31_status.state == VAT_BRAKE && !task31_status.hostage_fallback &&
          task31_status.hostage_age_ms == 0u && !servo_calls && !host_timer_active);
    CHECK(task31_cancel_and_verify("0") == 0);

    const char *const keys[] = {"g", "a", "0"};
    for (unsigned fault = 0u; fault < 6u; ++fault) {
        CHECK(task31_loss_search() == 0); xy_obj(wire_request, 0, 500, VAT_HOSTAGE_Y_PX);
        CHECK(task31_service_ms(300u) == 0); ++host_tick;
        if (fault < 3u) CHECK(task31_cancel_and_verify(keys[fault]) == 0);
        else {
            if (fault == 3u) { host_imu_valid = 0; wire_poll(); }
            else if (fault == 4u) { host_abort = 1; wire_poll(); }
            else wire_ack(wire_request, 2u, 1u);
            CHECK(s_seq_state == SQ_STOPPED && stopped() && !pulse_calls && !servo_calls && host_servo == 1500u);
        }
    }
    /* The new seventh job may open before a failed clock start, but never
     * creates a pulse, grips2100, guesses rank, or auto-retries. */
    CHECK(task31_loss_search() == 0); xy_obj(wire_request, 0, 500, VAT_HOSTAGE_Y_PX);
#if ROUTE31_HOSTAGE_PREGRAB_LEFT_MM > 0u
    CHECK(task31_service_ms(301u) == 0 && task31_service_ms(250u) == 0 && s_seq_hostage_offset);
    CHECK(task31_hostage_offset_resume() == 0); host_messages[0] = '\0'; host_timer_start_fail = 1;
    CHECK(task31_service_ms(270u) == 0);
#else
    host_messages[0] = '\0'; host_timer_start_fail = 1;
    CHECK(task31_service_ms(301u) == 0 && task31_service_ms(250u) == 0 &&
          !s_seq_hostage_offset && !s_route31_hostage_offset_done);
#endif
    CHECK(s_seq_state == SQ_STOPPED && !pulse_calls &&
          host_servo == 1150u && servo_calls == 1 && stopped());
    CHECK(strstr(host_messages, "LIFT_START_ERROR"));
    run_cmd("g"); CHECK(task31_service_ms(10000u) == 0 && !pulse_calls && servo_calls == 1 && stopped());
    puts("31 hostage recovery:300 new01 restarts strict300ms;braking and poll->take new01 cancels pendinggrab;g/a/0/IMU/abort/NACK deadline priority;seventhclock-startfault leaves1150/noSTEP/noauto-retry passed");
    return 0;
}

static int task31_return_turn_phase(unsigned phase)
{
    CHECK(phase < 5u && task31_mechanical_phase(R31_RACK_RETRACT_WAIT) == 0);
    CHECK(task31_service_ms(250u) == 0 && s_seq_stage == ROUTE31_RETURN180_STAGE &&
          s_seq_state == SQ_STILL && host_servo == 1500u);
    uint16_t request = wire_request;
    CHECK(task31_turn_phase(phase) == 0);
    CHECK(s_seq_stage == ROUTE31_RETURN180_STAGE && !s_seq_return_offset &&
          !s_target35_route && wire_request == request && host_servo == 1500u);
    return 0;
}
/* A completed182 must hand off directly. Keep the stale distance ledger as
 * evidence that neither a RIGHT15 nor an ostensibly harmless0mm leg started.
 * Target preparation still has its own stop/zero/settle, separate from turn. */
static int task31_return_target_phase(unsigned phase, uint16_t *old_request)
{
    CHECK(phase < 4u && task31_return_turn_phase(4u) == 0);
    uint16_t request = wire_request;
    int zeros = zero_calls;
    float dist_target = s_dist_target, dist_odo0 = s_dist_odo0;
    float lateral = host_lateral, heading = host_absolute_yaw;
    unsigned precise = host_precise_calls, integer = host_integer_calls;
    host_tick += 399u; wire_poll();
    CHECK(s_seq_stage == ROUTE31_RETURN180_STAGE && s_round == R_BRAKE &&
          !s_seq_return_offset && !s_target35_route && wire_request == request &&
          zero_calls == zeros && stopped());
    ++host_tick; wire_poll();
    CHECK(ROUTE31_RETURN_RIGHT_MM == 0u && s_seq_stage == ROUTE31_TARGET_STAGE &&
          s_seq_state == SQ_TASK && s_msel == 31 && s_round == R_RUN && !dist_mode() &&
          !s_seq_return_offset && s_target35_route && wire_task == 2u &&
          wire_request > request && s_target35_phase == TA_TASK_WAIT &&
          zero_calls == zeros && host_precise_calls == precise && host_integer_calls == integer && stopped());
    if (phase >= 1u) {
        host_tick += 20u; wire_poll();
        CHECK(s_target35_phase == TA_STILL && zero_calls == zeros && stopped());
    }
    if (phase >= 2u) {
        host_tick += T_DIST_STILL_MS; wire_poll();
        CHECK(s_target35_phase == TA_PREP_WAIT && zero_calls == zeros + 1 && stopped());
    }
    if (phase == 3u) {
        host_tick += NAV_SETTLE_MS; wire_poll();
        CHECK(s_target35_phase == TA_SEEK && zero_calls == zeros + 1 &&
              !s_target35_seen && !s_target35_good && !laser_state);
    }
    CHECK(!s_seq_return_offset && !dist_mode() && s_seq_state == SQ_TASK &&
          s_seq_stage == ROUTE31_TARGET_STAGE && !laser_state && host_servo == 1500u &&
          s_dist_target == dist_target && s_dist_odo0 == dist_odo0 &&
          host_lateral == lateral && host_absolute_yaw == heading);
    if (old_request) *old_request = request;
    return 0;
}

static int check_return_turn_stability_and_stops(void)
{
    const char *const keys[] = {"g", "a", "0"};
    for (unsigned phase=0u;phase<4u;++phase) for(unsigned key=0u;key<3u;++key){
        uint16_t old_request;
        CHECK(task31_return_target_phase(phase, &old_request)==0);
        uint16_t target_request=wire_request;unsigned commands=wire_commands;
        int pulses=pulse_calls,servos=servo_calls;
        /* Old bucket-request pixels and new target pixels before ACK cannot
         * aim/fire, including while the permitted blind search is running. */
        host_tick+=20u;xy_obj(old_request,8,250,10);xy_obj(target_request,8,250,10);
        CHECK(!s_seq_return_offset && s_target35_route && !laser_state &&
              !s_target35_seen && !s_target35_good && s_seq_stage==ROUTE31_TARGET_STAGE &&
              wire_request==target_request && wire_commands==commands);
        CHECK(task31_cancel_and_verify(keys[key])==0 && !s_seq_return_offset &&
              !s_target35_route && pulse_calls==pulses && servo_calls==servos &&
              wire_commands==commands && host_servo==1500u);
    }
    /* Actual release/rack-return reaches this second182; near-target
     * stronger commands must remain signed, precise and request-gated. */
    for (unsigned side=0u;side<2u;++side) {
        CHECK(task31_return_turn_phase(1u)==0);
        float offset=side?0.5f:-0.5f;
        int zeros=zero_calls;unsigned precise=host_precise_calls, integer=host_integer_calls;
        uint16_t request=wire_request;
        CHECK(turn_target_deg()==182.0f);
        host_yaw=182.0f+offset;host_tick+=20u;wire_poll();
        CHECK(s_round==R_RUN && s_seq_stage==ROUTE31_RETURN180_STAGE &&
              fabsf(last_w+(side?0.30f:-0.30f))<0.000001f && !last_x && !last_y &&
              host_precise_calls==precise+1u && host_integer_calls==integer &&
              zero_calls==zeros && wire_request==request && !s_target35_route && !laser_state);
        CHECK(task31_cancel_and_verify("0")==0);
    }
    for (unsigned phase = 0u; phase < 5u; ++phase)
        for (unsigned k = 0u; k < 3u; ++k) {
            CHECK(task31_return_turn_phase(phase) == 0);
            int pulses = pulse_calls, servos = servo_calls; uint16_t request = wire_request;
            for (unsigned frame = 0u; frame < 5u; ++frame) {
                host_tick += 20u; xy_obj(request, 8, ROUTE31_TARGET_CX, 10);
                CHECK(s_seq_stage == ROUTE31_RETURN180_STAGE && !s_seq_return_offset &&
                      !s_target35_route && !laser_state && wire_request == request &&
                      pulse_calls == pulses && servo_calls == servos);
            }
            CHECK(task31_cancel_and_verify(keys[k]) == 0);
        }
    CHECK(task31_return_turn_phase(1u) == 0);
    uint16_t old_goal_request=wire_request;int old_goal_zeros=zero_calls;
    host_yaw=180.0f;host_tick+=20u;wire_poll();
    CHECK(s_round==R_RUN && s_seq_state==SQ_RUN && s_seq_stage==ROUTE31_RETURN180_STAGE &&
          turn_target_deg()==182.0f && last_w>0.0f && !last_x && !last_y &&
          wire_request==old_goal_request && zero_calls==old_goal_zeros &&
          !s_target35_route && !laser_state);
    CHECK(task31_cancel_and_verify("0")==0);
    CHECK(task31_return_turn_phase(4u) == 0);
    uint16_t request = wire_request; int pulses = pulse_calls, servos = servo_calls;
    int zeros = zero_calls; float absolute_heading = host_absolute_yaw;
    host_messages[0] = '\0'; /* Keep this final-turn REC in the bounded host log. */
    host_tick += 399u; wire_poll();
    CHECK(!s_seq_return_offset && !s_target35_route && s_seq_state == SQ_RUN &&
          s_round == R_BRAKE && s_seq_stage == ROUTE31_RETURN180_STAGE && wire_request == request &&
          zero_calls == zeros && host_absolute_yaw == absolute_heading);
    ++host_tick; wire_poll(); wire_sync();
    CHECK(!s_seq_return_offset && s_seq_stage == ROUTE31_TARGET_STAGE && s_seq_state == SQ_TASK &&
          s_target35_route && wire_task == 2u && wire_request > request &&
          pulse_calls == pulses && servo_calls == servos && host_servo == 1500u && stopped() &&
          zero_calls == zeros && host_absolute_yaw == absolute_heading && !dist_mode() &&
          s_msel == 31 && s_round == R_RUN && ROUTE31_RETURN_RIGHT_MM == 0u);
    CHECK(strstr(host_messages,"cmd_deg=182") && strstr(host_messages,"abs_yaw_deg=") &&
          strstr(host_messages,"route_step=11"));
    CHECK(!s_seq_pair_turn && !s_seq_pair_offset && s_msel == 31 &&
          s_target35_phase == TA_TASK_WAIT && !s_target35_seen && !s_target35_good);
    uint16_t fresh_request = wire_request;
    for (unsigned frame = 0u; frame < 5u; ++frame) {
        host_tick += 20u; xy_obj(request, 8, ROUTE31_TARGET_CX, 10);
        xy_obj(fresh_request, 8, ROUTE31_TARGET_CX, 10);
        CHECK(stopped() && !s_target35_seen && !s_target35_good &&
              pulse_calls == pulses && servo_calls == servos && !laser_state);
    }
    CHECK(task31_cancel_and_verify("0") == 0);
    for (unsigned cause = 0u; cause < 3u; ++cause) {
        CHECK(task31_return_turn_phase(cause == 2u ? 3u : 1u) == 0);
        int p = pulse_calls, s = servo_calls;
        if (cause == 0u) host_imu_valid = 0;
        else if (cause == 1u) host_abort = 1;
        else host_tick = s_meas_t0 + T_TURN180_MAX_MS;
        wire_poll();
        if (cause == 2u) {
            CHECK(s_seq_state == SQ_RUN && s_round == R_BRAKE &&
                  !strcmp(s_turn_result, "TIMEOUT") && !s_target35_route && stopped());
            CHECK(task31_service_ms(ROUTE31_STABLE_MS) == 0);
        }
        CHECK(s_seq_state == SQ_STOPPED && !s_seq_return_offset && !s_target35_route &&
              pulse_calls == p && servo_calls == s && host_servo == 1500u && stopped());
        CHECK(task31_service_ms(10000u) == 0); run_cmd("g");
        CHECK(s_seq_state == SQ_STOPPED && pulse_calls == p && servo_calls == s && stopped());
    }
    puts("31 return182 direct TARGET/no return lateral or0mm job: old180 staysRUN;actual +/-0.5 precise +/-0.30,overshoot/399-400ms hold;15 turn and12 target-handoff/prep g/a/0 stops;original absoluteheading retained;old/preACK targetpixels inert;target preparation zero once only afterstill250;REC182 and3IMU/abort/turn-timeout faults passed");
    return 0;
}

static int task31_bucket_search_start(void)
{
    CHECK(task31_mechanical_phase(R31_LIFT_BALL_UP) == 0 &&
          task31_ball_up_done() == 0 && task31_finish_motion() == 0);
    wire_ack(wire_request,2u,0u);
    CHECK(task31_service_ms(VAT_ROUTE_BUCKET_MISSING_MS + 600u) == 0);
    CHECK(task31_status.state == VAT_ROUTE_BUCKET_SEARCH && last_x == -200.0f &&
          last_y == 0.0f && !task31_status.bucket_seen && !task31_status.bucket_fallback &&
          !host_timer_active && host_servo == task31_grip_expected());
    return 0;
}

static int check_bucket_recalibrated_strict_fine_gate(void)
{
    const int coordinates[] = {90,120,91,119};
    CHECK(VAT_ROUTE_BUCKET_X_PX == 105 && VAT_ROUTE_BUCKET_FINE_ERROR_PX == 15);
    CHECK(VAT_ROUTE_BUCKET_X_PX - 15 == 90 && VAT_ROUTE_BUCKET_X_PX + 15 == 120);
    for (unsigned sample = 0u; sample < 4u; ++sample) {
        CHECK(task31_bucket_search_start() == 0);
        int pulses = pulse_calls, servos = servo_calls;
        host_tick += 20u; xy_obj(wire_request,9,coordinates[sample],VAT_BUCKET_Y_PX);
        task31_snapshot();
        CHECK(task31_status.x_goal == 105 && !task31_status.good && !host_timer_active &&
              pulse_calls == pulses && servo_calls == servos && !laser_state);
        if (sample < 2u) {
            /* Strict15px: either exact endpoint is still coarse back200. */
            CHECK(task31_status.state == VAT_ROUTE_BUCKET_SEARCH && last_x == -200.0f &&
                  last_y == 0.0f && last_w == 0.0f);
        } else {
            /*14px brakes first; old search image cannot count as arrival. */
            CHECK(task31_status.state == VAT_BRAKE && stopped());
            xy_advance(260u); task31_snapshot();
            CHECK(task31_status.state == VAT_RECHECK && !task31_status.latest &&
                  !task31_status.good && stopped());
            host_tick += 20u; xy_obj(wire_request,9,sample == 2u ? 90 : 120,VAT_BUCKET_Y_PX);
            task31_snapshot();
            CHECK(task31_status.state == task31_fine_x_state() && last_x == (sample == 2u ? -20.0f : 20.0f) &&
                  last_y == 0.0f && last_w == 0.0f && !task31_status.good && !host_timer_active);
        }
        run_cmd("0"); CHECK(stopped() && !host_timer_active);
    }
    puts("31 bucket105 strict finegate:90/120 error15 remain coarseback200;91/119 error14 brake/recheck/newimage then latched fine20 even beyond gate,never arrival or arm passed");
    return 0;
}

static int check_bucket_seen_loss_routes_to_actual_release(void)
{
    CHECK(task31_bucket_search_start() == 0);
    uint16_t request = wire_request; int pulses = pulse_calls, servos = servo_calls;
    xy_obj((uint16_t)(request-1u),9,500,VAT_BUCKET_Y_PX); xy_obj(request,4,500,VAT_BUCKET_Y_PX);
    CHECK(task31_service_ms(5000u) == 0 && !task31_status.bucket_seen &&
          !host_timer_active && pulse_calls == pulses && servo_calls == servos);
    unsigned seq = xy_seq;
    xy_obj_poll(request,9,VAT_ROUTE_BUCKET_X_PX+10,VAT_BUCKET_Y_PX,0); /* Fine gate is |115-105|<15. */
    xy_obj_poll(request,-1,0,0,0); wire_poll(); task31_snapshot();
    CHECK(task31_status.bucket_seen && task31_status.bucket_age_ms == 0u &&
          !task31_status.bucket_fallback && stopped());
    CHECK(task31_service_ms(100u) == 0);
    unsigned next_seq = xy_seq; xy_seq = seq; xy_obj(request,9,500,VAT_BUCKET_Y_PX); xy_seq = next_seq;
    CHECK(task31_service_ms(200u) == 0 && task31_status.bucket_age_ms == 300u &&
          !host_timer_active && pulse_calls == pulses && servo_calls == servos);
    CHECK(task31_service_ms(1u) == 0 && task31_status.bucket_age_ms == 301u &&
          !strcmp(task31_status.reason,"BUCKET_LOST300_BRAKE") && stopped());
    CHECK(task31_service_ms(100u) == 0); ++host_counts[0]; wire_poll();
    CHECK(task31_service_ms(249u) == 0 && !host_timer_active && servo_calls == servos);
    CHECK(task31_service_ms(1u) == 0 && s_route31_lift_phase == R31_LIFT_BUCKET_DOWN &&
          task31_status.state == VAT_WAIT_BUCKET_ACTION && task31_status.bucket_fallback &&
          !task31_status.alignment_confirmed && !task31_status.good && host_servo == task31_grip_expected() &&
          servo_calls == servos && host_timer_active);
    CHECK(task31_bucket_down_done() == 0 && host_servo == 1150u && servo_calls == servos+1);
    CHECK(task31_bucket_up_done() == 0 && s_seq_stage == ROUTE31_RETURN180_STAGE &&
          host_servo == 1500u && pulse_calls == pulses + 43400);
    CHECK(task31_finish_motion() == 0 && s_seq_stage == ROUTE31_TARGET_STAGE &&
          !s_seq_return_offset && s_target35_route && wire_task == 2u);
    CHECK(task31_cancel_and_verify("0") == 0);
    const char *const keys[] = {"g","a","0"};
    for (unsigned k=0u;k<3u;++k) {
        CHECK(task31_bucket_search_start() == 0); xy_obj(wire_request,9,500,VAT_BUCKET_Y_PX);
        CHECK(task31_service_ms(300u) == 0);
        ++host_tick; CHECK(task31_cancel_and_verify(keys[k]) == 0);
    }
    puts("31 bucket:never-seen/wrong/old/replay cannot release;NEW seen then lost>300+wheelstill250 starts actual down20000 before1150;normal release2/up20000 immediate rack/no fakealignment;deadline g/a/0 priority passed");
    return 0;
}

#if ROUTE31_TARGET_STOP_YAW_ENABLE
static int task31_target_yaw_phase(unsigned phase)
{
    CHECK(phase < 3u && task31_at(ROUTE31_TARGET_STAGE, "123") == 0 && task31_target_fire(8) == 0);
    host_yaw = s_target35_heading + 1.3f;
    int zeros = zero_calls;
    xy_advance(2000u);
    CHECK(s_seq_state == SQ_TASK_YAW && s_seq_stage == ROUTE31_TARGET_STAGE &&
          s_route31_task_yaw_goal == s_target35_heading && !s_target35_route && !laser_state &&
          zero_calls == zeros && !s_route31_task_yaw_stable && stopped());
    if (phase) {
        xy_advance(20u);
        CHECK(last_x == 0.0f && last_y == 0.0f && last_w < 0.0f &&
              !s_route31_task_yaw_stable && zero_calls == zeros && !host_timer_active);
    }
    if (phase == 2u) {
        host_yaw = s_route31_task_yaw_goal + 0.2f;
        xy_advance(20u);
        CHECK(s_route31_task_yaw_stable && stopped() && zero_calls == zeros);
    }
    return 0;
}

static int check_target_original_yaw_before_corner(void)
{
    for (unsigned side=0u;side<2u;++side) {
        CHECK(task31_target_yaw_phase(0u)==0);
        int zeros=zero_calls;float goal=s_route31_task_yaw_goal;
        float sign=side?1.0f:-1.0f;
        host_yaw=goal+sign*0.6f;wire_poll();
        CHECK(s_seq_state==SQ_TASK_YAW && !s_route31_task_yaw_progress.boosted &&
              fabsf(last_w+sign*0.30f)<0.000001f && !last_x && !last_y && !laser_state);
        uint32_t began=s_route31_task_yaw_progress.since;
        host_tick=began+399u;wire_poll();
        CHECK(!s_route31_task_yaw_progress.boosted && fabsf(last_w+sign*0.30f)<0.000001f);
        ++host_tick;wire_poll();
        CHECK(s_route31_task_yaw_progress.boosted && fabsf(last_w+sign*0.60f)<0.000001f &&
              s_route31_task_yaw_goal==goal && zero_calls==zeros && !laser_state);
        host_yaw=goal-sign*0.6f;host_tick+=20u;wire_poll();
        CHECK(!s_route31_task_yaw_progress.boosted && fabsf(last_w-sign*0.30f)<0.000001f);
        host_tick+=400u;wire_poll();
        CHECK(s_route31_task_yaw_progress.boosted && fabsf(last_w-sign*0.60f)<0.000001f);
        CHECK(task31_cancel_and_verify(side?"g":"a")==0);
    }
    CHECK(task31_target_yaw_phase(1u) == 0);
    int zeros = zero_calls;
    unsigned commands = wire_commands;
    float goal = s_route31_task_yaw_goal;
    CHECK(task31_service_ms(1000u) == 0 && s_seq_state == SQ_TASK_YAW && last_w < 0.0f &&
          zero_calls == zeros && wire_commands == commands);
    host_yaw = goal + 0.4f; xy_advance(20u);
    CHECK(s_seq_state == SQ_TASK_YAW && !s_route31_task_yaw_stable && last_w < 0.0f);
    host_yaw = goal + 0.2f; wire_poll();
    CHECK(s_route31_task_yaw_stable && stopped());
    CHECK(task31_service_ms(399u) == 0);
    CHECK(s_seq_state == SQ_TASK_YAW && zero_calls == zeros);
    ++host_counts[0]; ++host_tick; wire_poll();
    CHECK(s_seq_state == SQ_TASK_YAW && s_route31_task_yaw_stable_t0 == host_tick &&
          s_route31_task_yaw_still_t0 == host_tick && zero_calls == zeros);
    CHECK(task31_service_ms(399u) == 0); CHECK(s_seq_state == SQ_TASK_YAW && zero_calls == zeros);
    ++host_tick; wire_poll();
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
          route_seq_leg()->distance_mm == ROUTE31_GREEN_TO_CORNER_MM && stopped() && zero_calls == zeros);
    CHECK(sequence_start_stage() == 0 && zero_calls == zeros + 1 && s_dist_heading0 == 0.0f);
    CHECK(task31_cancel_and_verify("0") == 0);

    CHECK(task31_target_yaw_phase(2u) == 0);
    zeros = zero_calls;
    CHECK(task31_service_ms(399u) == 0);
    host_yaw = s_route31_task_yaw_goal - 0.2f; ++host_tick; wire_poll();
    CHECK(s_seq_state == SQ_TASK_YAW && s_route31_task_yaw_stable_t0 == host_tick && zero_calls == zeros);
    CHECK(task31_service_ms(399u) == 0); CHECK(s_seq_state == SQ_TASK_YAW);
    host_yaw = s_route31_task_yaw_goal + 0.5f; ++host_tick; wire_poll();
    CHECK(!s_route31_task_yaw_stable && s_seq_state == SQ_TASK_YAW && last_w < 0.0f && zero_calls == zeros);
    CHECK(task31_target_finish_yaw() == 0);
    CHECK(task31_cancel_and_verify("0") == 0);

    const char *const keys[] = {"g", "a", "0"};
    for (unsigned phase = 0u; phase < 3u; ++phase) for (unsigned key = 0u; key < 3u; ++key) {
        CHECK(task31_target_yaw_phase(phase) == 0);
        CHECK(task31_cancel_and_verify(keys[key]) == 0);
    }
    for (unsigned fault = 0u; fault < 3u; ++fault) {
        CHECK(task31_target_yaw_phase(1u) == 0);
        if (!fault) host_imu_valid = 0;
        else if (fault == 1u) host_abort = 1;
        else host_yaw = NAN;
        host_tick = s_route31_task_yaw_t0 + ROUTE31_POST_YAW_MAX_MS;
        wire_poll();
        CHECK(s_seq_state == SQ_STOPPED && stopped() && !host_timer_active && !s_receiving);
        CHECK(strstr(host_messages, "TARGET31_POST_YAW_TIMEOUT_CONTINUE") == NULL);
    }
    puts("31 target residual:laserOFF actual+/- .30 then399noBoost/400 .60,overshoot sign resets .30 and g/a cancel;originalheading before anyzero/corner;strict<0.4,encoder/drift resetfull400ms;9g/a/0 plusIMU/abort/NaN deadline faults passed");
    return 0;
}

static int check_target_two_second_continue_is_not_alignment(void)
{
    static const float residuals[] = { -1.3f, 1.3f };
    static const char *const reports[] = { "residual_err_deg=1.30", "residual_err_deg=-1.30" };
    CHECK(ROUTE31_POST_YAW_MAX_MS == 2000u && T_DIST_ALIGN_MAX_MS == 12000u);
    for (unsigned sample = 0u; sample < 2u; ++sample) {
        CHECK(task31_target_yaw_phase(0u) == 0);
        int zeros = zero_calls;
        unsigned commands = wire_commands;
        float goal = s_route31_task_yaw_goal;
        uint32_t began = s_route31_task_yaw_t0;
        host_yaw = goal + residuals[sample];
        host_tick = began + 1999u; wire_poll();
        CHECK(s_seq_state == SQ_TASK_YAW && s_seq_stage == ROUTE31_TARGET_STAGE &&
              zero_calls == zeros && !laser_state && wire_commands == commands &&
              last_x == 0.0f && last_y == 0.0f && last_w * residuals[sample] < 0.0f);
        host_tick = began + 2000u; wire_poll();
        CHECK(s_seq_state == SQ_STILL && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
              zero_calls == zeros && stopped() && !s_target35_route &&
              !host_timer_active && wire_commands == commands &&
              strstr(host_messages, "TARGET31_POST_YAW_TIMEOUT_CONTINUE") &&
              strstr(host_messages, "yaw_timeout_accept=1 reached=0") &&
              strstr(host_messages, reports[sample]));
        CHECK(sequence_start_stage() == 0 && zero_calls == zeros + 1 && s_dist_heading0 == 0.0f);
        CHECK(task31_cancel_and_verify("0") == 0);
    }

    /*2s includes final400ms hold. At1999 this deliberately incomplete
     *299ms hold cannot be reported reached=1. */
    CHECK(task31_target_yaw_phase(0u) == 0);
    int zeros = zero_calls;
    uint32_t began = s_route31_task_yaw_t0;
    host_tick = began + 1700u; host_yaw = s_route31_task_yaw_goal + 0.2f; wire_poll();
    CHECK(s_route31_task_yaw_stable && s_route31_task_yaw_stable_t0 == host_tick && stopped());
    host_tick = began + 1999u; wire_poll();
    CHECK(s_seq_state == SQ_TASK_YAW && zero_calls == zeros && stopped());
    host_tick = began + 2000u; wire_poll();
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
          zero_calls == zeros && strstr(host_messages, "yaw_timeout_accept=1 reached=0") &&
          strstr(host_messages, "residual_err_deg=-0.20"));
    CHECK(task31_cancel_and_verify("0") == 0);

    const char *const keys[] = { "g", "a", "0" };
    for (unsigned key = 0u; key < 3u; ++key) {
        CHECK(task31_target_yaw_phase(1u) == 0);
        host_tick = s_route31_task_yaw_t0 + 1999u; wire_poll();
        CHECK(task31_cancel_and_verify(keys[key]) == 0);
        CHECK(strstr(host_messages, "TARGET31_POST_YAW_TIMEOUT_CONTINUE") == NULL);
    }
    puts("31 target post-laser:1999 stays correcting,2000 continues with laserOFF/reached0/signed prezero residual;2s includes400ms hold;g/a/0 deadline cannot resume passed");
    return 0;
}
#else
static int check_target_current_direct_corner_and_stop(void)
{
    const char *const keys[] = {"g", "a", "0"};
    for (unsigned side = 0u; side < 2u; ++side) {
        CHECK(task31_at(ROUTE31_TARGET_STAGE, "123") == 0 && task31_target_fire(8) == 0);
        host_yaw = s_target35_heading + (side ? 1.3f : -1.3f);
        int zeros = zero_calls;
        unsigned commands = wire_commands;
        host_tick = s_target35_phase_t0 + 1999u; wire_poll();
        CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_TARGET_STAGE && laser_state &&
              !last_x && !last_y && !last_w && zero_calls == zeros);
        ++host_tick; wire_poll();
        CHECK(s_seq_state == SQ_STILL && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE &&
              !s_target35_route && !laser_state && stopped() && zero_calls == zeros &&
              wire_commands == commands && route_seq_leg()->distance_mm == 430u &&
              strstr(host_messages, "TARGET31_POST_YAW_TIMEOUT_CONTINUE") == NULL);
        CHECK(sequence_start_stage() == 0 && s_msel == 15 && s_dist_target == 430.0f &&
              zero_calls == zeros + 1 && s_dist_heading0 == 0.0f &&
              last_x == 200.0f && fabsf(last_y - 200.0f * ROUTE31_CORNER_RIGHT_FF_RATIO) < 0.0001f);
        CHECK(task31_cancel_and_verify(side ? "g" : "a") == 0);
    }
    for (unsigned key = 0u; key < 3u; ++key) {
        CHECK(task31_at(ROUTE31_TARGET_STAGE, "123") == 0 && task31_target_fire(8) == 0);
        host_tick = s_target35_phase_t0 + 1999u; wire_poll();
        CHECK(task31_cancel_and_verify(keys[key]) == 0);
        CHECK(strstr(host_messages, "TARGET31_POST_YAW_TIMEOUT_CONTINUE") == NULL);
    }
    puts("CURRENT31 target:laser holds1999ms,2000 laserOFF directly queues510/430/350 corner without stationaryyaw;next-road normalbaseline and FF retained;g/a/0 cancel before deadline passed");
    return 0;
}
#endif

#if defined(ROUTE31_HOSTAGE_OFFSET_LEGACY_TEST)
static int task31_hostage_offset_phase(unsigned phase)
{
    CHECK(ROUTE31_HOSTAGE_PREGRAB_LEFT_MM == 20u && ROUTE31_BALL_TO_BUCKET_LEFT_MM ==
#if defined(ROUTE31_BALL_OFFSET_LEGACY_TEST)
          20u);
#else
          15u);
#endif
    CHECK(phase < 6u && task31_at(ROUTE31_HOSTAGE_STAGE, "123") == 0);
    wire_ack(wire_request, 2u, 0u); xy_advance(300u); task31_rank(wire_request, 0u, 2u);
    for (unsigned i = 0u; i < 120u && !s_seq_hostage_offset; ++i) {
        host_tick += 20u; xy_obj(wire_request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
    }
    task31_snapshot();
    CHECK(s_seq_state == SQ_STILL && s_seq_hostage_offset && s_route31_hostage_offset_done &&
          task31_status.state == VAT_WAIT_HOSTAGE_ACTION && task31_status.good == 5u &&
          route_seq_leg()->mode == 17u && route_seq_leg()->distance_mm == 20u &&
          host_servo == 1500u && !servo_calls && !pulse_calls && !host_timer_active);
    if (phase) CHECK(sequence_start_stage() == 0 && last_y < 0.0f && last_x == 0.0f &&
                     s_dist_align_enabled && s_dist_target == -20.0f);
    if (phase >= 2u) {
        host_lateral = s_dist_odo0 + s_dist_target;
        host_yaw = s_dist_heading0 + 0.7f;
        host_absolute_yaw = s_route31_hostage_offset_heading + 0.7f;
        wire_poll();
        CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && stopped());
    }
    if (phase >= 3u) {
        xy_advance(T_DIST_STILL_MS); xy_advance(20u);
        CHECK(s_round == R_ALIGN && s_seq_state == SQ_RUN && last_w < 0.0f &&
              last_x == 0.0f && last_y == 0.0f && !s_dist_align_hold);
    }
    if (phase >= 4u) {
        host_yaw = s_dist_heading0; host_absolute_yaw = s_route31_hostage_offset_heading;
        xy_advance(20u);
        CHECK(s_dist_align_hold && s_seq_state == SQ_RUN && stopped());
    }
    if (phase == 5u) {
        xy_advance(ROUTE31_STABLE_MS); task31_snapshot();
        CHECK(s_seq_state == SQ_TASK && !s_seq_hostage_offset && s_route31_hostage_offset_done &&
              task31_status.state == VAT_BRAKE && !task31_status.good &&
              !task31_status.latest && !task31_status.alignment_confirmed && stopped());
    }
    CHECK(!s_route31_hostage_opened && !servo_calls && !pulse_calls && !host_timer_active);
    return 0;
}

static int check_hostage_pregrab_offset_and_new_images(void)
{
    CHECK(task31_hostage_offset_phase(1u) == 0);
    uint16_t request = wire_request;
    host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
    task31_snapshot();
    CHECK(s_seq_hostage_offset && task31_status.state == VAT_WAIT_HOSTAGE_ACTION &&
          last_y < 0.0f && !pulse_calls && !servo_calls && !host_timer_active);
    host_lateral = s_dist_odo0 + s_dist_target; wire_poll();
    CHECK(s_round == R_BRAKE && stopped());
    host_tick += T_DIST_STILL_MS; wire_poll();
    host_yaw = s_dist_heading0; wire_poll();
    CHECK(s_round == R_ALIGN && s_dist_align_hold);
    CHECK(task31_service_ms(399u) == 0 && s_seq_hostage_offset);
    unsigned moving_seq = xy_seq;
    ++host_tick; xy_obj_poll(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX, 0);
    wire_poll(); task31_snapshot();
    CHECK(s_seq_state == SQ_TASK && !s_seq_hostage_offset && task31_status.state == VAT_BRAKE &&
          !task31_status.alignment_confirmed && !task31_status.good && !task31_status.latest &&
          task31_status.target_rank == 2u && wire_request == request && !pulse_calls && !servo_calls);
    xy_advance(260u); task31_snapshot();
    CHECK(task31_status.state == VAT_RECHECK && !task31_status.good && !host_timer_active);
    unsigned next_seq = xy_seq; xy_seq = moving_seq;
    host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX); xy_seq = next_seq;
    task31_snapshot(); CHECK(!task31_status.good && !host_timer_active && !servo_calls && !pulse_calls);
    for (unsigned fresh = 0u; fresh < 4u; ++fresh) {
        host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
        task31_snapshot();
        CHECK(task31_status.good == fresh + 1u && !host_timer_active && !s_seq_hostage_offset &&
              !task31_status.alignment_confirmed && !servo_calls && !pulse_calls);
    }
    host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX); task31_snapshot();
    CHECK(s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND && host_timer_active &&
          task31_status.good == 5u && task31_status.alignment_confirmed &&
          s_route31_hostage_offset_done && !s_seq_hostage_offset && servo_calls == 1 &&
          !pulse_calls && host_servo == 1150u && task31_status.target_rank == 2u);
    CHECK(task31_hostage_grab_done() == 0 && s_route31_hostage_hold);
    wire_poll(); CHECK(s_seq_stage == ROUTE31_HOSTAGE_EXIT_STAGE && !s_seq_hostage_offset &&
                       route_seq_leg()->distance_mm == 1395u && pulse_calls == 6400 && servo_calls == 2);
    CHECK(task31_cancel_and_verify("0") == 0);

    const char *const keys[] = {"g", "a", "0"};
    for (unsigned phase = 0u; phase < 6u; ++phase) for (unsigned key = 0u; key < 3u; ++key) {
        CHECK(task31_hostage_offset_phase(phase) == 0);
        CHECK(task31_cancel_and_verify(keys[key]) == 0);
        CHECK(!s_seq_hostage_offset && !s_route31_hostage_offset_done && !servo_calls && !pulse_calls);
    }
    puts("EXPLICIT FIXTURE HISTORY hostageLEFT20: before anyarm/servo,originalyaw+400ms restored; moving-window frame replay rejected,rank/request retained,NEW5frames then onegrab/no repeat;18g/a/0 cancel atPREP/RUN/BRAKE/CORRECT/HOLD/RECHECK passed");
    return 0;
}

static int check_hostage_offset_timeout_keeps_visual_heading_recheck(void)
{
    CHECK(task31_hostage_offset_phase(3u) == 0);
    uint16_t request = wire_request;
    unsigned commands = wire_commands;
    int zeros = zero_calls;
    float absolute_goal = s_route31_hostage_offset_heading;
    uint32_t began = s_dist_align_t0;
    host_tick = began + 1999u;
    xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
    unsigned moving_seq = (uint16_t)(xy_seq - 1u);
    task31_snapshot();
    CHECK(s_seq_state == SQ_RUN && s_seq_hostage_offset && s_round == R_ALIGN &&
          zero_calls == zeros && !s_dist_align_timeout_accepted && !servo_calls && !pulse_calls);
    host_tick = began + 2000u; wire_poll(); task31_snapshot();
    CHECK(s_seq_state == SQ_TASK && !s_seq_hostage_offset && s_route31_hostage_offset_done &&
          s_dist_align_timeout_accepted && s_dist_reason == 1u && zero_calls == zeros + 1 &&
          task31_status.state == VAT_BRAKE && task31_status.yaw_dirty &&
          task31_status.yaw_target == absolute_goal && task31_status.target_rank == 2u &&
          !task31_status.alignment_confirmed && !task31_status.good && !task31_status.latest &&
          wire_request == request && wire_commands == commands && !servo_calls && !pulse_calls &&
          strstr(host_messages, "yaw_timeout_accept=1 residual_err_deg=-0.70"));

    /* Replay from the movement window cannot become post-stop coordinates. */
    unsigned next_seq = xy_seq;
    xy_seq = moving_seq; host_tick += 20u;
    xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX); xy_seq = next_seq;
    task31_snapshot();
    CHECK(!task31_status.good && !task31_status.latest && !task31_status.alignment_confirmed &&
          !servo_calls && !pulse_calls);

    /* Keep genuine NEW observations arriving so the user's independent2s
     *seen-then-loss fallback does not mask the separate12s VAT yaw policy. */
    for (unsigned elapsed = 0u; elapsed < 500u; elapsed += 20u) {
        host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
    }
    task31_snapshot();
    CHECK(task31_status.state == VAT_YAW_FIX && task31_status.yaw_target == absolute_goal &&
          task31_status.yaw_dirty && last_w < 0.0f && !servo_calls && !pulse_calls);
    uint32_t visual_began = s_yaw_from;
    for (unsigned elapsed = 0u; elapsed < 2000u; elapsed += 20u) {
        host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
    }
    task31_snapshot();
    CHECK((uint32_t)(host_tick - visual_began) >= 2000u && task31_status.state == VAT_YAW_FIX &&
          !task31_status.alignment_confirmed && !task31_status.good && !servo_calls && !pulse_calls &&
          wire_request == request && !s_seq_hostage_offset);
    host_absolute_yaw = absolute_goal;
    for (unsigned elapsed = 0u; elapsed < 420u; elapsed += 20u) {
        host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
    }
    task31_snapshot();
    CHECK(task31_status.state == VAT_RECHECK && !task31_status.good &&
          !task31_status.alignment_confirmed && !task31_status.yaw_dirty && !servo_calls && !pulse_calls);
    for (unsigned fresh = 0u; fresh < 4u; ++fresh) {
        host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
        task31_snapshot();
        CHECK(task31_status.good == fresh + 1u && !task31_status.alignment_confirmed &&
              !servo_calls && !pulse_calls && !host_timer_active);
    }
    host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
    task31_snapshot();
    CHECK(task31_status.alignment_confirmed && task31_status.good == 5u &&
          !task31_status.hostage_fallback && task31_status.target_rank == 2u &&
          s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND && host_timer_active &&
          servo_calls == 1 && !pulse_calls && s_route31_hostage_offset_done && !s_seq_hostage_offset);
    CHECK(task31_cancel_and_verify("0") == 0);
    puts("31 hostage LEFT20:1999 stays yawcorrecting,2000 accepts logged residual then same-request/rank BRAKE; movingframe replay rejected;VAT originalabsolute yaw still corrects past2s before NEW5 pixels/no duplicate offset/no early arm passed");
    return 0;
}
#endif /* Explicit historical fixture only; current31 bypasses this road leg. */

#if !defined(ROUTE31_HOSTAGE_OFFSET_LEGACY_TEST)
/* Current default: a real fifth NEW stopped image commits the existing arm
 * job directly. No zero-distance road, post-offset recheck or chassis output
 * may occur between confirmation and extension/grip/retraction. */
static int check_hostage_no_offset_direct_grab_and_rank_exit(void)
{
    static const unsigned exits[] = {1415u, 1315u, 1215u};
    CHECK(ROUTE31_HOSTAGE_PREGRAB_LEFT_MM == 0u &&
          ROUTE31_BALL_TO_BUCKET_LEFT_MM ==
#if defined(ROUTE31_BALL_OFFSET_LEGACY_TEST)
          20u &&
#else
          15u &&
#endif
          ROUTE31_BALL_TO_BUCKET_LEFT_V_MMS == 80.0f);
    CHECK(ROUTE31_HOSTAGE_SEARCH_RIGHT_FF_RATIO == 0.05f &&
          ROUTE31_CORNER_RIGHT_FF_RATIO == 0.095f && ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO == 0.075f);
    for (uint8_t rank = 1u; rank <= 3u; ++rank) {
        CHECK(task31_at(ROUTE31_HOSTAGE_STAGE, "123") == 0);
        uint16_t request = wire_request;
        wire_ack(request, 2u, 0u); xy_advance(300u); task31_rank(request, 0u, rank);
        host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
        xy_advance(260u); task31_snapshot();
        CHECK(task31_status.state == VAT_RECHECK && !task31_status.good && task31_drive_stopped());
        unsigned precise = host_precise_calls, integer = host_integer_calls, commands = wire_commands;
        int zeros = zero_calls;
        host_messages[0] = '\0';
        for (unsigned fresh = 0u; fresh < 4u; ++fresh) {
            host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
            task31_snapshot();
            CHECK(task31_status.good == fresh + 1u && !task31_status.alignment_confirmed &&
                  s_seq_state == SQ_TASK && !s_seq_hostage_offset && !s_route31_hostage_offset_done &&
                  s_route31_lift_phase == R31_LIFT_OFF && !host_timer_active && !pulse_calls && !servo_calls &&
                  task31_drive_stopped() && host_precise_calls == precise && host_integer_calls == integer);
        }
        /* Old request / wrong shape / duplicate sequence do not manufacture
         * the fifth arrival frame or restart a disabled offset. */
        unsigned next = xy_seq;
        xy_seq = next - 1u; host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
        xy_seq = next; host_tick += 20u; xy_obj((uint16_t)(request - 1u), 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
        host_tick += 20u; xy_obj(request, 1, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
        task31_snapshot();
        CHECK(!task31_status.good && !host_timer_active && !servo_calls && !pulse_calls);
        for (unsigned fresh = 0u; fresh < 4u; ++fresh) {
            host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
            task31_snapshot();
            CHECK(task31_status.good == fresh + 1u && !host_timer_active && !servo_calls && !pulse_calls &&
                  !s_seq_hostage_offset && !s_route31_hostage_offset_done && task31_drive_stopped());
        }
        host_tick += 20u; xy_obj(request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX); task31_snapshot();
        CHECK(task31_status.state == VAT_WAIT_HOSTAGE_ACTION && task31_status.good == 5u &&
              task31_status.alignment_confirmed && !task31_status.hostage_fallback &&
              task31_status.target_rank == rank && s_seq_state == SQ_TASK &&
              !s_seq_hostage_offset && !s_route31_hostage_offset_done &&
              s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND && host_timer_active &&
              s_jog_clock.remaining == 3400u && host_jog_axis == 0 && host_jog_dir == 0 &&
              host_timer_pps == 5000u && host_servo == 1150u && servo_calls == 1 && !pulse_calls &&
              wire_request == request && wire_commands == commands && zero_calls == zeros &&
              host_precise_calls == precise && host_integer_calls == integer && task31_drive_stopped());
        CHECK(strstr(host_messages, "HOSTAGE31_PREGRAB_LEFT") == NULL &&
              strstr(host_messages, "HOSTAGE_OFFSET_RECHECK") == NULL);
        CHECK(task31_hostage_grab_done() == 0 && s_route31_hostage_hold &&
              pulse_calls == 6400 && host_jog_pulses[0][0] == 3400u && host_jog_pulses[0][1] == 3000u &&
              !host_jog_pulses[1][0] && !host_jog_pulses[1][1] && host_servo == task31_grip_expected() && servo_calls == 2 &&
              host_precise_calls == precise && host_integer_calls == integer && zero_calls == zeros &&
              !s_seq_hostage_offset && !s_route31_hostage_offset_done);
        wire_poll();
        CHECK(s_seq_state == SQ_STILL && s_seq_stage == ROUTE31_HOSTAGE_EXIT_STAGE &&
              route_seq_leg()->distance_mm == exits[rank - 1u] && wire_request == request);
        CHECK(sequence_start_stage() == 0 && last_x == 200.0f &&
              fabsf(last_y - 200.0f * 0.075f) < 0.0001f);
        CHECK(sequence_finish_stage() == 0 && s_seq_state == SQ_DONE && stopped() && host_servo == task31_grip_expected());
    }
    puts("31 current HOSTAGE_NOOFFSET: fiveNEW stoppedframes directly open1150/extend3400@5000/grip2100/retract3000; no chassis velocity command/zero-distance leg/newrequest or extra yaw-reset; ranks1/2/3 ->1415/1315/1215v200 FF.075 heldclaw terminalSTOP; searchFF.05/cornerFF.095 preserved passed");
    return 0;
}

static int check_hostage_no_offset_confirmation_and_loss_cancel_priority(void)
{
    const char *const keys[] = {"g", "a", "0"};
    for (unsigned loss = 0u; loss < 2u; ++loss) for (unsigned key = 0u; key < 3u; ++key) {
        if (loss) {
            CHECK(task31_loss_search() == 0); xy_obj(wire_request, 0, 500, VAT_HOSTAGE_Y_PX);
            CHECK(task31_service_ms(301u) == 0 && !strcmp(task31_status.reason, "HOSTAGE_LOST300_BRAKE"));
            CHECK(task31_service_ms(249u) == 0 && !servo_calls && !host_timer_active);
            ++host_tick; /* Stop wins the exact arm-commit/still250 deadline. */
        } else {
            CHECK(task31_workpoint_ready(2u) == 0);
            for (unsigned fresh = 0u; fresh < 4u; ++fresh) {
                host_tick += 20u; xy_obj(wire_request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
            }
            task31_snapshot(); CHECK(task31_status.good == 4u && !servo_calls && !host_timer_active);
            host_tick += 20u; xy_obj_poll(wire_request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX, 0);
        }
        CHECK(task31_cancel_and_verify(keys[key]) == 0 && !servo_calls && !pulse_calls &&
              !s_seq_hostage_offset && !s_route31_hostage_offset_done && host_servo == 1500u);
    }
    puts("31 current NOOFFSET: g/a/0 wins fifthNEW frame and loss>300ms+wheelstill250 arm-commit deadlines; no servo/STEP/body offset or auto-resume passed");
    return 0;
}
#endif

/* Isolation smoke checks only; the user's deferred43 suite is NOT claimed
 * physically accepted or generally reworked by this31-only change. */
static int check_delta31_does_not_leak_into43(void)
{
    CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
    run_cmd("43"); wire_sync();
    wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123"); run_cmd("g");
    /* Synthetic already-issued rack snapshot: this smoke check does not
     * traverse43's independently covered predeployment/bucket mechanics. */
    s_route43_rack_issued = 3200u; s_route43_rack_pps_issued = 5000u;
    task31_latch_and_prepare(ROUTE31_HOSTAGE_STAGE, "123"); host_servo = 1500u;
    CHECK(s_seq_mode == ROUTE_STEP_MODE && s_seq_state == SQ_TASK && !s_seq_hostage_offset);
    wire_ack(wire_request, 2u, 0u); xy_advance(300u); task31_rank(wire_request, 0u, 2u);
    for (unsigned i = 0u; i < 100u && s_route31_lift_phase == R31_LIFT_OFF; ++i) {
        host_tick += 20u; xy_obj(wire_request, 0, VAT_ROUTE_HOSTAGE_X_PX, VAT_HOSTAGE_Y_PX);
    }
    CHECK(s_seq_state == SQ_TASK && s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND &&
          host_timer_active && !s_seq_hostage_offset && !s_route31_hostage_offset_done && host_servo == 1150u);
    CHECK(task31_hostage_grab_done() == 0 && host_servo == 1900u &&
          host_jog_pulses[0][0] == 3200u && host_jog_pulses[0][1] == 3000u &&
          !host_jog_pulses[1][0] && !host_jog_pulses[1][1] && s_route31_hostage_hold);
    run_cmd("0"); CHECK(stopped() && !host_timer_active);

    CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
    run_cmd("43"); wire_sync();
    wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123"); run_cmd("g");
    task31_latch_and_prepare(ROUTE31_TARGET_STAGE, "123");
    wire_ack(wire_request, 2u, 0u); xy_advance(1100u);
    for (unsigned i = 0u; i < 100u && !laser_state; ++i) {
        host_tick += 20u; xy_obj(wire_request, 8, ROUTE43_TARGET_CX, 10);
    }
    CHECK(s_target35_phase == TA_FIRE && laser_state && !host_timer_active);
    host_yaw = 1.3f; xy_advance(2000u);
    CHECK(s_seq_mode == ROUTE_STEP_MODE && s_seq_state == SQ_STEP_WAIT &&
          s_seq_stage == ROUTE31_TARGET_CORNER_STAGE && !laser_state && !s_target35_route);
    run_cmd("0"); CHECK(stopped());
    puts("31-only delta isolation:43 real hostage extend3200/grip1900/retract3000 retains noLEFT20;43 target retains next-step-g/noSQ_TASK_YAW;42 default1900 and independent35 laser hold checked separately passed");
    return 0;
}

int main(void)
{
    CHECK(check_ball_recipe_private_grip_scope() == 0);
    CHECK(check_full_qr_ball_bucket_target_hostage_chain() == 0);
    CHECK(check_three_target_distances_and_independent_laser_scope() == 0);
#if ROUTE31_TARGET_STOP_YAW_ENABLE
    CHECK(check_target_original_yaw_before_corner() == 0);
    CHECK(check_target_two_second_continue_is_not_alignment() == 0);
#else
    CHECK(check_target_current_direct_corner_and_stop() == 0);
#endif
    CHECK(check_target31_gate_freshness_and_burst() == 0);
    CHECK(check_task_cancellation_and_stale_frames() == 0);
    CHECK(check_bucket_turn_order_frames_cancel_and_faults() == 0);
#if defined(ROUTE31_BALL_OFFSET_LEGACY_TEST)
    CHECK(check_first_bucket_left20_speed_cap_and_other_pv_scope() == 0);
#else
    CHECK(check_first_bucket_current_left15_handoff() == 0);
#endif
    CHECK(check_return_turn_stability_and_stops() == 0);
    CHECK(check_bucket_seen_loss_routes_to_actual_release() == 0);
    CHECK(check_bucket_recalibrated_strict_fine_gate() == 0);
    CHECK(check_release_ready_one_second_and_late_poll() == 0);
    CHECK(check_lift_report_delay_and_clock_failure() == 0);
    CHECK(check_last_pulse_during_lift_poll_wait_origin() == 0);
    CHECK(check_hostage_retract_hold_timing_and_faults() == 0);
    CHECK(check_mechanical_ownership_and_qr_gate() == 0);
    CHECK(check_route31_workpoint_boundaries_and_standalone_isolation() == 0);
    CHECK(check_route_ball_hostage_initial_search100() == 0);
    CHECK(check_current31_continuous_x_and_ball_loss_action() == 0);
    CHECK(check_route_ball_hostage_strict_coarse_boundaries() == 0);
    CHECK(check_route31_wait_task_ack_diagnostics() == 0);
    CHECK(check_route_search_private_gain_and_late_yaw() == 0);
    CHECK(check_hostage_real_rank_hold_and_final_route() == 0);
    CHECK(check_hostage_alignment_opens_before_rank_once() == 0);
#if defined(ROUTE31_HOSTAGE_OFFSET_LEGACY_TEST)
    CHECK(check_hostage_pregrab_offset_and_new_images() == 0);
    CHECK(check_hostage_offset_timeout_keeps_visual_heading_recheck() == 0);
#else
    CHECK(check_hostage_no_offset_direct_grab_and_rank_exit() == 0);
    CHECK(check_hostage_no_offset_confirmation_and_loss_cancel_priority() == 0);
#endif
    CHECK(check_delta31_does_not_leak_into43() == 0);
    CHECK(check_hostage_dropout_trigger_and_rank() == 0);
    CHECK(check_hostage_dropout_recovery_priority_and_fault() == 0);
    CHECK(check_ball_needs_no_rank_before_bucket_request() == 0);
#if defined(ROUTE31_HOSTAGE_OFFSET_LEGACY_TEST)
    puts("EXPLICIT HOST FIXTURE HISTORY: hostageLEFT20 enabled only by test compile macro; not current production default or vehicle acceptance");
#else
    puts("CURRENT PRODUCTION DEFAULT: hostageNOOFFSET directly grabs after existing confirmation/fallback; no cancelled zero-distance road job");
#endif
    puts("route31_task_chain_test: host software checks only; no vehicle/camera/laser/arm physical or whole-event acceptance");
    return 0;
}
