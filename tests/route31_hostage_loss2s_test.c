/* Host-only real UART/CRC/VAT/dispatcher regression. Time, camera pixels and
 * encoder values are injected; a timer start is NOT physical grasp evidence. */
#define ROUTE31_FAST_TIMING_FIXTURE_MAIN retained_hostage_loss_timing_main
#include "route31_fast_timing_test.c"
#undef ROUTE31_FAST_TIMING_FIXTURE_MAIN

static int loss_drive_stopped(void)
{ return !last_x && !last_y && !last_w && !laser_state; }

static int loss_no_arm(void)
{ return !host_timer_active && !host_timer_start_calls && !pulse_calls && !servo_calls; }

static void loss_at(uint32_t at)
{ host_tick = at; wire_poll(); fast_snapshot(); }

static void loss_rank(void)
{
    uint8_t p[14] = {0x62u,(uint8_t)wire_request,(uint8_t)(wire_request >> 8),
                    9u,0u,0x54u,0u,0u,0u,1u,3u,0u,1u,2u};
    route_wire_le16(p + 6u,xy_seq++); wire_feed(p,sizeof p,0);
}

static int loss_search(void)
{
    CHECK(fast_prepare(31u,ROUTE31_HOSTAGE_STAGE) == 0);
    CHECK(s_seq_state == SQ_TASK && wire_task == 3u && wire_digit == 3u);
    wire_ack(wire_request,2u,0u);
    host_tick += 600u; wire_poll();
    host_tick += VAT_POLL_MS; wire_poll();
    host_tick += 100u; wire_poll(); fast_snapshot();
    CHECK(fast_status.state == VAT_FINE_CONTINUOUS && last_x > 0.0f &&
          !fast_status.hostage_seen && loss_no_arm());
    return 0;
}

static uint32_t loss_seen(void)
{
    /* Only a selected NEW01 inside the exclusive20px fine gate authorizes
     * the seen-then-lost shortcut. Keep it outside the15px aligned band. */
    xy_obj(wire_request,0,VAT_ROUTE31_HOSTAGE_X_PX + 19,VAT_HOSTAGE_Y_PX);
    fast_snapshot(); return host_tick;
}

static int loss_actual_log_search(void)
{
    int32_t qr[3];
    CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
    run_cmd("31"); wire_sync(); wire_ack(wire_request,1u,0u); xy_qr(wire_request,"212");
    CHECK(proto_qr_get(qr) && qr[0] == 2 && qr[1] == 1 && qr[2] == 2);
    run_cmd("g"); CHECK(s_seq_state == SQ_STILL && fast_stopped());
    memcpy(s_seq_qr,qr,sizeof qr); step_vision_receive_end();
    /* Replay a real request5, not a hand-written receive/request cache. Prior
     * task requests are intentionally isolated from the motion/arm fixture. */
    while (wire_request < 4u) {
        CHECK(real_proto_send_target(PROTO_TASK_TARGET,1u)); wire_sync();
    }
    CHECK(wire_request == 4u);
    s_seq_stage = ROUTE31_HOSTAGE_STAGE; route_seq_prepare(); wire_sync();
    CHECK(s_seq_state == SQ_TASK && wire_request == 5u && wire_task == 3u && wire_digit == 2u);
    wire_ack(wire_request,2u,0u); host_tick += 600u; wire_poll();
    host_tick += VAT_POLL_MS; wire_poll(); host_tick += 100u; wire_poll(); fast_snapshot();
    CHECK(fast_status.state == VAT_FINE_CONTINUOUS && last_x > 0.0f &&
          !fast_status.hostage_seen && !fast_status.hostage_loss_armed && loss_no_arm());
    return 0;
}

static int check_loss_far_flash_and_fine_gate_boundary(void)
{
    CHECK(VAT_ROUTE31_HOSTAGE_X_PX == 210 && VAT_ROUTE31_HOSTAGE_TOL_PX == 15 &&
          VAT_ROUTE31_HOSTAGE_FINE_ERROR_PX == 20);
    CHECK(loss_actual_log_search() == 0);
    const uint32_t never_seen_from = host_tick;
    loss_at(never_seen_from + 5000u);
    CHECK(fast_status.state == VAT_FINE_CONTINUOUS && last_x > 0.0f &&
          !fast_status.hostage_seen && !fast_status.hostage_loss_armed && loss_no_arm());
    xy_seq = 4471u; xy_obj(wire_request,2,619,226); fast_snapshot();
    uint32_t flash = host_tick;
    CHECK(s_route31_target.seen && s_route31_target.frame.sequence == 4471u &&
          s_route31_target.frame.cx == 619 && fast_status.rx_cx == 619 &&
          fast_status.rx_sequence == 4471u && fast_status.hostage_seen &&
          !fast_status.hostage_loss_armed && fast_status.state == VAT_FINE_CONTINUOUS && last_x > 0.0f);
    const unsigned ages[] = {529u,2029u,5001u};
    for (unsigned i = 0u; i < sizeof ages / sizeof ages[0]; ++i) {
        loss_at(flash + ages[i]);
        CHECK(fast_status.state == VAT_FINE_CONTINUOUS && last_x > 0.0f &&
              !last_y && !s_route31_loss_braking && !fast_status.hostage_fallback &&
              !fast_status.hostage_loss_armed && loss_no_arm());
    }
    xy_obj(wire_request,-1,0,0);
    xy_obj((uint16_t)(wire_request - 1u),2,VAT_ROUTE31_HOSTAGE_X_PX,VAT_HOSTAGE_Y_PX);
    xy_obj(wire_request,1,VAT_ROUTE31_HOSTAGE_X_PX,VAT_HOSTAGE_Y_PX);
    uint8_t rank[14] = {0x62u,5u,0u,9u,0u,0x54u,0u,0u,2u,1u,3u,2u,0u,1u};
    route_wire_le16(rank + 6u,4600u); wire_feed(rank,sizeof rank,0); fast_snapshot();
    CHECK(!fast_status.hostage_loss_armed && last_x > 0.0f && loss_no_arm());
    xy_seq = 4601u; xy_obj(wire_request,2,VAT_ROUTE31_HOSTAGE_X_PX + 20,VAT_HOSTAGE_Y_PX);
    fast_snapshot(); CHECK(!fast_status.hostage_loss_armed && last_x > 0.0f);
    host_tick += VAT_POLL_MS;
    xy_obj(wire_request,2,VAT_ROUTE31_HOSTAGE_X_PX - 20,VAT_HOSTAGE_Y_PX);
    fast_snapshot(); CHECK(!fast_status.hostage_loss_armed &&
          fast_status.state == VAT_FINE_CONTINUOUS && !s_search_mid.phase);
    host_tick += 100u; wire_poll(); fast_snapshot();
    /* Smooth reversal may spend one poll at zero before accelerating backward. */
    host_tick += VAT_POLL_MS; wire_poll(); fast_snapshot();
    CHECK(last_x < 0.0f && !fast_status.hostage_loss_armed && loss_no_arm());
    xy_obj(wire_request,2,VAT_ROUTE31_HOSTAGE_X_PX + 19,VAT_HOSTAGE_Y_PX);
    fast_snapshot(); uint32_t fine_seen = host_tick;
    CHECK(fast_status.hostage_loss_armed && fast_status.state == VAT_FINE_CONTINUOUS &&
          !s_route31_loss_braking && loss_no_arm());
    loss_at(fine_seen + VAT_ROUTE31_HOSTAGE_LOST_MS - 1u);
    CHECK(!s_route31_loss_braking && loss_no_arm());
    loss_at(fine_seen + VAT_ROUTE31_HOSTAGE_LOST_MS);
    CHECK(s_route31_loss_braking && !strcmp(fast_status.reason,"HOSTAGE_LOST2S_BRAKE") &&
          loss_drive_stopped() && loss_no_arm());
    run_cmd("0"); CHECK(!s_route31_hostage_loss_armed && fast_stopped());
    puts("31 hostage goal210/tol15/fine20 realreq5/digit2:never-seen and x619/seq4471 flash keep search>2s/>5s;empty54oldreq/wrong cannot arm;abs20 excluded/abs19 arms2s passed");
    return 0;
}

static int loss_actual_grab(void)
{
    CHECK(s_seq_state == SQ_TASK && !s_seq_hostage_offset &&
          s_route31_lift_phase == R31_HOSTAGE_RACK_EXTEND &&
          s_route_action_taken && host_timer_active && host_timer_start_calls == 1u &&
          host_jog_axis == ROUTE31_RACK_AXIS && host_jog_dir == ROUTE31_RACK_EXTEND_DIR &&
          host_timer_pps == ROUTE31_RACK_PPS &&
          s_jog_clock.remaining == ROUTE31_HOSTAGE_RACK_EXTEND_STEPS &&
          host_servo == ARM_SERVO_START_US && servo_calls == 1 &&
          !pulse_calls && loss_drive_stopped());
    for (unsigned event = 0u; !pulse_calls && event < 50u; ++event)
        CHECK(jog_host_timer_event() == 0);
    CHECK(pulse_calls == 1 &&
          host_jog_pulses[ROUTE31_RACK_AXIS][ROUTE31_RACK_EXTEND_DIR] == 1u);
    run_cmd("0"); CHECK(fast_stopped());
    return 0;
}

static int check_loss_exact_boundary_and_wheel_still(void)
{
    CHECK(VAT_ROUTE31_HOSTAGE_LOST_MS == 2000u && VAT_ROUTE31_LOST_MS == 300u);
    CHECK(loss_search() == 0);
    uint32_t seen = loss_seen();
    CHECK(s_route31_target.seen && s_route31_target.at == seen && fast_status.hostage_loss_armed);
    const unsigned early[] = {300u,301u,VAT_ROUTE31_HOSTAGE_LOST_MS - 1u};
    for (unsigned i = 0u; i < sizeof early / sizeof early[0]; ++i) {
        loss_at(seen + early[i]);
        CHECK(!s_route31_loss_braking && !fast_status.hostage_fallback &&
              fast_status.hostage_age_ms == early[i] && loss_no_arm() &&
              !fast_status.alignment_confirmed);
    }
    loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS);
    CHECK(s_route31_loss_braking && fast_status.state == VAT_BRAKE &&
          !strcmp(fast_status.reason,"HOSTAGE_LOST2S_BRAKE") &&
          loss_drive_stopped() && loss_no_arm());
    loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS + T_DIST_STILL_MS - 1u);
    CHECK(fast_status.state == VAT_BRAKE && loss_no_arm());
    loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS + T_DIST_STILL_MS);
    CHECK(fast_status.state == VAT_WAIT_HOSTAGE_ACTION && fast_status.hostage_fallback &&
          !fast_status.alignment_confirmed && !fast_status.good);
    CHECK(loss_actual_grab() == 0);

    CHECK(loss_search() == 0); seen = loss_seen();
    loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS);
    loss_at(host_tick + 100u); ++host_counts[2]; wire_poll();
    uint32_t moved = host_tick;
    loss_at(moved + T_DIST_STILL_MS - 1u); CHECK(loss_no_arm());
    loss_at(moved + T_DIST_STILL_MS); CHECK(loss_actual_grab() == 0);
    puts("31 hostage:last NEW01 ages300/301/1999 no grab;exact2000 brake/noSTEP;250ms unchanged wheels then actual rack job;wheel change restarts250 passed");
    return 0;
}

static int check_loss_new_frames_restart_and_invalid_frames_do_not(void)
{
    CHECK(loss_search() == 0);
    uint32_t seen = loss_seen();
    for (unsigned renewal = 0u; renewal < 3u; ++renewal) {
        loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS - 1u); CHECK(loss_no_arm());
        host_tick += 20u; seen = loss_seen();
        CHECK(s_route31_target.at == seen && fast_status.hostage_age_ms == 0u &&
              !s_route31_loss_braking && loss_no_arm());
    }
    loss_at(seen + 100u);
    unsigned next = xy_seq, accepted = s_route31_target.frame.sequence;
    xy_seq = accepted;
    xy_obj(wire_request,0,VAT_ROUTE31_HOSTAGE_X_PX + 100,VAT_HOSTAGE_Y_PX);
    xy_seq = accepted - 1u;
    xy_obj(wire_request,0,VAT_ROUTE31_HOSTAGE_X_PX + 100,VAT_HOSTAGE_Y_PX);
    xy_seq = next;
    xy_obj((uint16_t)(wire_request - 1u),0,VAT_ROUTE31_HOSTAGE_X_PX + 100,VAT_HOSTAGE_Y_PX);
    xy_obj(wire_request,1,VAT_ROUTE31_HOSTAGE_X_PX + 100,VAT_HOSTAGE_Y_PX);
    xy_obj(wire_request,-1,0,0);
    loss_rank(); fast_snapshot();
    CHECK(s_route31_target.at == seen && fast_status.hostage_age_ms == 100u && loss_no_arm());
    loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS - 1u);
    CHECK(!s_route31_loss_braking && loss_no_arm());
    loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS);
    CHECK(s_route31_loss_braking && loss_no_arm());
    loss_at(host_tick + T_DIST_STILL_MS); CHECK(loss_actual_grab() == 0);
    puts("31 hostage:each NEW selected01 restarts2s;duplicate/backward sequence/old request/wrong target/empty/rank54 do not renew accepted timestamp passed");
    return 0;
}

static int check_loss_new_frame_cancels_brake_and_pending_take(void)
{
    for (unsigned pending = 0u; pending < 2u; ++pending) {
        CHECK(loss_search() == 0); uint32_t seen = loss_seen();
        loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS);
        CHECK(s_route31_loss_braking && loss_no_arm());
        if (pending) {
            host_tick += T_DIST_STILL_MS;
            /* Deliberately separate actual VAT poll from App owner take. */
            wire_sync(); vision_align_test_poll(); fast_snapshot();
            CHECK(fast_status.state == VAT_WAIT_HOSTAGE_ACTION &&
                  fast_status.hostage_fallback && !s_route_action_taken && loss_no_arm());
        } else host_tick += 100u;
        xy_obj_poll(wire_request,0,VAT_ROUTE31_HOSTAGE_X_PX + 100,VAT_HOSTAGE_Y_PX,0);
        wire_sync();
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        wire_poll(); fast_snapshot(); seen = host_tick;
        CHECK(s_route31_target.at == seen && !s_route31_loss_braking &&
              !fast_status.hostage_fallback && !s_route_action_taken && loss_no_arm());
        loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS - 1u); CHECK(loss_no_arm());
        loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS); CHECK(s_route31_loss_braking && loss_no_arm());
        loss_at(host_tick + T_DIST_STILL_MS); CHECK(loss_actual_grab() == 0);
    }
    puts("31 hostage:NEW01 during braking or after pending poll before take cancels fallback atomically;fresh2s deadline then stopped grab passed");
    return 0;
}

static int check_loss_never_seen_and_fault_priority(void)
{
    CHECK(fast_prepare(31u,ROUTE31_HOSTAGE_STAGE) == 0);
    xy_obj(wire_request,0,VAT_ROUTE31_HOSTAGE_X_PX + 100,VAT_HOSTAGE_Y_PX); /* noACK */
    wire_ack(wire_request,2u,0u); host_tick += 600u; wire_poll();
    xy_obj(wire_request,1,VAT_ROUTE31_HOSTAGE_X_PX + 100,VAT_HOSTAGE_Y_PX);
    xy_obj((uint16_t)(wire_request - 1u),0,VAT_ROUTE31_HOSTAGE_X_PX + 100,VAT_HOSTAGE_Y_PX);
    xy_obj(wire_request,-1,0,0); loss_rank();
    loss_at(host_tick + 5000u);
    CHECK(!s_route31_target.seen && !fast_status.hostage_seen &&
          !fast_status.hostage_fallback && loss_no_arm());
    run_cmd("0");
    const char *const keys[] = {"g","a","0"};
    for (unsigned pending = 0u; pending < 2u; ++pending)
        for (unsigned fault = 0u; fault < 6u; ++fault) {
            CHECK(loss_search() == 0); uint32_t seen = loss_seen();
            if (pending) {
                loss_at(seen + VAT_ROUTE31_HOSTAGE_LOST_MS);
                host_tick += T_DIST_STILL_MS; wire_sync(); vision_align_test_poll();
                CHECK(s_vat.state == VAT_WAIT_HOSTAGE_ACTION && !s_route_action_taken);
            } else host_tick = seen + VAT_ROUTE31_HOSTAGE_LOST_MS;
            uint16_t old_request = wire_request;
            if (fault < 3u) run_cmd(keys[fault]);
            else if (fault == 3u) host_imu_valid = 0;
            else if (fault == 4u) host_abort = 1;
            else wire_ack(wire_request,2u,1u);
            wire_poll(); fast_snapshot();
            CHECK(s_seq_state == SQ_STOPPED && loss_drive_stopped() && loss_no_arm() && !s_receiving);
            host_tick += 4000u;
            xy_obj(old_request,0,VAT_ROUTE31_HOSTAGE_X_PX,VAT_HOSTAGE_Y_PX);
            wire_poll(); CHECK(s_seq_state == SQ_STOPPED && loss_drive_stopped() && loss_no_arm());
        }
    puts("31 hostage:never-seen/preACK/wrong/empty/54 cannot grab;g/a/0/IMU/abort/NACK win at deadline and pending take;late coordinates inert passed");
    return 0;
}

static int check_loss_aligned_grab_unchanged(void)
{
    CHECK(loss_search() == 0);
    host_tick += VAT_POLL_MS; /* Allow the moving controller to consume this frame. */
    xy_obj(wire_request,0,VAT_ROUTE31_HOSTAGE_X_PX,VAT_HOSTAGE_Y_PX);
    host_tick += T_DIST_STILL_MS; wire_poll(); fast_snapshot();
    CHECK(fast_status.state == VAT_RECHECK && loss_no_arm());
    for (unsigned frame = 1u; frame <= VAT_GOOD_FRAMES; ++frame) {
        host_tick += VAT_POLL_MS;
        xy_obj(wire_request,0,VAT_ROUTE31_HOSTAGE_X_PX,VAT_HOSTAGE_Y_PX); fast_snapshot();
        if (frame < VAT_GOOD_FRAMES)
            CHECK(fast_status.good == frame && !fast_status.alignment_confirmed && loss_no_arm());
    }
    CHECK(fast_status.alignment_confirmed && !fast_status.hostage_fallback &&
          fast_status.state == VAT_WAIT_HOSTAGE_ACTION);
    CHECK(loss_actual_grab() == 0);
    puts("31 hostage:normal stopped five NEW aligned frames still start actual grab before2s;not forced into dropout branch passed");
    return 0;
}

static void loss_direct_frame(const ProtoFrame *frame)
{
    real_proto_stats_get(&host_proto_stats); vision_align_test_feed_frame(frame);
}
static void loss_direct_poll(uint32_t at)
{ host_tick = at; wire_sync(); vision_align_test_poll(); wire_sync(); }

static int loss_direct_start(unsigned owner, ProtoTask task)
{
    const int32_t qr[3] = {1,2,3};
    CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get;
    proto_set_on_frame(loss_direct_frame); xy_seq = 1u;
    CHECK(vision_align_test_start_route_scoped(task == PROTO_TASK_HOSTAGE ? 40u : 41u,
                                              qr,(uint8_t)owner));
    wire_sync();
    if (task == PROTO_TASK_BUCKET) {
        /* Unit boundary for the already completed turn. The actual31 arm/
         * turn chain is replayed separately in revised_tail, not fabricated here. */
        CHECK(vat_request(PROTO_TASK_BUCKET,0u,host_tick)); wire_sync();
        s_route_bucket_eligible = 1u;
    }
    uint8_t ack[5] = {0x61u,(uint8_t)wire_request,(uint8_t)(wire_request >> 8),2u,0u};
    wire_feed_raw(ack,sizeof ack,0); wire_sync(); CHECK(vat_ack_ready());
    loss_direct_poll(host_tick + 260u); loss_direct_poll(host_tick + 20u);
    return 0;
}

static int check_loss_ball_bucket_and_legacy_scope(void)
{
    const unsigned owners[] = {31u,43u,44u};
    const ProtoTask tasks[] = {PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    for (unsigned owner = 0u; owner < 3u; ++owner)
        for (unsigned task = 0u; task < 3u; ++task) {
            if (owners[owner] == 31u && tasks[task] == PROTO_TASK_HOSTAGE) continue;
            CHECK(loss_direct_start(owners[owner],tasks[task]) == 0);
            int model = tasks[task] == PROTO_TASK_BALL ? 4 : tasks[task] == PROTO_TASK_BUCKET ? 9 : 0;
            int y = tasks[task] == PROTO_TASK_BALL ? VAT_BALL_Y_PX :
                    tasks[task] == PROTO_TASK_BUCKET ? VAT_BUCKET_Y_PX : VAT_HOSTAGE_Y_PX;
            xy_obj_poll(wire_request,model,s_vat.x_goal + 100,y,0); wire_sync();
            uint32_t seen = host_tick; CHECK(s_route31_target.seen && s_route31_target.at == seen);
            if (owners[owner] == 31u) {
                /* Current31 ball/bucket no longer act on a far flash. Only a
                 * NEW coordinate strictly within15px permits lost300 action. */
                loss_direct_poll(seen + VAT_ROUTE31_LOST_MS + 1u);
                CHECK(!s_route31_loss_braking && !s_route31_smooth_near &&
                      s_vat.state == VAT_FINE_CONTINUOUS && last_x != 0.0f && loss_no_arm());
                xy_obj_poll(wire_request,model,s_vat.x_goal + 15,y,0); wire_sync();
                seen = host_tick; loss_direct_poll(seen + VAT_ROUTE31_LOST_MS + 1u);
                CHECK(!s_route31_loss_braking && !s_route31_smooth_near && loss_no_arm());
                xy_obj_poll(wire_request,model,s_vat.x_goal + 14,y,0); wire_sync();
                seen = host_tick; CHECK(s_route31_smooth_near && s_route31_target.at == seen);
            }
            loss_direct_poll(seen + VAT_ROUTE31_LOST_MS);
            CHECK(!s_route31_loss_braking && loss_no_arm());
            loss_direct_poll(seen + VAT_ROUTE31_LOST_MS + 1u);
            CHECK(s_route31_loss_braking && loss_drive_stopped() && loss_no_arm());
            loss_direct_poll(host_tick + T_DIST_STILL_MS);
            CHECK(s_vat.state == (tasks[task] == PROTO_TASK_HOSTAGE ? VAT_WAIT_HOSTAGE_ACTION :
                  tasks[task] == PROTO_TASK_BUCKET ? VAT_WAIT_BUCKET_ACTION : VAT_WAIT_BALL_ACTION));
            CHECK(vision_align_test_take_route_action() == (tasks[task] == PROTO_TASK_HOSTAGE ?
                  VAT_ROUTE_ACTION_HOSTAGE : tasks[task] == PROTO_TASK_BUCKET ? VAT_ROUTE_ACTION_BUCKET :
                  VAT_ROUTE_ACTION_BALL));
            CHECK(loss_no_arm()); /* Scoped API exposes action; only dispatcher owns STEP. */
            vision_align_test_cancel();
        }
    puts("31 ball/bucket NEWnear<15 only lost300;43/44 farseen legacy unchanged;realCRC/VAT300 waits/301 brakes/still250 exposes action passed");
    return 0;
}

int main(void)
{
    CHECK(check_loss_far_flash_and_fine_gate_boundary() == 0);
    CHECK(check_loss_exact_boundary_and_wheel_still() == 0);
    CHECK(check_loss_new_frames_restart_and_invalid_frames_do_not() == 0);
    CHECK(check_loss_new_frame_cancels_brake_and_pending_take() == 0);
    CHECK(check_loss_never_seen_and_fault_priority() == 0);
    CHECK(check_loss_aligned_grab_unchanged() == 0);
    CHECK(check_loss_ball_bucket_and_legacy_scope() == 0);
    puts("route31_hostage_loss2s_test:host software only;no physical grasp/stop acceptance");
    return 0;
}
