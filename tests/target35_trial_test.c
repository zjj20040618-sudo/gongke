/* Real mode35 command/state router; only time, camera protocol and peripherals
 * are stubbed. The separate parser replay checks CRC/request/selection binding.
 * No test drives the robot, flashes firmware or changes camera code. */
#define main shared_g_command_fixture_main
#include "g_command_stop_test.c"
#undef main

static uint16_t target_sequence;

static int target_stopped(void)
{
    return last_x == 0.0f && last_y == 0.0f && last_w == 0.0f;
}

static void target_cache_packet(int cls, int label, int cx, uint16_t sequence)
{
    ProtoFrame frame;
    memset(&frame, 0, sizeof frame);
    frame.type = PF_OBJ; frame.cls = cls; frame.label = label;
    frame.cx = cx; frame.cy = 160; frame.w = 30; frame.h = 40;
    frame.conf = 95; frame.img_w = 480u; frame.img_h = 320u;
    frame.sequence = sequence;
    host_proto_stats.obj++; host_proto_stats.lines++;
    test_vision_feed_frame(&frame);
}

static void target_packet(int cls, int label, int cx, uint16_t sequence)
{
    target_cache_packet(cls, label, cx, sequence);
    test_poll();
}

static void target_frame(int cx)
{
    target_packet(CLS_TARGET, s_target35_qr[1] - 1, cx, ++target_sequence);
}

static void target_empty(void)
{
    /* An accepted OBJECT packet with zero selected objects has no OBJ callback. */
    host_proto_stats.obj++; host_proto_stats.lines++;
    test_poll();
}

static int target_ready(void)
{
    reset_fixture(); target_sequence = 0u;
    run_cmd("35");
    CHECK(TARGET_TRIAL_MODE == 35 && T_MODE_MAX == 43);
    CHECK(s_msel == TARGET_TRIAL_MODE && s_target35_phase == TA_READY);
    CHECK(target_stopped() && !laser_state && !s_go && !pulse_calls && !servo_calls);
    CHECK(host_target_calls == 0);
    return 0;
}

static int target_qr_wait(void)
{
    CHECK(target_ready() == 0);
    run_cmd("g");
    CHECK(s_target35_phase == TA_QR_WAIT && target_stopped() && !laser_state);
    return 0;
}

static int target_task_wait(int digit)
{
    CHECK(target_qr_wait() == 0);
    host_scene_status = 1;
    host_qr_accept(3, digit, 1);
    test_poll();
    CHECK(s_target35_phase == TA_TASK_WAIT && target_stopped() && !laser_state);
    CHECK(host_target_calls == 1 && host_target_task == PROTO_TASK_TARGET && host_target_digit == digit);
    CHECK(s_target35_qr[0] == 3 && s_target35_qr[1] == digit && s_target35_qr[2] == 1);
    CHECK(host_scene_status == 0 && !host_receive_closed);
    return 0;
}

static int target_still(int digit)
{
    CHECK(target_task_wait(digit) == 0);
    test_poll(); /* QR+queued target is sufficient; neither ACK nor data is needed. */
    CHECK(s_target35_phase == TA_STILL && target_stopped() && !laser_state);
    CHECK(host_scene_status == 0 && !s_target35_sample.seen);
    return 0;
}

static int target_prep(int digit)
{
    CHECK(target_still(digit) == 0);
    unsigned zeros_before = (unsigned)zero_calls;
    host_tick += T_DIST_STILL_MS - 1u; target_frame(200);
    CHECK(s_target35_phase == TA_STILL && (unsigned)zero_calls == zeros_before);
    host_tick++; target_frame(200);
    CHECK(s_target35_phase == TA_PREP_WAIT && (unsigned)zero_calls == zeros_before + 1u);
    CHECK(target_stopped() && !laser_state && !prepare_calls);
    return 0;
}

static int target_seek(int digit)
{
    CHECK(target_prep(digit) == 0);
    host_tick += NAV_SETTLE_MS - 1u; target_frame(200);
    CHECK(s_target35_phase == TA_PREP_WAIT && target_stopped());
    host_tick++; target_frame(200); test_poll();
    CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && last_y == 0.0f && last_w == 0.0f);
    CHECK(!laser_state && !pulse_calls && !servo_calls && !prepare_calls && !s_go);
    return 0;
}

static int target_settle(int digit)
{
    CHECK(target_seek(digit) == 0);
    for (unsigned i = 0u; i < 5u; ++i) {
        host_tick += 20u; target_frame(255);
        CHECK(target_stopped() && !laser_state);
        CHECK(s_target35_phase == (i < 4u ? TA_SEEK : TA_SETTLE));
    }
    return 0;
}

static int target_fire(int digit)
{
    CHECK(target_settle(digit) == 0);
    for (unsigned i = 0u; i < 9u; ++i) {
        host_tick += 100u; target_frame(255);
        CHECK(s_target35_phase == TA_SETTLE && target_stopped() && !laser_state);
    }
    host_tick += 99u; target_frame(255);
    CHECK(s_target35_phase == TA_SETTLE && !laser_state);
    host_tick++; target_frame(255); test_poll();
    CHECK(s_target35_phase == TA_FIRE && target_stopped() && laser_state);
    CHECK(host_laser_on_calls == 1u);
    return 0;
}

static int check_qr_gate_and_second_digit(void)
{
    CHECK(target_ready() == 0);
    host_scene_status = 1; host_qr_accept(1, 2, 3); test_poll();
    CHECK(s_target35_phase == TA_READY && target_stopped() && host_target_calls == 0 && !laser_state);
    run_cmd("g"); test_poll();
    CHECK(s_target35_phase == TA_TASK_WAIT && host_target_digit == 2u && target_stopped());
    for (int digit = 1; digit <= 3; ++digit) {
        CHECK(target_qr_wait() == 0);
        host_tick += 60000u; test_poll();
        CHECK(s_target35_phase == TA_QR_WAIT && target_stopped() && host_target_calls == 0);
        host_scene_status = 1;
        static const int invalid[][3] = {{1,2,0},{0,2,3},{1,4,3},{1,2,4}};
        for (unsigned i = 0u; i < sizeof invalid / sizeof invalid[0]; ++i) {
            host_qr_accept(invalid[i][0], invalid[i][1], invalid[i][2]); test_poll();
            CHECK(s_target35_phase == TA_QR_WAIT && target_stopped() && host_target_calls == 0);
        }
        host_qr_accept(2, digit, 3); test_poll();
        CHECK(s_target35_phase == TA_TASK_WAIT && host_target_calls == 1 && host_target_digit == digit);
        host_tick += 60000u; test_poll();
        CHECK(s_target35_phase == TA_STILL && target_stopped() && !laser_state);
        CHECK(host_scene_status == 0 && !s_target35_sample.seen);
    }
    CHECK(target_qr_wait() == 0); host_scene_status = 1; host_target_result = 0;
    host_qr_accept(1,2,3); test_poll();
    CHECK(target_stopped() && !laser_state && s_target35_phase == TA_STOPPED);
    CHECK(target_task_wait(1) == 0); host_scene_status = -1; test_poll();
    CHECK(target_stopped() && !laser_state && s_target35_phase == TA_STOPPED);
    puts("target35 QR/request gate: pre-g legal cache/no movement, complete legal tuple, second digit1..3, no QR time release; queued target permits preparation without ACK/data; failure/NACK passed");
    return 0;
}

static int check_search_without_ack_or_coordinates(void)
{
    for (int acknowledged = 0; acknowledged <= 1; ++acknowledged) {
        CHECK(target_task_wait(2) == 0);
        host_scene_status = acknowledged;
        test_poll(); CHECK(s_target35_phase == TA_STILL && target_stopped());
        host_tick += T_DIST_STILL_MS; test_poll();
        CHECK(s_target35_phase == TA_PREP_WAIT && target_stopped() && zero_calls == 1);
        host_tick += NAV_SETTLE_MS; test_poll();
        CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && last_y == 0.0f && last_w == 0.0f);
        CHECK(!s_target35_sample.seen && !s_target35_good && !laser_state);
        CHECK(host_scene_status == acknowledged && host_target_calls == 1 && !pulse_calls && !servo_calls);
        host_tick += 60000u; test_poll();
        CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && !laser_state && !s_target35_good);
        host_scene_status = -1; test_poll();
        CHECK(s_target35_phase == TA_STOPPED && target_stopped() && !laser_state && host_receive_closed);
    }
    puts("target35 no-coordinate search: legal QR starts +50 after preparation with no target ACK/data or ACK-only; time never fakes aim, NACK/error stops passed");
    return 0;
}

static int check_new_scan_and_preparation_boundaries(void)
{
    /* Selection must not silently reuse a previously accepted boot QR. */
    reset_fixture(); proto_send_scene(SCENE_QR); host_scene_status = 1;
    host_qr_accept(1, 2, 3); CHECK(host_qr_valid);
    run_cmd("35");
    CHECK(s_target35_phase == TA_READY && !host_qr_valid && target_stopped());
    run_cmd("g"); test_poll();
    CHECK(s_target35_phase == TA_QR_WAIT && !host_target_calls && target_stopped());

    CHECK(target_seek(2) == 0); target_frame(255); run_cmd("g");
    CHECK(s_target35_phase == TA_STOPPED && host_receive_closed);
    int targets_before = host_target_calls;
    run_cmd("g"); test_poll();
    CHECK(s_target35_phase == TA_QR_WAIT && !host_receive_closed && !host_qr_valid);
    CHECK(!s_target35_qr[0] && !s_target35_qr[1] && !s_target35_qr[2]);
    CHECK(!s_target35_sample.seen && !s_target35_good && target_stopped() && !laser_state);
    host_tick += 60000u; target_packet(CLS_TARGET, LAB_G, 255, ++target_sequence);
    CHECK(s_target35_phase == TA_QR_WAIT && host_target_calls == targets_before && target_stopped());
    host_scene_status = 1; host_qr_accept(3, 1, 2); test_poll();
    CHECK(s_target35_phase == TA_TASK_WAIT && host_target_digit == 1u);

    CHECK(target_fire(3) == 0); host_tick += 30000u; test_poll();
    CHECK(s_target35_phase == TA_FIRE && laser_state && target_stopped() && host_receive_closed);
    run_cmd("a");
    CHECK(s_target35_phase == TA_STOPPED && !laser_state && host_receive_closed);
    targets_before = host_target_calls;
    run_cmd("g"); test_poll();
    CHECK(s_target35_phase == TA_QR_WAIT && !host_qr_valid && !s_target35_sample.seen);
    CHECK(host_target_calls == targets_before && target_stopped() && !laser_state);

    CHECK(target_still(1) == 0);
    unsigned zeros_before = (unsigned)zero_calls;
    host_tick += T_DIST_STILL_MS - 1u; host_counts[0]++; target_frame(255);
    host_tick += T_DIST_STILL_MS - 1u; target_frame(255);
    CHECK(s_target35_phase == TA_STILL && (unsigned)zero_calls == zeros_before);
    host_tick++; target_frame(255);
    CHECK(s_target35_phase == TA_PREP_WAIT && (unsigned)zero_calls == zeros_before + 1u);
    host_tick += NAV_SETTLE_MS - 1u; host_counts[1]++; target_frame(255);
    CHECK(s_target35_phase == TA_STILL && target_stopped());
    host_tick += T_DIST_STILL_MS; target_frame(255);
    CHECK(s_target35_phase == TA_PREP_WAIT && (unsigned)zero_calls == zeros_before + 2u);
    for (unsigned i = 0u; i < 5u; ++i) target_frame(255);
    CHECK(!s_target35_good && s_target35_phase == TA_PREP_WAIT && target_stopped());
    host_tick += NAV_SETTLE_MS; target_frame(255);
    CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && !s_target35_sample.seen && !s_target35_good);
    test_poll(); CHECK(!s_target35_good && last_x == 50.0f);
    target_frame(255); CHECK(s_target35_good == 1u && target_stopped());
    puts("target35 restart/preparation: selection/manual-stop restart require new QR; encoder motion resets still/prep; pre-preparation coordinates discarded passed");
    return 0;
}

static int check_direction_boundaries_and_independent_frames(void)
{
    CHECK(target_seek(2) == 0);
    /*31 now has a350-pixel coarse200/fine30 gate plus right6.5% approach.
     * Independent35 MUST NOT inherit any of those parameters or latch. */
    target_frame(400);
    CHECK(last_x == 50.0f && last_y == 0.0f && !s_target31_fine && !s_target35_route);
    target_frame(351);
    CHECK(last_x == 50.0f && last_y == 0.0f && !s_target31_fine);
    target_frame(350);
    CHECK(last_x == 50.0f && last_y == 0.0f && !s_target31_fine);
    target_empty();
    CHECK(last_x == 50.0f && last_y == 0.0f && !s_target31_fine);
    target_frame(249); CHECK(last_x == -50.0f && s_target35_phase == TA_SEEK);
    target_frame(261); CHECK(last_x == 50.0f && last_y == 0.0f);
    target_frame(250); CHECK(target_stopped() && s_target35_good == 1u);
    /* Polling a cached frame is not a new observation. */
    for (unsigned i = 0u; i < 8u; ++i) { host_tick += 10u; test_poll(); }
    CHECK(s_target35_phase == TA_SEEK && s_target35_good == 1u && target_stopped());
    for (unsigned i = 0u; i < 8u; ++i) {
        target_packet(CLS_TARGET, LAB_G, 255, target_sequence);
        CHECK(s_target35_good == 1u && s_target35_phase == TA_SEEK && target_stopped());
    }
    target_frame(260); CHECK(target_stopped() && s_target35_good == 2u);
    target_frame(255); target_frame(250);
    CHECK(s_target35_good == 4u && s_target35_phase == TA_SEEK);
    target_frame(260);
    CHECK(s_target35_phase == TA_SETTLE && target_stopped() && !laser_state);
    puts("target35 directions/boundaries: x400/351/350 and loss still50/no lateral/no31 latch; x249 -50, x261 +50; x250/260 included; brake on first good; cache/same-seq cannot complete five passed");
    return 0;
}

static void target31_trial_wire_diag(ProtoWireDiag *out)
{
    memset(out,0,sizeof *out);
    out->request=1u;out->receiving=(uint8_t)!host_receive_closed;
    out->ack=(uint8_t)(host_scene_status==1);out->failed=(uint8_t)(host_scene_status<0);
    out->mode=2u;out->task=(uint8_t)host_target_task;out->selection=host_target_digit;
}
static int check_route31_fine_gate_is_not_an_aim_gate(void)
{
    for (unsigned burst = 0u; burst < 2u; ++burst) {
        reset_fixture(); run_cmd("31"); run_cmd("g");
        s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
        host_wire_diag_hook=target31_trial_wire_diag;
        s_seq_stage = ROUTE31_TARGET_STAGE; route_seq_prepare();
        CHECK(s_target35_route && s_target35_phase == TA_TASK_WAIT && target_stopped());
        host_scene_status = 1; test_poll();
        host_tick += T_DIST_STILL_MS; test_poll();
        host_tick += NAV_SETTLE_MS; test_poll();
    CHECK(s_target35_phase == TA_SEEK && last_x == 200.0f && last_y == 9.0f && !s_target31_fine);
        target_packet(CLS_TARGET, LAB_G, 400, 10u);
        target_packet(CLS_TARGET, LAB_G, 360, 10u); /* Duplicate. */
        CHECK(last_x == 200.0f && last_y == 9.0f && !s_target31_fine);
        target_packet(CLS_TARGET, LAB_G, 359, 9u); /* Backwards. */
        CHECK(last_x == 200.0f && last_y == 9.0f && !s_target31_fine);
        target_packet(CLS_TARGET, LAB_G, 361, 11u);
        CHECK(last_x == 200.0f && last_y == 9.0f && !s_target31_fine);
        target_cache_packet(CLS_TARGET, LAB_G, 360, 12u);
        if (burst) { target_cache_packet(CLS_TARGET, LAB_G, 400, 13u); test_poll(); }
        else target_empty();
        CHECK(last_x == 30.0f && last_y == 0.0f && s_target31_fine &&
              s_target35_phase == TA_SEEK && !s_target35_good && !laser_state);
        host_tick += T_TARGET35_FRESH_MS + 1u; test_poll();
        CHECK(last_x == 30.0f && last_y == 0.0f && s_target31_fine && !laser_state);
        run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && target_stopped() && !laser_state);
    }
    puts("31-only direct RX:far400 then duplicate/backward360 cannot latch; fresh360->empty/far400 samepoll permanentlyfine30/noFF, never fakegood/laser;35 isolated passed");
    return 0;
}

static int check_route31_private_band_geometry(void)
{
    CHECK(ROUTE31_TARGET_CX == 250 && ROUTE31_TARGET_LOW_CX == 247 && ROUTE31_TARGET_HIGH_CX == 253);
    for (unsigned width = 253u; width <= 255u; ++width) {
        ProtoFrame frame;
        reset_fixture(); run_cmd("31"); run_cmd("g");
        s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
        host_wire_diag_hook = target31_trial_wire_diag;
        s_seq_stage = ROUTE31_TARGET_STAGE; route_seq_prepare();
        host_scene_status = 1; test_poll();
        host_tick += T_DIST_STILL_MS; test_poll();
        host_tick += NAV_SETTLE_MS; test_poll();
        CHECK(s_target35_phase == TA_SEEK && !s_target31_fine);
        memset(&frame, 0, sizeof frame);
        frame.type = PF_OBJ; frame.cls = CLS_TARGET; frame.label = LAB_G;
        frame.cx = 250; frame.cy = 160; frame.w = frame.h = 2;
        frame.conf = 95; frame.img_w = (uint16_t)width; frame.img_h = 320;
        frame.sequence = 1;
        host_proto_stats.obj++; host_proto_stats.lines++;
        test_vision_feed_frame(&frame); test_poll();
        if (width == 253u) CHECK(s_seq_state == SQ_STOPPED && !laser_state);
        else CHECK(s_target31_fine && s_target35_good == 1u && target_stopped() && !laser_state);
        CHECK(T_TARGET35_CX == 255 && T_TARGET35_LOW_CX == 250 && T_TARGET35_HIGH_CX == 260);
        run_cmd("0");
    }
    puts("31 private250/247..253: valid image widths254..255 latch fine at250,253 width rejected; standalone35 constants unchanged passed");
    return 0;
}

static int check_route31_private_band_endpoints(void)
{
    static const int points[] = {246,247,250,253,254};
    for (unsigned i = 0u; i < sizeof points / sizeof points[0]; ++i) {
        reset_fixture(); run_cmd("31"); run_cmd("g");
        s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
        host_wire_diag_hook = target31_trial_wire_diag;
        s_seq_stage = ROUTE31_TARGET_STAGE; route_seq_prepare();
        host_scene_status = 1; test_poll();
        host_tick += T_DIST_STILL_MS; test_poll();
        host_tick += NAV_SETTLE_MS; test_poll();
        CHECK(s_target35_phase == TA_SEEK && target35_point() == 250 &&
              target35_low() == 247 && target35_high() == 253);
        target_packet(CLS_TARGET, LAB_G, points[i], 1u);
        CHECK(s_target31_fine && s_target35_phase == TA_SEEK && !laser_state && !last_y && !last_w);
        if (points[i] < 247) CHECK(last_x == -30.0f && !s_target35_good);
        else if (points[i] > 253) CHECK(last_x == 30.0f && !s_target35_good);
        else CHECK(target_stopped() && s_target35_good == 1u);
        run_cmd("0"); CHECK(target_stopped() && !laser_state);
    }
    CHECK(target_ready() == 0 && target35_point() == 255 && target35_low() == 250 && target35_high() == 260);
    puts("31 band247/253 included and246/254 correct directions;250 center;standalone35 remains255/250..260 passed");
    return 0;
}

static int check_empty_wrong_color_and_expiry(void)
{
    CHECK(target_seek(2) == 0);
    target_frame(250); target_frame(255); CHECK(s_target35_good == 2u);
    target_packet(CLS_TARGET, LAB_R, 255, ++target_sequence);
    CHECK(s_target35_phase == TA_SEEK && !s_target35_good && last_x == 50.0f && !laser_state);
    target_frame(249); CHECK(last_x == -50.0f);
    target_empty(); CHECK(last_x == 50.0f && !s_target35_good && !laser_state);
    target_packet(CLS_BALL, LAB_G, 255, ++target_sequence);
    CHECK(last_x == 50.0f && !s_target35_good && s_target35_phase == TA_SEEK);
    target_frame(249); CHECK(last_x == -50.0f);
    host_tick += 300u; test_poll();
    CHECK(last_x == -50.0f && s_target35_phase == TA_SEEK);
    host_tick++; test_poll();
    CHECK(last_x == 50.0f && !s_target35_good && !laser_state && s_target35_phase == TA_SEEK);
    host_tick += 60000u; test_poll(); CHECK(last_x == 50.0f && !laser_state && !host_laser_on_calls);
    for (unsigned i = 0u; i < 4u; ++i) target_frame(255);
    CHECK(s_target35_phase == TA_SEEK && s_target35_good == 4u);
    target_frame(255); CHECK(s_target35_phase == TA_SETTLE);
    puts("target35 loss search: wrong selected color/class, empty packet and >300ms stale clear good/resume +50 rather than old reverse; fresh reacquisition needs five passed");
    return 0;
}

static int check_settle_revalidation_and_fire_timing(void)
{
    CHECK(target_settle(1) == 0);
    host_tick += 500u; target_frame(261);
    CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && !laser_state && !s_target35_good);
    for (unsigned i = 0u; i < 5u; ++i) target_frame(255);
    CHECK(s_target35_phase == TA_SETTLE && target_stopped());
    target_empty(); CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && !laser_state);
    CHECK(target_settle(3) == 0);
    host_tick += 1000u; test_poll();
    CHECK(s_target35_phase == TA_SEEK && last_x == 50.0f && !laser_state && !host_laser_on_calls && !s_target35_good);
    for (unsigned i = 0u; i < 4u; ++i) {
        target_frame(255);
        CHECK(s_target35_phase == TA_SEEK && target_stopped() && !laser_state && s_target35_good == i + 1u);
    }
    target_frame(255); CHECK(s_target35_phase == TA_SETTLE && target_stopped() && !laser_state);
    CHECK(target_fire(3) == 0);
    host_tick += 1999u; test_poll();
    CHECK(s_target35_phase == TA_FIRE && laser_state && target_stopped());
    host_tick++; test_poll();
    CHECK(s_target35_phase == TA_FIRE && laser_state && target_stopped() && host_receive_closed);
    CHECK(host_laser_on_calls == 1u && !pulse_calls && !servo_calls && !s_go && !host_abort);
    unsigned precise_before = host_precise_calls, integer_before = host_integer_calls;
    host_tick += 28000u; target_frame(100); test_poll();
    CHECK(s_target35_phase == TA_FIRE && target_stopped() && laser_state && host_receive_closed);
    CHECK(host_laser_on_calls == 1u && !pulse_calls && !servo_calls && !s_go);
    CHECK(host_precise_calls == precise_before && host_integer_calls == integer_before);
    run_cmd("g");
    CHECK(s_target35_phase == TA_STOPPED && target_stopped() && !laser_state && host_receive_closed);
    host_tick += 60000u; target_frame(100); test_poll();
    CHECK(s_target35_phase == TA_STOPPED && target_stopped() && !laser_state);
    puts("target35 settle/fire: out-of-band or empty restarts alignment, no stale shot; full fresh1s then continuous laser at2s/30s, stationary/local-close until manual stop passed");
    return 0;
}

static unsigned target_atomic_kind;

static void target_atomic_interrupt(int motor)
{
    if (motor != 0) return;
    host_counts_hook = NULL; /* One interrupt between old snapshot and final check. */
    switch (target_atomic_kind) {
        case 0u: host_proto_stats.obj++; host_proto_stats.lines++; break;
        case 1u: target_cache_packet(CLS_TARGET, LAB_G, 249, ++target_sequence); break;
        case 2u: target_cache_packet(CLS_TARGET, LAB_R, 255, ++target_sequence); break;
        case 3u: target_cache_packet(CLS_TARGET, LAB_G, 255, ++target_sequence); break;
        case 4u: host_imu_valid = 0; break;
        case 5u: host_abort = 1; break;
        case 6u: host_scene_status = -1; break;
        default: break;
    }
}

static int check_final_snapshot_transition(void)
{
    /* Inject at a production call site that is after aim's earlier snapshot but
     * before its final atomic snapshot. This is deterministic host fault
     * injection, not proof of interrupt timing on the physical MCU. */
    for (target_atomic_kind = 0u; target_atomic_kind < 7u; ++target_atomic_kind) {
        CHECK(target_settle(2) == 0);
        for (unsigned i = 0u; i < 9u; ++i) { host_tick += 100u; target_frame(255); }
        host_tick += 99u; target_frame(255);
        CHECK(s_target35_phase == TA_SETTLE && !laser_state && !host_laser_on_calls);
        host_counts_hook = target_atomic_interrupt;
        host_tick++; target_frame(255);
        CHECK(!host_counts_hook && !laser_state && !host_laser_on_calls && target_stopped());
        CHECK(s_target35_phase == TA_SETTLE);
        test_poll();
        if (target_atomic_kind == 3u) {
            CHECK(s_target35_phase == TA_FIRE && laser_state && host_laser_on_calls == 1u);
            run_cmd("g"); CHECK(!laser_state && target_stopped());
        } else if (target_atomic_kind >= 4u) {
            CHECK(s_target35_phase == TA_STOPPED && target_stopped() && host_receive_closed);
        } else {
            CHECK(s_target35_phase == TA_SEEK && !laser_state && !host_laser_on_calls && !s_target35_good);
            CHECK(last_x == (target_atomic_kind == 1u ? -50.0f : 50.0f));
        }
    }
    puts("target35 final snapshot fault injection: intervening empty/out-of-band/wrong-color/new-in-band packet/IMU invalid/abort/NACK cannot fire old cached aim; next tick consumes newest state passed");
    return 0;
}

static int enter_target_phase(unsigned phase)
{
    switch (phase) {
        case TA_QR_WAIT: return target_qr_wait();
        case TA_TASK_WAIT: return target_task_wait(2);
        case TA_STILL: return target_still(2);
        case TA_PREP_WAIT: return target_prep(2);
        case TA_SEEK: return target_seek(2);
        case TA_SETTLE: return target_settle(2);
        case TA_FIRE: return target_fire(2);
        default: CHECK(0); return 1;
    }
}

static int check_all_phase_stop_and_command_locks(void)
{
    static const char *const stops[] = {"g", "a", "0"};
    static const char *const writes[] = {"35", "15", "20", "31", "32", "33", "34", "r1",
        "v300", "d1000", "ykp1", "okp1", "kp1", "ki0.1", "lp0.5", "dead30",
        "fff0", "lff0", "rff0", "acc100", "dec100", "co", "cc", "su1600", "u1600",
        "n5", "nl5", "nr5", "bcx250", "tcx255", "hcx250", "kcx250", "vsg1", "b1d800"};
    for (unsigned phase = TA_QR_WAIT; phase <= TA_FIRE; ++phase) {
        for (unsigned key = 0u; key < sizeof stops / sizeof stops[0]; ++key) {
            CHECK(enter_target_phase(phase) == 0);
            if (phase == TA_FIRE) {
                host_tick += 2000u; test_poll();
                CHECK(s_target35_phase == TA_FIRE && laser_state && target_stopped());
                host_tick += 28000u; test_poll();
                CHECK(s_target35_phase == TA_FIRE && laser_state && target_stopped() && host_receive_closed);
            }
            unsigned laser_before = host_laser_on_calls;
            run_cmd(stops[key]);
            CHECK(s_target35_phase == TA_STOPPED && s_msel == 35 && target_stopped() && !laser_state);
            CHECK(host_receive_closed && !pulse_calls && !servo_calls && !s_go);
            host_tick += 60000u; host_scene_status = 1; target_frame(100); test_poll();
            CHECK(s_target35_phase == TA_STOPPED && target_stopped() && !laser_state);
            CHECK(host_laser_on_calls == laser_before);
            if (phase == TA_FIRE) {
                unsigned targets_before = (unsigned)host_target_calls;
                run_cmd("g"); test_poll();
                CHECK(s_target35_phase == TA_QR_WAIT && !host_receive_closed && !host_qr_valid);
                CHECK(!s_target35_sample.seen && !s_target35_good && !s_target35_qr[1]);
                CHECK((unsigned)host_target_calls == targets_before && target_stopped() && !laser_state);
            }
        }
        CHECK(enter_target_phase(phase) == 0);
        float x_before = last_x, y_before = last_y, w_before = last_w;
        uint16_t servo_before = host_servo;
        unsigned laser_before = host_laser_on_calls, packet_before = (unsigned)host_target_calls;
        for (unsigned i = 0u; i < sizeof writes / sizeof writes[0]; ++i) {
            run_cmd(writes[i]);
            CHECK(strstr(last_message, "ERR") != NULL);
            CHECK(s_msel == 35 && s_target35_phase == phase);
            CHECK(last_x == x_before && last_y == y_before && last_w == w_before);
            CHECK(host_servo == servo_before && !pulse_calls && !servo_calls && host_laser_on_calls == laser_before);
            CHECK((unsigned)host_target_calls == packet_before);
        }
        run_cmd("param"); run_cmd("diag"); run_cmd("?");
        CHECK(s_msel == 35 && s_target35_phase == phase && !pulse_calls && !servo_calls);
    }
    CHECK(target_ready() == 0); laser_state = 1; run_cmd("15");
    CHECK(s_msel == 15 && !laser_state && target_stopped() && host_receive_closed);
    CHECK(target_fire(2) == 0); host_tick += 30000u; test_poll();
    CHECK(s_target35_phase == TA_FIRE && laser_state && target_stopped());
    host_imu_valid = 0; test_poll();
    CHECK(s_target35_phase == TA_STOPPED && target_stopped() && !laser_state && host_receive_closed);
    CHECK(target_fire(2) == 0); host_tick += 30000u; test_poll();
    CHECK(s_target35_phase == TA_FIRE && laser_state && target_stopped());
    host_abort = 1; test_poll();
    CHECK(s_target35_phase == TA_STOPPED && target_stopped() && !laser_state && host_receive_closed);
    /* Idle-framed pending g must win before a fifth fresh frame can advance. */
    CHECK(target_seek(2) == 0);
    for (unsigned i = 0u; i < 4u; ++i) target_frame(255);
    test_feed('g'); host_tick += T_IDLE_MS; target_frame(255);
    CHECK(s_target35_phase == TA_STOPPED && target_stopped() && !laser_state);
    puts("target35 abort/locks: 21 g/a/0 active phases incl30s continuous FIRE and fresh-QR restart; 238 parameter/actuator/mode writes locked; read-only commands, mode-exit laser-off, IMU/run-abort fire safety/pending-g priority passed");
    return 0;
}

int main(void)
{
    CHECK(check_qr_gate_and_second_digit() == 0);
    CHECK(check_search_without_ack_or_coordinates() == 0);
    CHECK(check_new_scan_and_preparation_boundaries() == 0);
    CHECK(check_direction_boundaries_and_independent_frames() == 0);
    CHECK(check_route31_fine_gate_is_not_an_aim_gate() == 0);
    CHECK(check_route31_private_band_geometry() == 0);
    CHECK(check_route31_private_band_endpoints() == 0);
    CHECK(check_empty_wrong_color_and_expiry() == 0);
    CHECK(check_settle_revalidation_and_fire_timing() == 0);
    CHECK(check_final_snapshot_transition() == 0);
    CHECK(check_all_phase_stop_and_command_locks() == 0);
    puts("target35_trial_test: all host checks passed; no hardware driven");
    return 0;
}
