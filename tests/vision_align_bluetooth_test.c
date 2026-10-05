/* Real Bluetooth38..41 dispatcher, VAT engine and CRC/request parser.
 * Pixels and IMU values are explicitly injected, not physical acceptance. */
#define ROUTE31_QR_FIXTURE_MAIN xy_prior_route_fixture_main
#include "route31_qr_gate_test.c"

static unsigned xy_seq;

static void xy_advance(unsigned milliseconds)
{
    for (unsigned elapsed = 0u; elapsed < milliseconds; elapsed += 20u) {
        host_tick += 20u; wire_poll();
    }
}

static void xy_qr(uint16_t request, const char *digits)
{
    unsigned count = (unsigned)strlen(digits), size = count ? 7u : 4u;
    uint8_t p[12] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8),
                    (uint8_t)size, 0u, 0x53u, 1u, 0u, (uint8_t)(count != 0u)};
    if (count == 3u) memcpy(p + 9u, digits, 3u);
    wire_feed(p, size + 5u, 0);
}

static void xy_obj(uint16_t request, int model, int x, int y)
{
    unsigned inner_size = model < 0 ? 14u : 25u;
    uint8_t p[30] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8),
                    (uint8_t)inner_size, 0u, 0x01u};
    route_wire_le16(p + 6u, xy_seq++);
    p[8] = (uint8_t)(model >= 0);
    route_wire_le16(p + 9u, 640u); route_wire_le16(p + 11u, 480u);
    if (model >= 0) {
        p[19] = (uint8_t)model; route_wire_le16(p + 20u, 900u);
        route_wire_le16(p + 22u, (unsigned)x); route_wire_le16(p + 24u, (unsigned)y);
        route_wire_le16(p + 26u, 24u); route_wire_le16(p + 28u, 24u);
    }
    wire_feed(p, inner_size + 5u, 0);
}

static int xy_boot(unsigned mode)
{
    char command[8];
    CHECK(wire_boot() == 0);
    host_wire_diag_hook = real_proto_wire_diag_get;
    xy_seq = 1u; snprintf(command, sizeof command, "%u", mode);
    run_cmd(command); wire_sync();
    CHECK(s_xy_owner == mode && s_xy_state == XT_READY && stopped());
    CHECK(wire_request == 2u && wire_opcode == 0x60u && wire_mode == 1u);
    return 0;
}

static int xy_start(unsigned mode)
{
    CHECK(xy_boot(mode) == 0);
    run_cmd("g"); CHECK(s_xy_state == XT_QR_WAIT && stopped());
    wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123");
    CHECK(s_xy_state == XT_ACTIVE && wire_opcode == 0x63u && stopped());
    CHECK(wire_task == (mode == 39u ? 4u : mode == 40u ? 3u : 1u));
    CHECK(wire_digit == (mode == 39u ? 0u : mode == 40u ? 3u : 1u));
    wire_ack(wire_request, 2u, 0u); xy_advance(1100u);
    return 0;
}

static int xy_finish(int model, int y)
{
    for (unsigned i = 0u; i < 120u && s_xy_state == XT_ACTIVE; ++i) {
        host_tick += 20u;
        xy_obj(wire_request, model, 190, y);
        vision_align_test_status(&s_xy_snapshot);
        if (s_xy_snapshot.state == VAT_HOLD_BALL || s_xy_snapshot.state == VAT_HOLD_BUCKET) break;
    }
    vision_align_test_status(&s_xy_snapshot);
    CHECK(s_xy_snapshot.state == VAT_HOLD_BALL || s_xy_snapshot.state == VAT_HOLD_BUCKET || s_xy_state == XT_DONE);
    CHECK(stopped()); return 0;
}

static int check_fresh_qr_and_selected_tasks(void)
{
    for (unsigned mode = 38u; mode <= 41u; ++mode) {
        CHECK(xy_boot(mode) == 0);
        wire_ack(1u, 1u, 0u); xy_qr(1u, "123"); run_cmd("g");
        CHECK(s_xy_state == XT_QR_WAIT && stopped() && !host_target_calls);
        wire_ack(2u, 1u, 0u); xy_qr(2u, ""); xy_advance(2000u);
        CHECK(s_xy_state == XT_QR_WAIT && stopped() && !host_target_calls);
        xy_qr(2u, "103"); CHECK(s_xy_state == XT_QR_WAIT && stopped());
        xy_qr(2u, "123");
        CHECK(s_xy_state == XT_ACTIVE && stopped() && host_target_calls == 1);
        CHECK(wire_task == (mode == 39u ? 4u : mode == 40u ? 3u : 1u));
        CHECK(wire_digit == (mode == 39u ? 0u : mode == 40u ? 3u : 1u));
        run_cmd("g"); CHECK(s_xy_state == XT_STOPPED && stopped() && !s_receiving && !s_due);
        uint16_t previous = wire_request;
        run_cmd("g"); wire_sync();
        CHECK(s_xy_state == XT_QR_WAIT && wire_request > previous && wire_opcode == 0x60u && stopped());
        xy_qr(2u, "123"); xy_advance(1000u);
        CHECK(s_xy_state == XT_QR_WAIT && stopped() && host_target_calls == 1);
        run_cmd("0"); CHECK(stopped());
    }
    CHECK(xy_boot(38u) == 0);
    wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123");
    CHECK(s_xy_state == XT_READY && stopped() && !host_target_calls);
    run_cmd("g"); wire_poll();
    CHECK(s_xy_state == XT_ACTIVE && wire_task == 1u && stopped());
    puts("XY Bluetooth: fresh QR on selection/rerun, empty/illegal/old QR blocked in all4; firstg preserves selectionQR; request1/4/3 routing passed");
    return 0;
}

static int check_single_alignment_and_command_locks(void)
{
    CHECK(xy_start(39u) == 0); CHECK(stopped());
    xy_obj(wire_request, 9, 190, 370); host_tick += 20u; wire_poll();
    CHECK(last_x == 0.0f && last_y == -16.0f); /* y-small moves LEFT. */
    static const char *const writes[] = {"31","34","35","37","38","v100","d200","ykp1","fff0","co","cc","nr5","su1500"};
    for (unsigned i = 0u; i < sizeof writes / sizeof writes[0]; ++i) {
        run_cmd(writes[i]); CHECK(s_xy_owner == 39u && s_xy_state == XT_ACTIVE && !servo_calls && !pulse_calls);
    }
    xy_obj(wire_request, -1, 0, 0); host_tick += 20u; wire_poll(); CHECK(stopped());
    host_absolute_yaw = 3.0f; xy_advance(300u);
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w < 0.0f && !zero_calls);
    host_absolute_yaw = 0.0f; xy_advance(1000u);
    CHECK(stopped());
    CHECK(xy_finish(9, 400) == 0); CHECK(s_xy_state == XT_DONE && s_msel == 39 && !s_receiving);
    CHECK(s_seq_state == SQ_OFF && !prepare_calls && !laser_state);
    run_cmd("31"); CHECK(s_xy_state == XT_OFF && s_xy_owner == 0u && s_seq_state == SQ_READY);
    run_cmd("0"); run_cmd("35"); CHECK(s_msel == 35 && s_target35_phase == TA_READY && stopped());
    puts("XY Bluetooth: YsmallLEFT, missing-frame brake/absolute heading restoration without zero, arrivalDONE, locks and legacy31/35 isolation passed");
    return 0;
}

static int check_ball_turn_bucket_and_stops(void)
{
    CHECK(xy_start(41u) == 0);
    uint32_t test_id = s_active_test;
    CHECK(xy_finish(4, 420) == 0); CHECK(s_xy_snapshot.state == VAT_HOLD_BALL);
    xy_advance(4980u); CHECK(s_xy_state == XT_ACTIVE && stopped());
    xy_advance(20u); CHECK(s_xy_state == XT_TURN_STILL && wire_task == 4u && stopped());
    uint16_t before_turn = wire_request;
    wire_ack(before_turn, 2u, 0u); xy_obj(before_turn, 9, 190, 400);
    xy_advance(1100u); CHECK(s_xy_state == XT_TURN_RUN && s_msel == 22 && !prepare_calls);
    xy_advance(20u); CHECK(last_x == 0.0f && last_y == 0.0f && last_w > 0.0f);
    host_yaw = host_absolute_yaw = 180.0f; wire_poll(); CHECK(s_round == R_BRAKE);
    xy_advance(T_TURN_SETTLE_MS);
    CHECK(s_xy_state == XT_ACTIVE && s_msel == 41 && wire_task == 4u && wire_request > before_turn && stopped());
    CHECK(s_active_test == test_id && s_test_seq == test_id);
    xy_obj(before_turn, 9, 190, 400); xy_advance(1000u);
    CHECK(s_xy_state == XT_ACTIVE && stopped());
    wire_ack(wire_request, 2u, 0u);
    CHECK(xy_finish(9, 400) == 0); CHECK(s_xy_snapshot.state == VAT_HOLD_BUCKET);
    xy_advance(5000u); CHECK(s_xy_state == XT_DONE && s_msel == 41 && stopped() && !s_due && !s_receiving);
    CHECK(!prepare_calls && zero_calls == 1 && !laser_state && !pulse_calls && !servo_calls && s_seq_state == SQ_OFF);

    static const char *const stop_keys[] = {"g","a","0"};
    static const unsigned phases[] = {XT_QR_WAIT,XT_ACTIVE,XT_TURN_STILL,XT_TURN_WAIT,XT_TURN_RUN};
    for (unsigned p = 0u; p < sizeof phases / sizeof phases[0]; ++p) {
        for (unsigned k = 0u; k < 3u; ++k) {
            CHECK(xy_start(41u) == 0);
            s_xy_state = (uint8_t)phases[p];
            if (phases[p] == XT_TURN_RUN) s_msel = 22;
            run_cmd(stop_keys[k]); xy_advance(6000u);
            CHECK(s_xy_state == XT_STOPPED && s_msel == 41 && stopped() && !s_due && !s_receiving && !prepare_calls);
        }
    }
    puts("XY Bluetooth: ball5s/request4-beforeturn/shared22+180/newrequest4/bucket5sSTOP; one testid;15 g/a/0 owner-stop states passed");
    return 0;
}

int main(void)
{
    CHECK(check_fresh_qr_and_selected_tasks() == 0);
    CHECK(check_single_alignment_and_command_locks() == 0);
    CHECK(check_ball_turn_bucket_and_stops() == 0);
    puts("vision_align_bluetooth_test: all host checks passed; no physical camera/laser/arm/route acceptance");
    return 0;
}
