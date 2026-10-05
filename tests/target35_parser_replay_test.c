/* Mode35 command/state machine + real request/CRC/QR/object parser replay.
 * Uses the shared host peripheral fixture; does not drive physical hardware. */
#define ROUTE31_QR_FIXTURE_MAIN route31_qr_fixture_main
#include "route31_qr_gate_test.c"
#undef ROUTE31_QR_FIXTURE_MAIN

static uint8_t target_wire_packet[9];
static uint16_t target_wire_length;
static unsigned target_wire_calls, target_wire_bad;

static void target_wire_tx(const uint8_t *packet, uint16_t length)
{
    uint16_t crc;
    target_wire_calls++;
    target_wire_length = length;
    if (length > sizeof target_wire_packet || length < 4u) {
        target_wire_bad++;
        return;
    }
    memcpy(target_wire_packet, packet, length);
    crc = binary_crc(packet + 2u, length - 4u);
    if (packet[0] != 0xAAu || packet[1] != 0x55u ||
        packet[length - 2u] != (uint8_t)crc ||
        packet[length - 1u] != (uint8_t)(crc >> 8)) target_wire_bad++;
    if ((length == 8u && packet[2] == 0x60u) ||
        (length == 9u && packet[2] == 0x63u)) {
        wire_commands++;
        wire_request = (uint16_t)(packet[3] | ((uint16_t)packet[4] << 8));
        wire_mode = packet[2] == 0x63u ? 2u : packet[5];
    } else target_wire_bad++;
}

static void target_wire_on_frame(const ProtoFrame *frame)
{
    /* Production callback queries the parser's CURRENT packet counter. The
     * shared router fixture normally synchronizes its stats after feeding;
     * synchronize here too, before Mode35 snapshots stats.obj in its callback. */
    real_proto_stats_get(&host_proto_stats);
    test_vision_feed_frame(frame);
}

static void target_inner(uint16_t request, const uint8_t *inner,
                         unsigned length, int corrupt)
{
    uint8_t packet[41] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8),
                          (uint8_t)length, (uint8_t)(length >> 8)};
    memcpy(packet + 5u, inner, length);
    wire_feed(packet, length + 5u, corrupt);
}

static void target_qr(uint16_t request, uint16_t sequence, const char *tuple,
                      int corrupt)
{
    unsigned length = (unsigned)strlen(tuple);
    uint8_t inner[12] = {0x53u, (uint8_t)sequence, (uint8_t)(sequence >> 8), 1u};
    memcpy(inner + 4u, tuple, length);
    target_inner(request, inner, length + 4u, corrupt);
}

static void target_qr_empty(uint16_t request, uint16_t sequence)
{
    uint8_t inner[4] = {0x53u, (uint8_t)sequence, (uint8_t)(sequence >> 8), 0u};
    target_inner(request, inner, sizeof inner, 0);
}

static void target_le16(uint8_t *out, unsigned value)
{
    out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8);
}

static void target_object(uint16_t request, uint16_t sequence, int model,
                          unsigned cx, unsigned width, int corrupt)
{
    uint8_t inner[25] = {0x01u, (uint8_t)sequence, (uint8_t)(sequence >> 8),
                         (uint8_t)(model >= 0)};
    target_le16(inner + 4u, width); target_le16(inner + 6u, 320u);
    target_le16(inner + 8u, 10u); target_le16(inner + 10u, 20u);
    target_le16(inner + 12u, 30u);
    if (model >= 0) {
        inner[14] = (uint8_t)model;
        target_le16(inner + 15u, 900u); target_le16(inner + 17u, cx);
        target_le16(inner + 19u, 120u); target_le16(inner + 21u, 24u);
        target_le16(inner + 23u, 24u);
    }
    target_inner(request, inner, model >= 0 ? sizeof inner : 14u, corrupt);
}

static int target_model(int digit)
{
    static const int models[3] = {6, 8, 7}; /* red, green, blue TARGET */
    return models[digit - 1];
}

static int target_searching(void)
{
    return last_x == 50.0f && last_y == 0.0f && last_w == 0.0f &&
           !laser_state && !pulse_calls && !servo_calls && !s_go;
}

static int target_boot(void)
{
    CHECK(wire_boot() == 0);
    memset(target_wire_packet, 0, sizeof target_wire_packet);
    target_wire_length = 0u; target_wire_calls = target_wire_bad = 0u;
    proto_set_binary_tx(target_wire_tx); proto_set_on_frame(target_wire_on_frame);
    host_target_hook = real_proto_send_target;
    run_cmd("35"); wire_sync();
    CHECK(s_msel == TARGET_TRIAL_MODE && s_target35_phase == TA_READY && stopped());
    CHECK(wire_request > 1u && wire_mode == 1u && s_receiving && !target_wire_bad);
    CHECK(target_wire_length == 8u && target_wire_packet[2] == 0x60u);
    return 0;
}

static int target_start_task(const char *tuple)
{
    CHECK(target_boot() == 0);
    uint16_t qr_request = wire_request;
    run_cmd("g"); wire_poll();
    CHECK(s_target35_phase == TA_QR_WAIT && stopped() && wire_request == qr_request);
    wire_ack(qr_request, 1u, 0u); target_qr(qr_request, 1u, tuple, 0);
    CHECK(s_target35_phase == TA_TASK_WAIT && stopped());
    CHECK(host_target_calls == 1 && host_target_task == PROTO_TASK_TARGET &&
          host_target_digit == (uint8_t)(tuple[1] - '0'));
    CHECK(wire_request == (uint16_t)(qr_request + 1u) && wire_mode == 2u &&
          target_wire_length == 9u && target_wire_packet[2] == 0x63u &&
          target_wire_packet[5] == 2u && target_wire_packet[6] == (uint8_t)(tuple[1] - '0') &&
          !target_wire_bad);
    return 0;
}

static int target_prepare(void)
{
    int model = target_model((int)host_target_digit);
    CHECK(s_target35_phase == TA_TASK_WAIT || s_target35_phase == TA_STILL);
    wire_ack(wire_request, 2u, 0u);
    CHECK(s_target35_phase == TA_STILL && stopped()); /* Preparation no longer waits for a result. */
    target_object(wire_request, 1u, model, 230u, 480u, 0);
    CHECK(s_target35_phase == TA_STILL && stopped());
    host_tick += T_DIST_STILL_MS; wire_poll();
    CHECK(s_target35_phase == TA_PREP_WAIT && stopped());
    host_tick += NAV_SETTLE_MS; wire_poll();
    CHECK(s_target35_phase == TA_SEEK && !s_target35_sample.seen && !s_target35_good);
    wire_poll();
    CHECK(last_x == 50.0f && last_y == 0.0f && last_w == 0.0f && !laser_state);
    return 0;
}

static int check_target_qr_gate(void)
{
    CHECK(target_boot() == 0);
    uint16_t request = wire_request;
    run_cmd("g");
    target_qr(request, 1u, "123", 0); /* Before matching ACK. */
    wire_ack((uint16_t)(request - 1u), 1u, 0u);
    target_qr((uint16_t)(request - 1u), 2u, "123", 0);
    CHECK(s_target35_phase == TA_QR_WAIT && stopped() && !host_target_calls);
    wire_ack(request, 1u, 0u);
    {
        const uint8_t bare_qr[7] = {0x53u, 99u, 0u, 1u, '1', '2', '3'};
        wire_feed(bare_qr, sizeof bare_qr, 0); /* Valid CRC, no request-bound62 wrapper. */
        CHECK(s_target35_phase == TA_QR_WAIT && stopped() && !host_target_calls &&
              !proto_qr_get(NULL) && !s_target35_sample.seen && !notice_calls);
        CHECK(s_wire.legacy_qr == 1u);
    }
    target_qr(request, 3u, "120", 0); target_qr(request, 4u, "423", 0);
    target_qr(request, 5u, "12", 0); target_qr(request, 6u, "1231", 0);
    target_qr_empty(request, 7u); target_qr(request, 8u, "123", 1);
    CHECK(s_target35_phase == TA_QR_WAIT && stopped() && !host_target_calls && !notice_calls);
    CHECK(host_proto_stats.crc_bad == 1u);
    host_tick += 60000u; wire_poll();
    CHECK(s_target35_phase == TA_QR_WAIT && stopped() && !host_target_calls);
    target_qr(request, 9u, "231", 0);
    CHECK(s_target35_phase == TA_TASK_WAIT && stopped() && host_target_digit == 3u);
    CHECK(s_target35_qr[0] == 2 && s_target35_qr[1] == 3 && s_target35_qr[2] == 1);
    target_qr(request, 10u, "123", 0); /* Old QR stage cannot retarget an active target request. */
    CHECK(host_target_calls == 1 && host_target_digit == 3u && s_target35_qr[1] == 3);

    CHECK(target_boot() == 0); request = wire_request;
    wire_ack(request, 1u, 0u); target_qr(request, 1u, "312", 0);
    CHECK(s_target35_phase == TA_READY && stopped() && !host_target_calls);
    run_cmd("g"); wire_poll(); /* A manually presented READY QR is reusable, not old target data. */
    CHECK(s_target35_phase == TA_TASK_WAIT && host_target_digit == 1u && stopped());

    CHECK(target_boot() == 0); request = wire_request; run_cmd("g");
    wire_ack(request, 1u, 1u);
    CHECK(s_target35_phase == TA_STOPPED && stopped() && !s_receiving);
    target_qr(request, 1u, "123", 0); host_tick += 60000u; wire_poll();
    CHECK(s_target35_phase == TA_STOPPED && stopped() && !host_target_calls);
    puts("target35 real QR53: pre-ACK/old request/illegal or partial tuple/empty/CRC/NACK rejected; READY cache and no timed release passed");
    return 0;
}

static int check_target_selection_and_handshake(void)
{
    for (int a = 1; a <= 3; a++) for (int b = 1; b <= 3; b++) for (int c = 1; c <= 3; c++) {
        char tuple[4] = {(char)('0' + a), (char)('0' + b), (char)('0' + c), 0};
        CHECK(target_start_task(tuple) == 0);
        CHECK(s_task == 2u && s_selection == (uint8_t)b && s_target_cls == CLS_TARGET && s_target_label == b - 1);
        CHECK(target_prepare() == 0);
        target_object(wire_request, 2u, target_model(b), 255u, 480u, 0);
        CHECK(s_target35_sample.seen == 1u && s_target35_sample.frame.label == b - 1 && s_target35_good == 1u && stopped());
    }
    CHECK(target_start_task("123") == 0);
    uint16_t request = wire_request;
    unsigned commands = target_wire_calls;
    uint8_t first_packet[9]; memcpy(first_packet, target_wire_packet, sizeof first_packet);
    target_object(request, 1u, 8, 255u, 480u, 0); /* Before target ACK. */
    wire_ack((uint16_t)(request - 1u), 2u, 0u);
    target_object((uint16_t)(request - 1u), 2u, 8, 255u, 480u, 0);
    CHECK(s_target35_phase == TA_STILL && stopped() && !s_target35_sample.seen);
    host_tick += 500u; wire_poll();
    CHECK(target_wire_calls == commands + 1u && wire_request == request &&
          memcmp(first_packet, target_wire_packet, sizeof first_packet) == 0 && host_target_calls == 1);
    wire_ack(request, 2u, 0u);
    {
        uint8_t bare_obj[25] = {0x01u, 99u, 0u, 1u};
        target_le16(bare_obj + 4u, 480u); target_le16(bare_obj + 6u, 320u);
        bare_obj[14] = 8u; target_le16(bare_obj + 15u, 900u);
        target_le16(bare_obj + 17u, 255u); target_le16(bare_obj + 19u, 120u);
        target_le16(bare_obj + 21u, 24u); target_le16(bare_obj + 23u, 24u);
        wire_feed(bare_obj, sizeof bare_obj, 0); /* Matching class/color but not62 request-bound. */
        CHECK(s_target35_phase == TA_PREP_WAIT && stopped() && !s_target35_sample.seen &&
              host_scene_status == 0 && s_stats.obj == 0u);
        CHECK(s_wire.legacy_obj == 1u);
    }
    host_tick += 60000u; wire_poll();
    CHECK(s_target35_phase == TA_SEEK && target_searching() && !s_target35_sample.seen);
    target_object(request, 3u, 8, 255u, 480u, 1);
    CHECK(s_target35_phase == TA_SEEK && target_searching() && !s_target35_sample.seen);
    target_object(request, 4u, 8, 255u, 480u, 0);
    CHECK(s_target35_phase == TA_SEEK && stopped() && s_target35_good == 1u);

    for (unsigned failure = 0u; failure < 2u; failure++) {
        CHECK(target_start_task("123") == 0); request = wire_request;
        wire_ack(request, failure ? 1u : 2u, failure ? 0u : 1u);
        CHECK(s_target35_phase == TA_STOPPED && stopped() && !s_receiving);
        target_object(request, 1u, 8, 255u, 480u, 0);
        CHECK(s_target35_phase == TA_STOPPED && stopped() && !s_target35_sample.seen);
    }
    puts("target35 real63: all27 tuples use second digit and selected target mapping; same-request CRC retry, coordinates require ACK/request, bareQR53/bareOBJECT01/old/pre-ACK/CRC/NACK/wrong-mode gates passed");
    return 0;
}

static int check_target_search_without_coordinates(void)
{
    CHECK(target_start_task("123") == 0);
    uint16_t request = wire_request;
    CHECK(!s_ack && !s_fresh && !s_target35_sample.seen && stopped());
    wire_poll();
    CHECK(s_target35_phase == TA_STILL && stopped() && !s_ack);
    host_tick += T_DIST_STILL_MS; wire_poll();
    CHECK(s_target35_phase == TA_PREP_WAIT && stopped() && !s_ack);
    host_tick += NAV_SETTLE_MS; wire_poll();
    CHECK(s_target35_phase == TA_SEEK && target_searching() &&
          !s_ack && !s_fresh && !s_target35_sample.seen && host_scene_status == 0);
    target_object(request, 1u, 8, 249u, 480u, 0); /* Unacknowledged coordinate cannot reverse search. */
    CHECK(target_searching() && !s_target35_sample.seen && s_stats.obj == 0u);
    wire_ack(request, 2u, 0u); /* ACK alone leaves no coordinate; search is still permitted. */
    CHECK(s_ack && !s_fresh && target_searching() && !s_target35_sample.seen && host_scene_status == 0);
    target_object(request, 2u, 8, 249u, 480u, 0);
    CHECK(last_x == -50.0f && last_y == 0.0f && last_w == 0.0f &&
          s_target35_seen && !s_target35_good && !laser_state);
    target_object(request, 3u, -1, 0u, 480u, 0);
    CHECK(s_target35_phase == TA_SEEK && target_searching() && !s_target35_good);
    for (uint16_t seq = 4u; seq <= 8u; seq++) target_object(request, seq, 8, 255u, 480u, 0);
    CHECK(s_target35_phase == TA_SETTLE && stopped() && s_target35_good == 5u);
    host_tick += 300u; wire_poll();
    CHECK(s_target35_phase == TA_SETTLE && stopped() && s_target35_good == 5u);
    host_tick++; wire_poll(); /* Stale SETTLE must cancel firing and resume positive search. */
    CHECK(s_target35_phase == TA_SEEK && target_searching() && !s_target35_good);
    target_object(request, 9u, 8, 255u, 480u, 0);
    CHECK(s_target35_phase == TA_SEEK && stopped() && s_target35_good == 1u);
    for (uint16_t seq = 10u; seq <= 13u; seq++) target_object(request, seq, 8, 255u, 480u, 0);
    CHECK(s_target35_phase == TA_SETTLE && stopped() && s_target35_good == 5u);
    for (uint16_t seq = 14u; seq <= 17u; seq++) {
        host_tick += 250u; target_object(request, seq, 8, 255u, 480u, 0);
    }
    CHECK(s_target35_phase == TA_FIRE && laser_state && !s_receiving &&
          last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    run_cmd("g"); wire_poll(); CHECK(s_target35_phase == TA_STOPPED && stopped());
    puts("target35 no-coordinate search: legalQR prepares/forward50 without targetACK or first frame; ACK-only stays forward; x249 reverse -> empty forward; stale SETTLE cancels fire/forward and needs5 new frames passed");
    return 0;
}

static int check_target_frame_quality_and_fire(void)
{
    CHECK(target_start_task("123") == 0); CHECK(target_prepare() == 0);
    uint16_t request = wire_request;
    target_object(request, 2u, 8, 249u, 480u, 0);
    CHECK(last_x == -50.0f && last_y == 0.0f && s_target35_good == 0u && !laser_state);
    target_object(request, 3u, 8, 261u, 480u, 0);
    CHECK(target_searching() && s_target35_good == 0u);
    target_object(request, 4u, 8, 250u, 480u, 0);
    CHECK(stopped() && s_target35_good == 1u); /* Lower boundary included. */
    for (unsigned i = 0u; i < 5u; i++) {
        target_object(request, 4u, 8, 260u, 480u, 0); wire_poll();
        CHECK(stopped() && s_target35_good == 1u && s_target35_phase == TA_SEEK);
    }
    CHECK(host_proto_stats.duplicate >= 5u);
    target_object(request, 5u, 8, 260u, 480u, 0);
    CHECK(stopped() && s_target35_good == 2u); /* Upper boundary included. */
    unsigned seen = s_target35_sample.seen;
    target_object(request, 6u, 6, 255u, 480u, 0); /* Wrong target COLOR. */
    CHECK(target_searching() && s_target35_good == 0u && s_target35_sample.seen == seen);
    target_object(request, 7u, 8, 255u, 480u, 0);
    CHECK(stopped() && s_target35_good == 1u);
    target_object(request, 8u, 4, 255u, 480u, 0); /* Right color, wrong CLASS (ball). */
    CHECK(target_searching() && s_target35_good == 0u);
    target_object(request, 9u, 8, 255u, 480u, 0);
    CHECK(stopped() && s_target35_good == 1u);
    target_object(request, 10u, -1, 0u, 480u, 0); /* Valid empty object packet. */
    CHECK(target_searching() && s_target35_good == 0u);
    target_object(request, 11u, 8, 255u, 480u, 0);
    CHECK(stopped() && s_target35_good == 1u);
    host_tick += 300u; wire_poll();
    CHECK(stopped() && s_target35_good == 1u); /* Freshness includes exactly300ms. */
    host_tick++; wire_poll();
    CHECK(target_searching() && s_target35_good == 0u && s_target35_phase == TA_SEEK);
    target_object(request, 12u, 8, 255u, 480u, 0);
    CHECK(stopped() && s_target35_good == 1u);
    target_object(request, 14u, 8, 255u, 480u, 0); /* A missing seq breaks consecutive5. */
    CHECK(stopped() && s_target35_good == 1u);
    for (uint16_t seq = 15u; seq <= 18u; seq++) target_object(request, seq, 8, 255u, 480u, 0);
    CHECK(s_target35_phase == TA_SETTLE && s_target35_good == 5u && stopped());
    uint32_t aimed_at = host_tick;
    for (uint16_t seq = 19u; seq <= 21u; seq++) {
        host_tick += 250u; target_object(request, seq, 8, 255u, 480u, 0);
        CHECK(s_target35_phase == TA_SETTLE && stopped());
    }
    host_tick += 249u; target_object(request, 22u, 8, 255u, 480u, 0);
    CHECK(host_tick - aimed_at == 999u && s_target35_phase == TA_SETTLE && stopped());
    unsigned commands = target_wire_calls;
    host_tick++; target_object(request, 23u, 8, 255u, 480u, 0);
    CHECK(s_target35_phase == TA_FIRE && laser_state && !s_receiving && host_receive_closed);
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !pulse_calls && !servo_calls);
    CHECK(target_wire_calls == commands && target_wire_packet[2] == 0x63u); /* No IDLE/STOP packet. */
    host_tick += 2000u; wire_poll();
    CHECK(s_target35_phase == TA_FIRE && laser_state && !s_receiving && s_round != R_DONE &&
          last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    host_tick += 28000u; wire_poll();
    CHECK(s_target35_phase == TA_FIRE && laser_state && !s_receiving && s_round != R_DONE &&
          last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    CHECK(strstr(host_messages, "status=DONE") == NULL);
    run_cmd("g"); wire_poll();
    CHECK(s_target35_phase == TA_STOPPED && stopped() && !s_receiving && s_round == R_DONE);
    CHECK(strstr(host_messages, "status=STOP") != NULL && strstr(host_messages, "no_auto_return=1") != NULL);
    wire_ack(request, 2u, 0u); target_object(request, 24u, 8, 230u, 480u, 0);
    host_tick += 60000u; wire_poll();
    CHECK(s_target35_phase == TA_STOPPED && stopped() && target_wire_calls == commands);
    puts("target35 real objects: inclusive250..260, x249 backward50/x261 forward50, duplicate/seq-gap/wrong-color/class/empty/301ms stale break5 with forward search; 1s fresh-still then persistent laser/no timed-DONE and terminal local-close passed");
    return 0;
}

static int check_target_settle_interruptions(void)
{
    for (unsigned interruption = 0u; interruption < 4u; interruption++) {
        CHECK(target_start_task("123") == 0); CHECK(target_prepare() == 0);
        uint16_t request = wire_request;
        for (uint16_t seq = 2u; seq <= 6u; seq++) target_object(request, seq, 8, 255u, 480u, 0);
        CHECK(s_target35_phase == TA_SETTLE && stopped());
        host_tick += 250u;
        uint16_t next = 8u;
        if (interruption == 0u) target_object(request, 7u, -1, 0u, 480u, 0);
        else if (interruption == 1u) target_object(request, 7u, 6, 255u, 480u, 0);
        else if (interruption == 2u) { host_tick += 51u; wire_poll(); next = 7u; }
        else { target_object(request, 8u, 8, 255u, 480u, 0); next = 9u; }
        CHECK(s_target35_phase == TA_SEEK && s_target35_good < 5u && !laser_state);
        CHECK(interruption == 3u ? stopped() : target_searching());
        for (unsigned i = 0u; i < 5u; i++) target_object(request, next++, 8, 255u, 480u, 0);
        CHECK(s_target35_phase == TA_SETTLE && stopped());
        uint32_t new_settle = host_tick;
        host_tick += 250u; target_object(request, next++, 8, 255u, 480u, 0);
        CHECK(s_target35_phase == TA_SETTLE && host_tick - new_settle == 250u && stopped());
        run_cmd("g"); wire_poll(); CHECK(s_target35_phase == TA_STOPPED && stopped());
    }
    CHECK(target_start_task("123") == 0); CHECK(target_prepare() == 0);
    uint16_t failed_request = wire_request;
    for (uint16_t seq = 2u; seq <= 6u; seq++) target_object(failed_request, seq, 8, 255u, 480u, 0);
    CHECK(s_target35_phase == TA_SETTLE && stopped());
    unsigned commands = target_wire_calls, seen = s_target35_sample.seen;
    wire_ack(failed_request, 2u, 1u); /* Real matching63 NACK while aim dwell is pending. */
    CHECK(s_target35_phase == TA_STOPPED && stopped() && !s_receiving && host_receive_closed);
    CHECK(strstr(host_messages, "status=TARGET_LINK_ERROR") != NULL && target_wire_calls == commands);
    for (uint16_t seq = 7u; seq <= 12u; seq++) target_object(failed_request, seq, 8, 255u, 480u, 0);
    host_tick += 5000u; wire_poll();
    CHECK(s_target35_phase == TA_STOPPED && stopped() && !s_receiving &&
          s_target35_sample.seen == seen && target_wire_calls == commands);
    CHECK(target_start_task("321") == 0); CHECK(target_prepare() == 0);
    for (unsigned i = 0u; i < 5u; i++) {
        uint16_t seq = (uint16_t)(65533u + i);
        target_object(wire_request, seq, 8, 255u, 480u, 0);
    }
    CHECK(s_target35_phase == TA_SETTLE && s_target35_good == 5u && stopped());
    puts("target35 settle: empty/wrong-color/stale/sequence-gap re-arm full dwell; matching63 NACK stops/closes/no-fire;16bit seq rollover preserves independent5 passed");
    return 0;
}

static int check_target_stop_and_old_batch(void)
{
    static const char *const stop_keys[] = {"g", "a", "0"};
    for (unsigned key = 0u; key < 3u; key++) {
        CHECK(target_start_task("123") == 0); CHECK(target_prepare() == 0);
        uint16_t old = wire_request;
        target_object(old, 2u, 8, 230u, 480u, 0);
        CHECK(last_x == -50.0f);
        unsigned commands = target_wire_calls, seen = s_target35_sample.seen;
        run_cmd(stop_keys[key]); wire_poll();
        CHECK(s_target35_phase == TA_STOPPED && stopped() && !s_receiving && host_receive_closed);
        wire_ack(old, 2u, 0u);
        for (uint16_t seq = 3u; seq <= 12u; seq++) target_object(old, seq, 8, 255u, 480u, 0);
        target_qr((uint16_t)(old - 1u), 20u, "321", 0);
        host_tick += 60000u; wire_poll();
        CHECK(s_target35_phase == TA_STOPPED && stopped() && s_target35_sample.seen == seen && target_wire_calls == commands);
        run_cmd("g"); wire_poll(); /* Explicit new run creates new QR request. */
        uint16_t qr_request = wire_request;
        CHECK(s_target35_phase == TA_QR_WAIT && qr_request > old && wire_mode == 1u && stopped());
        target_qr((uint16_t)(old - 1u), 21u, "321", 0);
        wire_ack(qr_request, 1u, 0u); target_qr(qr_request, 1u, "231", 0);
        CHECK(s_target35_phase == TA_TASK_WAIT && wire_request > qr_request && host_target_digit == 3u && stopped());
        wire_ack(old, 2u, 0u); target_object(old, 22u, 7, 255u, 480u, 0);
        CHECK(s_target35_phase == TA_STILL && stopped() && !s_target35_sample.seen);
        CHECK(target_prepare() == 0);
        target_object(wire_request, 2u, 8, 230u, 480u, 0); /* Previous run's green is now wrong. */
        CHECK(!s_target35_sample.seen && !laser_state);
        target_object(wire_request, 3u, 7, 255u, 480u, 0);
        CHECK(s_target35_good == 1u && stopped());
    }
    puts("target35 parser stop: g/a/0 close current window; batched late ACK/QR/object never resumes; explicit restart isolates new QR,target request and color passed");
    return 0;
}

static int check_target_manual_fire_stop(void)
{
    static const char *const stop_keys[] = {"g", "a", "0"};
    for (unsigned key = 0u; key < 3u; key++) {
        CHECK(target_start_task("123") == 0); CHECK(target_prepare() == 0);
        uint16_t old = wire_request;
        for (uint16_t seq = 2u; seq <= 6u; seq++) target_object(old, seq, 8, 255u, 480u, 0);
        CHECK(s_target35_phase == TA_SETTLE && stopped());
        for (uint16_t seq = 7u; seq <= 10u; seq++) {
            host_tick += 250u; target_object(old, seq, 8, 255u, 480u, 0);
        }
        CHECK(s_target35_phase == TA_FIRE && laser_state && !s_receiving && host_receive_closed);
        unsigned commands = target_wire_calls, seen = s_target35_sample.seen;
        host_tick += 2000u; wire_poll();
        CHECK(s_target35_phase == TA_FIRE && laser_state && s_round != R_DONE &&
              last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        host_tick += 28000u; wire_poll();
        CHECK(s_target35_phase == TA_FIRE && laser_state && s_round != R_DONE &&
              last_x == 0.0f && last_y == 0.0f && last_w == 0.0f &&
              !pulse_calls && !servo_calls && !s_receiving && target_wire_calls == commands);
        CHECK(strstr(host_messages, "status=DONE") == NULL);
        run_cmd(stop_keys[key]); wire_poll();
        CHECK(s_target35_phase == TA_STOPPED && stopped() && !s_receiving && host_receive_closed);
        wire_ack(old, 2u, 0u);
        for (uint16_t seq = 11u; seq <= 20u; seq++) target_object(old, seq, 8, 255u, 480u, 0);
        CHECK(s_target35_phase == TA_STOPPED && stopped() && s_target35_sample.seen == seen &&
              target_wire_calls == commands);
        run_cmd("g"); wire_poll();
        uint16_t qr_request = wire_request;
        CHECK(s_target35_phase == TA_QR_WAIT && stopped() && qr_request > old && wire_mode == 1u &&
              !s_target35_sample.seen && s_target35_qr[0] == 0 && s_target35_qr[1] == 0 && s_target35_qr[2] == 0);
        wire_ack(old, 2u, 0u); target_object(old, 21u, 8, 255u, 480u, 0);
        target_qr((uint16_t)(old - 1u), 22u, "123", 0);
        CHECK(s_target35_phase == TA_QR_WAIT && stopped() && !proto_qr_get(NULL) && !s_target35_sample.seen);
        wire_ack(qr_request, 1u, 0u); target_qr(qr_request, 1u, "231", 0);
        CHECK(s_target35_phase == TA_TASK_WAIT && stopped() && wire_request > qr_request && host_target_digit == 3u);
        wire_ack(old, 2u, 0u); target_object(old, 23u, 7, 255u, 480u, 0);
        CHECK(s_target35_phase == TA_STILL && stopped() && !s_target35_sample.seen);
    }
    puts("target35 persistent fire: 2s/30s stays laserON/braked/no-DONE; g/a/0 turnsOFF; g restarts fresh QR and rejects old QR,target requests passed");
    return 0;
}

int main(void)
{
    CHECK(check_target_qr_gate() == 0);
    CHECK(check_target_selection_and_handshake() == 0);
    CHECK(check_target_search_without_coordinates() == 0);
    CHECK(check_target_frame_quality_and_fire() == 0);
    CHECK(check_target_settle_interruptions() == 0);
    CHECK(check_target_stop_and_old_batch() == 0);
    CHECK(check_target_manual_fire_stop() == 0);
    puts("target35 real parser replay passed; no physical UART, wheel, arm or laser operated");
    return 0;
}
