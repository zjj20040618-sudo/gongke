/* Real route31 command router plus real request/CRC QR parser. Host only:
 * power-on/READY/R1 scan is nonblocking, no wheel or laser starts before g. */
#define main g_router_fixture_main
#include "g_command_stop_test.c"
#undef main

#define proto_stats_get real_proto_stats_get
#define proto_wire_diag_get real_proto_wire_diag_get
#define proto_scene_status real_proto_scene_status
#define proto_send_scene real_proto_send_scene
#define proto_send_target real_proto_send_target
#define proto_qr_begin real_proto_qr_begin
#define proto_qr_cancel real_proto_qr_cancel
#define proto_receive_end real_proto_receive_end
#define proto_qr_get real_proto_qr_get
#include "../App/proto.c"
#undef proto_stats_get
#undef proto_wire_diag_get
#undef proto_scene_status
#undef proto_send_scene
#undef proto_send_target
#undef proto_qr_begin
#undef proto_qr_cancel
#undef proto_receive_end
#undef proto_qr_get

static unsigned wire_commands, notice_calls, wire_bad;
static uint16_t wire_request;
static uint8_t wire_mode, wire_opcode, wire_task, wire_digit;
static int32_t noticed_qr[3];

static int stopped(void)
{
    return last_x == 0.0f && last_y == 0.0f && last_w == 0.0f &&
           !laser_state && !pulse_calls && !servo_calls && !s_go;
}

static void wire_tx(const uint8_t *packet, uint16_t length)
{
    if ((length == 8u && packet[2] == 0x60u) ||
        (length == 9u && packet[2] == 0x63u)) {
        uint16_t crc = binary_crc(packet + 2u, length - 4u);
        if (packet[0] != 0xAAu || packet[1] != 0x55u ||
            packet[length - 2u] != (uint8_t)crc ||
            packet[length - 1u] != (uint8_t)(crc >> 8)) wire_bad++;
        wire_commands++;
        wire_request = (uint16_t)(packet[3] | ((uint16_t)packet[4] << 8));
        wire_opcode = packet[2];
        wire_mode = wire_opcode == 0x63u ? 2u : packet[5];
        wire_task = wire_opcode == 0x63u ? packet[5] : 0u;
        wire_digit = wire_opcode == 0x63u ? packet[6] : 0u;
    } else wire_bad++;
}
static void route_wire_on_frame(const ProtoFrame *frame)
{
    /* Production callbacks observe the current packet's incremented stats.obj. */
    real_proto_stats_get(&host_proto_stats);
    test_vision_feed_frame(frame);
}

static void wire_begin_hook(void)
{
    uint16_t old_request = s_request;
    real_proto_qr_begin();
    if (s_request != old_request) { host_scene = SCENE_QR; host_scene_calls++; }
    host_receive_closed = !s_receiving;
}
static void wire_cancel_hook(void)
{
    real_proto_qr_cancel();
    host_receive_closed = !s_receiving;
}
static void wire_receive_end_hook(void) { real_proto_receive_end(); }
static void wire_sync(void)
{
    proto_service();
    host_scene_status = real_proto_scene_status();
    real_proto_stats_get(&host_proto_stats);
}
static void wire_poll(void)
{
    wire_sync();
    /* Same owner/order as robot_bt_service: notice before route31 may close QR. */
    if (proto_qr_take_notice(noticed_qr)) notice_calls++;
    test_poll();
    wire_sync();
}
static void wire_feed_raw(const uint8_t *payload, unsigned length, int corrupt)
{
    uint16_t crc = binary_crc(payload, length);
    proto_feed_byte(0xAAu); proto_feed_byte(0x55u);
    for (unsigned i = 0u; i < length; i++) proto_feed_byte(payload[i]);
    proto_feed_byte((uint8_t)(crc ^ (corrupt ? 1u : 0u)));
    proto_feed_byte((uint8_t)(crc >> 8));
}
static void wire_feed(const uint8_t *payload, unsigned length, int corrupt)
{ wire_feed_raw(payload, length, corrupt); wire_poll(); }
static void wire_ack(uint16_t request, uint8_t mode, uint8_t result)
{
    uint8_t p[5] = {0x61u, (uint8_t)request, (uint8_t)(request >> 8), mode, result};
    wire_feed(p, sizeof p, 0);
}
static void wire_qr_with_poll(uint16_t request, unsigned sequence, const char *tuple,
                              int corrupt, int legacy, int poll)
{
    unsigned n = (unsigned)strlen(tuple), length = 13u + n;
    uint8_t qr[18] = {0x51u, (uint8_t)sequence, (uint8_t)(sequence >> 8), 1u};
    uint8_t p[23] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8)};
    qr[4] = (uint8_t)n; memcpy(qr + 5u, tuple, n);
    p[3] = (uint8_t)length; memcpy(p + 5u, qr, length);
    wire_feed_raw(legacy ? qr : p, legacy ? length : length + 5u, corrupt);
    if (poll) wire_poll();
}
static void wire_qr(uint16_t request, unsigned sequence, const char *tuple, int corrupt, int legacy)
{ wire_qr_with_poll(request, sequence, tuple, corrupt, legacy, 1); }
static void wire_empty(uint16_t request, unsigned sequence)
{
    uint8_t p[9] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8), 4u, 0u,
                   0x51u, (uint8_t)sequence, (uint8_t)(sequence >> 8), 0u};
    wire_feed(p, sizeof p, 0);
}
static void route_wire_le16(uint8_t *out, unsigned value)
{ out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8); }
static void route_wire_bucket(uint16_t request, unsigned sequence, int model, unsigned cx)
{
    uint8_t inner[25] = {0x01u, (uint8_t)sequence, (uint8_t)(sequence >> 8),
                         (uint8_t)(model >= 0)};
    uint8_t packet[30] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8)};
    unsigned length = model >= 0 ? sizeof inner : 14u;
    route_wire_le16(inner + 4u, 640u); route_wire_le16(inner + 6u, 320u);
    if (model >= 0) {
        inner[14] = (uint8_t)model;
        route_wire_le16(inner + 15u, 900u); route_wire_le16(inner + 17u, cx);
        route_wire_le16(inner + 19u, 120u); route_wire_le16(inner + 21u, 24u);
        route_wire_le16(inner + 23u, 24u);
    }
    route_wire_le16(packet + 3u, length); memcpy(packet + 5u, inner, length);
    wire_feed(packet, length + 5u, 0);
}
static int route_wire_bucket_wait_d(void)
{
    CHECK(s_seq_stage == ROUTE_TEST_ALIGN_STAGE && s_seq_state == SQ_BUCKET_ALIGN && stopped());
    wire_sync();
    uint16_t request = wire_request;
    CHECK(wire_opcode == 0x63u && wire_mode == 2u && wire_task == 4u && wire_digit == 0u && !wire_bad);
    CHECK(host_target_task == PROTO_TASK_BUCKET && host_target_digit == 0u && s_receiving);
    route_wire_bucket(request, 1u, 9, 500u); /* Coordinates alone cannot bypass ACK. */
    wire_ack((uint16_t)(request - 1u), 2u, 0u);
    route_wire_bucket((uint16_t)(request - 1u), 2u, 9, 500u);
    CHECK(!s_bucket36_sample.seen && stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    wire_ack(request, 2u, 0u);
    host_tick += T_DIST_STILL_MS; wire_poll();
    CHECK(s_bucket36_phase == BA_PREP_WAIT && stopped());
    host_tick += NAV_SETTLE_MS; wire_poll();
    CHECK(s_bucket36_phase == BA_SEEK && stopped());
    for (unsigned sequence = 1u; sequence <= 5u; ++sequence) {
        route_wire_bucket(request, sequence, 9, sequence == 1u ? 495u : 505u);
        CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
        CHECK(s_bucket36_good == sequence);
    }
    CHECK(s_bucket36_phase == BA_SETTLE);
    for (unsigned sequence = 6u; sequence <= 9u; ++sequence) {
        host_tick += 250u; route_wire_bucket(request, sequence, 9, 500u);
        CHECK(stopped());
        CHECK(s_seq_state == (sequence == 9u ? SQ_MANUAL_D_WAIT : SQ_BUCKET_ALIGN));
    }
    CHECK(s_seq_stage == ROUTE_TEST_BACK_STAGE && !s_bucket36_back_mm && !s_receiving && !s_due);
    unsigned commands = wire_commands, seen = s_bucket36_sample.seen;
    route_wire_bucket(request, 10u, 9, 450u); host_tick += 1000u; wire_poll();
    CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped() && wire_commands == commands && s_bucket36_sample.seen == seen);
    return 0;
}
static int wire_boot(void)
{
    reset_fixture(); proto_init(); proto_set_binary_mode(1);
    proto_set_binary_tx(wire_tx); proto_set_on_frame(route_wire_on_frame);
    host_scene_hook = real_proto_send_scene;
    host_target_hook = real_proto_send_target;
    host_qr_begin_hook = wire_begin_hook; host_qr_cancel_hook = wire_cancel_hook;
    host_receive_end_hook = wire_receive_end_hook;
    host_qr_get_hook = real_proto_qr_get;
    wire_commands = notice_calls = wire_bad = 0u; wire_request = 0u;
    wire_mode = wire_opcode = wire_task = wire_digit = 0u;
    memset(noticed_qr, 0, sizeof noticed_qr);
    proto_qr_begin(); /* Same nonblocking call as robot_init, after RX/cache init. */
    wire_poll();
    CHECK(wire_commands == 1u && wire_request == 1u && wire_mode == 1u);
    CHECK(s_msel == R_FREE && s_seq_state == SQ_OFF && stopped() && !notice_calls);
    return 0;
}
static int finish_r1_to_qr_wait(void)
{
    int scene_calls = host_scene_calls;
    CHECK(sequence_start_stage() == 0);
    CHECK(s_seq_stage == 0u && s_seq_state == SQ_RUN && s_msel == 17);
    host_lateral = s_dist_odo0 + s_dist_target; wire_poll();
    CHECK(s_seq_state == SQ_RUN && s_round == R_BRAKE);
    host_tick += T_DIST_STILL_MS; wire_poll();
    fixture_complete_distance_alignment(); wire_sync();
    CHECK(s_seq_state == SQ_QR_WAIT && s_seq_stage == 0u && stopped());
    CHECK(host_scene_calls == scene_calls && host_scene == SCENE_QR);
    return 0;
}
static int begin_qr_wait(void)
{
    CHECK(wire_boot() == 0); run_cmd("31"); run_cmd("g");
    return finish_r1_to_qr_wait();
}
static int check_early_and_r1_cache(void)
{
    for (unsigned phase = 0u; phase < 3u; phase++) {
        int32_t qr[3];
        CHECK(wire_boot() == 0);
        uint16_t request = wire_request;
        if (phase >= 1u) run_cmd("31");
        if (phase >= 2u) { run_cmd("g"); CHECK(sequence_start_stage() == 0); }
        wire_ack(request, 1u, 0u); wire_qr(request, 1u, "123", 0, 0);
        CHECK(proto_qr_get(qr) && qr[0] == 1 && qr[1] == 2 && qr[2] == 3);
        CHECK(notice_calls == 1u && !proto_qr_take_notice(qr));
        wire_qr(request, 2u, "321", 0, 0); /* First complete legal choice wins. */
        CHECK(notice_calls == 1u && proto_qr_get(qr) && qr[0] == 1 && qr[2] == 3);
        if (phase == 0u) { CHECK(stopped()); run_cmd("31"); }
        if (phase < 2u) { CHECK(stopped()); run_cmd("g"); CHECK(sequence_start_stage() == 0); }
        CHECK(s_seq_stage == 0u && s_seq_state == SQ_RUN && wire_commands == 1u && wire_request == request);
        host_lateral = s_dist_odo0 + s_dist_target; wire_poll();
        CHECK(s_round == R_BRAKE && s_seq_stage == 0u);
        host_tick += T_DIST_STILL_MS; wire_poll();
        fixture_complete_distance_alignment(); wire_sync();
        CHECK(s_seq_state == SQ_QR_WAIT && s_seq_stage == 0u && stopped());
        wire_poll();
        CHECK(s_seq_state == SQ_STILL && s_seq_stage == 1u && s_msel == 16 && stopped());
        CHECK(wire_commands == 1u && wire_mode == 1u && wire_request == request &&
              host_receive_closed && !s_receiving && !proto_qr_get(NULL));
        CHECK(s_seq_qr[0] == 1 && s_seq_qr[1] == 2 && s_seq_qr[2] == 3 && notice_calls == 1u);
        wire_qr(request, 3u, "321", 0, 0); /* Closed phase keeps draining, not receiving business data. */
        CHECK(s_seq_qr[0] == 1 && s_seq_qr[2] == 3 && notice_calls == 1u && !proto_qr_get(NULL));
        CHECK(wire_commands == 1u && wire_request == request && stopped());
        CHECK(sequence_start_stage() == 0 && last_x < 0.0f);
    }
    puts("route31 startup QR: power-on/READY/running R1 first-tuple cache; one success notice; R2 only after completed R1; same request reused passed");
    return 0;
}
static int check_gate_and_wire_rejections(void)
{
    CHECK(begin_qr_wait() == 0); uint16_t request = wire_request;
    wire_qr(request, 1u, "123", 0, 0); /* Before ACK. */
    wire_ack((uint16_t)(request + 1u), 1u, 0u);
    wire_qr((uint16_t)(request + 1u), 2u, "123", 0, 0);
    CHECK(!proto_qr_get(NULL) && !notice_calls && stopped());
    wire_ack(request, 1u, 0u);
    wire_qr(request, 3u, "123", 0, 1);
    wire_qr(request, 4u, "123", 1, 0);
    CHECK(host_proto_stats.crc_bad == 1u && !proto_qr_get(NULL));
    wire_qr(request, 5u, "12", 0, 0); wire_qr(request, 6u, "120", 0, 0);
    wire_qr(request, 7u, "423", 0, 0); wire_empty(request, 8u);
    host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_QR_WAIT && !proto_qr_get(NULL) && !notice_calls && stopped());
    CHECK(strstr(host_messages, "phase=QR_WAIT") != NULL);
    CHECK(strstr(last_message, "VR type=") != NULL);
    wire_qr(request, 9u, "231", 0, 0);
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == 1u && stopped() && notice_calls == 1u);
    CHECK(noticed_qr[0] == 2 && noticed_qr[1] == 3 && noticed_qr[2] == 1);

    CHECK(begin_qr_wait() == 0); request = wire_request;
    wire_ack(request, 1u, 1u); wire_qr(request, 1u, "123", 0, 0);
    host_tick += 60000u; wire_poll();
    CHECK(host_scene_status == -1 && s_seq_state == SQ_QR_WAIT && !proto_qr_get(NULL) && !notice_calls && stopped());
    for (int a = 1; a <= 3; a++) for (int b = 1; b <= 3; b++) for (int c = 1; c <= 3; c++) {
        char tuple[4] = {(char)('0'+a), (char)('0'+b), (char)('0'+c), 0};
        CHECK(begin_qr_wait() == 0); request = wire_request;
        wire_ack(request, 1u, 0u); wire_qr(request, 1u, tuple, 0, 0);
        CHECK(s_seq_stage == 1u && s_seq_state == SQ_STILL && stopped() && notice_calls == 1u);
        CHECK(s_seq_qr[0] == a && s_seq_qr[1] == b && s_seq_qr[2] == c);
    }
    puts("route31 real QR parser: 27 tuples; pre-ACK/old request/bad CRC/legacy/partial/out-of-range/empty/NACK/no-time-release passed");
    return 0;
}
static int check_cancel_restart_and_mode_change(void)
{
    static const char *const stops[] = {"g", "a", "0"};
    static const char *const writes[] = {"v300", "d600", "ykp1", "fff0", "acc50", "dec50",
        "31", "32", "33", "15", "r1", "co", "cc", "su1600", "n5"};
    for (unsigned key = 0u; key < 3u; key++) for (unsigned phase = 0u; phase < 3u; phase++) {
        CHECK(wire_boot() == 0); uint16_t old = wire_request;
        run_cmd("31"); run_cmd("g");
        if (phase >= 1u) CHECK(sequence_start_stage() == 0);
        if (phase >= 2u) {
            host_lateral = s_dist_odo0 + s_dist_target; wire_poll();
            host_tick += T_DIST_STILL_MS; wire_poll();
            fixture_complete_distance_alignment(); wire_sync();
            CHECK(s_seq_state == SQ_QR_WAIT);
        }
        unsigned commands_before_stop = wire_commands;
        run_cmd(stops[key]); wire_sync();
        CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 0u && stopped() && wire_mode == 1u &&
              wire_request == old && wire_commands == commands_before_stop && !s_receiving && !proto_qr_get(NULL));
        wire_ack(old, 1u, 0u); wire_qr(old, 1u, "123", 0, 0);
        host_tick += 60000u; wire_poll(); run_cmd("g");
        CHECK(s_seq_state == SQ_STOPPED && stopped() && !notice_calls && wire_commands == commands_before_stop);
        run_cmd("31"); run_cmd("g"); wire_sync(); uint16_t next = wire_request;
        CHECK(next != old && wire_mode == 1u && !proto_qr_get(NULL));
        CHECK(finish_r1_to_qr_wait() == 0);
        wire_ack(next, 1u, 0u); wire_qr(old, 2u, "123", 0, 0);
        CHECK(s_seq_state == SQ_QR_WAIT && stopped() && !notice_calls);
        wire_qr(next, 3u, "321", 0, 0);
        CHECK(s_seq_stage == 1u && noticed_qr[0] == 3 && notice_calls == 1u);
    }
    CHECK(wire_boot() == 0); uint16_t old = wire_request;
    wire_ack(old, 1u, 0u); wire_qr(old, 1u, "123", 0, 0);
    run_cmd("31"); run_cmd("15"); wire_sync();
    CHECK(wire_mode == 1u && wire_request == old && wire_commands == 1u &&
          !s_receiving && !proto_qr_get(NULL) && stopped());
    run_cmd("31"); wire_sync(); CHECK(wire_request != old && wire_mode == 1u && !proto_qr_get(NULL));
    wire_ack(wire_request, 1u, 0u); wire_qr(wire_request, 2u, "231", 0, 0);
    CHECK(notice_calls == 2u && noticed_qr[0] == 2);
    /* Manual 0/a before any selection also invalidate a boot tuple. */
    for (unsigned key = 1u; key < 3u; key++) {
        CHECK(wire_boot() == 0); old = wire_request;
        wire_ack(old, 1u, 0u); wire_qr(old, 1u, "123", 0, 0);
        run_cmd(stops[key]); wire_sync();
        CHECK(wire_mode == 1u && wire_request == old && wire_commands == 1u &&
              !s_receiving && !proto_qr_get(NULL) && stopped());
    }
    CHECK(begin_qr_wait() == 0);
    int scene_calls = host_scene_calls; uint32_t test_id = s_active_test;
    for (unsigned i = 0u; i < sizeof writes / sizeof writes[0]; i++) {
        run_cmd(writes[i]); CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
        CHECK(s_seq_state == SQ_QR_WAIT && stopped() && host_scene_calls == scene_calls && s_active_test == test_id);
    }
    run_cmd("param"); run_cmd("diag"); run_cmd("?");
    CHECK(s_seq_state == SQ_QR_WAIT && stopped() && host_scene_calls == scene_calls);
    /* Pending g is flushed before the now-ready QR can release R2. */
    wire_ack(wire_request, 1u, 0u);
    test_feed('g'); host_tick += T_IDLE_MS;
    wire_qr(wire_request, 2u, "123", 0, 0);
    CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 0u && stopped() && !proto_qr_get(NULL));
    puts("route31 QR cancellations: 9 R1/wait g/a/0 cases, boot a/0, mode change/reselection/new notice, late frames, 15 write locks and pending-g priority passed");
    return 0;
}
static int check_notice_transition_race(void)
{
    int32_t qr[3];
    for (unsigned i = 0u; i < 2u; i++) {
        CHECK(wire_boot() == 0); wire_ack(wire_request, 1u, 0u);
        uint16_t request = wire_request;
        CHECK(!proto_qr_take_notice(qr)); /* Task checked immediately before RX. */
        wire_qr_with_poll(wire_request, 1u, "231", 0, 0, 0);
        CHECK(proto_qr_get(qr) && qr[0] == 2 && !notice_calls);
        if (i == 0u) step_vision_receive_end();
        else proto_send_scene(SCENE_EOD);
        CHECK(!proto_qr_get(NULL)); /* Reporting a past success must not authorize R2. */
        if (i == 0u) {
            wire_sync();
            CHECK(wire_commands == 1u && wire_request == request && wire_mode == 1u && !s_receiving);
        }
        CHECK(proto_qr_take_notice(qr) && qr[0] == 2 && qr[1] == 3 && qr[2] == 1);
        CHECK(!proto_qr_take_notice(qr));
        if (i == 0u) {
            proto_qr_begin(); wire_sync();
            uint16_t next = wire_request;
            CHECK(next != request && wire_commands == 2u && wire_mode == 1u &&
                  s_receiving && !host_receive_closed && !proto_qr_get(NULL));
            wire_ack(next, 1u, 0u);
            wire_qr(request, 2u, "123", 0, 0);
            CHECK(!proto_qr_get(NULL) && !notice_calls);
            wire_qr(next, 1u, "312", 0, 0);
            CHECK(proto_qr_get(qr) && qr[0] == 3 && qr[1] == 1 && qr[2] == 2 &&
                  notice_calls == 1u && noticed_qr[0] == 3);
        }
    }
    /* Explicit abort differs from automatic local close; neither old success nor a
     * new QR request may issue an old unconsumed OK. */
    for (unsigned action = 0u; action < 4u; action++) {
        CHECK(wire_boot() == 0); wire_ack(wire_request, 1u, 0u);
        wire_qr_with_poll(wire_request, 1u, "123", 0, 0, 0);
        CHECK(proto_qr_get(NULL) && !notice_calls);
        if (action == 0u) proto_qr_cancel();
        else if (action == 1u) proto_send_scene(SCENE_QR);
        else if (action == 2u) proto_init();
        else {
            uint8_t nack[5] = {0x61u, (uint8_t)wire_request,
                (uint8_t)(wire_request >> 8), 1u, 1u};
            wire_feed_raw(nack, sizeof nack, 0);
        }
        CHECK(!proto_qr_take_notice(qr) && !proto_qr_get(NULL));
    }
    CHECK(wire_boot() == 0); wire_ack(wire_request, 1u, 0u);
    wire_qr_with_poll(wire_request, 1u, "123", 0, 0, 0);
    proto_send_scene(SCENE_EOD); proto_qr_cancel();
    CHECK(!proto_qr_take_notice(qr)); /* Manual cancel also drops notice in OBJECT. */
    puts("QR notice race: RX after notice poll -> local close/OBJECT preserves one OK without IDLE; explicit cancel/new QR/init/NACK clear pending snapshot passed");
    return 0;
}
static int check_complete_recipe_with_bucket(void)
{
    static const int modes[15] = {17,16,20,16,15,16,30,0,16,20,15,20,15,20,15};
    static const int commands[15] = {-530,-650,90,-650,80,-190,-92,0,-730,90,780,90,2450,90,2125};
    static const float speeds[15] = {100,100,100,300,20,100,100,50,100,100,100,100,100,100,100};
    CHECK(ROUTE_TEST_STAGES == 15u && ROUTE_TEST_ALIGN_STAGE == 7u && ROUTE_TEST_BACK_STAGE == 8u);
    CHECK(begin_qr_wait() == 0);
    CHECK(s_dist_target == -530.0f && s_v == 100.0f && !host_target_calls);
    uint16_t qr_request = wire_request;
    wire_ack(qr_request, 1u, 0u); wire_qr(qr_request, 1u, "213", 0, 0);
    CHECK(s_seq_stage == 1u && s_seq_state == SQ_STILL && stopped());
    unsigned qr_commands = wire_commands; /* R1 may have retried the same unACKed QR request. */
    for (unsigned stage = 1u; stage < ROUTE_TEST_STAGES; ++stage) {
        CHECK(s_seq_stage == stage && s_seq_mode == 31);
        CHECK(s_route_test_plan[stage].mode == modes[stage]);
        CHECK(s_route_test_plan[stage].heading_hold == (stage == 3u || stage == 4u ? 0u : 1u));
        if (stage == ROUTE_TEST_ALIGN_STAGE) {
            CHECK(wire_request != qr_request && host_target_calls == 1);
            CHECK(route_wire_bucket_wait_d() == 0);
            continue;
        }
        if (stage == ROUTE_TEST_BACK_STAGE) {
            CHECK(s_seq_state == SQ_MANUAL_D_WAIT && !s_bucket36_back_mm && stopped());
            run_cmd("d730");
            CHECK(s_seq_state == SQ_STILL && s_bucket36_back_mm == 730u && stopped());
        }
        CHECK(sequence_start_stage() == 0 && s_msel == modes[stage]);
        if (dist_mode()) {
            CHECK(s_dist_target == commands[stage] && s_v == speeds[stage] && s_dist_precise);
            CHECK(s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : 0.3f));
            CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f);
            CHECK(s_dist_ff_ratio == (stage == 10u || stage == 12u || stage == 14u ? -0.00625f :
                  stage == 1u || stage == 5u || stage == 8u ? 0.00625f : 0.0f));
        } else CHECK(turn_target_deg() == commands[stage]);
        CHECK(sequence_finish_stage() == 0); wire_sync();
        CHECK(s_seq_state == (stage == 6u ? SQ_BUCKET_ALIGN : stage == 14u ? SQ_DONE : SQ_STILL));
        CHECK(wire_commands == qr_commands + (stage >= 6u ? 1u : 0u) && !wire_bad && notice_calls == 1u);
        CHECK(s_seq_qr[0] == 2 && s_seq_qr[1] == 1 && s_seq_qr[2] == 3);
        CHECK(stopped() && !s_go);
    }
    CHECK(s_msel == 31 && s_round == R_DONE && !s_receiving && !s_due && host_target_calls == 1);
    run_cmd("g"); host_tick += 10000u; wire_poll();
    CHECK(s_seq_state == SQ_DONE && stopped() && wire_commands == qr_commands + 1u);
    puts("route31: legal R1 QR -> exact15 crossing-v300 recipe -> real bucket4/0/five frames/1s -> new d730 -> corridors; independent normal forward/reverse FF, post-yaw-before-next and terminal no-resume passed");
    return 0;
}
static int check_route_bucket_manual_stop(unsigned mode)
{
    static const char *const keys[] = {"g", "a", "0"};
    CHECK(mode == 31u || mode == 34u);
    for (unsigned phase = 0u; phase < 5u; ++phase)
        for (unsigned key = 0u; key < 3u; ++key) {
            CHECK(wire_boot() == 0); run_cmd(mode == 31u ? "31" : "34"); run_cmd("g");
            s_seq_stage = ROUTE_TEST_ALIGN_STAGE; route_seq_prepare(); wire_sync();
            CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
            uint16_t request = wire_request;
            if (phase == 4u) CHECK(route_wire_bucket_wait_d() == 0);
            else {
                wire_ack(request, 2u, 0u);
                if (phase >= 1u) {
                    host_tick += T_DIST_STILL_MS; wire_poll();
                    CHECK(s_bucket36_phase == BA_PREP_WAIT && stopped());
                }
                if (phase >= 2u) {
                    host_tick += NAV_SETTLE_MS; wire_poll();
                    CHECK(s_bucket36_phase == BA_SEEK && stopped());
                }
                if (phase >= 3u) {
                    for (unsigned seq = 1u; seq <= 5u; ++seq) route_wire_bucket(request, seq, 9, 500u);
                    CHECK(s_bucket36_phase == BA_SETTLE && stopped());
                }
            }
            unsigned commands = wire_commands, seen = s_bucket36_sample.seen;
            uint32_t run = s_seq_run;
            run_cmd(keys[key]); wire_sync();
            CHECK(s_seq_state == SQ_STOPPED && s_msel == (int)mode && stopped());
            CHECK(!s_bucket36_back_mm && s_bucket36_phase == BA_OFF && !s_receiving && !s_due);
            wire_ack(request, 2u, 0u); route_wire_bucket(request, 10u, 9, 450u);
            run_cmd("g"); run_cmd("d730"); host_tick += 60000u; wire_poll();
            CHECK(s_seq_state == SQ_STOPPED && stopped() && s_seq_run == run);
            CHECK(s_bucket36_sample.seen == seen && wire_commands == commands && !wire_bad);
        }
    puts("route bucket: 15 g/a/0 STILL/PREP_WAIT/SEEK/SETTLE/WAIT_D cancellations; late ACK/object/d cannot resume, local-close without IDLE passed");
    return 0;
}
#ifndef ROUTE31_QR_FIXTURE_MAIN
#define ROUTE31_QR_FIXTURE_MAIN main
#endif
int ROUTE31_QR_FIXTURE_MAIN(void)
{
    CHECK(check_early_and_r1_cache() == 0);
    CHECK(check_gate_and_wire_rejections() == 0);
    CHECK(check_cancel_restart_and_mode_change() == 0);
    CHECK(check_notice_transition_race() == 0);
    CHECK(check_complete_recipe_with_bucket() == 0);
    CHECK(check_route_bucket_manual_stop(31u) == 0);
    puts("route31 boot QR regression passed; no hardware driven");
    return 0;
}
