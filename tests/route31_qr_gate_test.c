/* Real route31 command router plus real request/CRC QR parser. Host only:
 * power-on/READY/R1 scan is nonblocking, no wheel or laser starts before g. */
#define main g_router_fixture_main
#include "g_command_stop_test.c"
#undef main

#define proto_stats_get real_proto_stats_get
#define proto_wire_diag_get real_proto_wire_diag_get
#define proto_scene_status real_proto_scene_status
#define proto_send_scene real_proto_send_scene
#define proto_qr_begin real_proto_qr_begin
#define proto_qr_cancel real_proto_qr_cancel
#define proto_receive_end real_proto_receive_end
#define proto_qr_get real_proto_qr_get
#include "../App/proto.c"
#undef proto_stats_get
#undef proto_wire_diag_get
#undef proto_scene_status
#undef proto_send_scene
#undef proto_qr_begin
#undef proto_qr_cancel
#undef proto_receive_end
#undef proto_qr_get

static unsigned wire_commands, notice_calls;
static uint16_t wire_request;
static uint8_t wire_mode;
static int32_t noticed_qr[3];

static int stopped(void)
{
    return last_x == 0.0f && last_y == 0.0f && last_w == 0.0f &&
           !laser_state && !pulse_calls && !servo_calls && !s_go;
}

static void wire_tx(const uint8_t *packet, uint16_t length)
{
    if (length == 8u && packet[0] == 0xAAu && packet[1] == 0x55u && packet[2] == 0x60u) {
        wire_commands++;
        wire_request = (uint16_t)(packet[3] | ((uint16_t)packet[4] << 8));
        wire_mode = packet[5];
    }
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
static int wire_boot(void)
{
    reset_fixture(); proto_init(); proto_set_binary_mode(1);
    proto_set_binary_tx(wire_tx); proto_set_on_frame(test_vision_feed_frame);
    host_scene_hook = real_proto_send_scene;
    host_qr_begin_hook = wire_begin_hook; host_qr_cancel_hook = wire_cancel_hook;
    host_receive_end_hook = wire_receive_end_hook;
    host_qr_get_hook = real_proto_qr_get;
    wire_commands = notice_calls = 0u; wire_request = 0u; wire_mode = 0u;
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
#ifndef ROUTE31_QR_FIXTURE_MAIN
#define ROUTE31_QR_FIXTURE_MAIN main
#endif
int ROUTE31_QR_FIXTURE_MAIN(void)
{
    CHECK(check_early_and_r1_cache() == 0);
    CHECK(check_gate_and_wire_rejections() == 0);
    CHECK(check_cancel_restart_and_mode_change() == 0);
    CHECK(check_notice_transition_race() == 0);
    puts("route31 boot QR regression passed; no hardware driven");
    return 0;
}
