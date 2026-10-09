/* Synthetic UART transport tests only: no MCU RNG, motion, GPIO or hardware.
 * Public proto API plus an independent wire encoder/CRC oracle. */
#include "proto.h"
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static uint32_t tick;
static unsigned tx_count, ascii_tx_count, frame_count, qr_count, obj_count;
static uint8_t tx_last[PROTO_BINARY_MAX_LEN];
static unsigned tx_length;
static ProtoFrame last_frame;
static const uint8_t client_a[8] = {1u, 2u, 3u, 4u, 5u, 6u, 7u, 8u};
static const uint8_t client_b[8] = {8u, 7u, 6u, 5u, 4u, 3u, 2u, 1u};
static const uint8_t server_a[8] = {0x10u, 0x20u, 0x30u, 0x40u, 0x50u, 0x60u, 0x70u, 0x80u};
static const uint8_t server_b[8] = {0x80u, 0x70u, 0x60u, 0x50u, 0x40u, 0x30u, 0x20u, 0x10u};
static const uint8_t zero_nonce[8] = {0u};

uint32_t HAL_GetTick(void) { return tick; }

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)value;
    p[1] = (uint8_t)(value >> 8);
}

static uint16_t crc_oracle(const uint8_t *data, unsigned length)
{
    uint16_t crc = 0xffffu;
    for (unsigned i = 0u; i < length; ++i) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (unsigned b = 0u; b < 8u; ++b) {
            const unsigned high = (unsigned)crc & 0x8000u;
            crc = (uint16_t)((uint32_t)crc << 1);
            if (high) crc ^= 0x1021u;
        }
    }
    return crc;
}

static unsigned encode_wire(uint8_t *out, const uint8_t *body, unsigned n)
{
    assert(n + 4u <= PROTO_BINARY_MAX_LEN);
    out[0] = 0xaau; out[1] = 0x55u;
    memcpy(out + 2u, body, n);
    put16(out + n + 2u, crc_oracle(body, n));
    return n + 4u;
}

static void tx_cb(const uint8_t *data, uint16_t length)
{
    assert(length >= 4u && length <= sizeof tx_last);
    assert(data[0] == 0xaau && data[1] == 0x55u);
    assert(crc_oracle(data + 2u, length - 4u) == le16(data + length - 2u));
    ++tx_count; tx_length = length;
    memcpy(tx_last, data, length);
}

static void ascii_tx_cb(const char *s) { assert(s); ++ascii_tx_count; }

static void frame_cb(const ProtoFrame *frame)
{
    assert(frame); ++frame_count;
    if (frame->type == PF_QR) ++qr_count;
    if (frame->type == PF_OBJ) ++obj_count;
    last_frame = *frame;
}

static void reset_at(uint32_t now, int binary)
{
    tick = now;
    tx_count = ascii_tx_count = frame_count = qr_count = obj_count = 0u;
    tx_length = 0u; memset(tx_last, 0, sizeof tx_last);
    memset(&last_frame, 0, sizeof last_frame);
    proto_init();
    proto_set_binary_mode(binary);
    proto_set_binary_tx(tx_cb);
    proto_set_tx(ascii_tx_cb);
    proto_set_on_frame(frame_cb);
}

static ProtoSessionDiag session(void)
{
    ProtoSessionDiag out;
    memset(&out, 0xa5, sizeof out);
    proto_session_diag_get(&out);
    return out;
}

static ProtoWireDiag wire(void)
{
    ProtoWireDiag out;
    memset(&out, 0, sizeof out);
    proto_wire_diag_get(&out);
    return out;
}

static ProtoStats stats(void)
{
    ProtoStats out;
    memset(&out, 0, sizeof out);
    proto_stats_get(&out);
    return out;
}

static void feed_bytes(const uint8_t *data, unsigned n)
{
    for (unsigned i = 0u; i < n; ++i) proto_feed_byte(data[i]);
}

static void feed_body(const uint8_t *body, unsigned n)
{
    uint8_t packet[PROTO_BINARY_MAX_LEN];
    feed_bytes(packet, encode_wire(packet, body, n));
}

static void feed_ascii(const char *s)
{
    while (*s) proto_feed_byte((uint8_t)*s++);
}

static void clear_incomplete(void)
{
    tick += PROTO_BINARY_GAP_MS + 1u;
    proto_feed_byte(0u);
}

static unsigned boot_body(uint8_t *out, uint8_t type, const uint8_t *client,
                          const uint8_t *server, uint8_t status)
{
    out[0] = type; out[1] = 1u;
    memcpy(out + 2u, client, 8u);
    if (type == 0x64u) return 10u;
    memcpy(out + 10u, server, 8u);
    if (type != 0x68u) return 18u;
    out[18] = status;
    return 19u;
}

static void feed_boot(uint8_t type, const uint8_t *client, const uint8_t *server,
                      uint8_t status)
{
    uint8_t body[19];
    unsigned n = boot_body(body, type, client, server, status);
    feed_body(body, n);
}

static unsigned envelope(uint8_t *out, const uint8_t *client, const uint8_t *server,
                         const uint8_t *inner, unsigned n)
{
    assert(n + 23u <= PROTO_BINARY_MAX_LEN);
    out[0] = 0x66u;
    memcpy(out + 1u, client, 8u); memcpy(out + 9u, server, 8u);
    put16(out + 17u, (uint16_t)n);
    memcpy(out + 19u, inner, n);
    return n + 19u;
}

static void feed_business(const uint8_t *client, const uint8_t *server,
                          const uint8_t *body, unsigned n)
{
    uint8_t wrapped[PROTO_BINARY_MAX_LEN];
    feed_body(wrapped, envelope(wrapped, client, server, body, n));
}

static void expected_tx(const uint8_t *body, unsigned n)
{
    uint8_t expected[PROTO_BINARY_MAX_LEN];
    unsigned length = encode_wire(expected, body, n);
    assert(tx_length == length && memcmp(tx_last, expected, length) == 0);
}

static void expected_boot_tx(uint8_t type, const uint8_t *client, const uint8_t *server)
{
    uint8_t body[19];
    unsigned n = boot_body(body, type, client, server, 0u);
    expected_tx(body, n);
}

static void expected_business_tx(const uint8_t *client, const uint8_t *server,
                                 uint16_t req, uint8_t mode,
                                 uint8_t task, uint8_t selection)
{
    uint8_t inner[5] = {0x60u, 0u, 0u, mode, 0u};
    uint8_t body[PROTO_BINARY_MAX_LEN];
    unsigned n = 4u;
    put16(inner + 1u, req);
    if (task) { inner[0] = 0x63u; inner[3] = task; inner[4] = selection; n = 5u; }
    expected_tx(body, envelope(body, client, server, inner, n));
}

static void begin_ready(const uint8_t *client, const uint8_t *server)
{
    assert(proto_session_begin(client) == 1);
    assert(session().enabled && session().state == PROTO_SESSION_HELLO);
    proto_service(); expected_boot_tx(0x64u, client, server);
    unsigned sent = tx_count;
    feed_boot(0x65u, client, server, 0u);
    assert(tx_count == sent && session().state == PROTO_SESSION_CONFIRM);
    proto_service(); expected_boot_tx(0x67u, client, server);
    sent = tx_count;
    feed_boot(0x68u, client, server, 0u);
    assert(tx_count == sent && session().state == PROTO_SESSION_READY);
    assert(memcmp(session().client, client, 8u) == 0);
    assert(memcmp(session().server, server, 8u) == 0);
}

static unsigned ack_body(uint8_t *out, uint16_t req, uint8_t mode, uint8_t status)
{
    out[0] = 0x61u; put16(out + 1u, req); out[3] = mode; out[4] = status;
    return 5u;
}

static void feed_ack(const uint8_t *client, const uint8_t *server,
                     uint16_t req, uint8_t mode, uint8_t status)
{
    uint8_t body[5];
    feed_business(client, server, body, ack_body(body, req, mode, status));
}

static unsigned result_body(uint8_t *out, uint16_t req, const uint8_t *inner, unsigned n)
{
    out[0] = 0x62u; put16(out + 1u, req); put16(out + 3u, (uint16_t)n);
    memcpy(out + 5u, inner, n);
    return n + 5u;
}

static void feed_result(const uint8_t *client, const uint8_t *server,
                        uint16_t req, const uint8_t *inner, unsigned n)
{
    uint8_t body[PROTO_BINARY_MAX_LEN];
    feed_business(client, server, body, result_body(body, req, inner, n));
}

static unsigned qr_body(uint8_t *out, uint16_t sequence, const char *digits)
{
    out[0] = 0x53u; put16(out + 1u, sequence); out[3] = digits ? 1u : 0u;
    if (!digits) return 4u;
    memcpy(out + 4u, digits, 3u);
    return 7u;
}

static void feed_qr(const uint8_t *client, const uint8_t *server,
                    uint16_t req, uint16_t sequence, const char *digits)
{
    uint8_t body[7];
    feed_result(client, server, req, body, qr_body(body, sequence, digits));
}

static unsigned object_body(uint8_t *out, uint16_t sequence, uint8_t model,
                            uint16_t cx, uint16_t cy, unsigned count)
{
    assert(count <= PROTO_BINARY_MAX_OBJECTS);
    memset(out, 0, 14u + 11u * count);
    out[0] = 0x01u; put16(out + 1u, sequence); out[3] = (uint8_t)count;
    put16(out + 4u, 640u); put16(out + 6u, 480u);
    for (unsigned i = 0u; i < count; ++i) {
        uint8_t *item = out + 14u + 11u * i;
        item[0] = model; put16(item + 1u, (uint16_t)(950u - i));
        put16(item + 3u, cx); put16(item + 5u, cy);
        put16(item + 7u, 30u); put16(item + 9u, 40u);
    }
    return 14u + 11u * count;
}

static void feed_object(const uint8_t *client, const uint8_t *server,
                        uint16_t req, uint16_t sequence, uint8_t model)
{
    uint8_t body[25];
    feed_result(client, server, req, body, object_body(body, sequence, model, 135u, 390u, 1u));
}

static void feed_rank(const uint8_t *client, const uint8_t *server,
                      uint16_t req, uint16_t sequence)
{
    uint8_t body[9] = {0x54u, 0u, 0u, 4u, 2u, 2u, 3u, 4u, 0xffu};
    put16(body + 1u, sequence);
    feed_result(client, server, req, body, sizeof body);
}

static void assert_no_authority(void)
{
    ProtoTargetRank rank;
    assert(proto_qr_get(NULL) == 0);
    assert(proto_qr_take_notice(NULL) == 0);
    assert(proto_target_rank_get(&rank) == 0);
    assert(!wire().ack && !wire().fresh);
}

static void test_legacy_is_opt_in(void)
{
    uint8_t body[7];
    reset_at(100u, 1);
    assert(!session().enabled && session().state == PROTO_SESSION_NONE);
    feed_body(body, qr_body(body, 1u, "123"));
    assert(qr_count == 1u); /* Unarmed standalone legacy replay still works. */
    assert(proto_qr_get(NULL) == 0);
    reset_at(100u, 0);
    feed_ascii("QR,1,2,3\n");
    assert(qr_count == 1u && !session().enabled);
    reset_at(100u, 1);
    proto_qr_begin(); proto_service();
    assert(tx_length == 8u && tx_last[2] == 0x60u && wire().request == 1u);
    assert(proto_session_begin(client_a) == 1);
    assert_no_authority();
    assert(session().state == PROTO_SESSION_HELLO && session().enabled);
    puts("boot session: explicit enable preserves default standalone legacy replay");
}

static void test_hello_confirm_retry_and_challenge_rotation(void)
{
    uint8_t saved[PROTO_BINARY_MAX_LEN];
    reset_at(1000u, 1);
    assert(proto_session_begin(client_a) == 1);
    proto_qr_begin();
    assert(wire().request == 1u && !wire().ack);
    feed_boot(0x68u, client_a, server_a, 0u); /* READY before challenge. */
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 1u, "331");
    assert(session().state == PROTO_SESSION_HELLO && frame_count == 0u);
    assert_no_authority();
    proto_service(); assert(tx_count == 1u);
    expected_boot_tx(0x64u, client_a, server_a);
    unsigned n = tx_length; memcpy(saved, tx_last, n);
    feed_bytes(saved, n); /* Loopback HELLO is never a challenge or business ACK. */
    assert(session().state == PROTO_SESSION_HELLO);
    tick += 499u; proto_service(); assert(tx_count == 1u);
    ++tick; proto_service(); assert(tx_count == 2u);
    assert(tx_length == n && memcmp(saved, tx_last, n) == 0);
    feed_boot(0x65u, client_b, server_a, 0u);
    feed_boot(0x65u, client_a, zero_nonce, 0u);
    assert(session().state == PROTO_SESSION_HELLO);
    uint8_t body[19]; unsigned length = boot_body(body, 0x65u, client_a, server_a, 0u);
    body[1] = 2u; feed_body(body, length);
    assert(session().state == PROTO_SESSION_HELLO);
    feed_boot(0x65u, client_a, server_a, 0u);
    assert(session().state == PROTO_SESSION_CONFIRM && tx_count == 2u);
    proto_service(); assert(tx_count == 3u); expected_boot_tx(0x67u, client_a, server_a);
    n = tx_length; memcpy(saved, tx_last, n);
    feed_bytes(saved, n); /* Loopback CONFIRM is not READY. */
    assert(session().state == PROTO_SESSION_CONFIRM);
    tick += 499u; proto_service(); assert(tx_count == 3u);
    ++tick; proto_service(); assert(tx_count == 4u && memcmp(saved, tx_last, n) == 0);
    feed_boot(0x65u, client_a, server_a, 0u);
    assert(tx_count == 4u); proto_service(); assert(tx_count == 5u);
    expected_boot_tx(0x67u, client_a, server_a);
    feed_boot(0x65u, client_a, server_b, 0u);
    assert(session().state == PROTO_SESSION_CONFIRM && memcmp(session().server, server_b, 8u) == 0);
    proto_service(); assert(tx_count == 6u); expected_boot_tx(0x67u, client_a, server_b);
    feed_boot(0x68u, client_a, server_a, 0u);
    feed_boot(0x68u, client_b, server_b, 0u);
    assert(session().state == PROTO_SESSION_CONFIRM && frame_count == 0u);
    feed_boot(0x68u, client_a, server_b, 0u);
    assert(session().state == PROTO_SESSION_READY && tx_count == 6u);
    proto_service(); assert(tx_count == 7u);
    expected_business_tx(client_a, server_b, 1u, 1u, 0u, 0u);
    feed_boot(0x65u, client_a, server_a, 0u); /* Late challenge must not reset READY. */
    feed_boot(0x68u, client_a, server_b, 1u); /* Late NACK must not reset READY. */
    assert(session().state == PROTO_SESSION_READY && memcmp(session().server, server_b, 8u) == 0);
    feed_ack(client_a, server_b, 1u, 1u, 0u);
    feed_qr(client_a, server_b, 1u, 1u, "331");
    int32_t qr[3] = {0};
    assert(proto_qr_get(qr) == 1 && qr[0] == 3 && qr[1] == 3 && qr[2] == 1);
    assert(qr_count == 1u && proto_scene_status() == 1);
    assert(session().tx >= 6u);
    puts("boot session: HELLO/CONFIRM 500ms retries, rotated challenge, queued QR and late packets");
}

static void test_reboot_old_nonce_request_one(void)
{
    reset_at(0u, 1); begin_ready(client_a, server_a);
    proto_qr_begin(); proto_service();
    assert(wire().request == 1u);
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 99u, "331");
    assert(proto_qr_get(NULL) && qr_count == 1u);
    reset_at(0u, 1); /* Same request1 after an MCU reboot, but a fresh client. */
    assert(proto_session_begin(client_b)); proto_qr_begin();
    assert(wire().request == 1u);
    feed_boot(0x68u, client_a, server_a, 0u);
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 99u, "331");
    feed_object(client_a, server_a, 1u, 99u, 4u);
    feed_rank(client_a, server_a, 1u, 99u);
    assert_no_authority(); assert(frame_count == 0u);
    proto_service(); expected_boot_tx(0x64u, client_b, server_b);
    feed_boot(0x65u, client_b, server_b, 0u); proto_service();
    expected_boot_tx(0x67u, client_b, server_b);
    feed_boot(0x68u, client_a, server_a, 0u);
    assert(session().state == PROTO_SESSION_CONFIRM);
    feed_boot(0x68u, client_b, server_b, 0u); proto_service();
    expected_business_tx(client_b, server_b, 1u, 1u, 0u, 0u);
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 99u, "331");
    assert_no_authority(); assert(frame_count == 0u);
    feed_ack(client_b, server_b, 1u, 1u, 0u);
    feed_qr(client_b, server_b, 1u, 99u, "123");
    assert(proto_qr_get(NULL) && qr_count == 1u);

    reset_at(0u, 1); begin_ready(client_b, server_b);
    assert(proto_send_target(PROTO_TASK_BALL, 1u)); proto_service();
    assert(wire().request == 1u);
    feed_ack(client_a, server_a, 1u, 2u, 0u);
    feed_object(client_a, server_a, 1u, 1u, 4u);
    feed_rank(client_a, server_a, 1u, 1u);
    assert_no_authority(); assert(frame_count == 0u);
    feed_ack(client_b, server_b, 1u, 2u, 0u);
    feed_rank(client_a, server_a, 1u, 1u);
    feed_object(client_a, server_a, 1u, 1u, 4u);
    ProtoTargetRank rank;
    assert(!proto_target_rank_get(&rank) && frame_count == 0u && !wire().fresh);
    feed_rank(client_b, server_b, 1u, 1u);
    assert(proto_target_rank_get(&rank) && rank.rank == 2u && rank.request == 1u);
    assert(frame_count == 0u && !wire().fresh);
    feed_object(client_b, server_b, 1u, 1u, 4u);
    assert(obj_count == 1u && proto_scene_status() == 1);
    puts("boot session: reboot/request1 rejects old READY, ACK, QR, OBJ and rank54 before/after READY");
}

static void test_qr_request_phase_and_bare_rejection(void)
{
    uint8_t body[25];
    reset_at(40u, 1); begin_ready(client_a, server_a);
    proto_qr_begin(); proto_service();
    expected_business_tx(client_a, server_a, 1u, 1u, 0u, 0u);
    unsigned sent = tx_count;
    feed_bytes(tx_last, tx_length); /* Correct session business request is still only an echo. */
    assert(!wire().ack && wire().command_echo == 1u && tx_count == sent);
    feed_qr(client_a, server_a, 1u, 1u, "123");
    assert_no_authority(); assert(wire().result_preack == 1u && qr_count == 0u);
    feed_body(body, ack_body(body, 1u, 1u, 0u));
    feed_body(body, qr_body(body, 1u, "123"));
    feed_body(body, object_body(body, 1u, 4u, 135u, 390u, 1u));
    assert_no_authority(); assert(frame_count == 0u && session().bare >= 3u);
    feed_ack(client_a, server_a, 2u, 1u, 0u);
    feed_ack(client_b, server_a, 1u, 1u, 0u);
    feed_ack(client_a, server_b, 1u, 1u, 0u);
    assert_no_authority();
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    assert(wire().ack && !wire().fresh && !proto_qr_get(NULL));
    feed_qr(client_a, server_a, 2u, 1u, "123");
    assert(!wire().fresh && !proto_qr_get(NULL));
    feed_qr(client_a, server_a, 1u, 1u, NULL);
    assert(wire().fresh && !proto_qr_get(NULL) && qr_count == 0u);
    feed_qr(client_a, server_a, 1u, 2u, "403");
    assert(!proto_qr_get(NULL) && qr_count == 0u);
    feed_qr(client_a, server_a, 1u, 3u, "331");
    assert(proto_qr_get(NULL) && qr_count == 1u);
    feed_qr(client_a, server_a, 1u, 3u, "331");
    assert(qr_count == 1u && stats().duplicate >= 1u);
    int32_t qr[3] = {0};
    assert(proto_qr_take_notice(qr) && qr[0] == 3 && qr[1] == 3 && qr[2] == 1);
    assert(!proto_qr_take_notice(NULL));
    proto_receive_end();
    assert_no_authority();
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 4u, "123");
    assert_no_authority(); assert(qr_count == 1u && wire().outside_phase >= 2u);
    proto_qr_begin(); proto_service();
    assert(wire().request == 2u);
    expected_business_tx(client_a, server_a, 2u, 1u, 0u, 0u);
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 3u, "331");
    assert_no_authority();
    feed_ack(client_a, server_a, 2u, 1u, 0u);
    feed_qr(client_a, server_a, 2u, 3u, "123");
    assert(proto_qr_get(qr) && qr[0] == 1 && qr[1] == 2 && qr[2] == 3 && qr_count == 2u);
    puts("boot session: nonce + request + ACK + active phase + legal QR remain separate gates");
}

static void test_object_rank_and_selected_task(void)
{
    reset_at(40u, 1); begin_ready(client_a, server_a);
    assert(proto_send_target(PROTO_TASK_BALL, 1u)); proto_service();
    expected_business_tx(client_a, server_a, 1u, 2u, 1u, 1u);
    feed_rank(client_a, server_a, 1u, 1u);
    feed_object(client_a, server_a, 1u, 1u, 4u);
    assert_no_authority(); assert(frame_count == 0u);
    feed_ack(client_a, server_a, 1u, 2u, 0u);
    feed_rank(client_a, server_a, 1u, 1u);
    ProtoTargetRank rank;
    assert(proto_target_rank_get(&rank) && rank.rank == 2u && rank.target_model == 4u);
    assert(rank.slots[0] == 3u && rank.slots[1] == 4u && rank.slots[2] == 0xffu);
    assert(proto_scene_status() == 0 && !wire().fresh && frame_count == 0u);
    feed_rank(client_a, server_a, 1u, 1u);
    assert(wire().rank_duplicate == 1u && frame_count == 0u && !wire().fresh);
    feed_object(client_a, server_a, 1u, 1u, 5u); /* Wrong selected color cannot reach consumers. */
    assert(obj_count == 0u);
    feed_object(client_a, server_a, 1u, 2u, 4u);
    assert(obj_count == 1u && last_frame.cls == CLS_BALL && last_frame.label == LAB_R);
    assert(last_frame.cx == 135 && last_frame.cy == 390 && last_frame.sequence == 2u);
    feed_object(client_a, server_a, 1u, 2u, 4u);
    assert(obj_count == 1u);
    uint8_t objects[124];
    unsigned n = object_body(objects, 3u, 4u, 135u, 390u, 10u);
    feed_result(client_a, server_a, 1u, objects, n); /* Largest 10-object envelope. */
    assert(obj_count == 2u); /* Existing best-confidence selection emits only one. */
    n = object_body(objects, 4u, 4u, 640u, 390u, 1u);
    feed_result(client_a, server_a, 1u, objects, n);
    assert(obj_count == 2u);
    assert(proto_send_target(PROTO_TASK_BUCKET, 0u)); proto_service();
    assert(wire().request == 2u && !proto_target_rank_get(&rank));
    feed_ack(client_a, server_a, 1u, 2u, 0u);
    feed_object(client_a, server_a, 1u, 4u, 4u);
    feed_rank(client_a, server_a, 1u, 2u);
    assert_no_authority(); assert(obj_count == 2u);
    feed_ack(client_a, server_a, 2u, 2u, 0u);
    feed_rank(client_a, server_a, 2u, 2u); /* Bucket cannot inherit ball order. */
    assert(!proto_target_rank_get(&rank));
    feed_object(client_a, server_a, 2u, 1u, 9u);
    assert(obj_count == 3u && last_frame.cls == CLS_BUCKET && last_frame.label == 0);
    proto_receive_end();
    feed_object(client_a, server_a, 2u, 2u, 9u);
    assert(obj_count == 3u && !wire().ack && !wire().fresh);
    puts("boot session: selected objects, independent historical rank54, duplicate and phase gates preserved");
}

static void test_malformed_split_and_resync(void)
{
    uint8_t body[PROTO_BINARY_MAX_LEN], packet[PROTO_BINARY_MAX_LEN], inner[PROTO_BINARY_MAX_LEN];
    reset_at(20u, 1);
    assert(proto_session_begin(client_a)); proto_qr_begin(); proto_service();
    unsigned n = boot_body(body, 0x65u, client_a, server_a, 0u);
    unsigned length = encode_wire(packet, body, n);
    feed_bytes(packet, 5u); assert(session().state == PROTO_SESSION_HELLO);
    tick += 50u; feed_bytes(packet + 5u, length - 5u);
    assert(session().state == PROTO_SESSION_CONFIRM);
    proto_service();
    n = boot_body(body, 0x68u, client_a, server_a, 0u);
    length = encode_wire(packet, body, n); packet[length - 1u] ^= 1u;
    feed_bytes(packet, length); assert(session().state == PROTO_SESSION_CONFIRM);
    assert(stats().crc_bad >= 1u);
    clear_incomplete();
    body[1] = 2u; feed_body(body, n); assert(session().state == PROTO_SESSION_CONFIRM);
    body[1] = 1u; body[18] = 2u; feed_body(body, n);
    assert(session().state == PROTO_SESSION_CONFIRM);
    body[18] = 0u; length = encode_wire(packet, body, n);
    feed_bytes(packet, 8u); tick += PROTO_BINARY_GAP_MS + 1u;
    feed_bytes(packet + 8u, length - 8u);
    assert(session().state == PROTO_SESSION_CONFIRM && stats().binary_gap >= 1u);
    clear_incomplete(); feed_body(body, n);
    assert(session().state == PROTO_SESSION_READY); proto_service();

    n = ack_body(inner, 1u, 1u, 0u);
    unsigned wrapped = envelope(body, client_a, server_a, inner, n);
    body[17] = 4u; body[18] = 0u; /* Wrong declared length with an otherwise valid outer CRC. */
    feed_body(body, wrapped); assert(!wire().ack);
    clear_incomplete();
    wrapped = envelope(body, client_a, server_a, inner, n);
    put16(body + 17u, 0xffffu); feed_body(body, wrapped); assert(!wire().ack);
    clear_incomplete();
    inner[5] = 0u; /* 61 has exactly 5 bytes, not 6. */
    feed_business(client_a, server_a, inner, 6u); assert(!wire().ack);
    clear_incomplete();
    n = boot_body(inner, 0x68u, client_a, server_a, 0u);
    feed_business(client_a, server_a, inner, n); /* No boot messages inside BUSINESS. */
    assert(!wire().ack && session().state == PROTO_SESSION_READY);
    clear_incomplete();
    n = ack_body(inner, 1u, 1u, 0u);
    wrapped = envelope(body, client_a, server_a, inner, n);
    feed_business(client_a, server_a, body, wrapped); /* No nested BUSINESS. */
    assert(!wire().ack); clear_incomplete();
    n = qr_body(inner, 1u, "123");
    wrapped = envelope(body, client_a, server_a, inner, n);
    uint8_t result[PROTO_BINARY_MAX_LEN];
    n = result_body(result, 1u, body, wrapped);
    feed_business(client_a, server_a, result, n); /* No nested 66 inside request62. */
    assert(!wire().ack && frame_count == 0u); clear_incomplete();
    inner[0] = 0x61u; inner[1] = 1u; inner[2] = 0u;
    feed_business(client_a, server_a, inner, 3u); assert(!wire().ack);
    clear_incomplete();
    n = ack_body(inner, 1u, 1u, 0u);
    wrapped = envelope(body, client_a, server_a, inner, n);
    length = encode_wire(packet, body, wrapped);
    feed_bytes(packet, 2u); tick += 10u; feed_bytes(packet + 2u, 9u);
    assert(!wire().ack);
    tick += 10u; feed_bytes(packet + 11u, length - 11u); assert(wire().ack);
    n = qr_body(inner, 1u, "123"); n = result_body(result, 1u, inner, n);
    wrapped = envelope(body, client_a, server_a, result, n);
    length = encode_wire(packet, body, wrapped);
    packet[length - 2u] ^= 0x10u; feed_bytes(packet, length);
    assert(!proto_qr_get(NULL) && qr_count == 0u);
    /* Corrupt frame followed by a complete valid frame recovers at AA55. */
    feed_body(body, wrapped); assert(proto_qr_get(NULL) && qr_count == 1u);
    assert(session().bad > 0u || wire().invalid_payload > 0u);
    puts("boot session: split/gapped frames, CRC, bad length/type, nesting and stream resynchronization");
}

static void test_fail_closed_and_explicit_init(void)
{
    reset_at(0u, 1); begin_ready(client_a, server_a);
    proto_qr_begin(); proto_service();
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 1u, "123");
    assert(proto_qr_get(NULL));
    proto_session_fail();
    assert(session().enabled && session().state == PROTO_SESSION_FAILED);
    assert_no_authority();
    unsigned sent = tx_count, callbacks = frame_count;
    proto_qr_begin(); tick += 10000u; proto_service();
    feed_boot(0x65u, client_a, server_a, 0u);
    feed_boot(0x68u, client_a, server_a, 0u);
    feed_ack(client_a, server_a, wire().request, 1u, 0u);
    feed_qr(client_a, server_a, wire().request, 2u, "331");
    uint8_t body[7]; feed_body(body, qr_body(body, 3u, "331"));
    assert(tx_count == sent && frame_count == callbacks && session().state == PROTO_SESSION_FAILED);
    proto_set_binary_mode(0);
    feed_ascii("QR,3,3,1\nOBJ,0,0,135,390,30,40,95\n");
    proto_send_scene(SCENE_QR); proto_send_ping(); proto_service();
    assert(session().enabled && session().state == PROTO_SESSION_FAILED);
    assert(frame_count == callbacks && tx_count == sent && ascii_tx_count == 0u);
    proto_init(); proto_set_on_frame(frame_cb); proto_set_tx(ascii_tx_cb);
    assert(!session().enabled && session().state == PROTO_SESSION_NONE);
    feed_ascii("QR,1,2,3\n"); assert(frame_count == callbacks + 1u);

    reset_at(0u, 1);
    assert(!proto_session_begin(NULL));
    assert(session().enabled && session().state == PROTO_SESSION_FAILED);
    assert_no_authority();
    reset_at(0u, 1);
    assert(!proto_session_begin(zero_nonce));
    assert(session().enabled && session().state == PROTO_SESSION_FAILED);
    proto_qr_begin(); proto_service(); assert(tx_count == 0u);
    reset_at(0u, 0);
    assert(!proto_session_begin(client_a));
    assert(session().enabled && session().state == PROTO_SESSION_FAILED);
    feed_ascii("QR,1,2,3\n"); assert(frame_count == 0u);

    reset_at(0u, 1);
    assert(proto_session_begin(client_a)); proto_qr_begin(); proto_service();
    feed_boot(0x65u, client_a, server_a, 0u); proto_service();
    feed_boot(0x68u, client_a, server_a, 1u);
    assert(session().state == PROTO_SESSION_FAILED); sent = tx_count;
    feed_boot(0x68u, client_a, server_a, 0u);
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 1u, "123");
    tick += 10000u; proto_service(); assert_no_authority();
    assert(tx_count == sent && frame_count == 0u && session().state == PROTO_SESSION_FAILED);

    reset_at(0u, 1); begin_ready(client_a, server_a);
    proto_qr_begin(); proto_service();
    feed_ack(client_a, server_a, 1u, 1u, 1u);
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 1u, "123");
    assert(proto_scene_status() == -1 && !proto_qr_get(NULL) && qr_count == 0u);
    puts("boot session: RNG/nonce/mode/NACK failure closes authority; only explicit init restores legacy");
}

static void test_strict_inner_validation_after_ack(void)
{
    uint8_t qr[7], result[PROTO_BINARY_MAX_LEN], body[PROTO_BINARY_MAX_LEN];
    uint8_t wrapped[PROTO_BINARY_MAX_LEN], raw_wire[PROTO_BINARY_MAX_LEN];
    reset_at(10u, 1); begin_ready(client_a, server_a);
    proto_qr_begin(); proto_service(); feed_ack(client_a, server_a, 1u, 1u, 0u);
    assert(wire().ack && !wire().fresh);
    unsigned qn = qr_body(qr, 7u, "123");
    unsigned rn = result_body(result, 1u, qr, qn);
    unsigned wn = envelope(wrapped, client_a, server_a, result, rn);
    feed_business(client_a, server_a, wrapped, wn); /* Nested66 containing a fully legal QR. */
    assert(!proto_qr_get(NULL) && qr_count == 0u && !wire().fresh);
    feed_business(client_a, server_a, qr, qn); /* Raw53 cannot replace request-bound62. */
    assert(!proto_qr_get(NULL) && qr_count == 0u && !wire().fresh);
    unsigned wire_n = encode_wire(raw_wire, result, rn);
    feed_result(client_a, server_a, 1u, raw_wire, wire_n); /* Inner AA55/CRC not permitted. */
    assert(!proto_qr_get(NULL) && qr_count == 0u && !wire().fresh);

    for (unsigned extra = 0u; extra < 2u; ++extra) {
        memcpy(body, result, rn);
        if (!extra) put16(body + 3u, (uint16_t)(qn - 1u));
        else put16(body + 3u, (uint16_t)(qn + 1u));
        feed_business(client_a, server_a, body, rn);
        assert(!proto_qr_get(NULL) && qr_count == 0u && !wire().fresh);
    }
    memcpy(body, result, rn); body[rn] = 0u;
    feed_business(client_a, server_a, body, rn + 1u); /* Extra QR byte is not ignored. */
    assert(!proto_qr_get(NULL) && qr_count == 0u && !wire().fresh);
    unsigned n = boot_body(body, 0x65u, client_a, server_b, 0u);
    feed_business(client_a, server_a, body, n); /* Boot challenge cannot be nested. */
    assert(session().state == PROTO_SESSION_READY && memcmp(session().server, server_a, 8u) == 0);
    assert(!proto_qr_get(NULL) && qr_count == 0u && !wire().fresh);
    uint8_t bad_ack[5] = {0x61u, 1u, 0u, 3u, 0u};
    feed_business(client_a, server_a, bad_ack, sizeof bad_ack);
    assert(wire().ack && !wire().fresh && proto_scene_status() == 0);
    bad_ack[3] = 1u; bad_ack[4] = 2u;
    feed_business(client_a, server_a, bad_ack, sizeof bad_ack);
    assert(wire().ack && !wire().fresh && proto_scene_status() == 0);
    feed_qr(client_a, server_a, 1u, 7u, "123");
    assert(proto_qr_get(NULL) && qr_count == 1u);
    assert(session().bad >= 8u);
    puts("boot session: authenticated ACK does not relax exact inner type/length/CRC-free envelope format");
}

static void test_duplicate_ready_and_begin_clear_authority(void)
{
    reset_at(10u, 1); begin_ready(client_a, server_a);
    proto_qr_begin(); proto_service();
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    feed_qr(client_a, server_a, 1u, 1u, "331");
    feed_boot(0x68u, client_a, server_a, 0u);
    assert(proto_qr_get(NULL) && wire().ack && wire().fresh);
    assert(proto_qr_take_notice(NULL));
    assert(proto_send_target(PROTO_TASK_BALL, 1u)); proto_service();
    feed_ack(client_a, server_a, 2u, 2u, 0u);
    feed_rank(client_a, server_a, 2u, 1u);
    feed_object(client_a, server_a, 2u, 1u, 4u);
    ProtoTargetRank rank;
    assert(proto_target_rank_get(&rank) && wire().ack && wire().fresh && obj_count == 1u);
    unsigned sent = tx_count;
    feed_boot(0x68u, client_a, server_a, 0u);
    proto_service();
    assert(tx_count == sent && wire().request == 2u && wire().ack && wire().fresh);
    assert(proto_target_rank_get(&rank) && rank.sequence == 1u);
    assert(proto_session_begin(client_b));
    assert_no_authority();
    assert(wire().request == 2u && frame_count == 2u);
    proto_service(); expected_boot_tx(0x64u, client_b, server_b);
    feed_boot(0x65u, client_b, server_b, 0u); proto_service();
    feed_boot(0x68u, client_b, server_b, 0u); proto_service();
    expected_business_tx(client_b, server_b, 2u, 2u, 1u, 1u);
    assert_no_authority();
    feed_ack(client_b, server_b, 2u, 2u, 0u);
    assert(!proto_target_rank_get(&rank) && !wire().fresh);
    feed_object(client_b, server_b, 2u, 1u, 4u); /* Sequence reuse is local to the new session. */
    assert(obj_count == 2u && wire().fresh);
    puts("boot session: duplicate READY preserves state, begin clears cached authority without request wrap");
}

static void test_retry_tick_wrap(void)
{
    reset_at(UINT32_MAX - 200u, 1);
    assert(proto_session_begin(client_a)); proto_qr_begin(); proto_service();
    unsigned sent = tx_count;
    tick += 499u; proto_service(); assert(tx_count == sent);
    ++tick; proto_service(); assert(tx_count == sent + 1u);
    expected_boot_tx(0x64u, client_a, server_a);
    feed_boot(0x65u, client_a, server_a, 0u); proto_service();
    tick = UINT32_MAX - 100u;
    /* Fresh challenge marks a due confirmation at a near-wrap tick. */
    feed_boot(0x65u, client_a, server_a, 0u); proto_service();
    expected_boot_tx(0x67u, client_a, server_a); sent = tx_count;
    tick += 499u; proto_service(); assert(tx_count == sent);
    ++tick; proto_service(); assert(tx_count == sent + 1u);
    feed_boot(0x68u, client_a, server_a, 0u);
    tick = UINT32_MAX - 50u; proto_service();
    expected_business_tx(client_a, server_a, 1u, 1u, 0u, 0u); sent = tx_count;
    tick += 499u; proto_service(); assert(tx_count == sent);
    ++tick; proto_service(); assert(tx_count == sent + 1u);
    expected_business_tx(client_a, server_a, 1u, 1u, 0u, 0u);
    feed_ack(client_a, server_a, 1u, 1u, 0u);
    sent = tx_count; tick += 5000u; proto_service(); assert(tx_count == sent);
    puts("boot session: unsigned HAL tick wrap preserves exact 499/500ms HELLO/CONFIRM/business retry");
}

int main(void)
{
    assert(crc_oracle((const uint8_t *)"123456789", 9u) == 0x29b1u);
    test_legacy_is_opt_in();
    test_hello_confirm_retry_and_challenge_rotation();
    test_reboot_old_nonce_request_one();
    test_qr_request_phase_and_bare_rejection();
    test_object_rank_and_selected_task();
    test_malformed_split_and_resync();
    test_fail_closed_and_explicit_init();
    test_strict_inner_validation_after_ack();
    test_duplicate_ready_and_begin_clear_authority();
    test_retry_tick_wrap();
    puts("proto boot session regression: PASS (synthetic transport only)");
    return 0;
}
