/* Host-only contract test: real STM32 parser, synthetic 0x63/ACK/results.
 * No camera code is imported or changed. Passing does NOT deploy support for
 * task selection on the camera and does NOT prove a physical UART link. */
#include "proto.h"
#include <stdio.h>
#include <string.h>

#define CHECK(condition) do { if (!(condition)) { \
    fprintf(stderr, "vision_task_select_test:%d: %s\n", __LINE__, #condition); \
    return 1; } } while (0)

typedef struct {
    uint8_t id;
    uint16_t score, cx, cy, w, h;
} Object;

static uint32_t now_ms;
static uint8_t sent[16];
static uint16_t sent_len;
static unsigned sent_count, object_count, qr_count;
static ProtoFrame last_object;

uint32_t HAL_GetTick(void) { return now_ms; }

static void transmit(const uint8_t *packet, uint16_t length)
{
    sent_count++;
    sent_len = length;
    if (length <= sizeof sent) memcpy(sent, packet, length);
}

static void receive(const ProtoFrame *f)
{
    if (f->type == PF_OBJ) { object_count++; last_object = *f; }
    if (f->type == PF_QR) qr_count++;
}

static void reset(void)
{
    now_ms = 0u;
    sent_count = object_count = qr_count = 0u;
    sent_len = 0u;
    memset(sent, 0, sizeof sent);
    memset(&last_object, 0, sizeof last_object);
    proto_init();
    proto_set_binary_mode(1);
    proto_set_binary_tx(transmit);
    proto_set_on_frame(receive);
}

static uint16_t le16(const uint8_t *p)
{ return (uint16_t)(p[0] | ((uint16_t)p[1] << 8)); }

static void put16(uint8_t *p, uint16_t value)
{ p[0] = (uint8_t)value; p[1] = (uint8_t)(value >> 8); }

static uint16_t crc16(const uint8_t *p, unsigned length)
{
    uint16_t crc = 0xFFFFu;
    while (length--) {
        crc ^= (uint16_t)((uint16_t)*p++ << 8);
        for (unsigned bit = 0u; bit < 8u; bit++)
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

static void feed_body(const uint8_t *p, unsigned length, int corrupt)
{
    uint16_t crc = crc16(p, length);
    proto_feed_byte(0xAAu); proto_feed_byte(0x55u);
    for (unsigned i = 0u; i < length; i++) proto_feed_byte(p[i]);
    proto_feed_byte((uint8_t)(crc ^ (corrupt ? 1u : 0u)));
    proto_feed_byte((uint8_t)(crc >> 8));
}

static void ack(uint16_t request, uint8_t mode, uint8_t status)
{
    uint8_t p[5] = {0x61u, 0u, 0u, mode, status};
    put16(p + 1u, request);
    feed_body(p, sizeof p, 0);
}

static void objects(uint16_t request, uint16_t sequence,
                    const Object *items, unsigned count, int corrupt)
{
    uint8_t p[5u + 14u + 11u * PROTO_BINARY_MAX_OBJECTS] = {0x62u};
    uint8_t *body = p + 5u;
    unsigned n = 14u + 11u * count;
    put16(p + 1u, request); put16(p + 3u, (uint16_t)n);
    body[0] = 0x01u; put16(body + 1u, sequence); body[3] = (uint8_t)count;
    put16(body + 4u, 480u); put16(body + 6u, 320u);
    put16(body + 8u, 3u); put16(body + 10u, 7u); put16(body + 12u, 11u);
    for (unsigned i = 0u; i < count; i++) {
        uint8_t *o = body + 14u + i * 11u;
        o[0] = items[i].id;
        put16(o + 1u, items[i].score); put16(o + 3u, items[i].cx);
        put16(o + 5u, items[i].cy); put16(o + 7u, items[i].w);
        put16(o + 9u, items[i].h);
    }
    feed_body(p, n + 5u, corrupt);
}

static void qr(uint16_t request, unsigned a, unsigned b, unsigned c)
{
    uint8_t p[21] = {0x62u};
    uint8_t *body = p + 5u;
    put16(p + 1u, request); put16(p + 3u, 16u);
    body[0] = 0x51u; put16(body + 1u, 1u); body[3] = 1u; body[4] = 3u;
    body[5] = (uint8_t)('0' + a); body[6] = (uint8_t)('0' + b);
    body[7] = (uint8_t)('0' + c);
    put16(body + 8u, 110u); put16(body + 10u, 65u);
    put16(body + 12u, 20u); put16(body + 14u, 30u);
    feed_body(p, sizeof p, 0);
}

static unsigned model_id(unsigned task, unsigned digit)
{
    static const unsigned ids[3][3] = {{4u, 5u, 3u}, {6u, 8u, 7u}, {1u, 2u, 0u}};
    return task == 4u ? 9u : ids[task - 1u][digit - 1u];
}

static int expected_label(unsigned task, unsigned digit)
{ return task == 4u ? 0 : task == 3u ? (int)digit + 2 : (int)digit - 1; }

static Object object(unsigned id, unsigned score, unsigned cx)
{
    Object o = {(uint8_t)id, (uint16_t)score, (uint16_t)cx, 80u, 30u, 40u};
    return o;
}

static int request_bytes(unsigned task, unsigned digit, uint16_t expected_request)
{
    CHECK(sent_len == 9u && sent[0] == 0xAAu && sent[1] == 0x55u);
    CHECK(sent[2] == 0x63u && le16(sent + 3u) == expected_request);
    CHECK(sent[5] == task && sent[6] == digit);
    CHECK(le16(sent + 7u) == crc16(sent + 2u, 5u));
    return 0;
}

static int selections_and_filter(void)
{
    for (unsigned task = 1u; task <= 4u; task++) {
        for (unsigned digit = task == 4u ? 0u : 1u;
             digit <= (task == 4u ? 0u : 3u); digit++) {
            Object all[10], duplicate[3], unrelated;
            ProtoWireDiag wire;
            ProtoStats stats;
            unsigned wanted = model_id(task, digit);
            int filter_cls = -9, filter_label = -9;
            reset();
            CHECK(proto_target_filter((ProtoTask)task, (uint8_t)digit, &filter_cls, &filter_label));
            CHECK(filter_cls == (int)task - 1);
            CHECK(filter_label == (task == 4u ? -1 : expected_label(task, digit)));
            CHECK(proto_target_filter((ProtoTask)task, (uint8_t)digit, NULL, NULL));
            CHECK(proto_send_target((ProtoTask)task, (uint8_t)digit));
            CHECK(sent_count == 0u); /* Caller only queues; DefaultTask transmits. */
            proto_service();
            CHECK(sent_count == 1u && !request_bytes(task, digit, 1u));
            for (unsigned i = 0u; i < 10u; i++) all[i] = object(i, 500u + i * 10u, 30u + i * 20u);

            objects(1u, 1u, all, 10u, 0); /* Correct body before current ACK. */
            CHECK(!object_count && proto_scene_status() == 0);
            ack(2u, 2u, 0u); /* A different request cannot acknowledge selection. */
            CHECK(proto_scene_status() == 0);
            ack(1u, 2u, 0u);
            CHECK(proto_scene_status() == 0); /* ACK alone is not a fresh result. */
            unrelated = object((wanted + 1u) % 10u, 900u, 150u);
            objects(1u, 0u, &unrelated, 1u, 0);
            CHECK(proto_scene_status() == 1 && !object_count);
            objects(1u, 1u, NULL, 0u, 0);
            CHECK(proto_scene_status() == 1 && !object_count);
            CHECK(!proto_qr_get(NULL) && !proto_qr_take_notice(NULL));

            objects(1u, 2u, all, 10u, 0);
            CHECK(object_count == 1u);
            CHECK(last_object.cls == (int)task - 1);
            CHECK(last_object.label == expected_label(task, digit));
            CHECK(last_object.cx == (int)(30u + wanted * 20u));
            CHECK(last_object.sequence == 2u);
            duplicate[0] = object(wanted, 700u, 50u);
            duplicate[1] = object(wanted, 950u, 110u);
            duplicate[2] = object(wanted, 950u, 160u);
            objects(1u, 3u, duplicate, 3u, 0);
            CHECK(object_count == 2u && last_object.cx == 110 && last_object.conf == 95);
            objects(1u, 3u, duplicate, 3u, 0);
            CHECK(object_count == 2u); /* Duplicate sequence cannot reissue coordinates. */
            proto_stats_get(&stats); CHECK(stats.duplicate == 1u);
            proto_wire_diag_get(&wire);
            CHECK(wire.request == 1u && wire.mode == 2u && wire.ack && wire.fresh);
            CHECK(wire.task == task && wire.selection == digit);
            CHECK(wire.result_preack == 1u && wire.ack_mismatch == 1u);
        }
    }
    puts("task selection: nine QR selectors plus bucket; queue/ACK/fresh gates; ten-class and best-score filtering passed");
    return 0;
}

static int qr_choices_and_phases(void)
{
    for (unsigned a = 1u; a <= 3u; a++) for (unsigned b = 1u; b <= 3u; b++)
    for (unsigned c = 1u; c <= 3u; c++) {
        int32_t latched[3];
        const unsigned tasks[4] = {1u, 4u, 2u, 3u};
        const unsigned digits[4] = {a, 0u, b, c};
        reset();
        proto_send_scene(SCENE_QR); proto_service();
        CHECK(sent_len == 8u && sent[2] == 0x60u && le16(sent + 3u) == 1u);
        ack(1u, 1u, 0u); qr(1u, a, b, c);
        CHECK(qr_count == 1u && proto_qr_get(latched));
        CHECK(latched[0] == (int32_t)a && latched[1] == (int32_t)b && latched[2] == (int32_t)c);
        CHECK(proto_qr_take_notice(NULL) && !proto_qr_take_notice(NULL));
        for (unsigned i = 0u; i < 4u; i++) {
            const uint16_t request = (uint16_t)(i + 2u);
            Object o = object(model_id(tasks[i], digits[i]), 900u, 150u);
            CHECK(proto_send_target((ProtoTask)tasks[i], (uint8_t)digits[i]));
            proto_service(); CHECK(!request_bytes(tasks[i], digits[i], request));
            ack(request, 2u, 0u); objects(request, 1u, &o, 1u, 0);
            CHECK(object_count == i + 1u);
            CHECK(last_object.cls == (int)tasks[i] - 1);
            CHECK(last_object.label == expected_label(tasks[i], digits[i]));
            CHECK(!proto_qr_take_notice(NULL));
        }
        /* Mission-owned copy, not a QR cache consumed by phase selection. */
        CHECK(latched[0] == (int32_t)a && latched[1] == (int32_t)b && latched[2] == (int32_t)c);
    }
    puts("task selection: all 27 QR tuples -> ball / bucket / target / hostage digit choices passed");
    return 0;
}

static int invalid_and_retry(void)
{
    static const unsigned invalid[][2] = {
        {0u, 1u}, {5u, 1u}, {1u, 0u}, {1u, 4u}, {2u, 0u}, {2u, 4u},
        {3u, 0u}, {3u, 4u}, {4u, 1u}, {4u, 255u}
    };
    ProtoWireDiag before, after;
    uint8_t first[9];
    Object wanted = object(5u, 900u, 150u);
    reset(); CHECK(proto_send_target((ProtoTask)1u, 2u)); proto_service();
    memcpy(first, sent, sizeof first); proto_wire_diag_get(&before);
    for (unsigned i = 0u; i < sizeof invalid / sizeof invalid[0]; i++) {
        CHECK(!proto_send_target((ProtoTask)invalid[i][0], (uint8_t)invalid[i][1]));
        proto_wire_diag_get(&after);
        CHECK(after.request == before.request && after.mode == before.mode);
        CHECK(after.task == before.task && after.selection == before.selection);
        CHECK(after.ack == before.ack && after.fresh == before.fresh && after.failed == before.failed);
    }
    now_ms = 499u; proto_service(); CHECK(sent_count == 1u);
    now_ms = 500u; proto_service(); CHECK(sent_count == 2u && !memcmp(first, sent, sizeof first));
    now_ms = 1000u; proto_service(); CHECK(sent_count == 3u && !memcmp(first, sent, sizeof first));
    proto_wire_diag_get(&after); CHECK(after.request == 1u);
    ack(1u, 2u, 0u); objects(1u, 1u, &wanted, 1u, 0);
    CHECK(object_count == 1u && proto_scene_status() == 1);
    for (unsigned i = 0u; i < sizeof invalid / sizeof invalid[0]; i++)
        CHECK(!proto_send_target((ProtoTask)invalid[i][0], (uint8_t)invalid[i][1]));
    now_ms = 1500u; proto_service(); CHECK(sent_count == 3u);
    objects(1u, 2u, &wanted, 1u, 0);
    CHECK(object_count == 2u && last_object.label == LAB_G && proto_scene_status() == 1);

    reset();
    for (unsigned i = 0u; i < 259u; i++) CHECK(proto_send_target((ProtoTask)1u, 1u));
    proto_service(); CHECK(!request_bytes(1u, 1u, 259u));
    CHECK(sent[3] == 3u && sent[4] == 1u); /* Request LE16 crosses the byte boundary. */
    puts("task selection: invalid inputs preserve active request/filter; exact 500ms retry and LE16 request passed");
    return 0;
}

static int stale_nack_and_idle(void)
{
    Object ball = object(4u, 900u, 110u), target = object(8u, 900u, 130u), all[10];
    reset(); CHECK(proto_send_target((ProtoTask)1u, 1u)); proto_service();
    ack(1u, 2u, 0u); objects(1u, 1u, &ball, 1u, 0);
    CHECK(object_count == 1u);
    CHECK(proto_send_target((ProtoTask)2u, 2u)); proto_service();
    ack(1u, 2u, 0u); objects(1u, 2u, &target, 1u, 0);
    CHECK(proto_scene_status() == 0 && object_count == 1u);
    objects(2u, 2u, &target, 1u, 0);
    CHECK(proto_scene_status() == 0 && object_count == 1u);
    ack(2u, 1u, 0u); /* Current request but wrong actual mode must fail. */
    CHECK(proto_scene_status() == -1);
    objects(2u, 3u, &target, 1u, 0); CHECK(object_count == 1u);
    now_ms = 1000u; proto_service(); CHECK(sent_count == 2u);
    CHECK(proto_send_target((ProtoTask)2u, 2u)); proto_service();
    ack(3u, 2u, 1u); CHECK(proto_scene_status() == -1);
    objects(3u, 4u, &target, 1u, 0); CHECK(object_count == 1u);

    CHECK(proto_send_target((ProtoTask)2u, 2u)); proto_service();
    ack(4u, 2u, 0u); objects(4u, 4u, &target, 1u, 1);
    CHECK(proto_scene_status() == 0 && object_count == 1u);
    objects(4u, 5u, NULL, 0u, 0);
    CHECK(proto_scene_status() == 1 && object_count == 1u);
    proto_send_scene(SCENE_IDLE); proto_service();
    CHECK(sent_len == 8u && sent[2] == 0x60u && le16(sent + 3u) == 5u && sent[5] == 0u);
    ack(4u, 2u, 0u); objects(4u, 6u, &target, 1u, 0);
    CHECK(proto_scene_status() == 0 && object_count == 1u);
    ack(5u, 0u, 0u); CHECK(proto_scene_status() == 1 && !proto_qr_get(NULL));
    {
        ProtoWireDiag wire;
        proto_wire_diag_get(&wire);
        CHECK(wire.task == 0u && wire.selection == 0u);
    }

    /* Generic 0x60 OBJECT remains the receive-only calibration contract. */
    proto_send_scene(SCENE_EOD); proto_service();
    CHECK(sent_len == 8u && sent[2] == 0x60u && le16(sent + 3u) == 6u && sent[5] == 2u);
    ack(6u, 2u, 0u);
    for (unsigned i = 0u; i < 10u; i++) all[i] = object(i, 800u, 30u + i * 20u);
    objects(6u, 1u, all, 10u, 0);
    CHECK(object_count == 11u && proto_scene_status() == 1);
    puts("task selection: stale/pre-ACK/CRC/NACK/mode/empty/IDLE gates; generic 0x60 calibration unchanged passed");
    return 0;
}

static int mapping_ascii_notice_and_wrap(void)
{
    int cls = 70, label = 80;
    int32_t notice[3];
    ProtoWireDiag wire;
    CHECK(!proto_target_filter((ProtoTask)0u, 1u, &cls, &label));
    CHECK(cls == 70 && label == 80);
    CHECK(!proto_target_filter((ProtoTask)1u, 0u, &cls, &label));
    CHECK(cls == 70 && label == 80);
    CHECK(!proto_target_filter((ProtoTask)4u, 1u, &cls, &label));
    CHECK(cls == 70 && label == 80);

    reset(); proto_set_binary_mode(0);
    CHECK(!proto_send_target(PROTO_TASK_BALL, 1u));
    proto_service(); proto_wire_diag_get(&wire);
    CHECK(!sent_count && !wire.request && !wire.task);

    reset(); proto_send_scene(SCENE_QR); proto_service();
    ack(1u, 1u, 0u); qr(1u, 1u, 2u, 3u);
    CHECK(proto_send_target(PROTO_TASK_BALL, 1u));
    CHECK(!proto_qr_get(NULL)); /* Current OBJECT request cannot serve as QR readiness. */
    CHECK(proto_qr_take_notice(notice));
    CHECK(notice[0] == 1 && notice[1] == 2 && notice[2] == 3);
    CHECK(!proto_qr_take_notice(NULL)); /* IRQ -> phase switch preserves just one report. */

    reset();
    for (unsigned i = 0u; i < 65535u; i++) CHECK(proto_send_target(PROTO_TASK_BALL, 1u));
    proto_service(); CHECK(!request_bytes(1u, 1u, 65535u));
    CHECK(!proto_send_target(PROTO_TASK_TARGET, 1u));
    proto_wire_diag_get(&wire);
    CHECK(wire.request == 65535u && wire.failed && proto_scene_status() == -1);
    now_ms = 1000u; proto_service(); CHECK(sent_count == 1u);
    puts("task selection: pure mapping/ASCII rejection; pending QR notice; request exhaustion never wraps passed");
    return 0;
}

static int echoed_selector(void)
{
    ProtoWireDiag wire;
    ProtoStats stats;
    reset(); CHECK(proto_send_target((ProtoTask)3u, 3u)); proto_service();
    CHECK(sent_len == 9u);
    for (unsigned i = 0u; i < sent_len; i++) proto_feed_byte(sent[i]);
    proto_wire_diag_get(&wire); proto_stats_get(&stats);
    CHECK(wire.command_echo == 1u && wire.last_reject_type == 0x63u);
    CHECK(wire.last_reject_len == 9u && wire.rx_bytes == 9u);
    CHECK(!object_count && !qr_count && !stats.accepted && !stats.crc_bad);
    CHECK(proto_scene_status() == 0 && !proto_qr_get(NULL));
    sent[8] ^= 1u;
    for (unsigned i = 0u; i < sent_len; i++) proto_feed_byte(sent[i]);
    proto_wire_diag_get(&wire); proto_stats_get(&stats);
    CHECK(wire.command_echo == 1u && stats.crc_bad == 1u);
    CHECK(proto_scene_status() == 0 && !object_count && !qr_count);
    puts("task selection: complete CRC-checked 0x63 echo diagnosed, never ACK/QR/coordinates passed");
    return 0;
}

static int qr53_and_local_end(void)
{
    uint8_t p[12] = {0x62u, 1u, 0u, 7u, 0u, 0x53u, 1u, 0u, 1u, '1', '2', '3'};
    int32_t tuple[3], notice[3];
    ProtoWireDiag wire;
    unsigned transmissions;
    reset(); proto_qr_begin(); proto_service();
    ack(1u, 1u, 0u); feed_body(p, sizeof p, 0);
    CHECK(qr_count == 1u && proto_qr_get(tuple));
    CHECK(tuple[0] == 1 && tuple[1] == 2 && tuple[2] == 3);
    transmissions = sent_count;
    proto_receive_end();
    CHECK(!proto_qr_get(NULL) && proto_scene_status() == 0);
    ack(1u, 1u, 1u); feed_body(p, sizeof p, 0);
    now_ms = 1000u; proto_service();
    CHECK(sent_count == transmissions && qr_count == 1u && proto_scene_status() == 0);
    proto_wire_diag_get(&wire);
    CHECK(!wire.receiving && wire.outside_phase == 2u && !wire.failed);
    CHECK(proto_qr_take_notice(notice) && !proto_qr_take_notice(NULL));
    CHECK(!memcmp(tuple, notice, sizeof tuple));
    proto_qr_begin(); proto_service();
    CHECK(sent_count == transmissions + 1u && le16(sent + 3u) == 2u && sent[5] == 1u);
    ack(2u, 1u, 0u);
    feed_body(p, sizeof p, 0); /* A late locked QR from request 1 cannot fill request 2. */
    CHECK(!proto_qr_get(NULL));
    p[1] = 2u; feed_body(p, sizeof p, 0);
    CHECK(proto_qr_get(NULL) && qr_count == 2u);
    proto_qr_cancel(); now_ms = 2000u; proto_service();
    CHECK(!proto_qr_take_notice(NULL) && !proto_qr_get(NULL));
    CHECK(sent_count == transmissions + 1u); /* No IDLE/STOP request on explicit cancellation. */

    reset(); CHECK(proto_send_target(PROTO_TASK_BALL, 1u));
    proto_receive_end(); now_ms = 1000u; proto_service();
    CHECK(!sent_count); /* Cancel a queued request before DefaultTask sends it. */
    puts("QR53: copied tuple/notice survives local end; late ACK/result ignored; new request and cancellation emit no camera stop passed");
    return 0;
}

int main(void)
{
    if (selections_and_filter() || qr_choices_and_phases() || invalid_and_retry()
        || stale_nack_and_idle() || echoed_selector() || mapping_ascii_notice_and_wrap()
        || qr53_and_local_end()) return 1;
    puts("vision task-selection protocol regression passed; camera support and physical UART remain unverified");
    return 0;
}
