/* Synthetic wire-only tests. No UART, motors, camera or physical rank claim. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define HOST_TEST_MAIN_H
static uint32_t host_tick, host_primask, host_irq_masks;
uint32_t HAL_GetTick(void) { return host_tick; }
static uint32_t __get_PRIMASK(void) { return host_primask; }
static void __disable_irq(void) { host_primask = 1u; host_irq_masks++; }
static void __set_PRIMASK(uint32_t value) { host_primask = value; }
#include "../App/proto.c"

static unsigned host_callbacks, host_obj_callbacks, host_qr_callbacks, host_tx;
static void host_frame(const ProtoFrame *f)
{
    host_callbacks++;
    host_obj_callbacks += f->type == PF_OBJ;
    host_qr_callbacks += f->type == PF_QR;
}
static void host_send(const uint8_t *p, uint16_t n)
{
    assert(n == 8u || n == 9u || n == 14u || n == 22u || n == 28u || n == 29u);
    assert(p[0] == 0xAAu && p[1] == 0x55u);
    assert(binary_crc(p + 2u, n - 4u) == read_le16(p + n - 2u));
    host_tx++;
}
static void put16(uint8_t *p, uint16_t x) { p[0] = (uint8_t)x; p[1] = (uint8_t)(x >> 8); }
static void host_feed(const uint8_t *p, unsigned n)
{
    for (unsigned i = 0; i < n; ++i) proto_feed_byte(p[i]);
}
static void packet(const uint8_t *body, unsigned n)
{
    uint8_t wire[PROTO_BINARY_MAX_LEN];
    assert(n + 4u <= sizeof wire);
    wire[0] = 0xAAu; wire[1] = 0x55u; memcpy(wire + 2, body, n);
    put16(wire + 2 + n, binary_crc(body, n)); host_feed(wire, n + 4u);
}
static void bound(uint16_t req, const uint8_t *inner, unsigned n)
{
    uint8_t outer[100]; assert(n + 5u <= sizeof outer);
    outer[0] = 0x62u; put16(outer + 1, req); put16(outer + 3, (uint16_t)n);
    memcpy(outer + 5, inner, n); packet(outer, n + 5u);
}
static void ack(uint16_t req, uint8_t mode, uint8_t error)
{
    uint8_t body[5] = {0x61u, 0, 0, mode, error}; put16(body + 1, req); packet(body, sizeof body);
}
static void rank_at(uint16_t req, uint16_t seq, uint8_t model, uint8_t rank,
                    uint8_t count, uint8_t a, uint8_t b, uint8_t c)
{
    uint8_t body[9] = {0x54u, 0, 0, model, rank, count, a, b, c};
    put16(body + 1, seq); bound(req, body, sizeof body);
}
static void rank_now(uint16_t seq, uint8_t model, uint8_t rank, uint8_t count,
                     uint8_t a, uint8_t b, uint8_t c)
{ rank_at(s_request, seq, model, rank, count, a, b, c); }
static void obj_at(uint16_t req, uint16_t seq, uint8_t model, int empty)
{
    uint8_t body[25] = {0x01u, 0, 0}; put16(body + 1, seq);
    body[3] = empty ? 0u : 1u; put16(body + 4, 640); put16(body + 6, 480);
    body[14] = model; put16(body + 15, 900); put16(body + 17, 180);
    put16(body + 19, 220); put16(body + 21, 20); put16(body + 23, 20);
    bound(req, body, empty ? 14u : 25u);
}
static void reset_host(void)
{
    host_tick = host_primask = host_irq_masks = 0u;
    host_callbacks = host_obj_callbacks = host_qr_callbacks = host_tx = 0u;
    proto_init(); proto_set_binary_mode(1); proto_set_binary_tx(host_send); proto_set_on_frame(host_frame);
    assert(host_primask == 0u);
}
static void start(ProtoTask task, uint8_t digit, int with_ack)
{
    reset_host(); assert(proto_send_target(task, digit)); proto_service();
    assert(host_tx == 1u); if (with_ack) ack(s_request, 2u, 0u);
}
static ProtoTargetRank get_rank(uint8_t value, uint8_t count)
{
    ProtoTargetRank r; assert(proto_target_rank_get(&r));
    assert(r.rank == value && r.seen_count == count && r.request == s_request);
    assert(r.task == s_task && r.digit == s_selection); return r;
}
static void no_rank(void)
{
    ProtoTargetRank r, zero; memset(&r, 0xCC, sizeof r); memset(&zero, 0, sizeof zero);
    assert(!proto_target_rank_get(&r)); assert(memcmp(&r, &zero, sizeof r) == 0);
    assert(!proto_target_rank_get(NULL));
}
static void invalid_first(uint8_t model, uint8_t rank, uint8_t count,
                          uint8_t a, uint8_t b, uint8_t c)
{
    start(PROTO_TASK_HOSTAGE, 2u, 1); rank_now(1, model, rank, count, a, b, c);
    no_rank(); assert(s_wire.invalid_payload == 1u && s_stats.binary_bad == 1u);
    assert(!host_callbacks && !s_wire.rank_packets && proto_scene_status() == 0);
}

static void test_unknown_latch_extend(void)
{
    start(PROTO_TASK_HOSTAGE, 2u, 1); no_rank();
    rank_now(5, 2, 0, 0, 255, 255, 255); get_rank(0, 0);
    rank_now(6, 2, 0, 1, 1, 255, 255); get_rank(0, 1);
    rank_now(7, 2, 2, 2, 1, 2, 255); ProtoTargetRank r = get_rank(2, 2);
    assert(r.target_model == 2u && r.sequence == 7u);
    rank_now(8, 2, 2, 3, 1, 2, 0); r = get_rank(2, 3); assert(r.slots[2] == 0u);
    rank_now(9, 2, 2, 3, 1, 2, 0); get_rank(2, 3);
    rank_now(9, 2, 2, 3, 1, 2, 0); get_rank(2, 3);
    assert(s_wire.rank_packets == 5u && s_wire.rank_duplicate == 1u);
    assert(s_stats.accepted == 0u && s_stats.obj == 0u && s_stats.qr == 0u && s_stats.duplicate == 0u);
    assert(host_callbacks == 0u && host_tx == 1u && proto_scene_status() == 0);
    host_tick += 10000u; get_rank(2, 3); /* Historical order does not expire like coordinates. */
}
static void test_all_shape_permutations(void)
{
    const uint8_t perm[6][3] = {{0,1,2},{0,2,1},{1,0,2},{1,2,0},{2,0,1},{2,1,0}};
    const uint8_t hostage[3] = {1, 2, 0};
    for (uint8_t digit = 1; digit <= 3; ++digit)
        for (unsigned p = 0; p < 6; ++p) {
            uint8_t target = hostage[digit-1];
            uint8_t a = perm[p][0], b = perm[p][1], c = perm[p][2];
            uint8_t rank = target == a ? 1u : target == b ? 2u : 3u;
            start(PROTO_TASK_HOSTAGE, digit, 1); rank_now(42, target, rank, 3, a, b, c);
            ProtoTargetRank r = get_rank(rank, 3); assert(r.target_model == target);
        }
}
static void test_ball54_and_ball01_independent_sequences(void)
{
    const uint8_t ball[3] = {4, 5, 3};
    for (uint8_t digit = 1; digit <= 3; ++digit) {
        start(PROTO_TASK_BALL, digit, 1);
        rank_now(8, ball[digit-1], 1, 1, ball[digit-1], 255, 255);
        ProtoTargetRank r = get_rank(1, 1);
        assert(r.target_model == ball[digit-1] && r.sequence == 8u);
        assert(s_wire.invalid_payload == 0u && s_wire.rank_packets == 1u);
        assert(!host_callbacks && proto_scene_status() == 0);
        obj_at(s_request, 8, ball[digit-1], 0);
        assert(host_obj_callbacks == 1u && s_stats.accepted == 1u && s_stats.obj == 1u);
        assert(!s_stats.duplicate && proto_scene_status() == 1); get_rank(1, 1);
        rank_now(9, 2, 1, 1, 2, 255, 255); /* Even a valid hostage54 cannot cross task domains. */
        assert(s_wire.invalid_payload == 1u); get_rank(1, 1);
        obj_at(s_request, 9, ball[digit-1], 0);
        assert(host_obj_callbacks == 2u && s_stats.accepted == 2u && s_stats.obj == 2u);
    }
}
static void test_ball_permutations_unknown_wrap_and_lifecycle(void)
{
    const uint8_t perm[6][3] = {{3,4,5},{3,5,4},{4,3,5},{4,5,3},{5,3,4},{5,4,3}};
    const uint8_t ball[3] = {4,5,3};
    for (uint8_t digit = 1u; digit <= 3u; ++digit)
        for (unsigned p = 0u; p < 6u; ++p) {
            uint8_t model = ball[digit-1u], position = model == perm[p][0] ? 1u : model == perm[p][1] ? 2u : 3u;
            start(PROTO_TASK_BALL, digit, 1);
            rank_now(65534u, model, 0u, 0u, 255u,255u,255u); get_rank(0u,0u);
            rank_now(65535u, model, position, 3u, perm[p][0],perm[p][1],perm[p][2]); get_rank(position,3u);
            rank_now(0u, model, position, 3u, perm[p][0],perm[p][1],perm[p][2]);
            assert(get_rank(position,3u).sequence == 0u);
            rank_now(65535u, model, position, 3u, perm[p][0],perm[p][1],perm[p][2]);
            assert(s_wire.rank_stale == 1u); get_rank(position,3u);
            uint16_t old = s_request;
            assert(proto_send_target(PROTO_TASK_BALL,digit)); no_rank();
            rank_now(1u,model,position,3u,perm[p][0],perm[p][1],perm[p][2]); no_rank();
            assert(s_wire.result_preack == 1u);
            ack(s_request,2u,0u);
            rank_at(old,1u,model,position,3u,perm[p][0],perm[p][1],perm[p][2]); no_rank();
            rank_now(1u,model,position,3u,perm[p][0],perm[p][1],perm[p][2]); get_rank(position,3u);
            assert(proto_send_target(PROTO_TASK_HOSTAGE,2u)); no_rank(); ack(s_request,2u,0u);
            rank_now(2u,model,position,3u,perm[p][0],perm[p][1],perm[p][2]); no_rank();
            proto_receive_end(); no_rank();
        }
}
static void test_payload_validation(void)
{
    invalid_first(1, 1, 1, 1, 255, 255); /* Wrong selected shape. */
    invalid_first(9, 1, 1, 9, 255, 255);
    invalid_first(2, 4, 1, 2, 255, 255); invalid_first(2, 1, 4, 2, 0, 1);
    invalid_first(2, 2, 1, 2, 255, 255); invalid_first(2, 1, 2, 1, 2, 255);
    invalid_first(2, 0, 1, 2, 255, 255); invalid_first(2, 1, 0, 255, 255, 255);
    invalid_first(2, 1, 2, 2, 2, 255); invalid_first(2, 1, 2, 2, 3, 255);
    invalid_first(2, 0, 1, 255, 255, 255); invalid_first(2, 1, 1, 2, 0, 255);
    invalid_first(2, 1, 1, 2, 255, 0); invalid_first(2, 1, 2, 2, 255, 255);
    start(PROTO_TASK_BALL, 1, 1); rank_now(1, 4, 1, 2, 4, 0, 255); no_rank();
}
static void test_conflicts_and_wrap(void)
{
    start(PROTO_TASK_HOSTAGE, 2, 1);
    rank_now(65534u, 2, 0, 1, 1, 255, 255); get_rank(0, 1);
    rank_now(65535u, 2, 2, 2, 1, 2, 255); get_rank(2, 2);
    rank_now(0, 2, 2, 3, 1, 2, 0); get_rank(2, 3);
    rank_now(0, 2, 2, 2, 1, 2, 255); assert(s_wire.rank_conflict == 1u); get_rank(2, 3);
    rank_now(65535u, 2, 2, 2, 1, 2, 255); assert(s_wire.rank_stale == 1u); get_rank(2, 3);
    rank_now(32768u, 2, 2, 3, 1, 2, 0); assert(s_wire.rank_stale == 2u); get_rank(2, 3);
    rank_now(1, 2, 1, 3, 2, 1, 0); assert(s_wire.rank_conflict == 2u); get_rank(2, 3);
    rank_now(2, 2, 2, 2, 1, 2, 255); assert(s_wire.rank_conflict == 3u); get_rank(2, 3);
    rank_now(100, 2, 2, 3, 1, 2, 0); assert(get_rank(2, 3).sequence == 100u);
    start(PROTO_TASK_HOSTAGE, 2, 1); rank_now(1, 2, 0, 1, 0, 255, 255);
    rank_now(2, 2, 0, 1, 1, 255, 255); assert(s_wire.rank_conflict == 1u); get_rank(0, 1);
    rank_now(3, 2, 1, 3, 2, 0, 1); assert(s_wire.rank_conflict == 2u); get_rank(0, 1);
    rank_now(4, 2, 2, 2, 0, 2, 255); get_rank(2, 2);
    rank_now(5, 2, 0, 1, 0, 255, 255); assert(s_wire.rank_conflict == 3u); get_rank(2, 2);
}
static void test_ack_domain_and_lifecycle(void)
{
    start(PROTO_TASK_HOSTAGE, 2, 0); rank_now(1, 2, 1, 1, 2, 255, 255); no_rank();
    assert(s_wire.result_preack == 1u); ack(99, 2, 0); no_rank();
    ack(s_request, 2, 0); rank_at(99, 1, 2, 1, 1, 2, 255, 255); no_rank();
    rank_now(1, 2, 1, 1, 2, 255, 255); get_rank(1, 1);
    assert(!proto_send_target(PROTO_TASK_HOSTAGE, 0)); get_rank(1, 1);
    ack(99, 0, 1); get_rank(1, 1); /* Unrelated failed ACK cannot kill this request. */
    ack(s_request, 2, 1); no_rank(); assert(proto_scene_status() == -1);
    ack(s_request, 2, 0); rank_now(2, 2, 1, 1, 2, 255, 255); no_rank();
    assert(proto_send_target(PROTO_TASK_HOSTAGE, 2)); no_rank(); ack(s_request, 2, 0);
    rank_now(1, 2, 1, 1, 2, 255, 255); get_rank(1, 1);
    uint16_t previous = s_request; assert(proto_send_target(PROTO_TASK_HOSTAGE, 2)); no_rank();
    ack(s_request, 2, 0); rank_at(previous, 2, 2, 1, 1, 2, 255, 255); no_rank();
    rank_now(1, 2, 1, 1, 2, 255, 255); get_rank(1, 1);
    proto_qr_cancel(); get_rank(1, 1); /* Only cancels QR, not unrelated selected reception. */
    proto_receive_end(); no_rank(); rank_now(2, 2, 1, 1, 2, 255, 255); no_rank();
    assert(s_wire.outside_phase > 0u); proto_send_scene(SCENE_RESCUE); ack(s_request, 2, 0);
    rank_now(1, 2, 1, 1, 2, 255, 255); no_rank(); /* Generic60 not selected63. */
    proto_qr_begin(); ack(s_request, 1, 0); rank_now(1, 2, 1, 1, 2, 255, 255); no_rank();
    start(PROTO_TASK_TARGET, 2, 1); rank_now(1, 8, 1, 1, 8, 255, 255); get_rank(1, 1);
    start(PROTO_TASK_BUCKET, 0, 1); rank_now(1, 9, 1, 1, 9, 255, 255); no_rank();
    start(PROTO_TASK_HOSTAGE, 2, 1); ack(s_request, 1, 0); assert(proto_scene_status() == -1); no_rank();
    start(PROTO_TASK_HOSTAGE, 2, 1); rank_now(1, 2, 1, 1, 2, 255, 255); get_rank(1, 1);
    proto_set_binary_mode(0); no_rank(); proto_set_binary_mode(1); no_rank();
    start(PROTO_TASK_HOSTAGE, 2, 1); rank_now(1, 2, 1, 1, 2, 255, 255);
    s_request = 65535u; assert(!proto_send_target(PROTO_TASK_HOSTAGE, 2)); no_rank();
    assert(!s_rank_valid && proto_scene_status() == -1); proto_init(); no_rank();
}
static void test_01_coexistence_no_fresh_side_effect(void)
{
    for (unsigned reverse = 0; reverse < 2; ++reverse) {
        start(PROTO_TASK_HOSTAGE, 2, 1);
        if (!reverse) rank_now(44, 2, 1, 1, 2, 255, 255);
        if (!reverse) { assert(proto_scene_status() == 0); assert(!s_binary_have_seq); }
        obj_at(s_request, 44, 2, 0);
        if (reverse) rank_now(44, 2, 1, 1, 2, 255, 255);
        get_rank(1, 1); assert(host_obj_callbacks == 1u && host_callbacks == 1u);
        assert(s_stats.accepted == 1u && s_stats.obj == 1u && !s_stats.duplicate);
        obj_at(s_request, 44, 2, 0); rank_now(44, 2, 1, 1, 2, 255, 255);
        assert(s_stats.duplicate == 1u && s_wire.rank_duplicate == 1u);
        assert(host_obj_callbacks == 1u && s_wire.rank_packets == 1u && proto_scene_status() == 1);
        obj_at(s_request, 45, 2, 1); rank_now(45, 2, 1, 1, 2, 255, 255);
        assert(s_stats.accepted == 2u && s_stats.obj == 2u && host_callbacks == 1u);
        get_rank(1, 1); assert(s_wire.rank_packets == 2u);
    }
    reset_host(); proto_qr_begin(); ack(s_request, 1, 0);
    const uint8_t qr[7] = {0x53u, 1, 0, 1, '3', '3', '1'}; bound(s_request, qr, sizeof qr);
    int32_t q[3]; assert(proto_qr_get(q) && q[0] == 3 && q[1] == 3 && q[2] == 1);
    assert(host_qr_callbacks == 1u && s_stats.qr == 1u); no_rank();
    assert(proto_send_target(PROTO_TASK_HOSTAGE, 1)); ack(s_request, 2, 0);
    rank_now(1, 1, 1, 1, 1, 255, 255); get_rank(1, 1);
    assert(proto_qr_take_notice(q) && q[2] == 1 && !proto_qr_take_notice(q));
}
static void test_crc_lengths_fragment_recovery(void)
{
    start(PROTO_TASK_HOSTAGE, 2, 1);
    const uint8_t valid[18] = {0xAA,0x55,0x62,1,0,9,0,0x54,1,0,2,1,1,2,255,255,0xD6,0x12};
    uint8_t wire[18]; memcpy(wire, valid, sizeof wire);
    put16(wire + 16, binary_crc(wire + 2, 14)); wire[17] ^= 1u;
    host_feed(wire, sizeof wire); no_rank(); assert(s_stats.crc_bad == 1u);
    uint8_t body[10] = {0x54,1,0,2,1,1,2,255,255,0};
    bound(s_request, body, 8); no_rank(); bound(s_request, body, 10); no_rank();
    assert(s_wire.invalid_payload == 2u);
    uint8_t malformed[6] = {0x62,1,0,255,255,0}; packet(malformed, sizeof malformed);
    no_rank(); assert(s_wire.bad_length > 0u);
    memcpy(wire, valid, sizeof wire); put16(wire + 16, binary_crc(wire + 2, 14));
    host_feed(wire, 7); host_tick = 101; host_feed(wire + 7, 11); no_rank();
    assert(s_stats.binary_gap == 1u); host_feed(wire, 3); host_feed(wire + 3, 15); get_rank(1, 1);
    start(PROTO_TASK_HOSTAGE, 2, 1); packet(body, 9); no_rank(); /* Naked54 never authorized. */
    rank_now(1, 2, 1, 1, 2, 255, 255); get_rank(1, 1);
}
static void test_snapshot_mask(void)
{
    start(PROTO_TASK_HOSTAGE, 2, 1); rank_now(1, 2, 1, 1, 2, 255, 255);
    unsigned masks = host_irq_masks; host_primask = 1u; get_rank(1, 1);
    assert(host_primask == 1u && host_irq_masks > masks);
    assert(proto_target_rank_get(NULL) && host_primask == 1u);
    proto_receive_end(); no_rank(); assert(host_primask == 1u);
    assert(proto_send_target(PROTO_TASK_HOSTAGE, 2) && host_primask == 1u);
    host_primask = 0u; ack(s_request, 2, 0); rank_now(1, 2, 1, 1, 2, 255, 255);
    get_rank(1, 1); assert(host_primask == 0u);
}

/* TARGET task2 extends the same historical54 contract. QR digit mapping is
 * 1=red6, 2=green8, 3=blue7; these tests never enable rank-based route travel. */
static void test_target_all_digits_permutations(void)
{
    const uint8_t target[3] = {6u, 8u, 7u};
    const uint8_t perm[6][3] = {{6,7,8},{6,8,7},{7,6,8},{7,8,6},{8,6,7},{8,7,6}};
    for (uint8_t digit = 1u; digit <= 3u; ++digit)
        for (unsigned p = 0u; p < 6u; ++p) {
            uint8_t model = target[digit - 1u];
            uint8_t position = model == perm[p][0] ? 1u : model == perm[p][1] ? 2u : 3u;
            start(PROTO_TASK_TARGET, digit, 1);
            rank_now(42u, model, position, 3u, perm[p][0], perm[p][1], perm[p][2]);
            ProtoTargetRank r = get_rank(position, 3u);
            assert(r.target_model == model && r.sequence == 42u);
            assert(memcmp(r.slots, perm[p], sizeof r.slots) == 0);
            assert(!s_wire.invalid_payload && s_wire.rank_packets == 1u);
            assert(!s_fresh && !s_binary_have_seq && !host_callbacks && proto_scene_status() == 0);
        }
}
static void test_target_unknown_prefix_wrap_conflicts(void)
{
    const uint8_t target[3] = {6u, 8u, 7u};
    for (uint8_t digit = 1u; digit <= 3u; ++digit) {
        uint8_t model = target[digit - 1u], first = target[digit % 3u], last = target[(digit + 1u) % 3u];
        start(PROTO_TASK_TARGET, digit, 1);
        rank_now(65532u, model, 0u, 0u, 255u, 255u, 255u); get_rank(0u, 0u);
        rank_now(65533u, model, 0u, 1u, first, 255u, 255u); get_rank(0u, 1u);
        rank_now(65534u, model, 2u, 2u, first, model, 255u); get_rank(2u, 2u);
        rank_now(65535u, model, 2u, 3u, first, model, last); get_rank(2u, 3u);
        rank_now(0u, model, 2u, 3u, first, model, last); assert(get_rank(2u, 3u).sequence == 0u);
        rank_now(0u, model, 2u, 3u, first, model, last); assert(s_wire.rank_duplicate == 1u);
        rank_now(0u, model, 2u, 2u, first, model, 255u); assert(s_wire.rank_conflict == 1u);
        rank_now(65535u, model, 2u, 3u, first, model, last); assert(s_wire.rank_stale == 1u);
        rank_now(32768u, model, 2u, 3u, first, model, last); assert(s_wire.rank_stale == 2u);
        rank_now(1u, model, 2u, 2u, first, model, 255u); assert(s_wire.rank_conflict == 2u);
        rank_now(2u, model, 1u, 3u, model, first, last); assert(s_wire.rank_conflict == 3u);
        rank_now(3u, model, 2u, 3u, last, model, first); assert(s_wire.rank_conflict == 4u);
        assert(get_rank(2u, 3u).sequence == 0u);
        rank_now(4u, model, 2u, 3u, first, model, last); assert(get_rank(2u, 3u).sequence == 4u);
        host_tick += 10000u; get_rank(2u, 3u);
        assert(!s_fresh && !host_callbacks && !s_binary_have_seq);
        start(PROTO_TASK_TARGET, digit, 1);
        rank_now(1u, model, 0u, 1u, first, 255u, 255u); get_rank(0u, 1u);
        rank_now(2u, model, 0u, 1u, last, 255u, 255u); assert(s_wire.rank_conflict == 1u);
        rank_now(3u, model, 1u, 2u, model, first, 255u); assert(s_wire.rank_conflict == 2u);
        get_rank(0u, 1u);
        rank_now(4u, model, 2u, 2u, first, model, 255u); get_rank(2u, 2u);
        rank_now(5u, model, 0u, 1u, first, 255u, 255u); assert(s_wire.rank_conflict == 3u);
        get_rank(2u, 2u);
    }
}
static void invalid_target(uint8_t model, uint8_t rank, uint8_t count,
                           uint8_t a, uint8_t b, uint8_t c)
{
    start(PROTO_TASK_TARGET, 1u, 1);
    rank_now(1u, model, rank, count, a, b, c); no_rank();
    assert(s_wire.invalid_payload == 1u && s_stats.binary_bad == 1u);
    assert(!s_wire.rank_packets && !s_fresh && !s_binary_have_seq && !host_callbacks);
}
static void test_target_payload_validation(void)
{
    invalid_target(8u, 1u, 1u, 8u, 255u, 255u); /* Wrong QR-selected target. */
    invalid_target(7u, 1u, 1u, 7u, 255u, 255u);
    invalid_target(0u, 1u, 1u, 0u, 255u, 255u);
    invalid_target(4u, 1u, 1u, 4u, 255u, 255u);
    invalid_target(9u, 1u, 1u, 9u, 255u, 255u);
    invalid_target(255u, 1u, 1u, 255u, 255u, 255u);
    invalid_target(6u, 4u, 1u, 6u, 255u, 255u);
    invalid_target(6u, 1u, 4u, 6u, 7u, 8u);
    invalid_target(6u, 2u, 1u, 6u, 255u, 255u);
    invalid_target(6u, 1u, 2u, 7u, 6u, 255u);
    invalid_target(6u, 0u, 1u, 6u, 255u, 255u);
    invalid_target(6u, 1u, 0u, 255u, 255u, 255u);
    invalid_target(6u, 1u, 2u, 6u, 6u, 255u);
    invalid_target(6u, 1u, 2u, 6u, 0u, 255u);
    invalid_target(6u, 1u, 2u, 6u, 3u, 255u);
    invalid_target(6u, 1u, 2u, 6u, 9u, 255u);
    invalid_target(6u, 0u, 1u, 255u, 255u, 255u);
    invalid_target(6u, 1u, 1u, 6u, 7u, 255u);
    invalid_target(6u, 1u, 1u, 6u, 255u, 8u);
    invalid_target(6u, 1u, 2u, 6u, 255u, 255u);
    invalid_target(6u, 1u, 1u, 7u, 255u, 255u);
    start(PROTO_TASK_TARGET, 1u, 1);
    uint8_t body[10] = {0x54u, 1u, 0u, 6u, 1u, 1u, 6u, 255u, 255u, 0u};
    bound(s_request, body, 8u); no_rank(); bound(s_request, body, 10u); no_rank();
    assert(s_wire.invalid_payload == 2u);
    packet(body, 9u); no_rank(); /* Naked54 is never selected-task authority. */
    rank_now(1u, 6u, 1u, 1u, 6u, 255u, 255u); get_rank(1u, 1u);
}
static void test_target_ack_request_lifecycle(void)
{
    const uint8_t target[3] = {6u, 8u, 7u};
    for (uint8_t digit = 1u; digit <= 3u; ++digit) {
        uint8_t model = target[digit - 1u];
        start(PROTO_TASK_TARGET, digit, 0);
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); no_rank();
        assert(s_wire.result_preack == 1u);
        ack(99u, 2u, 0u); no_rank(); assert(s_wire.ack_mismatch == 1u);
        ack(s_request, 2u, 0u);
        rank_at(99u, 1u, model, 1u, 1u, model, 255u, 255u); no_rank();
        assert(s_wire.result_mismatch == 1u);
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); get_rank(1u, 1u);
        assert(!proto_send_target(PROTO_TASK_TARGET, 0u)); get_rank(1u, 1u);
        assert(!proto_send_target(PROTO_TASK_TARGET, 4u)); get_rank(1u, 1u);
        ack(99u, 0u, 1u); get_rank(1u, 1u);
        ack(s_request, 2u, 1u); no_rank(); assert(proto_scene_status() == -1);
        ack(s_request, 2u, 0u); rank_now(2u, model, 1u, 1u, model, 255u, 255u); no_rank();
        uint16_t old = s_request;
        assert(proto_send_target(PROTO_TASK_TARGET, digit)); no_rank();
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); no_rank();
        ack(s_request, 2u, 0u);
        rank_at(old, 2u, model, 1u, 1u, model, 255u, 255u); no_rank();
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); get_rank(1u, 1u);
        proto_qr_cancel(); get_rank(1u, 1u); /* Unrelated QR cancel keeps selected rank. */
        proto_receive_end(); no_rank();
        rank_now(2u, model, 1u, 1u, model, 255u, 255u); no_rank(); assert(s_wire.outside_phase > 0u);
        assert(proto_send_target(PROTO_TASK_BALL, 1u)); ack(s_request, 2u, 0u);
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); no_rank();
        assert(proto_send_target(PROTO_TASK_HOSTAGE, 1u)); ack(s_request, 2u, 0u);
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); no_rank();
        proto_send_scene(SCENE_ANTI); ack(s_request, 2u, 0u);
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); no_rank();
        proto_qr_begin(); ack(s_request, 1u, 0u);
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); no_rank();
        start(PROTO_TASK_TARGET, digit, 1); ack(s_request, 1u, 0u);
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); no_rank(); assert(proto_scene_status() == -1);
        start(PROTO_TASK_TARGET, digit, 1);
        rank_now(1u, model, 1u, 1u, model, 255u, 255u); get_rank(1u, 1u);
        proto_set_binary_mode(0); no_rank(); proto_set_binary_mode(1); no_rank();
    }
    start(PROTO_TASK_TARGET, 2u, 1); rank_now(1u, 8u, 1u, 1u, 8u, 255u, 255u);
    s_request = 65535u; assert(!proto_send_target(PROTO_TASK_TARGET, 2u)); no_rank();
    assert(!s_rank_valid && proto_scene_status() == -1); proto_init(); no_rank();
}
static void test_target01_same_sequence_no_rank_freshness(void)
{
    const uint8_t target[3] = {6u, 8u, 7u};
    for (uint8_t digit = 1u; digit <= 3u; ++digit)
        for (unsigned reverse = 0u; reverse < 2u; ++reverse) {
            uint8_t model = target[digit - 1u];
            start(PROTO_TASK_TARGET, digit, 1); assert(proto_target_sequence_guard_enable());
            if (!reverse) {
                rank_now(44u, model, 1u, 1u, model, 255u, 255u);
                assert(!s_fresh && !s_binary_have_seq && proto_scene_status() == 0);
            }
            obj_at(s_request, 44u, model, 0);
            if (reverse) rank_now(44u, model, 1u, 1u, model, 255u, 255u);
            get_rank(1u, 1u); assert(host_callbacks == 1u && host_obj_callbacks == 1u);
            assert(s_stats.accepted == 1u && s_stats.obj == 1u && !s_stats.duplicate);
            obj_at(s_request, 44u, model, 0);
            rank_now(44u, model, 1u, 1u, model, 255u, 255u);
            assert(host_callbacks == 1u && s_stats.duplicate == 1u && s_wire.rank_duplicate == 1u);
            s_fresh = 0u; /* Test-only clearing makes another rank-only update observable. */
            rank_now(45u, model, 1u, 1u, model, 255u, 255u); get_rank(1u, 1u);
            assert(!s_fresh && s_binary_seq == 44u && proto_scene_status() == 0);
            obj_at(s_request, 43u, model, 0); assert(!s_fresh && host_callbacks == 1u);
            obj_at(s_request, 45u, model, 0);
            assert(s_fresh && host_callbacks == 2u && s_binary_seq == 45u);
            obj_at(s_request, 46u, model, 1); get_rank(1u, 1u);
            assert(s_stats.accepted == 3u && host_callbacks == 2u);
        }
}
static void test_target_fixed_wire_goldens(void)
{
    /* Request7, seq0x1234, task2/digit2 (green model8), CRC16-CCITT-FALSE.
     * These independently fixed bytes are handshake examples, not travel order. */
    const uint8_t wire[4][18] = {
        {0xAA,0x55,0x62,7,0,9,0,0x54,0x34,0x12,8,0,0,255,255,255,0xD8,0x08},
        {0xAA,0x55,0x62,7,0,9,0,0x54,0x34,0x12,8,1,3,8,6,7,0xD9,0x98},
        {0xAA,0x55,0x62,7,0,9,0,0x54,0x34,0x12,8,2,3,6,8,7,0x05,0x4E},
        {0xAA,0x55,0x62,7,0,9,0,0x54,0x34,0x12,8,3,3,6,7,8,0x85,0x05}
    };
    for (uint8_t rank = 0u; rank <= 3u; ++rank) {
        start(PROTO_TASK_TARGET, 2u, 0);
        for (unsigned i = 1u; i < 7u; ++i) assert(proto_send_target(PROTO_TASK_TARGET, 2u));
        assert(s_request == 7u); ack(s_request, 2u, 0u);
        assert(binary_crc(wire[rank] + 2u, 14u) == read_le16(wire[rank] + 16u));
        host_feed(wire[rank], sizeof wire[rank]);
        ProtoTargetRank r = get_rank(rank, rank ? 3u : 0u);
        assert(r.target_model == 8u && r.sequence == 0x1234u && !s_fresh && !host_callbacks);
    }
    /* Control-to-vision page examples: request7, task2/digit1 red model6,
     * seq1. Each is an alternative first report, not three updates to one
     * locked rank (nor conflicting packets to send with the same sequence). */
    const uint8_t red[3][18] = {
        {0xAA,0x55,0x62,7,0,9,0,0x54,1,0,6,1,1,6,255,255,0xA7,0x70},
        {0xAA,0x55,0x62,7,0,9,0,0x54,1,0,6,2,2,8,6,255,0xF1,0xB7},
        {0xAA,0x55,0x62,7,0,9,0,0x54,1,0,6,3,3,8,7,6,0x13,0x26}
    };
    for (uint8_t i = 0u; i < 3u; ++i) {
        start(PROTO_TASK_TARGET, 1u, 0);
        for (unsigned request = 1u; request < 7u; ++request)
            assert(proto_send_target(PROTO_TASK_TARGET, 1u));
        assert(s_request == 7u); ack(s_request, 2u, 0u);
        assert(binary_crc(red[i] + 2u, 14u) == read_le16(red[i] + 16u));
        host_feed(red[i], sizeof red[i]);
        ProtoTargetRank r = get_rank((uint8_t)(i + 1u), (uint8_t)(i + 1u));
        assert(r.target_model == 6u && r.sequence == 1u && !s_fresh && !host_callbacks);
    }
}
static const uint8_t boot_client[8] = {1u,2u,3u,4u,5u,6u,7u,8u};
static const uint8_t boot_server[8] = {0x10u,0x20u,0x30u,0x40u,0x50u,0x60u,0x70u,0x80u};
static void boot_reply(uint8_t type)
{
    uint8_t body[19] = {0u}; body[0] = type; body[1] = PROTO_SESSION_VERSION;
    memcpy(body + 2u, boot_client, 8u); memcpy(body + 10u, boot_server, 8u);
    packet(body, type == 0x65u ? 18u : 19u);
}
static void boot_business(const uint8_t *inner, unsigned n, int wrong_nonce)
{
    uint8_t body[100]; assert(n + 19u <= sizeof body); body[0] = 0x66u;
    memcpy(body + 1u, boot_client, 8u); memcpy(body + 9u, boot_server, 8u);
    if (wrong_nonce) body[1] ^= 0x80u;
    put16(body + 17u, (uint16_t)n); memcpy(body + 19u, inner, n); packet(body, n + 19u);
}
static void test_target_boot66_nonce_and_ack_gates(void)
{
    uint8_t selected[5] = {0x61u,1u,0u,2u,0u};
    uint8_t result[14] = {0x62u,1u,0u,9u,0u,0x54u,1u,0u,8u,2u,2u,6u,8u,255u};
    reset_host(); assert(proto_session_begin(boot_client));
    assert(proto_send_target(PROTO_TASK_TARGET, 2u)); proto_service();
    assert(s_session.state == PROTO_SESSION_HELLO);
    boot_business(result, sizeof result, 0); no_rank(); assert(s_session.pre_ready == 1u);
    boot_reply(0x65u); proto_service(); boot_reply(0x68u);
    assert(s_session.state == PROTO_SESSION_READY); proto_service();
    ack(s_request, 2u, 0u); rank_now(1u, 8u, 2u, 2u, 6u, 8u, 255u); no_rank();
    assert(s_session.bare == 2u && !s_ack);
    boot_business(result, sizeof result, 0); no_rank(); assert(s_wire.result_preack == 1u);
    boot_business(selected, sizeof selected, 1); no_rank(); assert(s_session.mismatch == 1u && !s_ack);
    boot_business(selected, sizeof selected, 0); assert(s_ack); no_rank();
    boot_business(result, sizeof result, 1); no_rank(); assert(s_session.mismatch == 2u);
    boot_business(result, sizeof result, 0); get_rank(2u, 2u);
    assert(!s_fresh && !host_callbacks && !s_binary_have_seq);
    boot_business(result, sizeof result, 0); assert(s_wire.rank_duplicate == 1u); get_rank(2u, 2u);
    boot_reply(0x68u); get_rank(2u, 2u); /* Duplicate READY cannot erase task rank. */
    proto_receive_end(); no_rank(); boot_business(result, sizeof result, 0); no_rank();
    assert(s_wire.outside_phase == 1u);
    assert(proto_send_target(PROTO_TASK_TARGET, 2u)); selected[1] = (uint8_t)s_request;
    boot_business(selected, sizeof selected, 0); boot_business(result, sizeof result, 0); no_rank();
    assert(s_wire.result_mismatch == 1u);
    result[1] = (uint8_t)s_request; boot_business(result, sizeof result, 0); get_rank(2u, 2u);
    selected[4] = 1u; boot_business(selected, sizeof selected, 0); no_rank();
    assert(proto_scene_status() == -1);
    boot_business(result, sizeof result, 0); no_rank();
    proto_session_fail(); no_rank(); boot_business(result, sizeof result, 0); no_rank();
    assert(s_session.state == PROTO_SESSION_FAILED && s_session.pre_ready == 2u);
}

/* Owned Python producer replay uses this executable without a second parser.
 * Lines: tick !taskdigit / tick hex / tick ? / tick #. All data host-synthetic. */
static int replay_main(void)
{
    char line[2048], value[2000]; unsigned long tick;
    reset_host();
    while (fgets(line, sizeof line, stdin)) {
        if (sscanf(line, "%lu %1999s", &tick, value) != 2) return 2;
        host_tick = (uint32_t)tick;
        if (value[0] == '!') {
            unsigned task, digit; if (sscanf(value + 1, "%1u%1u", &task, &digit) != 2) return 3;
            if (!proto_send_target((ProtoTask)task, (uint8_t)digit)) return 4;
            proto_service();
        } else if (!strcmp(value, "#")) proto_receive_end();
        else if (!strcmp(value, "?")) {
            ProtoTargetRank r; int ready = proto_target_rank_get(&r);
            printf("RANK,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\n", ready,
                r.task, r.digit, r.request, r.sequence, r.target_model, r.rank,
                r.seen_count, r.slots[0], r.slots[1], r.slots[2]);
            printf("COUNTERS,%u,%u,%lu,%lu,%lu,%lu,%lu,%d\n", host_obj_callbacks, host_qr_callbacks,
                (unsigned long)s_wire.rank_packets, (unsigned long)s_wire.rank_duplicate,
                (unsigned long)s_wire.rank_stale, (unsigned long)s_wire.rank_conflict,
                (unsigned long)s_stats.rejected, proto_scene_status());
        } else {
            size_t n = strlen(value); if (n % 2u) return 5;
            for (size_t i = 0; i < n; i += 2) {
                unsigned byte; if (sscanf(value + i, "%2x", &byte) != 1) return 6;
                proto_feed_byte((uint8_t)byte);
            }
        }
    }
    return 0;
}
int main(int argc, char **argv)
{
    if (argc == 2 && !strcmp(argv[1], "--replay")) return replay_main();
    test_unknown_latch_extend(); test_all_shape_permutations(); test_payload_validation();
    test_ball54_and_ball01_independent_sequences(); test_ball_permutations_unknown_wrap_and_lifecycle();
    test_conflicts_and_wrap(); test_ack_domain_and_lifecycle();
    test_01_coexistence_no_fresh_side_effect(); test_crc_lengths_fragment_recovery(); test_snapshot_mask();
    test_target_all_digits_permutations(); test_target_unknown_prefix_wrap_conflicts();
    test_target_payload_validation(); test_target_ack_request_lifecycle();
    test_target01_same_sequence_no_rank_freshness(); test_target_fixed_wire_goldens();
    test_target_boot66_nonce_and_ack_gates();
    puts("request-bound54 rank: BALL/TARGET/HOSTAGE validation/latch/prefix/wrap/01 coexistence/lifecycle/IRQ snapshot/boot66/goldens passed");
    return 0;
}
