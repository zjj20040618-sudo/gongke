/* Bucket integration for route34 + real 63/61/62 parser replay. Mode36 no
 * longer includes a bucket gate; retain these checks on its real owner.
 * Only physical
 * peripherals and time are inert host stubs; no robot is driven or flashed. */
#define ROUTE31_QR_FIXTURE_MAIN bucket36_qr_fixture_main
#include "route31_qr_gate_test.c"
#undef ROUTE31_QR_FIXTURE_MAIN

static uint8_t bucket_wire_packet[9];
static uint16_t bucket_wire_length;
static unsigned bucket_wire_calls, bucket_wire_bad;

static void bucket_wire_tx(const uint8_t *packet, uint16_t length)
{
    uint16_t crc;
    bucket_wire_calls++;
    bucket_wire_length = length;
    if (length > sizeof bucket_wire_packet || length < 4u) {
        bucket_wire_bad++;
        return;
    }
    memcpy(bucket_wire_packet, packet, length);
    crc = binary_crc(packet + 2u, length - 4u);
    if (packet[0] != 0xAAu || packet[1] != 0x55u ||
        packet[length - 2u] != (uint8_t)crc ||
        packet[length - 1u] != (uint8_t)(crc >> 8)) bucket_wire_bad++;
    if ((length == 8u && packet[2] == 0x60u) ||
        (length == 9u && packet[2] == 0x63u)) {
        wire_commands++;
        wire_request = (uint16_t)(packet[3] | ((uint16_t)packet[4] << 8));
        wire_mode = packet[2] == 0x63u ? 2u : packet[5];
    } else bucket_wire_bad++;
}

static void bucket_wire_on_frame(const ProtoFrame *frame)
{
    /* The production callback sees stats.obj after this packet has parsed. */
    real_proto_stats_get(&host_proto_stats);
    test_vision_feed_frame(frame);
}

static void bucket_le16(uint8_t *out, unsigned value)
{
    out[0] = (uint8_t)value; out[1] = (uint8_t)(value >> 8);
}

static void bucket_object_with_poll(uint16_t request, uint16_t sequence, int model,
                                    unsigned cx, unsigned width, int corrupt, int poll)
{
    uint8_t inner[25] = {0x01u, (uint8_t)sequence, (uint8_t)(sequence >> 8),
                         (uint8_t)(model >= 0)};
    uint8_t packet[30] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8)};
    unsigned length = model >= 0 ? sizeof inner : 14u;
    bucket_le16(inner + 4u, width); bucket_le16(inner + 6u, 320u);
    bucket_le16(inner + 8u, 10u); bucket_le16(inner + 10u, 20u);
    bucket_le16(inner + 12u, 30u);
    if (model >= 0) {
        inner[14] = (uint8_t)model;
        bucket_le16(inner + 15u, 900u); bucket_le16(inner + 17u, cx);
        bucket_le16(inner + 19u, 120u); bucket_le16(inner + 21u, 24u);
        bucket_le16(inner + 23u, 24u);
    }
    bucket_le16(packet + 3u, length); memcpy(packet + 5u, inner, length);
    wire_feed_raw(packet, length + 5u, corrupt);
    if (poll) wire_poll();
}

static void bucket_object(uint16_t request, uint16_t sequence, int model,
                          unsigned cx, unsigned width, int corrupt)
{ bucket_object_with_poll(request, sequence, model, cx, width, corrupt, 1); }

static int bucket_boot(void)
{
    CHECK(wire_boot() == 0);
    memset(bucket_wire_packet, 0, sizeof bucket_wire_packet);
    bucket_wire_length = 0u; bucket_wire_calls = bucket_wire_bad = 0u;
    proto_set_binary_tx(bucket_wire_tx); proto_set_on_frame(bucket_wire_on_frame);
    host_target_hook = real_proto_send_target;
    run_cmd("34"); wire_sync();
    CHECK(ROUTE_TEST_STAGES == 15u && route_seq_bucket_enabled());
    CHECK(s_msel == 34 && s_seq_mode == 34 && s_seq_state == SQ_READY && stopped());
    CHECK(!s_receiving && !s_due && !proto_qr_get(NULL));
    CHECK(!host_target_calls && !bucket_wire_calls && !bucket_wire_bad);
    return 0;
}

static int bucket_prefix(void)
{
    static const int modes[7] = {17,16,20,16,15,16,30};
    static const int commands[7] = {-530,-650,90,-650,80,-190,-92};
    static const float speeds[7] = {100,100,100,300,20,100,100};
    CHECK(bucket_boot() == 0); run_cmd("g"); wire_sync();
    CHECK(s_seq_run == 1u && s_seq_state == SQ_STILL && stopped());
    for (unsigned stage = 0u; stage < 7u; ++stage) {
        CHECK(s_seq_stage == stage && sequence_start_stage() == 0);
        CHECK(s_seq_mode == 34 && s_msel == modes[stage] && !s_receiving);
        CHECK(!host_target_calls && !bucket_wire_calls && !notice_calls && !s_seq_qr[0]);
        if (dist_mode()) {
            CHECK(s_dist_target == commands[stage] && s_v == speeds[stage] && s_dist_precise);
            CHECK(s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : 0.3f));
            CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f);
            CHECK(s_dist_ff_ratio == (stage == 1u || stage == 5u ? 0.00625f : 0.0f));
        } else CHECK(turn_target_deg() == commands[stage]);
        CHECK(sequence_finish_stage() == 0); wire_sync();
        CHECK(s_seq_state != SQ_QR_WAIT && stopped());
        CHECK(!pulse_calls && !servo_calls && !laser_state && !s_go);
    }
    CHECK(s_seq_stage == ROUTE_TEST_ALIGN_STAGE && s_seq_state == SQ_BUCKET_ALIGN);
    CHECK(host_target_calls == 1 && host_target_task == PROTO_TASK_BUCKET && host_target_digit == 0u);
    CHECK(wire_mode == 2u && bucket_wire_length == 9u && bucket_wire_calls == 1u);
    CHECK(bucket_wire_packet[2] == 0x63u && bucket_wire_packet[5] == 4u &&
          bucket_wire_packet[6] == 0u && !bucket_wire_bad && s_receiving);
    CHECK(!notice_calls && !s_seq_qr[0] && !proto_qr_get(NULL));
    return 0;
}

static int bucket_prepare(int ack)
{
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
    if (ack) wire_ack(wire_request, 2u, 0u);
    host_tick += T_DIST_STILL_MS; wire_poll();
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
    host_tick += NAV_SETTLE_MS; wire_poll();
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
    return 0;
}

static int bucket_aim(void)
{
    CHECK(bucket_prefix() == 0); CHECK(bucket_prepare(1) == 0);
    for (uint16_t seq = 1u; seq <= 5u; ++seq) {
        bucket_object(wire_request, seq, 9, 500u, 640u, 0);
        CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
    }
    for (uint16_t seq = 6u; seq <= 9u; ++seq) {
        host_tick += 250u; bucket_object(wire_request, seq, 9, 500u, 640u, 0);
    }
    CHECK(s_seq_stage == ROUTE_TEST_BACK_STAGE && s_seq_state == SQ_MANUAL_D_WAIT && stopped());
    CHECK(!s_receiving && !s_due && !laser_state && !host_laser_on_calls);
    return 0;
}

static int check_complete_recipe_and_distance_wait(void)
{
    static const int modes[7] = {16,20,15,20,15,20,15};
    static const int commands[7] = {-730,90,780,90,2450,90,2125};
    CHECK(bucket_aim() == 0);
    const uint16_t request = wire_request;
    const unsigned tx = bucket_wire_calls;
    host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped() && bucket_wire_calls == tx);
    bucket_object(request, 10u, 9, 450u, 640u, 0);
    CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped() && !s_receiving);
    run_cmd("d730");
    CHECK(s_seq_stage == 8u && s_seq_state == SQ_STILL && s_msel == 16 && stopped());
    for (unsigned index = 0u; index < 7u; ++index) {
        CHECK(s_seq_stage == index + 8u && sequence_start_stage() == 0);
        CHECK(s_msel == modes[index] && s_seq_mode == 34);
        if (dist_mode()) {
            CHECK(s_dist_target == commands[index] && s_v == 100.0f && s_dist_precise);
            CHECK(s_dist_heading_kp == 0.3f);
        } else CHECK(turn_target_deg() == commands[index]);
        CHECK(sequence_finish_stage() == 0); wire_sync();
        CHECK(stopped() && bucket_wire_calls == tx && !s_receiving && !s_due);
        CHECK(!laser_state && !host_laser_on_calls && !pulse_calls && !servo_calls && !s_go);
    }
    CHECK(s_seq_state == SQ_DONE && s_msel == 34 && s_seq_mode == 34 && s_round == R_DONE);
    uint32_t run = s_seq_run;
    run_cmd("g"); run_cmd("d500"); host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_DONE && stopped() && s_seq_run == run && bucket_wire_calls == tx);
    puts("bucket integration on34: exact15 nodes, task4/digit0, five fresh frames + still hold -> NEW d730 back100 -> whole2450/2125 tail; no QR/laser/arm/auto-distance/resume passed");
    return 0;
}

static int check_mode31_34_integrated_recipe_isolation(void)
{
    static const unsigned modes[15] = {17,16,20,16,15,16,30,0,16,20,15,20,15,20,15};
    static const unsigned distances[15] = {530,650,0,650,80,190,0,0,0,0,780,0,2450,0,2125};
    static const float speeds[15] = {100,100,100,300,20,100,100,50,100,100,100,100,100,100,100};
    {
        CHECK(wire_boot() == 0); run_cmd("34");
        CHECK(s_seq_mode == 34u && route_seq_stage_count() == 15u && ROUTE_TEST_STAGES == 15u);
        for (unsigned stage = 0u; stage < 15u; ++stage) {
            s_seq_stage = (uint8_t)stage;
            const RouteTestLeg *leg = route_seq_leg();
            CHECK(leg == &s_route_test_plan[stage]);
            CHECK(leg->mode == modes[stage] && leg->distance_mm == distances[stage]);
            CHECK(leg->speed_mms == speeds[stage]);
            CHECK(leg->heading_hold == (stage == 3u || stage == 4u ? 0u : 1u));
        }
        CHECK(ROUTE_TEST_ALIGN_STAGE == 7u && ROUTE_TEST_BACK_STAGE == 8u);
    }
    {
        static const unsigned modes31[9] = {17,16,20,16,15,16,18,16,30};
        static const unsigned distances31[9] = {535,630,0,650,0,190,800,760,0};
        CHECK(wire_boot() == 0); run_cmd("31");
        CHECK(s_seq_mode == 31u && route_seq_stage_count() == 16u && ROUTE31_STAGES == 16u);
        CHECK(!route_seq_bucket_enabled());
        for (unsigned stage = 0u; stage < 9u; ++stage) {
            s_seq_stage = (uint8_t)stage;
            const RouteTestLeg *leg = route_seq_leg();
            CHECK(leg == &s_route31_plan[stage]);
            CHECK(leg->mode == modes31[stage] && leg->distance_mm == distances31[stage]);
            CHECK(leg->speed_mms == (stage == 3u ? 300.0f : stage == 4u ? 40.0f :
                                    stage == 0u || stage == 6u ? 250.0f :
                                    stage == 2u || stage == 8u ? 100.0f : 200.0f));
            CHECK(leg->heading_hold == (stage == 3u || stage == 4u ? 0u : 1u));
            CHECK(leg->mode != 0u && ((leg->distance_mm != 0u) ==
                  (leg->mode != 20u && leg->mode != 30u && stage != 4u)));
        }
    }
    CHECK(s_route_test_plan[3].speed_mms == 300.0f && s_bucket36_plan[3].speed_mms == 300.0f);
    CHECK(s_route_test_plan[4].mode == 15u && s_route_test_plan[4].distance_mm == 80u);
    CHECK(s_bucket36_plan[4].mode == 15u && s_bucket36_plan[4].distance_mm == 80u);
    CHECK(s_route31_plan[6].mode == 18u && s_route31_plan[6].distance_mm == 800u &&
          s_route31_plan[6].speed_mms == 250.0f);
    CHECK(s_bucket36_plan[6].mode == 18u && s_bucket36_plan[6].distance_mm == 730u &&
          s_bucket36_plan[6].speed_mms == 100.0f);
    CHECK(s_bucket36_plan[7].mode == 16u && s_bucket36_plan[7].distance_mm == 780u &&
          s_bucket36_plan[7].speed_mms == 100.0f && s_bucket36_plan[7].heading_hold == 1u);
    CHECK(s_bucket36_plan[8].mode == 30u && s_bucket36_plan[9].mode == 15u);
    CHECK(BUCKET_ROUTE_STAGES == 10u);
    CHECK(s_cross37_plan[1].distance_mm == 80u && CROSS37_STAGES == 3u);
    CHECK(s_route_test_plan[0].distance_mm == 530u && s_bucket36_plan[0].distance_mm == 530u);
    CHECK(wire_boot() == 0); run_cmd("36");
    CHECK(route_seq_leg()->distance_mm == 530u && route_seq_leg()->speed_mms == 100.0f);
    puts("recipe isolation:31 independent16-task stages with nine-road prefix/left535/R2back630/tilt-contact-v40/right800/back760; no OLD bucket-anchor/manuald;34 retains15-node bucket/530/80/manual-d730,36 ten/530/80/right730,37 three/80; global left92 unchanged passed");
    return 0;
}

typedef struct { float x, y, heading; } BucketFieldPose;

static void bucket_nominal_field_step(BucketFieldPose *pose, unsigned mode, float distance)
{
    float vx = 0.0f, vy = 0.0f;
    float heading = pose->heading * 0.017453292519943295f;
    if (mode == 20u) { pose->heading += 90.0f; return; }
    if (mode == 30u) { pose->heading -= 90.0f; return; }
    if (mode == 15u) vx = distance;
    else if (mode == 16u) vx = -distance;
    else if (mode == 17u) vy = -distance;
    else if (mode == 18u) vy = distance;
    pose->x += vx * cosf(heading) - vy * sinf(heading);
    pose->y += vx * sinf(heading) + vy * cosf(heading);
}

static int check_nominal_field_vector_equivalence(void)
{
    /* Mirrored 31/34 prefix before its bucket gate. These are exact cardinal
     * geometry checks, NOT a physical simulation using compensated angles. */
    static const unsigned old_modes[7] = {17u,16u,30u,15u,16u,15u,20u};
    static const float distances[7] = {530.0f,650.0f,0.0f,650.0f,80.0f,190.0f,0.0f};
    BucketFieldPose before = {0.0f,0.0f,0.0f}, after = {0.0f,0.0f,0.0f};
    CHECK(TURN90_TARGET_DEG == 90.0f && T_TURN_RIGHT_TARGET_DEG == 90.0f);
    CHECK(T_TURN_LEFT_TARGET_DEG == -92.0f && T_TURN90_LEFT_COMP_DEG == 2.0f);
    for (unsigned stage = 0u; stage < 7u; ++stage) {
        BucketFieldPose old0 = before, new0 = after;
        bucket_nominal_field_step(&before, old_modes[stage], distances[stage]);
        bucket_nominal_field_step(&after, s_route_test_plan[stage].mode, distances[stage]);
        CHECK(fabsf((before.x - old0.x) - (after.x - new0.x)) < 0.001f);
        CHECK(fabsf((before.y - old0.y) - (after.y - new0.y)) < 0.001f);
        CHECK(fabsf(before.x - after.x) < 0.001f && fabsf(before.y - after.y) < 0.001f);
        if (stage >= 2u && stage <= 5u) CHECK(after.heading - before.heading == 180.0f);
        else CHECK(after.heading == before.heading);
    }
    CHECK(fabsf(after.x + 650.0f) < 0.001f && fabsf(after.y + 1290.0f) < 0.001f);
    CHECK(after.heading == 0.0f && before.heading == 0.0f);
    puts("route31/34 cardinal geometry: mirrored prefix preserves every field displacement and final nominal heading; left-92 calibration not treated as measured exact90 or physical no-slip acceptance");
    return 0;
}

static int check_post_cross_heading_restart(void)
{
    CHECK(bucket_boot() == 0);
    for (unsigned run = 1u; run <= 2u; ++run) {
        if (run > 1u) run_cmd("34");
        run_cmd("ykp1.2"); run_cmd("fff0.02"); run_cmd("g");
        CHECK(s_seq_run == run && s_route_heading_kp == 1.2f && step_heading_kp_deg() == 0.3f);
        s_seq_stage = 4u; route_seq_prepare();
        CHECK(sequence_start_stage() == 0 && s_msel == 15 && s_dist_target == 80.0f && s_v == 20.0f);
        CHECK(s_dist_heading_kp == 0.0f && s_dist_ff_ratio == 0.0f);
        host_yaw = 6.0f; tick(); CHECK(last_w == 0.0f && last_x == 20.0f && last_y == 0.0f);
        host_yaw = -6.0f; tick(); CHECK(last_w == 0.0f && last_y == 0.0f);
        unsigned before_finish = (unsigned)zero_calls;
        CHECK(sequence_finish_stage() == 0);
        CHECK(s_seq_stage == 5u && s_seq_state == SQ_STILL && stopped());
        CHECK(zero_calls == (int)before_finish + 1);
        unsigned before_node = (unsigned)zero_calls;
        host_yaw = run == 1u ? 7.0f : -7.0f; host_counts[2]++;
        host_tick += T_DIST_STILL_MS; wire_poll();
        CHECK(s_seq_state == SQ_STILL && zero_calls == (int)before_node && stopped());
        host_tick += T_DIST_STILL_MS - 1u; wire_poll();
        CHECK(s_seq_state == SQ_STILL && zero_calls == (int)before_node && stopped());
        host_tick++; wire_poll();
        CHECK(s_seq_state == SQ_WAIT && zero_calls == (int)before_node + 1 && host_yaw == 0.0f && stopped());
        CHECK(NAV_SETTLE_MS == 750u);
        host_tick += NAV_SETTLE_MS - 1u; wire_poll();
        CHECK(s_seq_state == SQ_WAIT && zero_calls == (int)before_node + 1 && stopped());
        host_tick++; wire_poll();
        CHECK(s_seq_state == SQ_RUN && s_round == R_RUN && s_msel == 16);
        CHECK(s_dist_target == -190.0f && s_v == 100.0f && s_dist_heading0 == 0.0f);
        CHECK(s_dist_heading_kp == 1.2f && s_dist_precise && s_dist_ff_ratio == 0.00625f);
        CHECK(last_x == -100.0f && last_y == -0.625f && last_w == 0.0f && host_yaw == 0.0f);
        CHECK(fabsf(s_route_forward_ff_ratio - 0.02f) < 0.000001f && test_forward_ff_ratio() == 0.0125f);
        host_yaw = 3.0f; tick(); CHECK(fabsf(last_w + 1.2f * 3.0f * 0.0174533f) < 0.000001f);
        host_yaw = -3.0f; tick(); CHECK(fabsf(last_w - 1.2f * 3.0f * 0.0174533f) < 0.000001f);
        CHECK(sequence_finish_stage() == 0 && s_seq_stage == 6u);
        CHECK(sequence_start_stage() == 0 && s_msel == 30 && turn_target_deg() == -92.0f);
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    }
    puts("bucket34 two-run node: forward80/v20 mechanical square-up has no yaw/FF; wheel change delays zero until250ms still; freshzero + full750ms ->back190/v100 restoresRAMykp + independentBFF; thenleft92; global/manual untouched passed");
    return 0;
}

static int check_request_gates_and_missing_coordinates(void)
{
    CHECK(bucket_prefix() == 0); const uint16_t request = wire_request;
    CHECK(bucket_prepare(0) == 0);
    bucket_object(request, 1u, 9, 450u, 640u, 0); /* Before matching ACK. */
    wire_ack((uint16_t)(request - 1u), 2u, 0u);
    bucket_object((uint16_t)(request - 1u), 2u, 9, 550u, 640u, 0);
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
    host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped() && host_target_calls == 1);
    CHECK(bucket_wire_calls > 1u && wire_request == request && !bucket_wire_bad);
    wire_ack(request, 2u, 0u);
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
    bucket_object(request, 3u, 9, 550u, 640u, 1); /* Bad CRC must not move. */
    CHECK(stopped() && host_proto_stats.crc_bad == 1u);
    bucket_object(request, 4u, 8, 500u, 640u, 0); /* Target, not bucket. */
    CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    bucket_object(request, 5u, -1, 0u, 640u, 0);
    CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    bucket_object(request, 6u, 9, 494u, 640u, 0);
    CHECK(last_x == -50.0f && last_y == 0.0f && s_seq_state == SQ_BUCKET_ALIGN);
    bucket_object(request, 7u, 9, 506u, 640u, 0);
    CHECK(last_x == 50.0f && last_y == 0.0f && s_seq_state == SQ_BUCKET_ALIGN);
    bucket_object(request, 8u, -1, 0u, 640u, 0);
    CHECK(stopped());
    bucket_object(request, 9u, 9, 494u, 640u, 0);
    CHECK(last_x == -50.0f);
    host_tick += 300u; wire_poll(); CHECK(last_x == -50.0f);
    host_tick++; wire_poll(); CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    run_cmd("g");
    CHECK(stopped() && s_seq_state == SQ_STOPPED && !s_receiving && !s_due);
    puts("bucket36 real parser: 63/4/0 CRC request/retry, pre-ACK/old request/badCRC/wrong class/empty stop; no ACK or missing bucket never search; cx494 backward50/cx506 forward50, 300ms current and301ms stale brake passed");
    return 0;
}

static int check_independent_frames_and_still_confirmation(void)
{
    CHECK(bucket_prefix() == 0); CHECK(bucket_prepare(1) == 0);
    const uint16_t request = wire_request;
    bucket_object(request, 1u, 9, 495u, 640u, 0); CHECK(stopped());
    for (unsigned n = 0u; n < 8u; ++n) {
        host_tick += 200u; bucket_object(request, 1u, 9, 505u, 640u, 0);
        CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    }
    CHECK(host_proto_stats.duplicate == 8u);
    for (uint16_t seq = 2u; seq <= 6u; ++seq)
        bucket_object(request, seq, 9, seq & 1u ? 495u : 505u, 640u, 0);
    CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    for (uint16_t seq = 7u; seq <= 9u; ++seq) {
        host_tick += 250u; bucket_object(request, seq, 9, 500u, 640u, 0);
    }
    host_counts[2]++; /* One encoder changes before the one-second hold ends. */
    host_tick += 249u; bucket_object(request, 10u, 9, 500u, 640u, 0);
    CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    host_tick++; bucket_object(request, 11u, 9, 500u, 640u, 0);
    CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    for (uint16_t seq = 12u; seq <= 14u; ++seq) {
        host_tick += 250u; bucket_object(request, seq, 9, 500u, 640u, 0);
        CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    }
    host_tick += 249u; bucket_object(request, 15u, 9, 500u, 640u, 0);
    CHECK(stopped() && s_seq_state == SQ_MANUAL_D_WAIT && !s_receiving);

    CHECK(bucket_prefix() == 0); CHECK(bucket_prepare(1) == 0);
    for (uint16_t seq = 1u; seq <= 4u; ++seq)
        bucket_object(wire_request, seq, 9, 500u, 640u, 0);
    bucket_object(wire_request, 6u, 9, 500u, 640u, 0); /* Missing seq restarts five. */
    for (uint16_t seq = 7u; seq <= 9u; ++seq) {
        host_tick += 250u; bucket_object(wire_request, seq, 9, 500u, 640u, 0);
        CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    }
    bucket_object(wire_request, 10u, 9, 500u, 640u, 0);
    host_tick += 301u; wire_poll(); /* stale hold cannot finish */
    CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    for (uint16_t seq = 11u; seq <= 15u; ++seq)
        bucket_object(wire_request, seq, 9, 500u, 640u, 0);
    host_tick += 250u; bucket_object(wire_request, 16u, -1, 0u, 640u, 0);
    CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    host_tick += 1000u; wire_poll(); CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
    run_cmd("g");
    puts("bucket36 aim gate: inclusive495..505 brakes, duplicate packets cannot supply5; skipped seq/stale/empty invalidate aim; independent5+fresh1s with all four wheel counts still before manual distance wait passed");
    return 0;
}

static int check_distance_validation_and_active_locks(void)
{
    static const char *const bad_d[] = {"d", "d0", "d-1", "d20001", "d1.5", "d1x", "d999999999999999999"};
    static const char *const locked[] = {"31","34","35","36","17","v300","fff0","ykp2","kcx255","cc","co","su1500","n5"};
    CHECK(bucket_prefix() == 0); run_cmd("d1234");
    CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped() && strstr(last_message, "ERR") != NULL);
    run_cmd("g");
    CHECK(bucket_aim() == 0);
    for (unsigned n = 0u; n < sizeof bad_d / sizeof bad_d[0]; ++n) {
        run_cmd(bad_d[n]);
        CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped() && strstr(last_message, "ERR") != NULL);
    }
    for (unsigned n = 0u; n < sizeof locked / sizeof locked[0]; ++n) {
        run_cmd(locked[n]);
        CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped() && strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
    }
    static const char *const reports[] = {"?", "diag", "param", "route"};
    for (unsigned n = 0u; n < sizeof reports / sizeof reports[0]; ++n) {
        run_cmd(reports[n]); CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped());
    }
    run_cmd("d1"); CHECK(s_seq_state == SQ_STILL && stopped());
    CHECK(sequence_start_stage() == 0 && s_dist_target == -1.0f && s_v == 100.0f);
    run_cmd("d500"); CHECK(s_seq_state == SQ_RUN && s_dist_target == -1.0f);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    CHECK(bucket_aim() == 0); run_cmd("d20000");
    CHECK(sequence_start_stage() == 0 && s_dist_target == -20000.0f && s_v == 100.0f);
    run_cmd("g");
    /* No previous arbitrary manual-distance value may run automatically. */
    CHECK(wire_boot() == 0);
    proto_set_binary_tx(bucket_wire_tx); proto_set_on_frame(bucket_wire_on_frame);
    host_target_hook = real_proto_send_target;
    run_cmd("15"); run_cmd("d1234"); run_cmd("34"); run_cmd("g");
    s_seq_stage = ROUTE_TEST_ALIGN_STAGE; route_seq_prepare(); wire_sync();
    CHECK(bucket_prepare(1) == 0);
    for (uint16_t seq = 1u; seq <= 9u; ++seq) {
        if (seq > 5u) host_tick += 250u;
        bucket_object(wire_request, seq, 9, 500u, 640u, 0);
    }
    CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped());
    host_tick += 60000u; wire_poll(); CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped());
    run_cmd("g");
    puts("bucket36 d command: only manual wait accepts d1..d20000 and automatically prepares backward100; malformed/negative/zero/overflow/early/repeated-running writes rejected, all active-mode/actuator/tuning writes locked, old d not reused passed");
    return 0;
}

static int check_phase_cancellation_and_errors(void)
{
    static const char *const keys[] = {"g", "a", "0"};
    /* Cover all route translation/turn phases including both tail corners. */
    for (unsigned stage = 0u; stage < ROUTE_TEST_STAGES; ++stage) {
        if (stage == ROUTE_TEST_ALIGN_STAGE) continue;
        for (unsigned phase = 0u; phase < 4u; ++phase)
            for (unsigned key = 0u; key < 3u; ++key) {
                CHECK(bucket_boot() == 0); run_cmd("g");
                s_seq_stage = (uint8_t)stage; route_seq_prepare();
                if (stage == ROUTE_TEST_BACK_STAGE) run_cmd("d500");
                if (phase == 1u) {
                    host_tick += T_DIST_STILL_MS; wire_poll(); CHECK(s_seq_state == SQ_WAIT);
                } else if (phase >= 2u) CHECK(sequence_start_stage() == 0);
                if (phase == 3u) {
                    if (dist_mode()) {
                        if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
                        else host_fore = s_dist_odo0 + s_dist_target;
                    } else host_yaw = turn_target_deg();
                    wire_poll(); CHECK(s_round == R_BRAKE);
                }
                uint32_t run = s_seq_run;
                run_cmd(keys[key]); wire_sync();
                CHECK(s_seq_state == SQ_STOPPED && s_msel == 34 && stopped() && !s_receiving && !s_due);
                run_cmd("g"); run_cmd("d500"); host_tick += 60000u; wire_poll();
                CHECK(s_seq_state == SQ_STOPPED && s_seq_run == run && stopped());
            }
    }
    /* Standstill, preparation, seek, settling and the user distance gate. */
    for (unsigned phase = 0u; phase < 5u; ++phase)
        for (unsigned key = 0u; key < 3u; ++key) {
            CHECK(bucket_prefix() == 0);
            if (phase == 1u) { host_tick += T_DIST_STILL_MS; wire_poll(); }
            else if (phase >= 2u) CHECK(bucket_prepare(1) == 0);
            if (phase >= 3u) for (uint16_t seq = 1u; seq <= 5u; ++seq)
                bucket_object(wire_request, seq, 9, 500u, 640u, 0);
            if (phase == 4u) for (uint16_t seq = 6u; seq <= 9u; ++seq) {
                host_tick += 250u; bucket_object(wire_request, seq, 9, 500u, 640u, 0);
            }
            uint16_t old = wire_request; unsigned tx = bucket_wire_calls;
            run_cmd(keys[key]);
            CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving && !s_due);
            wire_ack(old, 2u, 0u); bucket_object(old, 10u, 9, 450u, 640u, 0);
            run_cmd("g"); run_cmd("d500"); host_tick += 60000u; wire_poll();
            CHECK(s_seq_state == SQ_STOPPED && stopped() && bucket_wire_calls == tx);
        }
    for (unsigned error = 0u; error < 3u; ++error) {
        CHECK(bucket_prefix() == 0); CHECK(bucket_prepare(1) == 0);
        if (error == 0u) wire_ack(wire_request, 2u, 1u);
        else if (error == 1u) host_imu_valid = 0;
        else host_abort = 1;
        wire_poll(); CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving && !s_due);
    }
    puts("bucket34 stop/error: g/a/0 stop14 motion nodes x4 phases, fivebucket phases inclWAIT_D; noresume/latecoordinates/autoreverse/laser; NACK/IMU/run-abort stop passed");
    return 0;
}

static unsigned bucket_atomic_kind;
static uint16_t bucket_atomic_seq;

static void bucket_atomic_interrupt(int motor)
{
    if (motor != 0) return;
    host_counts_hook = NULL;
    if (bucket_atomic_kind <= 3u) {
        int model = bucket_atomic_kind == 0u ? -1 : (bucket_atomic_kind == 2u ? 8 : 9);
        unsigned cx = bucket_atomic_kind == 1u ? 494u : 500u;
        bucket_object_with_poll(wire_request, ++bucket_atomic_seq, model, cx, 640u, 0, 0);
        real_proto_stats_get(&host_proto_stats);
    } else if (bucket_atomic_kind == 4u) host_imu_valid = 0;
    else if (bucket_atomic_kind == 5u) host_abort = 1;
    else {
        uint8_t nack[5] = {0x61u, (uint8_t)wire_request, (uint8_t)(wire_request >> 8), 2u, 1u};
        wire_feed_raw(nack, sizeof nack, 0);
        host_scene_status = real_proto_scene_status();
    }
}

static int check_atomic_transition_and_image_width(void)
{
    for (bucket_atomic_kind = 0u; bucket_atomic_kind < 7u; ++bucket_atomic_kind) {
        CHECK(bucket_prefix() == 0); CHECK(bucket_prepare(1) == 0);
        for (uint16_t seq = 1u; seq <= 5u; ++seq)
            bucket_object(wire_request, seq, 9, 500u, 640u, 0);
        for (uint16_t seq = 6u; seq <= 14u; ++seq) {
            host_tick += 100u; bucket_object(wire_request, seq, 9, 500u, 640u, 0);
        }
        host_tick += 99u; bucket_object(wire_request, 15u, 9, 500u, 640u, 0);
        CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
        bucket_atomic_seq = 16u; host_counts_hook = bucket_atomic_interrupt;
        host_tick++; bucket_object(wire_request, 16u, 9, 500u, 640u, 0);
        CHECK(!host_counts_hook && stopped() && s_seq_state == SQ_BUCKET_ALIGN && !laser_state);
        wire_poll();
        if (bucket_atomic_kind == 3u)
            CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped() && !s_receiving);
        else if (bucket_atomic_kind >= 4u)
            CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving);
        else {
            CHECK(s_seq_state == SQ_BUCKET_ALIGN && !laser_state);
            CHECK(bucket_atomic_kind == 1u ? last_x == -50.0f : stopped());
        }
        run_cmd("g");
    }
    for (unsigned width = 480u; width <= 506u; width += width == 480u ? 25u : 1u) {
        CHECK(bucket_prefix() == 0); CHECK(bucket_prepare(1) == 0);
        bucket_object(wire_request, 1u, 9, width == 480u ? 400u : 500u, width, 0);
        if (width <= 505u) {
            CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving);
            CHECK(strstr(last_message, "BUCKET_IMAGE_WIDTH") != NULL);
        } else CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
        run_cmd("g");
    }
    puts("bucket36 final snapshot: intervening empty/out-of-band/wrong-class/new-in-band/IMU/abort/NACK cannot accept old aim; next tick consumes new state;500px target rejects480/505-wide image while506 is valid passed");
    return 0;
}

#ifndef BUCKET36_FIXTURE_MAIN
#define BUCKET36_FIXTURE_MAIN main
#endif
int BUCKET36_FIXTURE_MAIN(void)
{
    CHECK(check_mode31_34_integrated_recipe_isolation() == 0);
    CHECK(check_nominal_field_vector_equivalence() == 0);
    CHECK(check_post_cross_heading_restart() == 0);
    CHECK(check_complete_recipe_and_distance_wait() == 0);
    CHECK(check_request_gates_and_missing_coordinates() == 0);
    CHECK(check_independent_frames_and_still_confirmation() == 0);
    CHECK(check_distance_validation_and_active_locks() == 0);
    CHECK(check_phase_cancellation_and_errors() == 0);
    CHECK(check_atomic_transition_and_image_width() == 0);
    puts("bucket36_route_test: all host checks passed; not physical angle, traction, UART or camera acceptance");
    return 0;
}
