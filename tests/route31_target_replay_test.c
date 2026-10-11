/* Host-only selected TARGET replay checks. Reuse the same real Bluetooth,
 * request/ACK/CRC parser and route owner fixture as route31_task_chain_test.
 * Time, encoders, IMU and actuators are injected, not physical acceptance.
 * Only test entry skips the already-covered roads to task2; SEEK/FIRE,
 * coordinates, accepted sequence history and good counts are never assigned.
 */
#define VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN target_replay_prior_xy_main
#include "vision_align_bluetooth_test.c"
#undef VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN

#undef CHECK
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "route31 target replay line %d: %s\n", __LINE__, #expr); \
    return 1; \
} } while (0)

static int replay_target_entry(int boot)
{
    if (boot) {
        CHECK(wire_boot() == 0);
        host_wire_diag_hook = real_proto_wire_diag_get;
    }
    run_cmd("31"); wire_sync();
    CHECK(s_seq_mode == 31u && s_seq_state == SQ_READY && stopped());
    CHECK(wire_mode == 1u);
    wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123");
    CHECK(proto_qr_get(NULL) && stopped());
    run_cmd("g");
    CHECK(s_seq_state == SQ_STILL && proto_qr_get(s_seq_qr));
    CHECK(s_seq_qr[0] == 1 && s_seq_qr[1] == 2 && s_seq_qr[2] == 3);
    /* Identical direct task-entry method to the task-chain cancellation tests.
     * The QR tuple is read from the real parser, not fabricated here. */
    step_vision_receive_end();
    s_seq_stage = ROUTE31_TARGET_STAGE;
    route_seq_prepare(); wire_sync();
    CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_TARGET_STAGE &&
          s_target35_route && s_target35_phase == TA_TASK_WAIT &&
          wire_opcode == 0x63u && wire_task == 2u && wire_digit == 2u &&
          !s_target35_sample.seen && !s_target35_good && stopped());
    return 0;
}

static int replay_target_seek(void)
{
    wire_ack(wire_request, 2u, 0u); xy_advance(1100u);
    CHECK(s_target35_phase == TA_SEEK && !s_target31_fine &&
          !s_target35_sample.seen && !s_target35_good && !laser_state &&
          last_x == route31_straight_speed());
    return 0;
}

static void replay_object_send(uint16_t sequence, int model, int x)
{
    xy_seq = sequence; host_tick += 20u;
    xy_obj(wire_request, model, x, 10);
}

static void replay_target_send(uint16_t sequence, int x)
{ replay_object_send(sequence, 8, x); }

static int replay_cache_unchanged(const VisionDiagSample *prior, float command)
{
    CHECK(s_target35_sample.frame.sequence == prior->frame.sequence &&
          s_target35_sample.frame.cx == prior->frame.cx &&
          s_target35_sample.tick == prior->tick &&
          s_target35_sample.packet_index == prior->packet_index &&
          s_target35_sample.seen == prior->seen);
    CHECK(last_x == command && last_y == 0.0f &&
          !laser_state && s_seq_state == SQ_TASK && s_target31_fine);
    return 0;
}

static int check_old_series_and_duplicates(void)
{
    VisionDiagSample accepted;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 360);
    replay_target_send(101u, 400);
    CHECK(s_target31_fine && s_target35_sequence == 101u &&
          last_x == 30.0f && !s_target35_good && !laser_state);
    accepted = s_target35_sample;
    /* Adjacent duplicate and later same-number replay must both be rejected
     * by the request's opt-in strict OBJECT sequence domain. */
    replay_target_send(101u, 100);
    CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 && !s_target35_good);
    replay_target_send(99u, 100);
    CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 && !s_target35_good);
    replay_target_send(101u, 100);
    CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 && !s_target35_good);
    /* This was the confirmed bug: old90..94 at the target used to produce
     * good1..5 and laserON despite having already consumed sequence101. */
    for (uint16_t sequence = 90u; sequence <= 94u; ++sequence) {
        replay_target_send(sequence, ROUTE31_TARGET_CX);
        CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 &&
              !s_target35_good && s_target35_sequence == 101u);
    }
    /* A genuine next frame still works; rejected packets neither poison
     * the accepted sequence base nor permanently disable valid aiming. */
    replay_target_send(102u, 100);
    CHECK(s_target35_sample.frame.sequence == 102u &&
          s_target35_sample.frame.cx == 100 && s_target35_sequence == 102u &&
          last_x == -30.0f && !s_target35_good && !laser_state);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 old99/same101/old90..94: no cache/tick refresh, reverse, good or laser; NEW102 remains accepted passed");
    return 0;
}

static int check_replay_cannot_supply_fifth_good_frame(void)
{
    VisionDiagSample accepted;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 360);
    for (uint16_t sequence = 101u; sequence <= 104u; ++sequence)
        replay_target_send(sequence, ROUTE31_TARGET_CX);
    CHECK(s_target35_good == 4u && !laser_state && last_x == 0.0f);
    accepted = s_target35_sample;
    replay_target_send(103u, ROUTE31_TARGET_CX);
    CHECK(s_target35_sample.tick == accepted.tick &&
          s_target35_sample.packet_index == accepted.packet_index &&
          s_target35_sample.frame.sequence == 104u &&
          s_target35_good == 4u && !laser_state);
    for (uint16_t sequence = 90u; sequence <= 94u; ++sequence) {
        replay_target_send(sequence, ROUTE31_TARGET_CX);
        CHECK(s_target35_sample.tick == accepted.tick &&
              s_target35_sample.frame.sequence == 104u &&
              s_target35_good == 4u && !laser_state);
    }
    /* Rejected transport cannot reset or add good frames. Exactly NEW105
     * supplies the fifth genuine consecutive image. */
    replay_target_send(105u, ROUTE31_TARGET_CX);
    CHECK(laser_state && s_target35_phase == TA_FIRE &&
          s_target35_good == T_TARGET35_GOOD_FRAMES &&
          s_target35_sequence == 105u);
    run_cmd("a"); CHECK(!laser_state && s_seq_state == SQ_STOPPED && stopped());
    xy_advance(4000u); CHECK(!laser_state && stopped());
    puts("TARGET31 replay cannot turn four valid images into five; genuine new images can fire, a cancels without relight passed");
    return 0;
}

static int check_empty_breaks_replay_lease(void)
{
    VisionDiagSample accepted;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 360); replay_target_send(101u, 100);
    CHECK(last_x == -30.0f && s_target35_sequence == 101u);
    accepted = s_target35_sample;
    /* A genuine empty image is NOT an ignored replay. It must retain the
     * established no-coordinate forward-search behavior immediately. */
    xy_seq = 102u; host_tick += 20u; xy_obj(wire_request, -1, 0, 0);
    CHECK(last_x == 30.0f && !s_target35_good && !laser_state);
    replay_target_send(101u, 100);
    CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 && !s_target35_good);
    for (uint16_t sequence = 90u; sequence <= 94u; ++sequence) {
        replay_target_send(sequence, ROUTE31_TARGET_CX);
        CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 && !s_target35_good);
    }
    /* A valid new selected coordinate, not a replay, can end empty search. */
    replay_target_send(103u, 100);
    CHECK(s_target35_sample.frame.sequence == 103u &&
          s_target35_sequence == 103u && last_x == -30.0f && !laser_state);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 realEMPTY102 breaks replay lease;old101/90..94 cannot revive reverse aim or fire;NEW103 restores valid reverse passed");
    return 0;
}

static int check_new_empty_blocks_intermediate_delayed_coordinates(void)
{
    VisionDiagSample accepted;
    uint32_t empty_packet;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 360); replay_target_send(101u, 100);
    CHECK(last_x == -30.0f && s_target35_sequence == 101u);
    accepted = s_target35_sample;
    /* This empty103 overtakes selected102 in transport. The sequence domain
     * must include empty images, not merely selected-coordinate callbacks. */
    replay_object_send(103u, -1, 0);
    CHECK(last_x == 30.0f && !s_target35_good && !laser_state);
    empty_packet = host_proto_stats.obj;
    for (uint16_t sequence = 100u; sequence <= 103u; ++sequence) {
        replay_target_send(sequence, sequence == 102u ? 100 : ROUTE31_TARGET_CX);
        CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 &&
              host_proto_stats.obj == empty_packet && !s_target35_good &&
              s_target35_sequence == 101u);
    }
    /* Repeated delayed102 after the original lease expires cannot revive it. */
    xy_advance(T_TARGET35_FRESH_MS + 20u);
    replay_target_send(102u, 100);
    CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 &&
          host_proto_stats.obj == empty_packet && !s_target35_good);
    replay_target_send(104u, 100);
    CHECK(s_target35_sample.frame.sequence == 104u &&
          s_target35_sequence == 104u && last_x == -30.0f &&
          host_proto_stats.obj == empty_packet + 1u && !laser_state);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 selected101->newEMPTY103:delayed100..103 including102 cannot cache/reverse/addgood/fire or countobj,even after300ms;NEW104 accepted passed");
    return 0;
}

static int check_old_empty_cannot_clear_latest_selected_image(void)
{
    VisionDiagSample accepted;
    uint32_t accepted_packet;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 360); replay_target_send(103u, 100);
    CHECK(last_x == -30.0f && s_target35_sequence == 103u);
    accepted = s_target35_sample; accepted_packet = host_proto_stats.obj;
    for (uint16_t sequence = 100u; sequence <= 103u; ++sequence) {
        replay_object_send(sequence, -1, 0);
        CHECK(replay_cache_unchanged(&accepted, -30.0f) == 0 &&
              host_proto_stats.obj == accepted_packet && !s_target35_good);
    }
    /* Old empty images cannot destroy four legitimate consecutive aim frames
     * either; the fifth NEW image, never an empty/replayed image, may fire. */
    for (uint16_t sequence = 104u; sequence <= 107u; ++sequence)
        replay_target_send(sequence, ROUTE31_TARGET_CX);
    CHECK(s_target35_good == 4u && !laser_state && last_x == 0.0f);
    accepted = s_target35_sample; accepted_packet = host_proto_stats.obj;
    replay_object_send(106u, -1, 0);
    CHECK(replay_cache_unchanged(&accepted, 0.0f) == 0 &&
          host_proto_stats.obj == accepted_packet && s_target35_good == 4u);
    replay_target_send(108u, ROUTE31_TARGET_CX);
    CHECK(laser_state && s_target35_phase == TA_FIRE &&
          s_target35_good == 5u && s_target35_sequence == 108u);
    run_cmd("a"); CHECK(!laser_state && s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 oldEMPTY100..103 cannot clear valid103/tick/packet or flipreverse;oldEMPTY106 leavesgood4,NEW108 alone suppliesfifth passed");
    return 0;
}

static int check_new_wrong_target_also_advances_object_sequence(void)
{
    VisionDiagSample accepted;
    uint32_t wrong_packet;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 360); replay_target_send(101u, 100);
    CHECK(last_x == -30.0f && s_target35_sequence == 101u);
    accepted = s_target35_sample;
    /* QR123 selected green8. A legitimate new red6 image has no selected
     * target, but its sequence still belongs to this OBJECT request. */
    replay_object_send(103u, 6, ROUTE31_TARGET_CX);
    CHECK(last_x == 30.0f && !s_target35_good && !laser_state);
    wrong_packet = host_proto_stats.obj;
    replay_target_send(102u, 100);
    replay_target_send(103u, ROUTE31_TARGET_CX);
    CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 &&
          host_proto_stats.obj == wrong_packet && !s_target35_good);
    replay_target_send(104u, 100);
    CHECK(s_target35_sample.frame.sequence == 104u &&
          s_target35_sequence == 104u && last_x == -30.0f && !laser_state);
    accepted = s_target35_sample; wrong_packet = host_proto_stats.obj;
    replay_object_send(103u, 6, 400);
    CHECK(replay_cache_unchanged(&accepted, -30.0f) == 0 &&
          host_proto_stats.obj == wrong_packet && !s_target35_good);
    run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 selected101->newWRONG_TARGET103 blocksdelayed102/same103;NEW104 restoresreverse andold wrongtarget103 cannot clear it passed");
    return 0;
}

static int check_invalid_high_sequence_and_guard_idempotence(void)
{
    VisionDiagSample accepted;
    uint32_t accepted_packet;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 360); replay_target_send(101u, 100);
    CHECK(last_x == -30.0f && !s_target35_good);
    accepted = s_target35_sample; accepted_packet = host_proto_stats.obj;
    CHECK(proto_target_sequence_guard_enable() && proto_target_sequence_guard_enable());
    replay_target_send(100u, ROUTE31_TARGET_CX);
    CHECK(replay_cache_unchanged(&accepted, -30.0f) == 0 &&
          host_proto_stats.obj == accepted_packet && !s_target35_good);
    /* img_w=640, so cx=640 is CRC-correct but geometrically invalid.
     * Its high sequence10000 cannot poison the next valid sequence102. */
    replay_target_send(10000u, 640);
    CHECK(replay_cache_unchanged(&accepted, -30.0f) == 0 &&
          host_proto_stats.obj == accepted_packet && !s_target35_good);
    replay_target_send(102u, 100);
    CHECK(s_target35_sample.frame.sequence == 102u &&
          s_target35_sequence == 102u && last_x == -30.0f && !laser_state);
    accepted = s_target35_sample;
    replay_object_send(103u, -1, 0); accepted_packet = host_proto_stats.obj;
    CHECK(last_x == 30.0f && !laser_state);
    CHECK(proto_target_sequence_guard_enable());
    replay_target_send(102u, 100);
    CHECK(replay_cache_unchanged(&accepted, 30.0f) == 0 &&
          host_proto_stats.obj == accepted_packet && !s_target35_good);
    replay_target_send(104u, 100);
    CHECK(s_target35_sample.frame.sequence == 104u &&
          s_target35_sequence == 104u && last_x == -30.0f && !laser_state);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 repeatedguardEnable doesnot reset requesthead;CRCvalid invalidgeom sequence10000 cannot poison NEW102;enableafterEMPTY103 keepslate102 blocked passed");
    return 0;
}

static int check_replay_cannot_extend_300ms_freshness(void)
{
    VisionDiagSample accepted;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 360); replay_target_send(101u, 100);
    CHECK(last_x == -30.0f && !s_target35_good);
    accepted = s_target35_sample;
    /* Alternate stale numbers so this is not merely the parser's old
     * adjacent-identical-sequence shortcut. Strict31 rejects both numbers. */
    for (unsigned i = 1u; i <= 20u; ++i) {
        replay_target_send((uint16_t)(i & 1u ? 99u : 98u), 400);
        CHECK((uint32_t)(host_tick - accepted.tick) == i * 20u);
        CHECK(replay_cache_unchanged(&accepted,
              i * 20u <= T_TARGET35_FRESH_MS ? -30.0f : 30.0f) == 0);
        CHECK(!s_target35_good && s_target35_sequence == 101u);
    }
    replay_target_send(102u, 100);
    CHECK(s_target35_sample.frame.sequence == 102u &&
          s_target35_sample.tick > accepted.tick && last_x == -30.0f && !laser_state);
    run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 replay chain retains only original age:300ms still valid,320ms forward search,no refresh/fire;NEW102 renews lease passed");
    return 0;
}

static unsigned replay_atomic_injected;

static void replay_old_irq_during_aim_brake(void)
{
    host_motion_brake_hook = NULL;
    replay_atomic_injected++;
    xy_seq = 102u;
    xy_obj_poll(wire_request, 8, 100, 10, 0);
}

static int check_final_atomic_snapshot_ignores_replay(void)
{
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 360);
    for (uint16_t sequence = 101u; sequence <= 104u; ++sequence)
        replay_target_send(sequence, ROUTE31_TARGET_CX);
    CHECK(s_target35_good == 4u && !laser_state);
    replay_atomic_injected = 0u;
    host_motion_brake_hook = replay_old_irq_during_aim_brake;
    uint32_t fifth_tick = host_tick + 20u;
    replay_target_send(105u, ROUTE31_TARGET_CX);
    CHECK(replay_atomic_injected == 1u && laser_state && s_target35_phase == TA_FIRE &&
          s_target35_good == 5u && s_target35_sequence == 105u &&
          s_target35_sample.frame.sequence == 105u &&
          s_target35_sample.frame.cx == ROUTE31_TARGET_CX &&
          s_target35_sample.tick == fifth_tick);
    run_cmd("a"); CHECK(!laser_state && s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 replay IRQ at fifth-frame brake/final-check seam:fires only unchanged genuine105 sample,old102 neither replaces nor adds good passed");
    return 0;
}

static int check_mid_yaw_clear_keeps_replay_barrier(void)
{
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(100u, 400);
    CHECK(!s_target31_fine && s_target31_coarse_sequence == 100u &&
          last_x == route31_straight_speed());
    host_yaw = s_target35_heading + 3.0f; wire_poll();
    CHECK(s_target31_mid_phase == 1u && !last_x && !last_y && !last_w);
    /* Selected coordinates really arrive while the chassis is correcting. */
    replay_target_send(101u, 390);
    CHECK(s_target35_sample.frame.sequence == 101u && !s_target31_fine);
    xy_advance(T_DIST_STILL_MS);
    CHECK(s_target31_mid_phase == 2u && last_w < 0.0f);
    host_yaw = s_target35_heading; wire_poll();
    CHECK(s_target31_mid_phase == 3u && !last_x && !last_y && !last_w);
    xy_advance(ROUTE31_STABLE_MS);
    CHECK(!s_target31_mid_phase && s_target31_mid_need_new &&
          s_target31_mid_guarded && !s_target35_sample.seen && !s_target35_good &&
          !s_target31_fine && last_x == route31_straight_speed());
    replay_target_send(100u, ROUTE31_TARGET_CX);
    replay_target_send(101u, ROUTE31_TARGET_CX);
    CHECK(!s_target35_sample.seen && s_target31_mid_need_new &&
          !s_target31_fine && !s_target35_good && !laser_state &&
          last_x == route31_straight_speed());
    replay_target_send(102u, ROUTE31_TARGET_CX);
    CHECK(s_target35_sample.frame.sequence == 102u && !s_target31_mid_need_new &&
          s_target31_fine && s_target35_sequence == 102u &&
          s_target35_good == 1u && !laser_state && last_x == 0.0f);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 mid-yaw accepted101 survives aim-cache clear;old100/same101 cannot unlock NEW-frame gate or fire;NEW102 alone suppliesgood1 passed");
    return 0;
}

static int check_u16_wrap_and_backward_direction(void)
{
    VisionDiagSample accepted;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(65534u, 360);
    replay_target_send(65535u, 400);
    CHECK(s_target35_sequence == 65535u && last_x == 30.0f);
    replay_target_send(0u, 100);
    CHECK(s_target35_sequence == 0u &&
          s_target35_sample.frame.sequence == 0u && last_x == -30.0f);
    accepted = s_target35_sample;
    replay_target_send(0u, 400);
    CHECK(replay_cache_unchanged(&accepted, -30.0f) == 0 && !s_target35_good);
    replay_target_send(65535u, 400);
    CHECK(replay_cache_unchanged(&accepted, -30.0f) == 0 && !s_target35_good);
    replay_target_send(65530u, ROUTE31_TARGET_CX);
    CHECK(replay_cache_unchanged(&accepted, -30.0f) == 0 && !s_target35_good);
    for (uint16_t sequence = 1u; sequence <= 5u; ++sequence) {
        replay_target_send(sequence, ROUTE31_TARGET_CX);
        CHECK(s_target35_sequence == sequence && s_target35_good == sequence &&
              laser_state == (sequence == 5u));
    }
    run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 u16:65535->0 accepted,0->65535 and same0 rejected without changing reverse command;NEW1..5 fire;0 cancels passed");
    return 0;
}

static int check_new_request_and_new_run_reset(void)
{
    uint16_t previous_request;
    unsigned previous_run;
    CHECK(replay_target_entry(1) == 0 && replay_target_seek() == 0);
    replay_target_send(500u, 360); replay_target_send(501u, 400);
    CHECK(s_target35_sequence == 501u && s_target31_fine);
    previous_request = wire_request;
    /* Real task2 re-entry issues a new request; no sample/guard write here. */
    route31_task_begin(); wire_sync();
    CHECK(wire_request != previous_request && s_target35_phase == TA_TASK_WAIT &&
          !s_target35_sample.seen && !s_target35_good && stopped());
    xy_seq = 502u; host_tick += 20u;
    xy_obj(previous_request, 8, ROUTE31_TARGET_CX, 10);
    CHECK(!s_target35_sample.seen && !laser_state && stopped());
    /* A preACK high sequence must not poison this new request's low1 start. */
    replay_target_send(10000u, ROUTE31_TARGET_CX);
    CHECK(!s_target35_sample.seen && !s_target35_good && !laser_state && stopped());
    CHECK(replay_target_seek() == 0);
    replay_target_send(1u, 360); replay_target_send(2u, 400);
    CHECK(s_target35_sequence == 2u && s_target31_fine && last_x == 30.0f);
    previous_request = wire_request; previous_run = s_seq_run;
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped()); /* No auto-resume. */
    CHECK(replay_target_entry(0) == 0 && s_seq_run == previous_run + 1u &&
          wire_request != previous_request);
    CHECK(replay_target_seek() == 0);
    xy_seq = 503u; host_tick += 20u;
    xy_obj(previous_request, 8, ROUTE31_TARGET_CX, 10);
    CHECK(!s_target35_sample.seen && !s_target35_good && !laser_state);
    replay_target_send(1u, 360); replay_target_send(2u, 100);
    CHECK(s_target35_sequence == 2u && s_target31_fine && last_x == -30.0f);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("TARGET31 new request/new31 run resets sequence domain;old request/preACK high10000 cannot cache/poison lowNEW1;stopped g does not resume passed");
    return 0;
}

int main(void)
{
    CHECK(check_old_series_and_duplicates() == 0);
    CHECK(check_replay_cannot_supply_fifth_good_frame() == 0);
    CHECK(check_empty_breaks_replay_lease() == 0);
    CHECK(check_new_empty_blocks_intermediate_delayed_coordinates() == 0);
    CHECK(check_old_empty_cannot_clear_latest_selected_image() == 0);
    CHECK(check_new_wrong_target_also_advances_object_sequence() == 0);
    CHECK(check_invalid_high_sequence_and_guard_idempotence() == 0);
    CHECK(check_replay_cannot_extend_300ms_freshness() == 0);
    CHECK(check_final_atomic_snapshot_ignores_replay() == 0);
    CHECK(check_mid_yaw_clear_keeps_replay_barrier() == 0);
    CHECK(check_u16_wrap_and_backward_direction() == 0);
    CHECK(check_new_request_and_new_run_reset() == 0);
    puts("route31_target_replay_test: real parser/software replay only;no change to motion parameters,300ms seen-loss or35/43;no physical acceptance");
    return 0;
}
