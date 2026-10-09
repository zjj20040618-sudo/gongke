/* Real mode31 nine-node road prefix and retained mode34 bucket route,
 * plus real QR53/63/61/62 parsing. Filename is the historical runner entry.
 * Host stubs only: no device is flashed, moved, or physically accepted. */
#define BUCKET36_FIXTURE_MAIN integrated_bucket_fixture_main
#include "bucket36_route_test.c"
#undef BUCKET36_FIXTURE_MAIN

static uint16_t integrated_qr_request;

static void integrated_qr53(uint16_t request, uint16_t sequence,
                            const char *tuple, int corrupt)
{
    uint8_t inner[7] = {0x53u, (uint8_t)sequence, (uint8_t)(sequence >> 8), 1u};
    uint8_t outer[12] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8), 7u, 0u};
    memcpy(inner + 4u, tuple, 3u); memcpy(outer + 5u, inner, sizeof inner);
    wire_feed(outer, sizeof outer, corrupt);
}

static int integrated_boot(unsigned mode, int ready_qr)
{
    CHECK(mode == 31u || mode == 34u);
    CHECK(wire_boot() == 0);
    integrated_qr_request = wire_request;
    memset(bucket_wire_packet, 0, sizeof bucket_wire_packet);
    bucket_wire_calls = bucket_wire_bad = 0u; bucket_wire_length = 0u;
    proto_set_binary_tx(bucket_wire_tx); proto_set_on_frame(bucket_wire_on_frame);
    host_target_hook = real_proto_send_target;
    run_cmd(mode == 31u ? "31" : "34"); wire_sync();
    CHECK(s_msel == (int)mode && s_seq_mode == mode && s_seq_state == SQ_READY && stopped());
    CHECK(route_seq_stage_count() == (mode == 31u ? 16u : 15u) && !host_target_calls);
    if (mode == 31u) {
        CHECK(s_receiving && !proto_qr_get(NULL));
        if (ready_qr) {
            wire_ack(integrated_qr_request, 1u, 0u);
            integrated_qr53(integrated_qr_request, 1u, "331", 0);
            CHECK(proto_qr_get(NULL) && notice_calls == 1u && stopped());
        }
    } else {
        CHECK(!s_receiving && !s_due && !proto_qr_get(NULL));
        wire_ack(integrated_qr_request, 1u, 0u);
        integrated_qr53(integrated_qr_request, 1u, "331", 0);
        CHECK(!proto_qr_get(NULL) && !notice_calls && stopped());
    }
    return 0;
}

static int integrated_prefix(unsigned mode)
{
    static const int modes[7] = {17,16,20,16,15,16,30};
    static const int commands[7] = {-530,-650,90,-650,80,-190,-92};
    static const float speeds[7] = {100,100,100,300,20,100,100};
    CHECK(mode == 34u);
    CHECK(integrated_boot(mode, 1) == 0); run_cmd("g");
    CHECK(s_seq_state == SQ_STILL && s_seq_run == 1u && stopped());
    for (unsigned stage = 0u; stage < 7u; ++stage) {
        CHECK(s_seq_stage == stage && sequence_start_stage() == 0);
        CHECK(s_seq_mode == mode && s_msel == modes[stage]);
        CHECK(s_route_test_plan[stage].mode == modes[stage]);
        CHECK(s_route_test_plan[stage].heading_hold == (stage == 3u || stage == 4u ? 0u : 1u));
        if (dist_mode()) {
            CHECK(s_dist_target == commands[stage] && s_v == speeds[stage] && s_dist_precise);
            CHECK(s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : 0.3f));
            CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f);
            CHECK(s_dist_ff_ratio == (stage == 1u || stage == 5u ? 0.00625f : 0.0f));
        } else CHECK(turn_target_deg() == commands[stage]);
        CHECK(!host_target_calls && !bucket_wire_calls);
        CHECK(sequence_finish_stage() == 0); wire_sync();
        CHECK(stopped() && !pulse_calls && !servo_calls && !laser_state && !s_go);
        if (stage < 6u) CHECK(!s_receiving && !s_due);
    }
    CHECK(s_seq_stage == ROUTE_TEST_ALIGN_STAGE && s_seq_state == SQ_BUCKET_ALIGN);
    CHECK(s_msel == (int)mode && host_target_calls == 1 && s_receiving);
    CHECK(host_target_task == PROTO_TASK_BUCKET && host_target_digit == 0u);
    CHECK(wire_request != integrated_qr_request && wire_mode == 2u);
    CHECK(bucket_wire_length == 9u && bucket_wire_packet[2] == 0x63u);
    CHECK(bucket_wire_packet[5] == 4u && bucket_wire_packet[6] == 0u && !bucket_wire_bad);
    CHECK(notice_calls == (mode == 31u ? 1u : 0u));
    CHECK(s_seq_qr[0] == (mode == 31u ? 3 : 0) && !proto_qr_get(NULL));
    return 0;
}

static int integrated_aim_current(void)
{
    CHECK(bucket_prepare(1) == 0);
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

static int check_route31_qr53_road_prefix_to_ball(void)
{
    static const int modes[9] = {17,16,20,16,15,16,18,16,30};
    static const int commands[9] = {-535,-630,90,-650,0,-190,800,-760,-90};
    static const float speeds[9] = {250,200,100,300,40,200,250,200,100};
    CHECK(integrated_boot(31u, 1) == 0);
    CHECK(s_seq_state == SQ_READY && stopped() && notice_calls == 1u && proto_qr_get(NULL));
    uint16_t qr_request = wire_request;
    unsigned qr_commands = wire_commands;
    run_cmd("g");
    for (unsigned stage = 0u; stage < 9u; ++stage) {
        CHECK(s_seq_stage == stage && s_seq_state == (stage == 8u ? SQ_ARM_PREP : SQ_STILL) && sequence_start_stage() == 0);
        CHECK(route_seq_leg() == &s_route31_plan[stage] && s_msel == modes[stage] && s_seq_mode == 31u);
        CHECK(!route_seq_bucket_enabled() && !host_target_calls && !bucket_wire_calls);
        if (dist_mode()) {
            CHECK(s_dist_target == commands[stage] && s_v == speeds[stage] && s_dist_precise);
            CHECK(s_dist_align_enabled == (stage == 3u || stage == 4u ? 0u : 1u));
            CHECK(s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : stage == 6u ? 3.0f : 0.3f));
            CHECK(s_dist_ff_ratio == (stage == 1u || stage == 5u || stage == 7u ? 0.00625f : 0.0f));
            CHECK(stage == 4u ? last_x == 40.0f && last_y == 0.0f :
                  dist_lateral() ? last_y * commands[stage] > 0.0f : last_x * commands[stage] > 0.0f);
        } else {
            CHECK(turn_target_deg() == commands[stage]);
            CHECK(last_w * commands[stage] > 0.0f);
        }
        CHECK(sequence_finish_stage() == 0); wire_sync();
        CHECK(s_seq_state == (stage == 8u ? SQ_TASK : stage == 7u ? SQ_ARM_PREP : SQ_STILL));
        CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        if (stage != 7u) CHECK(stopped());
        CHECK(s_seq_state != SQ_BUCKET_ALIGN && s_seq_state != SQ_MANUAL_D_WAIT);
        if (stage < 8u)
            CHECK(!host_target_calls && !bucket_wire_calls && wire_request == qr_request && wire_commands == qr_commands);
        CHECK(!laser_state && !host_laser_on_calls && pulse_calls == (stage == 8u ? 3200 : 0) && !servo_calls && !s_go);
    }
    CHECK(s_seq_qr[0] == 3 && s_seq_qr[1] == 3 && s_seq_qr[2] == 1 && notice_calls == 1u);
    CHECK(s_receiving && !s_bucket36_back_mm && s_seq_stage == 9u && s_msel == 31);
    CHECK(host_target_calls == 1 && bucket_wire_calls == 1u && host_target_task == PROTO_TASK_BALL && host_target_digit == 3u);
    CHECK(!proto_qr_get(NULL)); /* route factory uses the validated R1 snapshot */
    run_cmd("g"); run_cmd("d500"); host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving);
    puts("integrated31 realQR53: early331 stays through roadprefix, R2back630/tilt-contact-v40/right800/back760/left90; after nine road actions ball request starts from latched QR, no old2450/2125 road-only tail passed");
    return 0;
}

static int check_route31_all_motion_node_cancellations(void)
{
    static const char *const keys[] = {"g", "a", "0"};
    for (unsigned stage = 0u; stage < 9u; ++stage)
        for (unsigned phase = 0u; phase < 4u; ++phase)
            for (unsigned key = 0u; key < 3u; ++key) {
                CHECK(integrated_boot(31u, 1) == 0); run_cmd("g");
                s_seq_stage = (uint8_t)stage; route_seq_prepare();
                if (phase >= 1u) CHECK(fixture_complete_route31_predeploy() == 0);
                if (phase == 1u) {
                    host_tick += T_DIST_STILL_MS; wire_poll(); CHECK(s_seq_state == SQ_WAIT);
                } else if (phase >= 2u) CHECK(sequence_start_stage() == 0);
                if (phase == 3u) {
                    if (route31_owner() && s_seq_stage == 4u) {
                        CHECK(fixture_trigger_board_contact() == 0);
                    } else if (dist_mode()) {
                        if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
                        else host_fore = s_dist_odo0 + s_dist_target;
                    } else host_yaw = turn_target_deg();
                    wire_poll(); CHECK(s_round == R_BRAKE);
                }
                unsigned commands = wire_commands;
                uint32_t run = s_seq_run;
                run_cmd(keys[key]); wire_sync();
                CHECK(s_seq_state == SQ_STOPPED && s_msel == 31 && stopped() && !s_receiving && !s_due);
                CHECK(!host_target_calls && !bucket_wire_calls && !s_bucket36_back_mm);
                integrated_qr53(integrated_qr_request, 2u, "123", 0);
                bucket_object(integrated_qr_request, 1u, 9, 500u, 640u, 0);
                run_cmd("g"); run_cmd("d500"); host_tick += 60000u; wire_poll();
                CHECK(s_seq_state == SQ_STOPPED && s_seq_run == run && stopped());
                CHECK(wire_commands == commands && !host_target_calls && !bucket_wire_calls);
            }
    puts("integrated31 cancellation:108 g/a/0 across9 roadprefix nodes x4phases; lateQR/object cannot drive orresume passed");
    return 0;
}

static int check_route31_turn_target_scope(void)
{
    CHECK(integrated_boot(31u, 1) == 0); run_cmd("g");
    static const unsigned stages[] = {2u, 8u, 13u};
    for (unsigned n = 0u; n < 3u; ++n) {
        s_seq_stage = (uint8_t)stages[n]; route_seq_prepare();
        CHECK(sequence_start_stage() == 0);
        CHECK(turn_target_deg() == (stages[n] == 8u ? -90.0f : 90.0f));
        CHECK(strstr(host_messages, "comp=0deg") != NULL);
    }
    run_cmd("g"); run_cmd("30"); run_cmd("g");
    CHECK(s_seq_state == SQ_OFF && turn_target_deg() == -92.0f && strstr(last_message, "comp=2deg") != NULL);
    reset_fixture(); run_cmd("22"); run_cmd("g");
    CHECK(turn_target_deg() == 180.0f && s_seq_state == SQ_OFF);
    CHECK(wire_boot() == 0); run_cmd("36"); run_cmd("g");
    s_seq_stage = 8u; route_seq_prepare(); CHECK(sequence_start_stage() == 0);
    CHECK(s_msel == 30 && turn_target_deg() == -92.0f);
    run_cmd("g");
    puts("turn scope:31 active route right90/left90 with0comp; standalone30 and36 keep calibratedleft92,22 keeps180 passed");
    return 0;
}

static int check_fifteen_nodes_and_manual_distance(void)
{
    static const int modes[7] = {16,20,15,20,15,20,15};
    static const int commands[7] = {-730,90,780,90,2450,90,2125};
    {
        const unsigned mode = 34u;
        CHECK(integrated_prefix(mode) == 0); CHECK(integrated_aim_current() == 0);
        CHECK(strstr(last_message, mode == 31u ? "BUCKET31_WAIT_D" : "BUCKET34_WAIT_D") != NULL);
        unsigned requests = bucket_wire_calls; uint16_t old_bucket = wire_request;
        host_tick += 60000u; wire_poll();
        CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped());
        CHECK(bucket_wire_calls == requests && s_bucket36_back_mm == 0u);
        bucket_object(old_bucket, 10u, 9, 550u, 640u, 0);
        integrated_qr53(integrated_qr_request, 2u, "123", 0);
        CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped() && !s_receiving);
        run_cmd("d730"); CHECK(s_seq_state == SQ_STILL && stopped());
        for (unsigned n = 0u; n < 7u; ++n) {
            CHECK(s_seq_stage == n + 8u && sequence_start_stage() == 0);
            CHECK(s_msel == modes[n] && s_seq_mode == mode);
            if (dist_mode()) CHECK(s_dist_target == commands[n] && s_v == 100.0f && s_dist_precise);
            else CHECK(turn_target_deg() == commands[n]);
            CHECK(sequence_finish_stage() == 0); wire_sync();
            CHECK(stopped() && !s_receiving && !s_due && bucket_wire_calls == requests);
            CHECK(!laser_state && !host_laser_on_calls && !pulse_calls && !servo_calls && !s_go);
        }
        CHECK(s_seq_state == SQ_DONE && s_seq_stage == 14u && s_msel == (int)mode && s_round == R_DONE);
        CHECK(s_bucket36_back_mm == 0u);
        uint32_t run = s_seq_run;
        run_cmd("g"); run_cmd("d500"); host_tick += 60000u; wire_poll();
        CHECK(s_seq_state == SQ_DONE && s_seq_run == run && stopped() && bucket_wire_calls == requests);
        CHECK(strstr(host_messages, "fwd200") == NULL);
    }
    puts("retained34 exact15: left530/back650/right90/back650v300/forward80v20/back190/left92 -> bucket4/0 -> NEW d back100 -> right90/forward780/right90/whole2450/right90/whole2125; no extra200, laser, arm, STOP/IDLE or automatic return passed");
    return 0;
}

static int check_real_qr_gate_and_request_separation(void)
{
    CHECK(integrated_boot(31u, 0) == 0); run_cmd("g");
    CHECK(sequence_start_stage() == 0);
    host_lateral = s_dist_odo0 + s_dist_target; wire_poll();
    CHECK(s_round == R_BRAKE); host_tick += T_DIST_STILL_MS; wire_poll();
    fixture_complete_distance_alignment(); wire_sync();
    CHECK(s_seq_state == SQ_QR_WAIT && s_seq_stage == 0u && stopped());
    integrated_qr53(integrated_qr_request, 1u, "331", 0); /* No matching ACK yet. */
    wire_ack((uint16_t)(integrated_qr_request + 1u), 1u, 0u);
    integrated_qr53((uint16_t)(integrated_qr_request + 1u), 2u, "331", 0);
    CHECK(!notice_calls && s_seq_state == SQ_QR_WAIT && stopped());
    wire_ack(integrated_qr_request, 1u, 0u);
    integrated_qr53(integrated_qr_request, 3u, "330", 0);
    integrated_qr53(integrated_qr_request, 4u, "331", 1);
    host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_QR_WAIT && stopped() && !notice_calls);
    integrated_qr53(integrated_qr_request, 5u, "331", 0);
    CHECK(s_seq_stage == 1u && s_seq_state == SQ_STILL && stopped() && notice_calls == 1u);
    CHECK(s_seq_qr[0] == 3 && s_seq_qr[1] == 3 && s_seq_qr[2] == 1 && !s_receiving);
    unsigned qr_tx = bucket_wire_calls; /* No-ACK QR may have retried before release. */
    for (unsigned stage = 1u; stage <= 6u; ++stage) {
        CHECK(s_seq_stage == stage && sequence_start_stage() == 0);
        CHECK(sequence_finish_stage() == 0); wire_sync();
    }
    CHECK(s_seq_stage == 7u && s_seq_state == SQ_STILL && !route_seq_bucket_enabled());
    CHECK(wire_request == integrated_qr_request && bucket_wire_calls == qr_tx && !host_target_calls && !s_receiving);
    wire_ack(integrated_qr_request, 1u, 0u);
    integrated_qr53(integrated_qr_request, 6u, "123", 0);
    bucket_object(integrated_qr_request, 1u, 9, 550u, 640u, 0);
    CHECK(stopped() && s_seq_state == SQ_STILL && notice_calls == 1u && !s_bucket36_sample.seen);
    run_cmd("g"); unsigned requests = bucket_wire_calls;
    wire_ack(integrated_qr_request, 2u, 0u); bucket_object(integrated_qr_request, 4u, 9, 506u, 640u, 0);
    integrated_qr53(integrated_qr_request, 7u, "123", 0);
    CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving && bucket_wire_calls == requests);
    puts("integrated31 QR53: R1 waits indefinitely until matching61 + legal complete331; preACK/wrongrequest/badCRC/illegal rejected; afterright800 no bucket63 or WAIT_D; locallyclosed oldQR/bucket packets cannot drive; cancel drains late packets without STOP/IDLE passed");
    return 0;
}

static int check_missing_coordinates_and_new_frame_confirmation(void)
{
    {
        const unsigned mode = 34u;
        CHECK(integrated_prefix(mode) == 0); CHECK(bucket_prepare(0) == 0);
        uint16_t request = wire_request;
        bucket_object(request, 1u, 9, 550u, 640u, 0);
        host_tick += 60000u; wire_poll();
        CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped() && wire_request == request);
        wire_ack(request, 2u, 0u);
        host_tick += 60000u; wire_poll(); CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
        bucket_object(request, 2u, 9, 494u, 640u, 0); CHECK(last_x == -50.0f && last_y == 0.0f);
        bucket_object(request, 3u, 9, 506u, 640u, 0); CHECK(last_x == 50.0f && last_y == 0.0f);
        bucket_object(request, 4u, -1, 0u, 640u, 0); CHECK(stopped());
        bucket_object(request, 5u, 8, 500u, 640u, 0); CHECK(stopped());
        bucket_object(request, 6u, 9, 494u, 640u, 1); CHECK(stopped());
        bucket_object(request, 7u, 9, 494u, 640u, 0); CHECK(last_x == -50.0f);
        host_tick += 301u; wire_poll(); CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
        bucket_object(request, 8u, 9, 495u, 640u, 0);
        for (unsigned repeat = 0u; repeat < 6u; ++repeat) {
            host_tick += 200u; bucket_object(request, 8u, 9, 500u, 640u, 0);
            CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
        }
        for (uint16_t seq = 9u; seq <= 13u; ++seq)
            bucket_object(request, seq, 9, seq & 1u ? 495u : 505u, 640u, 0);
        for (uint16_t seq = 14u; seq <= 16u; ++seq) {
            host_tick += 250u; bucket_object(request, seq, 9, 500u, 640u, 0);
        }
        host_counts[1]++; host_tick += 249u; bucket_object(request, 17u, 9, 500u, 640u, 0);
        CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
        for (uint16_t seq = 18u; seq <= 21u; ++seq) {
            host_tick += 250u; bucket_object(request, seq, 9, 500u, 640u, 0);
            if (seq < 21u) CHECK(s_seq_state == SQ_BUCKET_ALIGN);
        }
        CHECK(s_seq_stage == 8u && s_seq_state == SQ_MANUAL_D_WAIT && stopped() && !s_receiving);
        run_cmd("g");
    }
    puts("retained34 aim: no ACK/no coordinates stays stopped; ID9 only/CRC/freshness, cx494 back50/cx506 forward50, duplicate seq cannot count five; inclusive band and full1s wheel-still gate reset by wheel motion passed");
    return 0;
}

static int check_distance_gate_restart_and_errors(void)
{
    static const char *const bad_d[] = {"d", "d0", "d-1", "d20001", "d1.5", "d1x", "d999999999999999999"};
    static const char *const locked[] = {"31","34","35","36","37","15","v600","ykp2","fff0","cc","co","su1500","n5"};
    {
        const unsigned mode = 34u;
        CHECK(integrated_boot(mode, 1) == 0); run_cmd("d1234");
        CHECK(s_seq_state == SQ_READY && stopped() && strstr(last_message, "ERR") != NULL);
        run_cmd("g"); s_seq_stage = ROUTE_TEST_ALIGN_STAGE; route_seq_prepare(); wire_sync();
        run_cmd("d1234"); CHECK(s_seq_state == SQ_BUCKET_ALIGN && stopped());
        CHECK(integrated_aim_current() == 0);
        for (unsigned n = 0u; n < sizeof bad_d / sizeof bad_d[0]; ++n) {
            run_cmd(bad_d[n]); CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped());
            CHECK(strstr(last_message, "ERR") != NULL && s_bucket36_back_mm == 0u);
        }
        for (unsigned n = 0u; n < sizeof locked / sizeof locked[0]; ++n) {
            run_cmd(locked[n]); CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped());
            CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
        }
        run_cmd("d731"); CHECK(s_seq_state == SQ_STILL && s_d == 731.0f && stopped());
        CHECK(sequence_start_stage() == 0 && s_msel == 16 && s_dist_target == -731.0f && s_v == 100.0f);
        run_cmd("d800"); CHECK(s_dist_target == -731.0f && s_seq_state == SQ_RUN);
        uint16_t old_bucket = wire_request; unsigned requests = bucket_wire_calls;
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped() && s_bucket36_back_mm == 0u);
        run_cmd(mode == 31u ? "31" : "34"); wire_sync();
        CHECK(s_seq_state == SQ_READY && s_bucket36_back_mm == 0u && stopped());
        if (mode == 31u) {
            CHECK(wire_request != old_bucket && wire_mode == 1u && !proto_qr_get(NULL));
            integrated_qr53(integrated_qr_request, 10u, "331", 0); CHECK(!proto_qr_get(NULL));
        }
        run_cmd("g"); s_seq_stage = ROUTE_TEST_ALIGN_STAGE; route_seq_prepare(); wire_sync();
        CHECK(wire_request != old_bucket && s_seq_state == SQ_BUCKET_ALIGN);
        CHECK(bucket_prepare(0) == 0);
        wire_ack(old_bucket, 2u, 0u); bucket_object(old_bucket, 30u, 9, 550u, 640u, 0);
        CHECK(stopped() && s_seq_state == SQ_BUCKET_ALIGN);
        CHECK(integrated_aim_current() == 0);
        host_tick += 60000u; wire_poll(); CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped());
        CHECK(s_bucket36_back_mm == 0u); run_cmd("g");
        CHECK(bucket_wire_calls >= requests && !bucket_wire_bad);
        for (unsigned error = 0u; error < 3u; ++error) {
            CHECK(integrated_prefix(mode) == 0); CHECK(bucket_prepare(1) == 0);
            if (error == 0u) wire_ack(wire_request, 2u, 1u);
            else if (error == 1u) host_imu_valid = 0;
            else host_abort = 1;
            wire_poll(); CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving && !s_due);
        }
    }
    puts("retained34 d gate: positive NEW d only in WAIT_D starts backwardv100 automatically; malformed/early/running writes locked; cancellation/reselection clears oldd and oldQR/bucket requests; NACK/IMU/abort stop passed");
    return 0;
}

static int check_all_fifteen_stage_cancellations(void)
{
    static const char *const keys[] = {"g","a","0"};
    {
        const unsigned mode = 34u;
        for (unsigned stage = 0u; stage < 15u; ++stage) {
            if (stage == ROUTE_TEST_ALIGN_STAGE) continue;
            for (unsigned phase = 0u; phase < 4u; ++phase)
                for (unsigned key = 0u; key < 3u; ++key) {
                    CHECK(integrated_boot(mode, 1) == 0); run_cmd("g");
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
                    uint32_t run = s_seq_run; unsigned requests = bucket_wire_calls;
                    run_cmd(keys[key]); wire_sync();
                    CHECK(s_seq_state == SQ_STOPPED && s_msel == (int)mode && stopped());
                    CHECK(!s_receiving && !s_due && s_bucket36_back_mm == 0u);
                    run_cmd("g"); run_cmd("d500"); host_tick += 60000u; wire_poll();
                    CHECK(s_seq_state == SQ_STOPPED && s_seq_run == run && stopped() && bucket_wire_calls == requests);
                }
        }
        for (unsigned phase = 0u; phase < 5u; ++phase)
            for (unsigned key = 0u; key < 3u; ++key) {
                CHECK(integrated_prefix(mode) == 0);
                if (phase == 1u) { host_tick += T_DIST_STILL_MS; wire_poll(); }
                else if (phase >= 2u) CHECK(bucket_prepare(1) == 0);
                if (phase >= 3u) for (uint16_t seq = 1u; seq <= 5u; ++seq)
                    bucket_object(wire_request, seq, 9, 500u, 640u, 0);
                if (phase == 4u) for (uint16_t seq = 6u; seq <= 9u; ++seq) {
                    host_tick += 250u; bucket_object(wire_request, seq, 9, 500u, 640u, 0);
                }
                uint16_t request = wire_request; unsigned requests = bucket_wire_calls;
                run_cmd(keys[key]); wire_sync();
                CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving && !s_due);
                wire_ack(request, 2u, 0u); bucket_object(request, 10u, 9, 550u, 640u, 0);
                run_cmd("g"); run_cmd("d500"); host_tick += 60000u; wire_poll();
                CHECK(s_seq_state == SQ_STOPPED && stopped() && bucket_wire_calls == requests);
            }
    }
    puts("retained34 cancellation:168 g/a/0 across14 motion nodes x4 phases plus15 bucket prepare/seek/settle/manual-wait cases; all15 nodes stop, cleard/localRX, reject late coordinates and do not resume or send IDLE passed");
    return 0;
}

int main(void)
{
    CHECK(check_mode31_34_integrated_recipe_isolation() == 0);
    CHECK(check_route31_qr53_road_prefix_to_ball() == 0);
    CHECK(check_route31_all_motion_node_cancellations() == 0);
    CHECK(check_route31_turn_target_scope() == 0);
    CHECK(check_fifteen_nodes_and_manual_distance() == 0);
    CHECK(check_real_qr_gate_and_request_separation() == 0);
    CHECK(check_missing_coordinates_and_new_frame_confirmation() == 0);
    CHECK(check_distance_gate_restart_and_errors() == 0);
    CHECK(check_all_fifteen_stage_cancellations() == 0);
    puts("route31_bucket_integration_test: all host checks passed; not physical distance, obstacle, UART, camera, aim or road-projection acceptance");
    return 0;
}
