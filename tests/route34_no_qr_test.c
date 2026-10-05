/* Mode34 uses the real test.c router and real proto.c parser from this fixture.
 * Only peripheral motion/time are stubbed; no hardware is driven or flashed. */
#define ROUTE31_QR_FIXTURE_MAIN route31_qr_suite_main
#include "route31_qr_gate_test.c"
#include <math.h>

static int route34_near(float a, float b)
{ return fabsf(a - b) < 0.000001f; }

static int check_complete_recipe_without_qr(void)
{
    static const int modes[15] = {17,16,20,16,15,16,30,0,16,20,15,20,15,20,15};
    static const int commands[15] = {-530,-650,90,-650,80,-190,-92,0,-730,90,780,90,2450,90,2125};
    static const float speeds[15] = {100,100,100,300,20,100,100,0,100,100,100,100,100,100,100};
    CHECK(ROUTE_TEST_STAGES == 15u && ROUTE_TEST_ALIGN_STAGE == 7u && ROUTE_TEST_BACK_STAGE == 8u);
    CHECK(wire_boot() == 0);
    unsigned background_commands = wire_commands;
    run_cmd("34");
    CHECK(s_msel == 34 && s_seq_mode == 34 && s_seq_state == SQ_READY && stopped());
    CHECK(!s_receiving && !s_due && !proto_qr_get(NULL));
    run_cmd("fff0.01000"); run_cmd("ykp1.2");
    CHECK(route34_near(s_route_forward_ff_ratio, 0.01f) && route34_near(s_route_heading_kp, 1.2f));
    CHECK(test_forward_ff_ratio() == 0.0125f && step_heading_kp_deg() == 0.3f);
    host_messages[0] = '\0';
    run_cmd("g");
    CHECK(strstr(host_messages, "ROUTE_PROFILE mode=34 source=LOCAL") != NULL);
    CHECK(strstr(host_messages, "source=ROUTE34 global_ykp=0.300") != NULL);
    CHECK(s_seq_run == 1u && s_seq_state == SQ_STILL && stopped());
    for (unsigned stage = 0u; stage < ROUTE_TEST_STAGES; ++stage) {
        if (stage == ROUTE_TEST_ALIGN_STAGE) {
            CHECK(s_route_test_plan[stage].mode == 0u && s_route_test_plan[stage].heading_hold == 1u);
            CHECK(route_wire_bucket_wait_d() == 0);
            CHECK(!notice_calls && !s_seq_qr[0] && !laser_state && !pulse_calls && !servo_calls);
            continue;
        }
        if (stage == ROUTE_TEST_BACK_STAGE) {
            CHECK(s_seq_state == SQ_MANUAL_D_WAIT && stopped());
            run_cmd("d730");
            CHECK(s_seq_state == SQ_STILL && s_bucket36_back_mm == 730u && stopped());
        }
        CHECK(s_seq_stage == stage && s_seq_mode == 34 && sequence_start_stage() == 0);
        CHECK(s_msel == modes[stage] && s_seq_state != SQ_QR_WAIT);
        CHECK(s_route_test_plan[stage].heading_hold == (stage == 3u || stage == 4u ? 0u : 1u));
        if (dist_mode()) {
            const float expected_kp = stage == 3u || stage == 4u ? 0.0f : 1.2f;
            CHECK(s_dist_target == commands[stage] && s_v == speeds[stage]);
            CHECK(s_dist_precise && route34_near(s_dist_heading_kp, expected_kp));
            CHECK(route34_near(s_route_heading_kp, 1.2f) && step_heading_kp_deg() == 0.3f);
            CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f);
            CHECK(route34_near(s_dist_ff_ratio, stage == 10u || stage == 12u || stage == 14u ? 0.01f :
                  stage == 1u || stage == 5u || stage == 8u ? 0.00625f : 0.0f));
            unsigned precise_before = host_precise_calls, integer_before = host_integer_calls;
            host_yaw = 3.0f; tick();
            CHECK(route34_near(last_w, -expected_kp * 3.0f * 0.0174533f) &&
                  host_precise_calls == precise_before + 1u && host_integer_calls == integer_before);
            host_yaw = -3.0f; tick();
            CHECK(route34_near(last_w, expected_kp * 3.0f * 0.0174533f));
            host_yaw = 0.0f;
        } else {
            CHECK(turn_target_deg() == commands[stage]);
        }
        CHECK(sequence_finish_stage() == 0);
        wire_sync();
        CHECK(s_seq_state != SQ_QR_WAIT && wire_commands == background_commands + (stage >= 6u ? 1u : 0u));
        CHECK(!notice_calls && !s_seq_qr[0]);
        CHECK(s_receiving == (stage == 6u) && !s_due);
        CHECK(s_seq_state == (stage == 14u ? SQ_DONE : stage == 6u ? SQ_BUCKET_ALIGN : SQ_STILL));
        CHECK(!pulse_calls && !servo_calls && !laser_state && !s_go);
    }
    CHECK(s_msel == 34 && s_round == R_DONE && s_seq_state == SQ_DONE);
    CHECK(strstr(host_messages, "QR_WAIT") == NULL && !host_abort);
    uint32_t run_number = s_seq_run;
    run_cmd("g"); host_tick += 10000u; wire_poll();
    CHECK(stopped() && s_seq_state == SQ_DONE && s_seq_run == run_number);
    CHECK(strstr(last_message, "select34_then_g_to_rerun") != NULL);
    run_cmd("34"); run_cmd("g"); wire_sync();
    CHECK(s_seq_run == run_number + 1u && s_seq_state == SQ_STILL);
    CHECK(wire_commands == background_commands + 1u && !s_receiving && !wire_bad);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped() && s_msel == 34);
    puts("route34: shared31 15-node recipe, reverse650/v300, forward80/v20, back190/yaw/BFF, real bucket4/0 ACK/five frames/1s/manual d730, no QR/arm/laser, post-yaw-before-next, terminal restart passed");
    return 0;
}

static int check_pending_background_and_late_packets(void)
{
    CHECK(wire_boot() == 0);
    /* Model a boot QR request still queued before the first DefaultTask TX. */
    proto_init(); proto_set_binary_mode(1); proto_set_binary_tx(wire_tx);
    proto_set_on_frame(test_vision_feed_frame);
    wire_commands = notice_calls = 0u;
    proto_qr_begin();
    CHECK(s_due && s_receiving && !wire_commands);
    uint16_t old_request = s_request;
    run_cmd("34"); wire_sync();
    CHECK(!s_due && !s_receiving && !wire_commands && s_seq_state == SQ_READY);
    wire_ack(old_request, 1u, 0u);
    wire_qr(old_request, 1u, "123", 0, 0);
    CHECK(!proto_qr_get(NULL) && !notice_calls && stopped());
    ProtoWireDiag diag; real_proto_wire_diag_get(&diag);
    CHECK(diag.outside_phase == 2u && !diag.receiving && diag.tx_attempts == 0u);
    run_cmd("g"); CHECK(sequence_start_stage() == 0);
    wire_ack(old_request, 1u, 0u);
    wire_qr(old_request, 2u, "321", 0, 0);
    CHECK(sequence_finish_stage() == 0);
    CHECK(s_seq_stage == 1u && s_seq_state == SQ_STILL && !wire_commands);
    CHECK(!notice_calls && !s_seq_qr[0] && !s_receiving);
    run_cmd("g"); wire_sync();
    CHECK(s_seq_state == SQ_STOPPED && !wire_commands && stopped());
    puts("route34: queued boot QR canceled locally before TX; late ACK/QR drain without success or route gate; no camera IDLE/STOP passed");
    return 0;
}

static int check_all_phase_cancellations_and_write_locks(void)
{
    static const char *const stop_keys[] = {"g", "a", "0"};
    for (unsigned stage = 0u; stage < ROUTE_TEST_STAGES; ++stage)
        for (unsigned phase_index = 0u; phase_index < 4u; ++phase_index)
            for (unsigned key = 0u; key < 3u; ++key) {
                if (stage == ROUTE_TEST_ALIGN_STAGE) continue; /* Real bucket phases below. */
                CHECK(wire_boot() == 0); run_cmd("34"); run_cmd("g");
                if (stage == ROUTE_TEST_BACK_STAGE) s_bucket36_back_mm = 730u;
                s_seq_stage = (uint8_t)stage; route_seq_prepare();
                if (phase_index == 1u) {
                    host_tick += T_DIST_STILL_MS; wire_poll(); CHECK(s_seq_state == SQ_WAIT);
                } else if (phase_index >= 2u) CHECK(sequence_start_stage() == 0);
                if (phase_index == 3u) {
                    if (dist_mode()) {
                        if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
                        else host_fore = s_dist_odo0 + s_dist_target;
                    } else host_yaw = turn_target_deg();
                    wire_poll(); CHECK(s_round == R_BRAKE);
                }
                uint32_t run_number = s_seq_run;
                run_cmd(stop_keys[key]);
                CHECK(s_seq_state == SQ_STOPPED && s_msel == 34 && stopped());
                CHECK(s_seq_stage == stage && !s_receiving && !s_due && wire_commands == 1u);
                run_cmd("g"); host_tick += 10000u; wire_poll();
                CHECK(s_seq_state == SQ_STOPPED && s_seq_run == run_number && stopped());
                CHECK(!notice_calls && wire_commands == 1u);
            }
    static const char *const writes[] = {"31", "32", "33", "34", "15", "r1", "v300", "d1000",
                                        "ykp3", "fff0", "cc", "co", "su1500", "n5", "b1d800"};
    CHECK(wire_boot() == 0); run_cmd("34"); run_cmd("g");
    for (unsigned i = 0u; i < sizeof writes / sizeof writes[0]; ++i) {
        run_cmd(writes[i]);
        CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
        CHECK(s_seq_mode == 34 && s_seq_stage == 0u && s_seq_state == SQ_STILL && stopped());
        CHECK(wire_commands == 1u && !s_receiving);
    }
    run_cmd("g");
    CHECK(wire_boot() == 0); run_cmd("34"); run_cmd("g");
    host_imu_valid = 0; wire_poll();
    CHECK(s_seq_state == SQ_STOPPED && stopped() && wire_commands == 1u);
    CHECK(strstr(last_message, "status=IMUERR") != NULL);
    puts("route34: 168 g/a/0 motion PREP/WAIT/RUN/BRAKE cancellations, no resume/return or camera TX, 15 write locks and IMU stop passed");
    return 0;
}

static int check_post_cross_back_to_forward_preparation(void)
{
    CHECK(wire_boot() == 0); run_cmd("34"); run_cmd("ykp1.2"); run_cmd("g");
    s_seq_stage = 4u; route_seq_prepare();
    CHECK(sequence_start_stage() == 0 && s_dist_target == 80.0f && s_v == 20.0f);
    CHECK(s_dist_heading_kp == 0.0f && route34_near(s_route_heading_kp, 1.2f));
    host_yaw = 6.0f; tick(); CHECK(last_w == 0.0f);
    host_yaw = -6.0f; tick(); CHECK(last_w == 0.0f);
    unsigned before_finish = (unsigned)zero_calls;
    CHECK(sequence_finish_stage() == 0);
    CHECK(s_seq_stage == 5u && s_seq_state == SQ_STILL && stopped());
    CHECK(zero_calls == (int)before_finish + 1); /* Existing distance-result reset. */
    unsigned before_node = (unsigned)zero_calls;
    host_yaw = 4.0f; host_counts[0]++;
    host_tick += T_DIST_STILL_MS; wire_poll();
    CHECK(s_seq_state == SQ_STILL && stopped() && zero_calls == (int)before_node);
    host_tick += T_DIST_STILL_MS - 1u; wire_poll();
    CHECK(s_seq_state == SQ_STILL && stopped() && zero_calls == (int)before_node);
    host_tick++; wire_poll();
    CHECK(s_seq_state == SQ_WAIT && stopped() && zero_calls == (int)before_node + 1 && host_yaw == 0.0f);
    host_tick += NAV_SETTLE_MS - 1u; wire_poll();
    CHECK(s_seq_state == SQ_WAIT && stopped() && zero_calls == (int)before_node + 1);
    host_tick++; wire_poll();
    CHECK(s_seq_state == SQ_RUN && s_round == R_RUN && s_msel == 16);
    CHECK(s_dist_target == -190.0f && s_v == 100.0f && s_dist_precise && s_dist_ff_ratio == 0.00625f);
    CHECK(s_dist_heading0 == 0.0f && route34_near(s_dist_heading_kp, 1.2f));
    CHECK(host_yaw == 0.0f && last_w == 0.0f && step_heading_kp_deg() == 0.3f);
    host_yaw = 3.0f; tick(); CHECK(route34_near(last_w, -1.2f * 3.0f * 0.0174533f));
    host_yaw = -3.0f; tick(); CHECK(route34_near(last_w, 1.2f * 3.0f * 0.0174533f));
    CHECK(!s_receiving && wire_commands == 1u && !notice_calls && !prepare_calls);
    run_cmd("g");
    puts("route34 post-cross: forward80/v20 mechanical contact -> changed wheel delays zero ->250ms still ->zero ->full NAV_SETTLE_MS ->back190/v100 restores yaw and independentBFF passed");
    return 0;
}

static int check_shared_tuning_and_legacy_isolation(void)
{
    CHECK(wire_boot() == 0); run_cmd("34"); run_cmd("ykp2"); run_cmd("fff0.02");
    host_messages[0] = '\0'; run_cmd("param");
    CHECK(strstr(host_messages, "ROUTE_PROFILE mode=34 source=LOCAL") != NULL);
    CHECK(strstr(host_messages, "fff=0.02000 bff=0.00625 ykp=2.000") != NULL);
    CHECK(test_forward_ff_ratio() == 0.0125f && step_heading_kp_deg() == 0.3f);
    run_cmd("31"); wire_sync();
    CHECK(s_seq_mode == 31 && s_route_heading_kp == 2.0f && route34_near(s_route_forward_ff_ratio, 0.02f));
    CHECK(s_receiving && s_mode == 1u && wire_commands == 2u); /* 31 still requests QR. */
    run_cmd("34"); wire_sync(); CHECK(!s_receiving && wire_commands == 2u);
    run_cmd("32");
    CHECK(s_msel == 32 && s_seq_state == SQ_OFF && step_heading_kp_deg() == 0.3f);
    CHECK(test_forward_ff_ratio() == 0.0125f && trial_report_calls > 0);
    run_cmd("15"); run_cmd("v100"); run_cmd("d100"); run_cmd("g");
    CHECK(s_dist_heading_kp == 0.3f && s_dist_ff_ratio == -0.00625f);
    run_cmd("a");
    run_cmd("20"); CHECK(turn_target_deg() == 90.0f);
    run_cmd("22"); CHECK(turn_target_deg() == 180.0f);
    run_cmd("30"); CHECK(turn_target_deg() == -92.0f);
    CHECK(wire_boot() == 0); run_cmd("34");
    CHECK(s_route_heading_kp == 0.3f && s_route_forward_ff_ratio == -0.00625f);
    run_cmd("38"); CHECK(s_msel == 34 && strstr(last_message, "MODE_RANGE") != NULL);
    puts("route34 tuning: shared31 RAM ykp/fff, explicit MODE34 report,31 reopens QR; manual/32/global/90/180 unchanged and power-on defaults passed");
    return 0;
}

int main(void)
{
    CHECK(check_complete_recipe_without_qr() == 0);
    CHECK(check_pending_background_and_late_packets() == 0);
    CHECK(check_all_phase_cancellations_and_write_locks() == 0);
    CHECK(check_route_bucket_manual_stop(34u) == 0);
    CHECK(check_post_cross_back_to_forward_preparation() == 0);
    CHECK(check_route_cross_heading_restart(34u) == 0);
    CHECK(check_shared_tuning_and_legacy_isolation() == 0);
    puts("route34_no_qr_test: all host checks passed; no hardware or visual source changed");
    return 0;
}
