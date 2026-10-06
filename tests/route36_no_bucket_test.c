/* Current independent36 ten-node motion-only route. Real parser/dispatcher,
 * inert hardware; no flashing, wheel movement, or physical acceptance. */
#define CROSS37_FIXTURE_MAIN route36_cross_fixture_main
#include "cross37_sequence_test.c"
#undef CROSS37_FIXTURE_MAIN

static int route36_boot(void)
{
    CHECK(wire_boot() == 0); run_cmd("36"); wire_sync();
    CHECK(s_msel == 36 && s_seq_mode == 36 && s_seq_state == SQ_READY && stopped());
    CHECK(route_seq_stage_count() == 10u && !route_seq_bucket_enabled());
    CHECK(!s_receiving && !s_due && !proto_qr_get(NULL) && !host_target_calls);
    return 0;
}

static int check_complete_ten_node_runner(void)
{
    static const int modes[10] = {17,16,20,16,15,16,18,16,30,15};
    static const int commands[10] = {-530,-650,90,-650,80,-190,730,-780,-92,200};
    static const float speeds[10] = {100,100,100,300,20,100,100,100,100,100};
    CHECK(route36_boot() == 0);
    unsigned requests = wire_commands; uint16_t boot_qr = wire_request;
    host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_READY && stopped() && wire_commands == requests);
    host_messages[0] = '\0'; run_cmd("g");
    CHECK(s_seq_state == SQ_STILL && s_seq_run == 1u && stopped());
    CHECK(strstr(host_messages, "ROUTE_PROFILE mode=36 source=LOCAL") != NULL);
    for (unsigned stage = 0u; stage < 10u; ++stage) {
        CHECK(s_seq_stage == stage && route_seq_leg() == &s_bucket36_plan[stage]);
        CHECK(sequence_start_stage() == 0 && s_msel == modes[stage]);
        if (dist_mode()) {
            CHECK(s_dist_target == commands[stage] && s_v == speeds[stage] && s_dist_precise);
            CHECK(s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : 0.3f));
            CHECK(s_dist_ff_ratio == (stage == 1u || stage == 5u || stage == 7u ? 0.00625f :
                  stage == 9u ? -0.00625f : 0.0f));
            CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f);
            if (stage == 6u) CHECK(last_x == 0.0f && last_y == 100.0f);
            if (stage == 7u) CHECK(last_x == -100.0f && last_y == -0.625f);
        } else CHECK(turn_target_deg() == commands[stage]);
        wire_ack(boot_qr, 1u, 0u); wire_qr(boot_qr, stage + 1u, "123", 0, 0);
        CHECK(!s_receiving && !proto_qr_get(NULL) && !notice_calls);
        CHECK(sequence_finish_stage() == 0); wire_sync();
        CHECK(stopped() && wire_commands == requests && !host_target_calls && !s_receiving && !s_due);
        CHECK(!laser_state && !host_laser_on_calls && !pulse_calls && !servo_calls && !s_go);
        CHECK(s_seq_state == (stage == 9u ? SQ_DONE : SQ_STILL));
        CHECK(s_seq_state != SQ_BUCKET_ALIGN && s_seq_state != SQ_MANUAL_D_WAIT && s_seq_state != SQ_QR_WAIT);
    }
    CHECK(s_seq_state == SQ_DONE && s_seq_stage == 9u && s_msel == 36 && s_round == R_DONE);
    CHECK(strstr(host_messages, "BUCKET36_REQUEST") == NULL && strstr(host_messages, "WAIT_D") == NULL);
    uint32_t run = s_seq_run;
    run_cmd("g"); host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_DONE && s_seq_run == run && stopped() && wire_commands == requests);
    run_cmd("36"); run_cmd("g"); CHECK(s_seq_run == run + 1u && s_seq_state == SQ_STILL);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && stopped());
    puts("route36 exact10: left530/back650/right90/back650v300/forward80v20/back190/right730/back780/left92/forward200STOP; no QR/bucket/manual-d/target/arm/laser, automaticPARAM, no auto resume passed");
    return 0;
}

static int check_all_motion_phases_cancel(void)
{
    static const char *const keys[] = {"g", "a", "0"};
    for (unsigned stage = 0u; stage < 10u; ++stage)
        for (unsigned phase = 0u; phase < 4u; ++phase)
            for (unsigned key = 0u; key < 3u; ++key) {
                CHECK(route36_boot() == 0); run_cmd("g");
                s_seq_stage = (uint8_t)stage; route_seq_prepare();
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
                unsigned requests = wire_commands; uint32_t run = s_seq_run;
                run_cmd(keys[key]); wire_sync();
                CHECK(s_seq_state == SQ_STOPPED && s_msel == 36 && stopped() && !s_receiving && !s_due);
                run_cmd("g"); host_tick += 60000u; wire_poll();
                CHECK(s_seq_state == SQ_STOPPED && s_seq_run == run && stopped() && wire_commands == requests);
                CHECK(!host_target_calls && !host_laser_on_calls);
            }
    CHECK(route36_boot() == 0); run_cmd("g");
    static const char *const writes[] = {"d500", "37", "v200", "ykp1", "fff0", "cc", "su1500", "n5"};
    for (unsigned n = 0u; n < sizeof writes / sizeof writes[0]; ++n) {
        run_cmd(writes[n]); CHECK(s_seq_state == SQ_STILL && stopped());
        CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
    }
    run_cmd("g");
    puts("route36 stop:120 g/a/0 cancellations across10 nodes x PREP/WAIT/RUN/BRAKE; early stop/active parameter locks, no automatic reverse/restart or camera command passed");
    return 0;
}

static int check_no_camera_and_imu_errors(void)
{
    CHECK(wire_boot() == 0);
    proto_init(); proto_set_binary_mode(1); proto_set_binary_tx(wire_tx);
    proto_set_on_frame(test_vision_feed_frame);
    wire_commands = notice_calls = 0u; proto_qr_begin();
    CHECK(s_due && s_receiving && !wire_commands);
    uint16_t old = s_request;
    run_cmd("36"); wire_sync();
    CHECK(!s_due && !s_receiving && !wire_commands && stopped());
    wire_ack(old, 1u, 0u); wire_qr(old, 1u, "123", 0, 0);
    CHECK(!proto_qr_get(NULL) && !notice_calls && !wire_commands && stopped());
    run_cmd("g"); CHECK(s_seq_state == SQ_STILL && !wire_commands); run_cmd("g");
    for (unsigned stage = 0u; stage < 10u; ++stage) {
        CHECK(route36_boot() == 0); run_cmd("g"); s_seq_stage = (uint8_t)stage; route_seq_prepare();
        CHECK(sequence_start_stage() == 0); host_imu_valid = 0; wire_poll();
        CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving && !s_due && !host_target_calls);
        CHECK(strstr(last_message, "status=IMUERR") != NULL);
    }
    CHECK(route36_boot() == 0); run_cmd("g"); host_abort = 1; wire_poll();
    CHECK(s_seq_state == SQ_STOPPED && stopped() && !host_target_calls);
    puts("route36 isolation/error: pendingbootQR cancelled before TX, lateQR ignored,10 IMU-invalid cases and run-abort stop; no target/bucket/laser/arm request passed");
    return 0;
}

int main(void)
{
    CHECK(check_mode36_ten_step_recipe_and_shared_crossing() == 0);
    CHECK(check_complete_ten_node_runner() == 0);
    CHECK(check_all_motion_phases_cancel() == 0);
    CHECK(check_no_camera_and_imu_errors() == 0);
    puts("route36_no_bucket_test: all host checks passed; not physical obstacle, traction, contact, distance, yaw or road-projection acceptance");
    return 0;
}
