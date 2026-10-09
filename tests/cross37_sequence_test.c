/* Mode37 uses the real Bluetooth route state machine and request parser with
 * inert motion/time peripherals. This test never drives or flashes hardware. */
#define ROUTE31_QR_FIXTURE_MAIN cross37_qr_fixture_main
#include "route31_qr_gate_test.c"
#undef ROUTE31_QR_FIXTURE_MAIN

static int cross37_boot(void)
{
    CHECK(wire_boot() == 0);
    run_cmd("37"); wire_sync();
    CHECK(CROSS_ONLY_MODE == 37 && CROSS37_STAGES == 3u);
    CHECK(T_MODE_MAX == 43 && s_msel == 37 && s_seq_mode == 37 && s_seq_state == SQ_READY);
    CHECK(route_seq_stage_count() == 3u && stopped() && !s_receiving && !s_due && !proto_qr_get(NULL));
    CHECK(!host_target_calls && !pulse_calls && !servo_calls && !host_laser_on_calls && !s_go);
    return 0;
}

static int check_exact_three_stage_recipe(void)
{
    static const int modes[3] = {16,15,16};
    static const int commands[3] = {-650,80,-190};
    static const float speeds[3] = {300.0f,20.0f,100.0f};
    CHECK(cross37_boot() == 0);
    unsigned requests = wire_commands;
    uint16_t old_qr_request = wire_request;
    host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_READY && stopped() && wire_commands == requests);
    host_messages[0] = '\0'; run_cmd("g");
    CHECK(s_seq_run == 1u && s_seq_state == SQ_STILL && stopped());
    CHECK(strstr(host_messages, "ROUTE_PROFILE mode=37 source=LOCAL") != NULL);
    for (unsigned stage = 0u; stage < 3u; ++stage) {
        CHECK(s_seq_stage == stage && route_seq_leg() == &s_cross37_plan[stage]);
        CHECK(route_seq_leg()->mode == modes[stage] && route_seq_leg()->speed_mms == speeds[stage]);
        CHECK(route_seq_leg()->heading_hold == (stage == 2u ? 1u : 0u));
        CHECK(sequence_start_stage() == 0 && s_seq_mode == 37 && s_msel == modes[stage]);
        CHECK(dist_mode() && !turn_closed_loop_mode());
        CHECK(s_dist_target == commands[stage] && s_v == speeds[stage] && s_dist_precise);
        CHECK(s_dist_heading_kp == (stage == 2u ? 0.3f : 0.0f) &&
              s_dist_ff_ratio == (stage == 2u ? 0.00625f : 0.0f));
        CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f);
        CHECK(last_x == (stage == 1u ? speeds[stage] : -speeds[stage]));
        CHECK(last_y == (stage == 2u ? -0.625f : 0.0f) && last_w == 0.0f);
        host_yaw = 3.0f; tick();
        CHECK(fabsf(last_w + (stage == 2u ? 0.3f : 0.0f) * 3.0f * 0.0174533f) < 0.000001f);
        host_yaw = -3.0f; tick();
        CHECK(fabsf(last_w - (stage == 2u ? 0.3f : 0.0f) * 3.0f * 0.0174533f) < 0.000001f);
        wire_ack(old_qr_request, 1u, 0u); wire_qr(old_qr_request, stage + 1u, "123", 0, 0);
        CHECK(!s_receiving && !notice_calls && !proto_qr_get(NULL));
        CHECK(sequence_finish_stage() == 0); wire_sync();
        CHECK(stopped() && wire_commands == requests && !s_due && !host_target_calls);
        CHECK(!laser_state && !host_laser_on_calls && !pulse_calls && !servo_calls && !s_go);
        CHECK(s_seq_state == (stage == 2u ? SQ_DONE : SQ_STILL));
    }
    CHECK(s_msel == 37 && s_round == R_DONE && s_seq_state == SQ_DONE && s_seq_stage == 2u);
    CHECK(strstr(host_messages, "QR_WAIT") == NULL && strstr(host_messages, "BUCKET36_REQUEST") == NULL);
    run_cmd("g"); host_tick += 60000u; wire_poll();
    CHECK(s_seq_state == SQ_DONE && s_seq_run == 1u && stopped() && wire_commands == requests);
    CHECK(strstr(last_message, "select37_then_g_to_rerun") != NULL);
    run_cmd("37"); CHECK(s_seq_state == SQ_READY && stopped());
    run_cmd("g"); CHECK(s_seq_run == 2u && s_seq_state == SQ_STILL && stopped());
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && s_msel == 37 && stopped());
    puts("cross37 exact3: backward650/v300 ->forward80/v20 ->backward190/v100, yaw/FF0/0/normalBFF; lastleg post-yaw beforeDONE; no offset/turn/QR/camera/arm/laser; PARAM, terminal/noresume and rerun passed");
    return 0;
}

static int check_heading_restart_after_mechanical_alignment(void)
{
    CHECK(cross37_boot() == 0); run_cmd("ykp1.2"); run_cmd("fff0.02"); run_cmd("g");
    s_seq_stage = 1u; route_seq_prepare();
    CHECK(sequence_start_stage() == 0 && s_msel == 15 && s_dist_target == 80.0f && s_v == 20.0f);
    CHECK(s_dist_heading_kp == 0.0f && s_dist_ff_ratio == 0.0f);
    host_yaw = 6.0f; tick(); CHECK(last_w == 0.0f && last_x == 20.0f && last_y == 0.0f);
    CHECK(sequence_finish_stage() == 0 && s_seq_stage == 2u && stopped());
    unsigned zero_before = (unsigned)zero_calls;
    host_yaw = 7.0f; host_counts[3]++;
    host_tick += T_DIST_STILL_MS; wire_poll();
    CHECK(s_seq_state == SQ_STILL && zero_calls == (int)zero_before && stopped());
    host_tick += T_DIST_STILL_MS - 1u; wire_poll();
    CHECK(s_seq_state == SQ_STILL && zero_calls == (int)zero_before && stopped());
    host_tick++; wire_poll();
    CHECK(s_seq_state == SQ_WAIT && zero_calls == (int)zero_before + 1 && host_yaw == 0.0f && stopped());
    CHECK(NAV_SETTLE_MS == 750u);
    host_tick += NAV_SETTLE_MS - 1u; wire_poll();
    CHECK(s_seq_state == SQ_WAIT && stopped());
    host_tick++; wire_poll();
    CHECK(s_seq_state == SQ_RUN && s_msel == 16 && s_dist_target == -190.0f && s_v == 100.0f);
    CHECK(s_dist_heading0 == 0.0f && s_dist_heading_kp == 1.2f && s_dist_ff_ratio == 0.00625f);
    CHECK(last_x == -100.0f && last_y == -0.625f && last_w == 0.0f);
    host_yaw = 3.0f; tick(); CHECK(fabsf(last_w + 1.2f * 3.0f * 0.0174533f) < 0.000001f);
    host_yaw = -3.0f; tick(); CHECK(fabsf(last_w - 1.2f * 3.0f * 0.0174533f) < 0.000001f);
    CHECK(step_heading_kp_deg() == 0.3f && test_forward_ff_ratio() == 0.0125f);
    CHECK(sequence_finish_stage() == 0 && s_seq_state == SQ_DONE && stopped());
    puts("cross37 post-board: forward80/v20 has no yaw/FF; moving wheel delays zero until250ms still, then reset+750ms ->backward190/v100 restores localykp and independentBFF; global/manual untouched passed");
    return 0;
}

static int check_all_36_stops_and_write_locks(void)
{
    static const char *const keys[] = {"g","a","0"};
    for (unsigned stage = 0u; stage < 3u; ++stage)
        for (unsigned phase = 0u; phase < 4u; ++phase)
            for (unsigned key = 0u; key < 3u; ++key) {
                CHECK(cross37_boot() == 0); run_cmd("g");
                s_seq_stage = (uint8_t)stage; route_seq_prepare();
                if (phase == 1u) {
                    host_tick += T_DIST_STILL_MS; wire_poll(); CHECK(s_seq_state == SQ_WAIT);
                } else if (phase >= 2u) CHECK(sequence_start_stage() == 0);
                if (phase == 3u) {
                    if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
                    else host_fore = s_dist_odo0 + s_dist_target;
                    wire_poll(); CHECK(s_round == R_BRAKE);
                }
                uint32_t run = s_seq_run; unsigned tx = wire_commands;
                run_cmd(keys[key]); wire_sync();
                CHECK(s_seq_state == SQ_STOPPED && s_msel == 37 && stopped() && !s_due && !s_receiving);
                run_cmd("g"); host_tick += 60000u; wire_poll();
                CHECK(s_seq_state == SQ_STOPPED && s_seq_run == run && stopped() && wire_commands == tx);
            }
    static const char *const writes[] = {"31","34","35","36","37","15","r1","v600","d1000",
                                        "fff0","ykp10","cc","co","su1500","n5","b1d800"};
    CHECK(cross37_boot() == 0); run_cmd("g");
    for (unsigned n = 0u; n < sizeof writes / sizeof writes[0]; ++n) {
        run_cmd(writes[n]);
        CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
        CHECK(s_seq_mode == 37 && s_seq_stage == 0u && s_seq_state == SQ_STILL && stopped());
    }
    static const char *const reports[] = {"?","diag","param","route"};
    for (unsigned n = 0u; n < sizeof reports / sizeof reports[0]; ++n) {
        run_cmd(reports[n]); CHECK(s_seq_state == SQ_STILL && s_seq_mode == 37 && stopped());
    }
    run_cmd("g");
    puts("cross37 stops:36 g/a/0 cancellations across3 nodes x PREP/WAIT/RUN/BRAKE; early g cancels before motion; no return/resume,16 mode/parameter/actuator writes locked and reports remain read-only passed");
    return 0;
}

static int check_boot_request_cancel_and_imu_abort(void)
{
    CHECK(wire_boot() == 0);
    proto_init(); proto_set_binary_mode(1); proto_set_binary_tx(wire_tx);
    proto_set_on_frame(test_vision_feed_frame);
    wire_commands = notice_calls = 0u; proto_qr_begin();
    CHECK(s_due && s_receiving && !wire_commands);
    uint16_t old = s_request;
    run_cmd("37"); wire_sync(); CHECK(!s_due && !s_receiving && !wire_commands && stopped());
    wire_ack(old, 1u, 0u); wire_qr(old, 1u, "123", 0, 0);
    CHECK(!proto_qr_get(NULL) && !notice_calls && !wire_commands && stopped());
    run_cmd("g"); CHECK(s_seq_state == SQ_STILL && !wire_commands); run_cmd("g");
    for (unsigned stage = 0u; stage < 3u; ++stage)
        for (unsigned running = 0u; running < 2u; ++running) {
            CHECK(cross37_boot() == 0); run_cmd("g");
            s_seq_stage = (uint8_t)stage; route_seq_prepare();
            if (running) CHECK(sequence_start_stage() == 0);
            host_imu_valid = 0; wire_poll();
            CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving && !s_due);
            CHECK(strstr(last_message, "status=IMUERR") != NULL);
        }
    CHECK(cross37_boot() == 0); run_cmd("g"); host_abort = 1; wire_poll();
    CHECK(s_seq_state == SQ_STOPPED && stopped() && !s_receiving && !s_due);
    CHECK(strstr(last_message, "status=ABORT") != NULL);
    puts("cross37 safety: queued bootQR cancelled without camera command; late QR cannot become success;6 IMU-invalid prep/run cases incl yaw-disabled legs and run-abort stop passed");
    return 0;
}

static int check_mode36_ten_step_recipe_and_shared_crossing(void)
{
    static const unsigned modes[10] = {17u,16u,20u,16u,15u,16u,18u,16u,30u,15u};
    static const unsigned distances[10] = {530u,650u,0u,650u,80u,190u,730u,780u,0u,200u};
    static const float speeds[10] = {100,100,100,300,20,100,100,100,100,100};
    CHECK(wire_boot() == 0); run_cmd("36");
    CHECK(s_seq_mode == 36 && route_seq_stage_count() == 10u && BUCKET_ROUTE_STAGES == 10u);
    for (unsigned stage = 0u; stage < 10u; ++stage) {
        s_seq_stage = (uint8_t)stage;
        const RouteTestLeg *leg = route_seq_leg();
        CHECK(leg == &s_bucket36_plan[stage]);
        CHECK(leg->mode == modes[stage] && leg->distance_mm == distances[stage] && leg->speed_mms == speeds[stage]);
        CHECK(leg->heading_hold == (stage == 3u || stage == 4u ? 0u : 1u));
    }
    CHECK(!route_seq_bucket_enabled());
    CHECK(T_TURN_RIGHT_TARGET_DEG == 90.0f && T_TURN_LEFT_TARGET_DEG == -92.0f);
    run_cmd("37"); CHECK(s_seq_mode == 37 && route_seq_stage_count() == 3u);
    static const unsigned base_stages[3] = {3u,4u,5u};
    for (unsigned stage = 0u; stage < 3u; ++stage) {
        s_seq_stage = (uint8_t)stage;
        const RouteTestLeg *leg = route_seq_leg();
        const RouteTestLeg *base = &s_bucket36_plan[base_stages[stage]];
        const RouteTestLeg *integrated = &s_route_test_plan[base_stages[stage]];
        CHECK(leg == &s_cross37_plan[stage] && leg->mode == base->mode);
        CHECK(leg->distance_mm == base->distance_mm && leg->heading_hold == base->heading_hold);
        CHECK(leg->speed_mms == base->speed_mms);
        CHECK(integrated->mode == leg->mode && integrated->distance_mm == leg->distance_mm &&
              integrated->speed_mms == leg->speed_mms && integrated->heading_hold == leg->heading_hold);
    }
    CHECK(ROUTE_CROSS_BACK_MM == 650u && ROUTE_CROSS_BACK_V_MMS == 300.0f);
    CHECK(ROUTE_POST_CROSS_ALIGN_MM == 80u && ROUTE_POST_CROSS_ALIGN_V_MMS == 20.0f);
    CHECK(ROUTE_POST_CROSS_CLEAR_MM == 190u && ROUTE_POST_CROSS_CLEAR_V_MMS == 100.0f);
    CHECK(!route_seq_bucket_enabled());
    CHECK(s_cross37_plan[0].speed_mms == 300.0f && s_bucket36_plan[3].speed_mms == 300.0f);
    CHECK(s_route31_plan[4].distance_mm == 0u && s_route31_plan[4].speed_mms == 40.0f && ROUTE31_STAGES == 16u);
    puts("cross37 current recipe:34/36/37 retain back650v300/forward80v20/back190v100;31-only tilt-contact-v40 and16 task stages;36 ten and37 three withoutbucket/manuald;34 retainsbucket tail passed");
    return 0;
}

#ifndef CROSS37_FIXTURE_MAIN
#define CROSS37_FIXTURE_MAIN main
#endif
int CROSS37_FIXTURE_MAIN(void)
{
    CHECK(check_mode36_ten_step_recipe_and_shared_crossing() == 0);
    CHECK(check_exact_three_stage_recipe() == 0);
    CHECK(check_heading_restart_after_mechanical_alignment() == 0);
    CHECK(check_all_36_stops_and_write_locks() == 0);
    CHECK(check_boot_request_cancel_and_imu_abort() == 0);
    puts("cross37_sequence_test: all host checks passed; not real obstacle, slip, contact, distance or angle acceptance");
    return 0;
}
