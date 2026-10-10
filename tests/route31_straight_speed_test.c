/* Real mode31 router/executor/VAT and request-bound parser. Host only:
 * these assertions do not establish motor speed, grip, or road clearance. */
#define ROUTE31_QR_FIXTURE_MAIN speed_qr_fixture_main
#include "route31_qr_gate_test.c"
#undef ROUTE31_QR_FIXTURE_MAIN

static int speed_boot_with_lateral(unsigned speed, unsigned lateral)
{
    char line[16];
    CHECK(wire_boot() == 0);
    host_wire_diag_hook = real_proto_wire_diag_get;
    run_cmd("31");
    snprintf(line, sizeof line, "v%u", speed); run_cmd(line);
    snprintf(line, sizeof line, "pv%u", lateral); run_cmd(line);
    CHECK(s_seq_state == SQ_READY && s_route31_straight_v == (float)speed &&
          s_route31_lateral_v == (float)lateral);
    wire_ack(wire_request, 1u, 0u); wire_qr(wire_request, 1u, "111", 0, 0);
    CHECK(proto_qr_get(NULL) && stopped());
    run_cmd("g");
    s_seq_qr[0] = s_seq_qr[1] = s_seq_qr[2] = 1;
    return 0;
}

static int speed_prepare_with_lateral(unsigned stage, unsigned speed, unsigned lateral)
{
    CHECK(speed_boot_with_lateral(speed, lateral) == 0);
    step_vision_receive_end();
    s_route31_rack_deployed = 1u; /* Isolate speed from the separately tested rack job. */
    s_route31_hostage_rank = 1u;
    if (stage == 4u) CHECK(fixture_prepare_route31_board() == 0);
    else { s_seq_stage = (uint8_t)stage; route_seq_prepare(); }
    wire_sync();
    CHECK(s_seq_state != SQ_STOPPED && s_route31_straight_v == (float)speed &&
          s_route31_lateral_v == (float)lateral);
    return 0;
}

static int speed_prepare(unsigned stage, unsigned speed)
{
    return speed_prepare_with_lateral(stage, speed, 100u);
}

static void speed_wire_object(uint16_t request, unsigned sequence, int model,
                              unsigned cx, unsigned cy)
{
    /* A full real-parser object frame with room for ballY390/bucketY420.
     * The old bucket-only helper's320px height is invalid when optionalY is
     * compiled; that geometry must not turn a speed test into a fault test. */
    uint8_t inner[25] = {0x01u, (uint8_t)sequence, (uint8_t)(sequence >> 8), 1u};
    uint8_t packet[30] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8)};
    route_wire_le16(inner + 4u, 640u); route_wire_le16(inner + 6u, 480u);
    inner[14] = (uint8_t)model;
    route_wire_le16(inner + 15u, 900u); route_wire_le16(inner + 17u, cx);
    route_wire_le16(inner + 19u, cy); route_wire_le16(inner + 21u, 24u);
    route_wire_le16(inner + 23u, 24u);
    route_wire_le16(packet + 3u, sizeof inner); memcpy(packet + 5u, inner, sizeof inner);
    wire_feed(packet, sizeof packet, 0);
}

static int speed_router_bounds_and_retention(void)
{
    static const unsigned valid[] = {1u, 100u, 200u, 300u, 600u};
    static const char *const invalid[] = {"v0", "v601", "v-1", "v", "v100x", "v100.5", "v2147483648"};
    char line[16];
    reset_fixture(); run_cmd("31");
    CHECK(s_route31_straight_v == 200.0f && !pulse_calls && !servo_calls);
    run_cmd("ykp1.2"); run_cmd("fff0.03"); run_cmd("bff0.04");
    run_cmd("lff0.01"); run_cmd("rff0.02");
    float kp = s_route_heading_kp, fff = s_route_forward_ff_ratio, bff = s_route_backward_ff_ratio;
    float lff = s_route_left_ff_ratio, rff = s_route_right_ff_ratio;
    for (unsigned i = 0u; i < sizeof valid / sizeof valid[0]; ++i) {
        snprintf(line, sizeof line, "v%u", valid[i]);
        host_messages[0] = '\0'; run_cmd(line);
        CHECK(s_route31_straight_v == (float)valid[i] && s_seq_state == SQ_READY &&
              !pulse_calls && !servo_calls && last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
        CHECK(s_route_heading_kp == kp && s_route_forward_ff_ratio == fff &&
              s_route_backward_ff_ratio == bff && s_route_left_ff_ratio == lff && s_route_right_ff_ratio == rff);
        snprintf(line, sizeof line, "straight_v=%u", valid[i]);
        CHECK(strstr(host_messages, "ROUTE31_SPEED") && strstr(host_messages, line));
    }
    for (unsigned i = 0u; i < sizeof invalid / sizeof invalid[0]; ++i) {
        run_cmd(invalid[i]);
        CHECK(s_route31_straight_v == 600.0f && s_seq_state == SQ_READY && !pulse_calls &&
              !servo_calls && strstr(last_message, "ERR"));
    }
    run_cmd("31"); CHECK(s_route31_straight_v == 600.0f && s_v == 600.0f);
    route_seq_end("STOP"); run_cmd("v200");
    CHECK(s_seq_state == SQ_STOPPED && s_route31_straight_v == 200.0f);
    route_seq_end("DONE"); run_cmd("v300");
    CHECK(s_seq_state == SQ_DONE && s_route31_straight_v == 300.0f);
    run_cmd("31"); CHECK(s_seq_state == SQ_READY && s_route31_straight_v == 300.0f);
    run_cmd("15"); run_cmd("v250"); CHECK(s_v == 250.0f && s_route31_straight_v == 300.0f);
    run_cmd("31"); CHECK(s_route31_straight_v == 300.0f && s_v == 300.0f);
    reset_fixture(); run_cmd("31"); CHECK(s_route31_straight_v == 200.0f && s_v == 200.0f);
    puts("route31 v:1/100/200/300/600, malformed/range rejection; READY/STOPPED/DONE RAM slot, reselect retention/init reset and auto PARAM; all non-speed tuning unchanged passed");
    return 0;
}

static int lateral_router_bounds_and_retention(void)
{
    static const unsigned valid[] = {1u, 100u, 200u, 300u, 600u};
    static const char *const invalid[] = {
        "pv0", "pv601", "pv-1", "pv", "pv100x", "pv100.5", "pv2147483648"
    };
    static const unsigned other_modes[] = {15u, 17u, 18u, 24u, 28u, 34u, 35u, 36u, 37u, 39u, 42u};
    char line[24];
    reset_fixture(); host_messages[0] = '\0'; run_cmd("31");
    CHECK(s_route31_lateral_v == 250.0f && s_route31_straight_v == 200.0f && stopped());
    CHECK(strstr(host_messages, "ROUTE31_SPEED") && strstr(host_messages, "lateral_v=250") &&
          strstr(host_messages, "straight_v=200"));
    run_cmd("v200"); run_cmd("ykp1.2"); run_cmd("fff0.03"); run_cmd("bff0.04");
    run_cmd("lff0.01"); run_cmd("rff0.02");
    float kp = s_route_heading_kp, fff = s_route_forward_ff_ratio, bff = s_route_backward_ff_ratio;
    float lff = s_route_left_ff_ratio, rff = s_route_right_ff_ratio, ordinary_v = s_v;
    for (unsigned i = 0u; i < sizeof valid / sizeof valid[0]; ++i) {
        snprintf(line, sizeof line, "pv%u", valid[i]);
        host_messages[0] = '\0'; run_cmd(line);
        CHECK(s_route31_lateral_v == (float)valid[i] && s_route31_straight_v == 200.0f &&
              s_v == ordinary_v && s_seq_state == SQ_READY && stopped());
        CHECK(s_route_heading_kp == kp && s_route_forward_ff_ratio == fff &&
              s_route_backward_ff_ratio == bff && s_route_left_ff_ratio == lff && s_route_right_ff_ratio == rff);
        snprintf(line, sizeof line, "lateral_v=%u", valid[i]);
        CHECK(strstr(host_messages, "ROUTE31_SPEED") && strstr(host_messages, line) &&
              strstr(host_messages, "straight_v=200"));
    }
    for (unsigned i = 0u; i < sizeof invalid / sizeof invalid[0]; ++i) {
        run_cmd(invalid[i]);
        CHECK(s_route31_lateral_v == 600.0f && s_route31_straight_v == 200.0f &&
              s_seq_state == SQ_READY && stopped() && strstr(last_message, "ERR"));
    }
    run_cmd("v300"); CHECK(s_route31_straight_v == 300.0f && s_route31_lateral_v == 600.0f);
    run_cmd("31"); CHECK(s_seq_state == SQ_READY && s_route31_lateral_v == 600.0f);
    route_seq_end("STOP"); run_cmd("pv200");
    CHECK(s_seq_state == SQ_STOPPED && s_route31_lateral_v == 200.0f);
    route_seq_end("DONE"); run_cmd("pv300");
    CHECK(s_seq_state == SQ_DONE && s_route31_lateral_v == 300.0f);
    run_cmd("31"); CHECK(s_seq_state == SQ_READY && s_route31_lateral_v == 300.0f);
    for (unsigned i = 0u; i < sizeof other_modes / sizeof other_modes[0]; ++i) {
        snprintf(line, sizeof line, "%u", other_modes[i]); run_cmd(line);
        float prior_v = s_v;
        unsigned prior_state = s_seq_state, prior_round = s_round;
        run_cmd("pv500");
        CHECK(strstr(last_message, "ERR ROUTE31_PV_ONLY") && s_v == prior_v &&
              s_seq_state == prior_state && s_round == (int)prior_round &&
              s_route31_lateral_v == 300.0f && s_route31_straight_v == 300.0f && !pulse_calls && !servo_calls);
    }
    run_cmd("31"); CHECK(s_route31_lateral_v == 300.0f && s_route31_straight_v == 300.0f);
    reset_fixture(); run_cmd("pv200");
    CHECK(strstr(last_message, "ERR ROUTE31_PV_ONLY") && s_route31_lateral_v == 250.0f && stopped());
    run_cmd("31"); CHECK(s_route31_lateral_v == 250.0f && s_route31_straight_v == 200.0f);
    puts("route31 pv:1/100/200/300/600, malformed/range rejection; straight v independent, READY/STOPPED/DONE/reselect retention/init reset and auto PARAM; other-mode rejection with no writes/motion passed");
    return 0;
}

static int speed_default_stage_scope(void)
{
    /* No v/pv commands: pin actual RAM initialization and executor defaults,
     * separately from the explicit overrides exercised below. */
    for (unsigned stage = 0u; stage < 16u; ++stage) {
        CHECK(wire_boot() == 0);
        host_wire_diag_hook = real_proto_wire_diag_get;
        run_cmd("31");
        CHECK(s_route31_straight_v == 200.0f && s_route31_lateral_v == 250.0f);
        wire_ack(wire_request, 1u, 0u); wire_qr(wire_request, 1u, "111", 0, 0);
        CHECK(proto_qr_get(NULL) && stopped());
        run_cmd("g");
        s_seq_qr[0] = s_seq_qr[1] = s_seq_qr[2] = 1;
        step_vision_receive_end();
        s_route31_rack_deployed = 1u;
        s_route31_hostage_rank = 1u;
        if (stage == 4u) CHECK(fixture_prepare_route31_board() == 0);
        else { s_seq_stage = (uint8_t)stage; route_seq_prepare(); }
        wire_sync();
        int strafe = stage == 0u || stage == 6u;
        int ordinary = stage == 1u || stage == 5u || stage == 7u || stage == 12u || stage == 15u;
        int search = stage == 9u || stage == 11u || stage == 14u;
        float expected = strafe ? 250.0f : ordinary || search ? 200.0f :
                         stage == 3u ? 300.0f : stage == 4u ? 40.0f : 100.0f;
        CHECK(route_seq_speed_mms() == expected && s_v == expected);
        if (stage == 14u) CHECK(fabsf(s_route_search_ff - 0.05f) < 0.00001f);
        if (stage == 2u) CHECK(turn_target_deg() == 90.0f);
        if (stage == ROUTE31_HOSTAGE_TURN_STAGE) CHECK(turn_target_deg() == 93.0f);
        if (strafe || ordinary || stage == 3u || stage == 4u) {
            CHECK(sequence_start_stage() == 0 && s_v == expected);
            if (stage == 7u) CHECK(s_dist_target == -810.0f);
            if (stage == 15u) CHECK(route_seq_leg()->distance_mm == 1415u && s_dist_target == 1415.0f);
            if (strafe) CHECK(fabsf(last_y) == 250.0f);
            else CHECK(last_x == (s_msel == 16 ? -expected : expected));
            if (stage == 15u) CHECK(fabsf(s_dist_ff_ratio + 0.075f) < 0.00001f &&
                                  fabsf(last_y - expected * 0.075f) < 0.00001f);
        }
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !host_timer_active && !laser_state);
    }
    puts("route31 defaults without overrides: all16 stages straight/search200 and lateral250; fixed turns100 with preCrossRIGHT90/hostageRIGHT93, rank1final1415/cross300/contact40 and cancellation preserved passed");
    return 0;
}

static int speed_every_stage_scope(void)
{
    static const unsigned speeds[] = {1u, 100u, 200u, 600u};
    for (unsigned n = 0u; n < sizeof speeds / sizeof speeds[0]; ++n) {
        unsigned speed = speeds[n];
        for (unsigned stage = 0u; stage < 16u; ++stage) {
            CHECK(speed_prepare(stage, speed) == 0);
            int ordinary = stage == 1u || stage == 5u || stage == 7u || stage == 12u || stage == 15u;
            int search = stage == 9u || stage == 11u || stage == 14u;
            float expected = ordinary || search ? (float)speed : stage == 3u ? 300.0f : stage == 4u ? 40.0f : 100.0f;
            CHECK(route_seq_speed_mms() == expected && s_v == expected);
            if (stage == 14u) CHECK(fabsf(s_route_search_ff - 0.05f) < 0.00001f);
            if (stage == 2u) CHECK(turn_target_deg() == 90.0f);
            if (stage == ROUTE31_HOSTAGE_TURN_STAGE) CHECK(turn_target_deg() == 93.0f);
            if (ordinary || stage == 0u || stage == 3u || stage == 4u || stage == 6u) {
                CHECK(sequence_start_stage() == 0 && s_v == expected);
                CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f && s_dist_precise);
                CHECK(s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : stage == 6u ? 3.0f : 0.3f));
                CHECK(stage == 0u ? s_dist_target == -575.0f :
                      stage == 1u ? s_dist_target == -610.0f :
                      stage == 3u ? s_dist_target == -620.0f :
                      stage == 4u ? s_dist_target == 0.0f :
                      stage == 7u ? s_dist_target == -810.0f : 1);
                if (stage == 15u) CHECK(route_seq_leg()->distance_mm == 1415u && s_dist_target == 1415.0f);
                if (ordinary) CHECK(last_x == (s_msel == 16 ? -expected : expected));
                else if (stage == 0u || stage == 6u) CHECK(fabsf(last_y) == 100.0f);
                if (stage == 15u) CHECK(fabsf(s_dist_ff_ratio + 0.075f) < 0.00001f &&
                                      fabsf(last_y - expected * 0.075f) < 0.00001f);
            }
            run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !host_timer_active);
        }
    }
    /* The same private slot cannot affect another route's executor. */
    reset_fixture(); run_cmd("31"); run_cmd("v600"); run_cmd("34");
    s_seq_stage = 1u; CHECK(route_seq_speed_mms() == 100.0f);
    run_cmd("36"); s_seq_stage = 1u; CHECK(route_seq_speed_mms() == 100.0f);
    run_cmd("37"); s_seq_stage = 0u; CHECK(route_seq_speed_mms() == 300.0f);
    puts("route31 v:all16 stages at1/100/200/600; ordinary back/forward only; explicit lateral100 override, turns, cross300/contact40, limits/acc/dec/ykp retained; 34/36/37 isolated passed");
    return 0;
}

static int speed_internal_turn_offsets(void)
{
    static const unsigned requested[]={1u,40u,80u,250u,600u};
    for(unsigned n=0u;n<5u;++n){
        unsigned lateral=requested[n];float capped=lateral<80u?(float)lateral:80.0f;
        CHECK(speed_boot_with_lateral(600u,lateral)==0);
        step_vision_receive_end();s_route31_rack_deployed=1u;
        s_seq_stage=ROUTE31_PAIR_STAGE;
        s_seq_pair_turn=0u;s_seq_pair_offset=1u;s_seq_return_offset=0u;
        route_seq_prepare();wire_sync();
        CHECK(s_seq_state==SQ_STILL && route_seq_leg()->mode==17u &&
              route_seq_leg()->distance_mm==15u && route_seq_leg()->heading_hold &&
              route_seq_speed_mms()==capped && s_v==capped &&
              s_route31_straight_v==600.0f && s_route31_lateral_v==(float)lateral);
        CHECK(sequence_start_stage()==0 && s_dist_target==-15.0f &&
              s_dist_align_enabled && last_y==-capped && !last_x);
        run_cmd("g");CHECK(s_seq_state==SQ_STOPPED&&!s_seq_pair_offset&&!s_seq_return_offset&&stopped());
    }
    CHECK(ROUTE31_RETURN_RIGHT_MM == 0u); /* Return-turn handoff is tested by the real task chain. */
    puts("31 firstLEFT15: actual executor min(pv,80) atpv1/40/80/250/600, straightv600 independent, heading/end-correction enabled; return lateral offset disabled passed");
    return 0;
}

static int speed_active_write_lock(void)
{
    static const char *const writes[] = {"v100", "v600", "pv100", "pv600", "d100", "ykp2", "fff0", "31", "co", "su1600"};
    for (unsigned phase = 0u; phase < 9u; ++phase) {
        CHECK(speed_prepare(phase == 8u ? 6u : phase == 7u ? 0u :
                            phase == 6u ? 9u : phase == 5u ? 8u : 1u, 200u) == 0);
        if (phase == 1u) { host_tick += T_DIST_STILL_MS; wire_poll(); CHECK(s_seq_state == SQ_WAIT); }
        if (phase >= 2u && phase <= 3u) CHECK(sequence_start_stage() == 0);
        if (phase >= 7u) CHECK(sequence_start_stage() == 0);
        if (phase == 3u) { host_fore = s_dist_odo0 + s_dist_target; wire_poll(); CHECK(s_round == R_BRAKE); }
        if (phase == 4u) s_seq_state = SQ_QR_WAIT;
        if (phase == 5u) { s_route31_rack_deployed = 0u; route_seq_prepare(); CHECK(s_seq_state == SQ_ARM_PREP); }
        float speed = s_v, distance = s_d, kp = s_route_heading_kp, ff = s_route_forward_ff_ratio;
        unsigned state = s_seq_state, round = s_round, timer = host_timer_active;
        float x = last_x, y = last_y, w = last_w;
        for (unsigned i = 0u; i < sizeof writes / sizeof writes[0]; ++i) {
            run_cmd(writes[i]);
            CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") && s_route31_straight_v == 200.0f &&
                  s_route31_lateral_v == 100.0f && s_v == speed && s_d == distance &&
                  s_route_heading_kp == kp && s_route_forward_ff_ratio == ff && !servo_calls &&
                  s_seq_state == state && s_round == (int)round && host_timer_active == (int)timer &&
                  last_x == x && last_y == y && last_w == w);
        }
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !host_timer_active);
    }
    puts("route31 v/pv:STILL/WAIT/RUN/BRAKE/QR_WAIT/PREDEPLOY/TASK/LEFT_RUN/RIGHT_RUN locks preserve active speed/distance/tuning and motor commands; g cancellation intact passed");
    return 0;
}

static int lateral_every_stage_scope(void)
{
    static const unsigned speeds[] = {1u, 100u, 200u, 600u};
    for (unsigned n = 0u; n < sizeof speeds / sizeof speeds[0]; ++n) {
        unsigned lateral = speeds[n];
        for (unsigned stage = 0u; stage < 16u; ++stage) {
            CHECK(speed_prepare_with_lateral(stage, 200u, lateral) == 0);
            int strafe = stage == 0u || stage == 6u;
            int ordinary = stage == 1u || stage == 5u || stage == 7u || stage == 12u || stage == 15u;
            int search = stage == 9u || stage == 11u || stage == 14u;
            float expected = strafe ? (float)lateral : ordinary || search ? 200.0f :
                             stage == 3u ? 300.0f : stage == 4u ? 40.0f : 100.0f;
            CHECK(route_seq_speed_mms() == expected && s_v == expected);
            if (strafe || ordinary || stage == 3u || stage == 4u) {
                host_messages[0] = '\0';
                CHECK(sequence_start_stage() == 0 && s_v == expected);
                CHECK(strstr(host_messages, "ROUTE31_SPEED") && strstr(host_messages, "straight_v=200"));
                char report[24]; snprintf(report, sizeof report, "lateral_v=%u", lateral);
                CHECK(strstr(host_messages, report));
                CHECK(s_dist_ramp.acc == 700.0f && s_dist_ramp.dec == 350.0f && s_dist_precise);
                CHECK(s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : stage == 6u ? 3.0f : 0.3f));
                if (strafe) {
                    CHECK(s_msel == (stage == 0u ? 17 : 18) && fabsf(last_y) == (float)lateral &&
                          last_x == 0.0f && last_w == 0.0f);
                    CHECK(s_dist_target == (stage == 0u ? -575.0f : 780.0f));
                } else CHECK(last_x == (s_msel == 16 ? -expected : expected));
            }
            run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !host_timer_active);
        }
    }
    /* Both existing strafe-FF slots continue to scale by the selected strafe
     * speed and retain their existing vehicle-frame sign, without setting v. */
    for (unsigned i = 0u; i < 2u; ++i) {
        CHECK(speed_boot_with_lateral(200u, 300u) == 0);
        run_cmd("g"); run_cmd("lff0.02"); run_cmd("rff0.03");
        run_cmd("g");
        step_vision_receive_end();
        s_route31_rack_deployed = 1u; s_seq_stage = i == 0u ? 0u : 6u;
        route_seq_prepare(); wire_sync();
        CHECK(sequence_start_stage() == 0 && s_v == 300.0f);
        CHECK(fabsf(last_x - (i == 0u ? -6.0f : 9.0f)) < 0.001f &&
              last_y == (i == 0u ? -300.0f : 300.0f) && s_route31_straight_v == 200.0f);
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED);
    }
    reset_fixture(); run_cmd("31"); run_cmd("v200"); run_cmd("pv600"); run_cmd("34");
    s_seq_stage = 0u; CHECK(route_seq_speed_mms() == 100.0f);
    run_cmd("36"); s_seq_stage = 0u; CHECK(route_seq_speed_mms() == 100.0f);
    run_cmd("37"); s_seq_stage = 0u; CHECK(route_seq_speed_mms() == 300.0f);
    puts("route31 pv:all16 stages at1/100/200/600; actual LEFT/RIGHT executor commands and compensated signs/units; straight200, cross300/contact40, turns/search/fine unchanged; 34/36/37 isolated passed");
    return 0;
}

static int speed_search_and_fine(void)
{
    static const unsigned stages[] = {9u, 11u, 14u};
    static const int models[] = {4, 6, 1}; /* QR111 selects red ball/red target/cylinder. */
    static const unsigned image_y[] = {390u, 120u, 220u};
    CHECK(VAT_ROUTE_BALL_X_PX == 135 && VAT_ROUTE_BUCKET_X_PX == 105 && VAT_ROUTE43_BUCKET_X_PX == 125 &&
          VAT_ROUTE_HOSTAGE_X_PX == 215 && VAT_TOL_PX == 10);
    for (unsigned i = 0u; i < 3u; ++i) {
        CHECK(speed_prepare_with_lateral(stages[i], 200u, 600u) == 0);
        if (i == 1u) CHECK(target35_point() == 250 && target35_low() == 247 && target35_high() == 253 &&
                           target35_fine_enter() == 360);
        else CHECK(s_vat.x_goal == (i == 0u ? 135 : 215));
        if (i != 1u) {
            CHECK(VAT_ROUTE_SEARCH_ACC_MMS2 == 700.0f);
            CHECK(fabsf(s_route_search_ff - (i == 2u ? 0.05f : 0.03f)) < 0.00001f);
            /* Explicit trial FF exercises the public setter while still
             * braked. It must scale with actual ramp speed, not cruise200. */
            CHECK(vision_align_test_route_search_ff_set(0.03f));
        }
        wire_ack(wire_request, 2u, 0u);
        host_tick += T_DIST_STILL_MS; wire_poll();
        host_tick += NAV_SETTLE_MS; wire_poll();
        if (i == 1u) {
            CHECK(last_x == 200.0f); /* target35 route coarse remains instant. */
            host_tick += VAT_POLL_MS; wire_poll();
        } else {
            CHECK(s_vat.state == VAT_ROUTE_SEARCH && last_x == 0.0f && last_y == 0.0f);
            host_tick += VAT_POLL_MS; wire_poll();
            if (fabsf(last_x - 14.0f) >= 0.00001f || fabsf(last_y - 14.0f * 0.03f) >= 0.00001f)
                fprintf(stderr,"search ramp task=%u tick=%u from=%u vx=%.9f vy=%.9f ff=%.9f state=%u\n",
                        stages[i],host_tick,s_route_search_from,last_x,last_y,s_route_search_ff,(unsigned)s_vat.state);
            CHECK(fabsf(last_x - 14.0f) < 0.00001f && fabsf(last_y - 14.0f * 0.03f) < 0.00001f);
            host_tick += VAT_POLL_MS; wire_poll();
            CHECK(fabsf(last_x - 28.0f) < 0.00001f && fabsf(last_y - 28.0f * 0.03f) < 0.00001f);
            host_tick += 245u; wire_poll();
            CHECK(fabsf(last_x - 199.5f) < 0.0001f &&
                  fabsf(last_y - last_x * 0.03f) < 0.00001f);
            host_tick += 1u; wire_poll();
            CHECK(last_x == 200.0f && fabsf(last_y - 6.0f) < 0.00001f);
        }
        CHECK(last_x == 200.0f && s_seq_state == SQ_TASK);
        speed_wire_object(wire_request, 1u, models[i], 400u, image_y[i]);
        CHECK(last_x == 200.0f);
        unsigned near_x = i == 1u ? 300u : (unsigned)s_vat.x_goal + 14u;
        host_tick += VAT_POLL_MS; speed_wire_object(wire_request, 2u, models[i], near_x, image_y[i]);
        if (i == 1u) {
            CHECK(s_target31_fine && last_x == 30.0f);
            host_tick += VAT_POLL_MS; speed_wire_object(wire_request, 3u, models[i], 400u, image_y[i]);
            CHECK(last_x == 30.0f); /* one-way fine latch never reopens v200 search. */
        } else {
            CHECK(last_x == 0.0f && s_vat.state == VAT_BRAKE);
            host_tick += T_DIST_STILL_MS; wire_poll();
            host_tick += VAT_POLL_MS; speed_wire_object(wire_request, 3u, models[i], near_x, image_y[i]);
            CHECK(s_vat.state == (VAT_Y_ALIGN_ENABLE ? VAT_STEP_MOVE : VAT_FINE_CONTINUOUS) &&
                  last_x == 20.0f && last_y == 0.0f);
        }
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !laser_state);
    }
    /* Search-speed setter cannot leak into the independent bucket mode. */
    reset_fixture(); run_cmd("31"); run_cmd("v600"); run_cmd("pv600"); run_cmd("39"); run_cmd("g");
    CHECK(!vision_align_test_route_search_speed_set(600.0f) && VAT_ROUTE_BUCKET_SEARCH_SPEED_MMS == -20.0f);
    run_cmd("g"); run_cmd("35"); CHECK(s_v == 50.0f && target35_fine_speed() == 50.0f &&
                                      target35_point() == 255 && target35_low() == 250 && target35_high() == 260);
    run_cmd("42"); CHECK(s_grab42_pps == 500u && s_grab42_u == 1900u && s_route31_straight_v == 600.0f &&
                         s_route31_lateral_v == 600.0f);
    puts("route31 v/pv:actual ACK ball/hostage acc700 ->0/14/28/199.5/clamp200 with FF on actual speed;target coarse instant200 despite pv600;farcx400 retains coarse,abs_error14 fineX20/target360 fine30 one-way latch;independent39/35/42 isolated passed");
    return 0;
}

int main(void)
{
    CHECK(speed_router_bounds_and_retention() == 0);
    CHECK(lateral_router_bounds_and_retention() == 0);
    CHECK(speed_default_stage_scope() == 0);
    CHECK(speed_every_stage_scope() == 0);
    CHECK(lateral_every_stage_scope() == 0);
    CHECK(speed_internal_turn_offsets() == 0);
    CHECK(speed_active_write_lock() == 0);
    CHECK(speed_search_and_fine() == 0);
    puts("route31 independent straight/lateral speed host regression passed; no hardware accepted");
    return 0;
}
