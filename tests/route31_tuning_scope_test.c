/* Actual Bluetooth router/state machine with the existing inert peripheral
 * fixture. These checks pin parameter ownership and command semantics only;
 * they do not simulate wheel traction, camera feedback or physical routes. */
#include <math.h>
#define main old_g_command_stop_main
#include "g_command_stop_test.c"
#undef main

#undef CHECK
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "route31 tuning scope line %d: %s\n", __LINE__, #expr); \
    return 1; \
} } while (0)

static int ff_equal(float a, float b)
{ return fabsf(a - b) < 0.000001f; }

static int check_report(const char *value, int local)
{
    char expected[40];
    host_messages[0] = '\0';
    run_cmd("param");
    snprintf(expected, sizeof expected, "fff=%s", value);
    CHECK(strstr(host_messages, expected) != NULL);
    CHECK((strstr(host_messages, "ROUTE_PROFILE mode=31 source=LOCAL") != NULL) == local);
    if (local) {
        const char *route_profile = strstr(host_messages, "ROUTE_PROFILE mode=31 source=LOCAL");
        const char *local_fff = strstr(route_profile, expected);
        const char *newline = strchr(route_profile, '\n');
        CHECK(local_fff != NULL && newline != NULL && local_fff < newline);
        snprintf(expected, sizeof expected, "ykp=%.3f", s_route_heading_kp);
        CHECK(strstr(route_profile, expected) != NULL && strstr(route_profile, expected) < newline);
        CHECK(strstr(host_messages, "source=ROUTE31 global_ykp=0.300") != NULL);
        CHECK(strstr(host_messages, expected) != NULL);
    } else {
        CHECK(strstr(host_messages, "source=ROUTE31") == NULL);
    }
    return 0;
}

static int check_defaults_and_direct_values(void)
{
    reset_fixture();
    CHECK(ROUTE_TEST_FORWARD_FF_SEED == -0.00625f);
    CHECK(s_forward_ff_ratio == -0.00625f && test_forward_ff_ratio() == 0.0125f);
    CHECK(s_route_forward_ff_ratio == -0.00625f);
    CHECK(ROUTE_TEST_HEADING_KP_SEED == 0.3f && s_route_heading_kp == 0.3f);
    CHECK(step_heading_kp_deg() == 0.3f);
    /* These legacy offset legs remain available to43, not executed by31. */
    CHECK(s_route31_plan[7].distance_mm == 760u &&
          s_route31_pair_offset_leg.distance_mm == 40u &&
          strcmp(s_route31_pair_offset_leg.name, "BALL_TO_BUCKET_LEFT40") == 0);
    CHECK(s_route31_return_offset_leg.mode == 17u &&
          s_route31_return_offset_leg.distance_mm == 20u &&
          s_route31_return_offset_leg.heading_hold == 1u);
    CHECK(s_route43_tune.road_mm[7] == 760u && s_route43_tune.pair_left_mm == 40u &&
          s_route43_tune.return_left_mm == 20u &&
          s_route43_tune.corner_mm[0] == 520u && s_route43_tune.corner_mm[1] == 420u &&
          s_route43_tune.corner_mm[2] == 320u);
    CHECK(check_report("0.01250", 0) == 0);
    run_cmd("31");
    CHECK(s_seq_state == SQ_READY && s_route_forward_ff_ratio == -0.00625f);
    CHECK(check_report("-0.00625", 1) == 0);

    static const char *const inputs[] = { "fff0.01000", "fff0", "fff-0.02000", "fff0.04999", "fff-0.04999" };
    static const float expected[] = { 0.01f, 0.0f, -0.02f, 0.04999f, -0.04999f };
    for (unsigned i = 0; i < sizeof inputs / sizeof inputs[0]; ++i) {
        run_cmd(inputs[i]);
        if (!ff_equal(s_route_forward_ff_ratio, expected[i]))
            fprintf(stderr, "input=%s actual=%.9f expected=%.9f last=%s\n",
                    inputs[i], s_route_forward_ff_ratio, expected[i], last_message);
        CHECK(ff_equal(s_route_forward_ff_ratio, expected[i]));
        CHECK(s_forward_ff_ratio == -0.00625f && test_forward_ff_ratio() == 0.0125f);
    }
    run_cmd("fff0.00625"); CHECK(check_report("0.00625", 1) == 0);
    run_cmd("15"); run_cmd("v100"); run_cmd("d500"); run_cmd("g"); tick();
    CHECK(s_dist_ff_ratio == -0.00625f && last_x == 100.0f && last_y == 0.625f);
    run_cmd("g"); host_tick += T_DIST_STILL_MS; test_poll();
    run_cmd("32");
    CHECK(s_msel == 32 && s_seq_state == SQ_OFF && test_forward_ff_ratio() == 0.0125f);
    CHECK(check_report("0.01250", 0) == 0);
    return 0;
}

static int check_all_route_legs(void)
{
    static const int modes[9] = {17,16,20,16,15,16,18,16,30};
    static const int commands[9] = {-535,-630,90,-650,0,-190,800,-760,-90};
    static const float speeds[9] = {250,200,100,300,40,200,250,200,100};
    const float direct_ff = 0.01f;
    unsigned forward_legs = 0u, translation_legs = 0u, ff_legs = 0u, heading_legs = 0u;
    reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("g");
    CHECK(ROUTE31_STAGES == 16u && route_seq_stage_count() == 16u && !route_seq_bucket_enabled());
    for (unsigned stage = 0; stage < 9u; ++stage) {
        CHECK(s_seq_stage == stage && sequence_start_stage() == 0);
        CHECK(s_msel == modes[stage]);
        CHECK(s_route31_plan[stage].heading_hold == (stage == 3u || stage == 4u ? 0u : 1u));
        CHECK(ff_equal(s_route_forward_ff_ratio, direct_ff) && test_forward_ff_ratio() == 0.0125f);
        if (dist_mode()) {
            ++translation_legs;
            CHECK(s_dist_target == (float)commands[stage] && s_v == speeds[stage]);
            const float expected_kp = stage == 3u || stage == 4u ? 0.0f : stage == 6u ? 3.0f : 0.3f;
            CHECK(s_dist_heading_kp == expected_kp && s_route_heading_kp == 0.3f);
            if (expected_kp != 0.0f) ++heading_legs;
            CHECK(step_heading_kp_deg() == 0.3f);
            CHECK(ff_equal(s_dist_ff_ratio, stage == 1u || stage == 5u || stage == 7u ? 0.00625f : 0.0f));
            if (s_dist_ff_ratio != 0.0f) ++ff_legs;
            if (s_msel == 15) {
                ++forward_legs;
                CHECK(last_x == speeds[stage] &&
                      ff_equal(last_y, stage == 4u ? 0.0f : -direct_ff * speeds[stage]));
            } else if (s_msel == 16) {
                CHECK(last_x == -speeds[stage] && last_y == -s_dist_ff_ratio * speeds[stage]);
            } else {
                CHECK(last_x == 0.0f && last_y == (s_msel == 18 ? speeds[stage] : -speeds[stage]));
            }
            host_yaw = 3.0f; tick();
            CHECK(ff_equal(last_w, -expected_kp * 3.0f * 0.0174533f));
            host_yaw = -3.0f; tick();
            CHECK(ff_equal(last_w, expected_kp * 3.0f * 0.0174533f));
            host_yaw = 0.0f;
        } else {
            CHECK(turn_target_deg() == (float)commands[stage]);
        }
        CHECK(sequence_finish_stage() == 0);
        CHECK(host_target_calls == (stage == 8u ? 1 : 0) && s_seq_state != SQ_BUCKET_ALIGN && s_seq_state != SQ_MANUAL_D_WAIT);
    }
    CHECK(forward_legs == 1u && ff_legs == 3u && translation_legs == 7u && heading_legs == 5u);
    CHECK(s_seq_state == SQ_TASK && s_seq_stage == ROUTE31_PAIR_STAGE && vision_align_test_active() && s_vat.mode == 41u);
    CHECK(s_route_heading_kp == 0.3f && step_heading_kp_deg() == 0.3f);
    CHECK(ff_equal(s_route_forward_ff_ratio, direct_ff) && test_forward_ff_ratio() == 0.0125f);
    CHECK(check_report("0.01000", 1) == 0);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !vision_align_test_active()); run_cmd("31");
    CHECK(s_seq_state == SQ_READY && ff_equal(s_route_forward_ff_ratio, direct_ff));
    CHECK(check_report("0.01000", 1) == 0);
    run_cmd("22"); CHECK(turn_target_deg() == 180.0f);
    run_cmd("30"); CHECK(turn_target_deg() == -92.0f);
    run_cmd("20"); CHECK(turn_target_deg() == 90.0f);
    CHECK(pulse_calls == 3200 && !host_timer_active && servo_calls == 0 && !laser_state && !s_go);
    return 0;
}

static int check_route_ff_sign_dispatch(void)
{
    static const char *const selections[] = { "31", "34" };
    static const char *const commands[] = { "", "fff0", "fff-0.02000", "fff0.02000" };
    static const float ratios[] = { -0.00625f, 0.0f, -0.02f, 0.02f };
    static const float lateral[] = { 0.625f, 0.0f, 2.0f, -2.0f };
    for (unsigned mode = 0u; mode < 2u; ++mode) {
        for (unsigned input = 0u; input < 4u; ++input) {
            reset_fixture(); run_cmd(selections[mode]);
            if (commands[input][0]) run_cmd(commands[input]);
            CHECK(ff_equal(s_route_forward_ff_ratio, ratios[input]) && test_forward_ff_ratio() == 0.0125f);
            run_cmd("g");
            s_seq_qr[0] = 1; s_seq_qr[1] = 1; s_seq_qr[2] = 3; /* Already locked at R1; task12 uses target color. */
            s_seq_stage = mode == 0u ? ROUTE31_TARGET_CORNER_STAGE : 10u; route_seq_prepare();
            CHECK(sequence_start_stage() == 0);
            const float expected_speed = mode == 0u ? 200.0f : 100.0f;
            CHECK(s_msel == 15 && s_dist_target == (mode == 0u ? 520.0f : 780.0f) && s_v == expected_speed);
            const float effective = mode == 0u ? -ROUTE31_CORNER_RIGHT_FF_RATIO : ratios[input];
            const float expected_lateral = mode == 0u ? 200.0f * ROUTE31_CORNER_RIGHT_FF_RATIO : lateral[input];
            CHECK(ff_equal(s_dist_ff_ratio, effective) && last_x == expected_speed && ff_equal(last_y, expected_lateral));
            CHECK(ff_equal(s_route_forward_ff_ratio, ratios[input])); /* Stage12 must not rewrite ordinary routeFFF. */
            CHECK(step_heading_kp_deg() == 0.3f && s_forward_ff_ratio == -0.00625f);
            run_cmd("g"); run_cmd("0"); test_init();
            CHECK(s_route_forward_ff_ratio == -0.00625f && test_forward_ff_ratio() == 0.0125f);
        }
    }
    puts("route31stage12 always private CORNER_RIGHT_FF independent of ordinaryFFF inputs; retained34 FFF zero/negative/positive lateral signs; routeRAM/manual/32 unchanged passed");
    return 0;
}

static int check_ram_retention_and_separation(void)
{
    static const char *const stop_keys[] = { "g", "a", "0" };
    for (unsigned key = 0; key < sizeof stop_keys / sizeof stop_keys[0]; ++key) {
        reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("ykp2"); run_cmd("g");
        CHECK(sequence_start_stage() == 0);
        CHECK(s_dist_heading_kp == 2.0f);
        run_cmd(stop_keys[key]);
        CHECK(s_seq_state == SQ_STOPPED && ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
        CHECK(test_forward_ff_ratio() == 0.0125f);
        CHECK(check_report("0.01000", 1) == 0);
        run_cmd("31"); CHECK(s_seq_state == SQ_READY && ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(s_route_heading_kp == 2.0f);
        run_cmd("15"); run_cmd("fff0.02000");
        CHECK(ff_equal(s_forward_ff_ratio, 0.02f) && test_forward_ff_ratio() == 0.0125f &&
              ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(check_report("0.02000", 0) == 0);
        run_cmd("32");
        CHECK(test_forward_ff_ratio() == 0.0125f && ff_equal(s_route_forward_ff_ratio, 0.01f));
        CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
        CHECK(check_report("0.01250", 0) == 0);
        run_cmd("31"); CHECK(check_report("0.01000", 1) == 0);
        run_cmd("0"); test_init();
        CHECK(s_seq_state == SQ_OFF && s_route_forward_ff_ratio == -0.00625f);
        CHECK(s_forward_ff_ratio == -0.00625f && test_forward_ff_ratio() == 0.0125f);
        CHECK(s_route_heading_kp == 0.3f && step_heading_kp_deg() == 0.3f);
    }
    return 0;
}

static void tuning_pair_wire_diag(ProtoWireDiag *out)
{
    memset(out, 0, sizeof *out);
    out->request = (uint16_t)(100u + (unsigned)host_target_calls);
    out->mode = 2u; out->controlled = 1u;
    out->receiving = (uint8_t)!host_receive_closed;
    out->ack = (uint8_t)(host_scene_status == 1);
    out->task = (uint8_t)host_target_task; out->selection = host_target_digit;
}

static int check_pair_turn_offset_scope(void)
{
    const int32_t qr[3] = {1, 2, 3};
    for (unsigned i = 0u; i < 2u; ++i) {
        const unsigned hold = i ? 700u : 400u;
        ProtoFrame old_bucket = { .type=PF_OBJ, .cls=CLS_BUCKET, .label=0,
            .cx=125, .cy=420, .w=20, .h=20, .conf=90,
            .sequence=10u, .img_w=640u, .img_h=480u };
        reset_fixture(); run_cmd(i ? "43" : "31"); run_cmd("g");
        memcpy(s_seq_qr, qr, sizeof qr); host_wire_diag_hook = tuning_pair_wire_diag;
        s_seq_stage = ROUTE31_PAIR_STAGE; route_seq_prepare();
        CHECK(s_seq_state == SQ_TASK && host_target_task == PROTO_TASK_BALL);
        /* Executor entry only: rank transport/grabbing is covered separately.
         * This does not bypass or weaken the production WAIT_BALL_RANK gate. */
        s_vat.ball_rank = 1u; vat_ball_begin_bucket_turn(); test_poll();
        CHECK(s_seq_pair_turn && s_vat.state == VAT_TURN_ACTIVE && host_target_calls == 2);
        if (i) { CHECK(s_seq_state == SQ_STEP_WAIT); run_cmd("g"); }
        CHECK(sequence_start_stage() == 0 && s_msel == 22 && turn_target_deg() == 180.0f);
        host_scene_status = 1; ++host_proto_stats.obj;
        vision_align_test_feed_frame(&old_bucket);
        CHECK(s_sample.seen && s_sample.frame.sequence == 10u);
        const uint16_t preturn_request = s_vat.request;
        host_yaw = turn_target_deg(); test_poll();
        CHECK(s_round == R_BRAKE && s_seq_pair_turn && host_target_calls == 2 &&
              !s_seq_pair_offset && s_seq_stage == ROUTE31_PAIR_STAGE && !last_x && !last_y && !last_w);
        CHECK(turn_settle_ms() == hold);
        host_tick += hold - 1u; test_poll();
        CHECK(s_round == R_BRAKE && s_seq_pair_turn && host_target_calls == 2 && !s_seq_pair_offset);
        ++host_tick; test_poll();
        CHECK(!s_seq_pair_turn && s_seq_stage == ROUTE31_PAIR_STAGE && !s_target35_route && !laser_state);
        if (i) {
            CHECK(s_seq_pair_offset && s_seq_state == SQ_STEP_WAIT &&
                  route_seq_leg()->mode == 17u && route_seq_leg()->distance_mm == 40u &&
                  host_target_calls == 2 && s_vat.request == preturn_request &&
                  s_vat.state == VAT_TURN_ACTIVE && !last_x && !last_y && !last_w);
            host_tick += 1000u; test_poll();
            CHECK(s_seq_state == SQ_STEP_WAIT && host_target_calls == 2);
        } else {
            CHECK(!s_seq_pair_offset && !s_seq_return_offset && s_seq_state == SQ_TASK &&
                  s_msel == 31 && host_target_calls == 3 && host_target_task == PROTO_TASK_BUCKET &&
                  host_target_digit == 0u && s_vat.request != preturn_request &&
                  s_vat.state == VAT_BRAKE && s_need_new && !s_have_seq &&
                  !s_sample.seen && !s_vat.latest && !s_vat.good && !host_scene_status &&
                  !last_x && !last_y && !last_w);
            ++host_proto_stats.obj; old_bucket.sequence++;
            vision_align_test_feed_frame(&old_bucket); /* No matching newACK yet. */
            CHECK(!s_sample.seen && !vat_ack_ready());
            host_tick += T_DIST_STILL_MS + VAT_POLL_MS; test_poll();
            CHECK(s_seq_state == SQ_TASK && !s_vat.good && !s_sample.seen &&
                  !last_x && !last_y && !last_w && !laser_state);
            host_scene_status = 1;
            CHECK(vat_ack_ready() && !s_sample.seen && !s_vat.good);
            ++host_proto_stats.obj; old_bucket.sequence++;
            vision_align_test_feed_frame(&old_bucket);
            CHECK(s_sample.seen && s_sample.frame.sequence == old_bucket.sequence &&
                  s_sample.packet != s_session_packet);
        }
        run_cmd("a");
        CHECK(s_seq_state == SQ_STOPPED && !vision_align_test_active() && host_receive_closed &&
              !s_seq_pair_turn && !s_seq_pair_offset && !s_seq_return_offset &&
              !last_x && !last_y && !last_w && !laser_state);
    }
    puts("pair180:31 holds400 then new bucket request, discards turn pixels and gates matchingACK/new frame;43 holds700 then preserves gatedLEFT40; manual stop passed");
    return 0;
}

static int check_new_task_tail_and_dynamic_motion(void)
{
    static const uint8_t tail_modes[7] = {41u,22u,35u,15u,20u,40u,15u};
    static const uint16_t target_to_corner[3] = {520u,420u,320u};
    CHECK(ROUTE31_LASER_MS == 2000u);
    CHECK(ROUTE31_BALL_GRIP_WAIT_MS == 2000u && ROUTE31_BUCKET_RELEASE_WAIT_MS == 2000u &&
          ROUTE31_BUCKET_LIFT_WAIT_MS == 0u && ROUTE31_HOSTAGE_HOLD_MS == 0u);
    CHECK(ROUTE31_TARGET_SEARCH_RIGHT_FF_RATIO == 0.065f && ROUTE31_CORNER_RIGHT_FF_RATIO == 0.065f &&
          ROUTE31_HOSTAGE_SEARCH_RIGHT_FF_RATIO == 0.03f && ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO == 0.02f);
    for(unsigned i=0u;i<7u;i++) {
        CHECK(s_route31_plan[9u+i].mode == tail_modes[i]);
        CHECK(s_route31_plan[9u+i].distance_mm == 0u); /* No old2450/2125 appended. */
    }
    for(unsigned color=1u;color<=3u;color++) {
        reset_fixture();run_cmd("31");run_cmd("fff0.01000");run_cmd("ykp1.2");run_cmd("g");
        s_seq_qr[0]=1;s_seq_qr[1]=(int32_t)color;s_seq_qr[2]=3;
        s_seq_stage=ROUTE31_TARGET_CORNER_STAGE;route_seq_prepare();
        CHECK(s_seq_state==SQ_STILL && s_d==(float)target_to_corner[color-1u]);
        CHECK(route_seq_leg()->distance_mm==target_to_corner[color-1u]);
        CHECK(sequence_start_stage()==0);
        CHECK(s_msel==15 && s_dist_target==(float)target_to_corner[color-1u] && s_v==200.0f);
        CHECK(ff_equal(s_dist_ff_ratio,-ROUTE31_CORNER_RIGHT_FF_RATIO) && s_dist_heading_kp==1.2f &&
              last_x==200.0f && ff_equal(last_y,200.0f*ROUTE31_CORNER_RIGHT_FF_RATIO) &&
              step_heading_kp_deg()==0.3f);
        CHECK(ff_equal(s_route_forward_ff_ratio,0.01f));
        CHECK(sequence_finish_stage()==0 && s_seq_stage==ROUTE31_HOSTAGE_TURN_STAGE);
        CHECK(sequence_start_stage()==0 && s_msel==20 && turn_target_deg()==90.0f);
        CHECK(sequence_finish_stage()==0 && s_seq_stage==ROUTE31_HOSTAGE_STAGE && s_seq_state==SQ_TASK);
        CHECK(vision_align_test_active() && s_vat.mode==40u && s_vat.state==VAT_BRAKE &&
              host_target_task==PROTO_TASK_HOSTAGE && host_target_digit==3u);
        CHECK(ff_equal(s_route_search_ff,ROUTE31_HOSTAGE_SEARCH_RIGHT_FF_RATIO));
        CHECK(!laser_state && !pulse_calls && !servo_calls && !s_go);
        run_cmd("g");CHECK(s_seq_state==SQ_STOPPED && !vision_align_test_active() && host_receive_closed);
    }
    /*31 goes directly from the completed stable return180 to selected task2.
     *43 retains LEFT20 and its next-g gate; neither may bypass turn stability. */
    for (unsigned i=0u;i<2u;++i) {
        const unsigned hold=i ? 700u : 400u;
        reset_fixture();run_cmd(i ? "43" : "31");run_cmd("g");
        s_seq_qr[0]=1;s_seq_qr[1]=2;s_seq_qr[2]=3;
        s_seq_stage=ROUTE31_RETURN180_STAGE;route_seq_prepare();
        CHECK(sequence_start_stage()==0 && s_msel==22 && turn_target_deg()==180.0f);
        host_yaw=turn_target_deg();test_poll();
        CHECK(s_round==R_BRAKE && !s_seq_return_offset && !s_target35_route &&
              s_seq_stage==ROUTE31_RETURN180_STAGE && !host_target_calls && !laser_state);
        CHECK(turn_settle_ms()==hold);
        host_tick+=hold-1u;test_poll();
        CHECK(s_round==R_BRAKE && s_seq_stage==ROUTE31_RETURN180_STAGE &&
              !s_seq_return_offset && !host_target_calls && !last_x && !last_y && !last_w);
        ++host_tick;test_poll();
        if (i) {
            CHECK(s_seq_return_offset && s_seq_stage==ROUTE31_RETURN180_STAGE &&
                  s_seq_state==SQ_STEP_WAIT && !s_target35_route && !host_target_calls &&
                  route_seq_leg()->mode==17u && route_seq_leg()->distance_mm==20u);
            host_tick+=1000u;test_poll();
            CHECK(s_seq_state==SQ_STEP_WAIT && !s_target35_route && !host_target_calls);
            run_cmd("g");CHECK(sequence_start_stage()==0 && s_msel==17 &&
                  s_dist_target==-20.0f && s_dist_align_enabled && s_dist_heading_kp==0.3f);
        } else {
            CHECK(!s_seq_return_offset && !s_seq_pair_offset &&
                  s_seq_stage==ROUTE31_TARGET_STAGE && s_seq_state==SQ_TASK);
            CHECK(s_target35_route && host_target_calls==1 && host_target_task==PROTO_TASK_TARGET &&
                  host_target_digit==2u && !laser_state && !last_x && !last_y && !last_w);
            CHECK(target35_point()==240 && target35_low()==237 && target35_high()==243);
        }
        run_cmd("a");CHECK(s_seq_state==SQ_STOPPED && !s_target35_route && !laser_state);
    }
    run_cmd("35"); CHECK(target35_point()==255 && target35_low()==250 && target35_high()==260);
    puts("route31 new tail:return180 stable400 then direct target240[237..243], noLEFT20;43 stable700 retains gatedLEFT20; dynamic520/420/320/right90/hostage40/dynamicrankexit15;16stages,grip/release2s retained;standalone35 stays255;parameter ownership/handoffs and stop passed");
    return 0;
}

static int check_route_workpoint_isolation(void)
{
    const int32_t qr[3] = {1,2,3};
    CHECK(VAT_ROUTE_BALL_X_PX == 135 && VAT_ROUTE_BUCKET_X_PX == 125 &&
          VAT_ROUTE_HOSTAGE_X_PX == 215 && VAT_X_PX == 190);
    CHECK(VAT_TOL_PX == 10);
    CHECK(VAT_BALL_Y_PX == 390 && VAT_BUCKET_Y_PX == 420 && VAT_HOSTAGE_Y_PX == 220);

    reset_fixture();
    CHECK(vision_align_test_start_route(41u, qr));
    CHECK(s_route_owned && s_vat.task == PROTO_TASK_BALL && s_vat.x_goal == 135);
    /* Exercise the actual route request goal dispatcher independently of
     * the mechanical/turn chain, whose integration is tested separately. */
    CHECK(vat_request(PROTO_TASK_BUCKET, 0u, host_tick));
    CHECK(s_route_owned && s_vat.task == PROTO_TASK_BUCKET && s_vat.x_goal == 125);
    CHECK(!pulse_calls && !servo_calls && !laser_state && !s_go);

    reset_fixture();
    CHECK(vision_align_test_start_route(40u, qr));
    CHECK(s_route_owned && s_vat.task == PROTO_TASK_HOSTAGE && s_vat.x_goal == 215);
    CHECK(!pulse_calls && !servo_calls && !laser_state && !s_go);

    /* The route-private hostage215 must not change any standalone38..41 goal. */
    for (unsigned mode = 38u; mode <= 41u; ++mode) {
        reset_fixture();
        if (mode != 39u) {
            proto_send_scene(SCENE_QR);
            host_scene_status = 1; /* Explicit fixture ACK before legal QR. */
            host_qr_accept(1,2,3);
        }
        CHECK(vision_align_test_start((uint8_t)mode, mode == 39u ? NULL : qr));
        CHECK(!s_route_owned && s_vat.x_goal == 190);
        CHECK(!pulse_calls && !servo_calls && !laser_state && !s_go);
    }
    puts("route31 workpoints: ball135/bucket125/hostage215 private; tolerance10/Y390/420/220 unchanged and standalone38..41 remainX190 passed");
    return 0;
}

static int check_hostage_exit_parameter_scope(void)
{
    static const unsigned distances[] = {1415u,1315u,1215u};
    static const char *const commands[] = {"fff0", "fff-0.02000", "fff0.01000"};
    static const float ratios[] = {0.0f,-0.02f,0.01f};
    for (unsigned rank = 1u; rank <= 3u; ++rank)
        for (unsigned input = 0u; input < 3u; ++input) {
            reset_fixture(); run_cmd("31"); run_cmd(commands[input]); run_cmd("ykp1.2"); run_cmd("g");
            /* Executor-only fixture: inject an already validated historical
             * rank. Real54 parsing/alignment/no extra hold is tested in task_chain. */
            s_seq_qr[0]=1; s_seq_qr[1]=2; s_seq_qr[2]=(int32_t)(rank % 3u + 1u);
            s_route31_hostage_rank=(uint8_t)rank;
            s_seq_stage=ROUTE31_HOSTAGE_EXIT_STAGE; route_seq_prepare();
            CHECK(s_seq_state==SQ_STILL && s_d==(float)distances[rank-1u]);
            CHECK(route_seq_leg()->distance_mm==distances[rank-1u] && route_seq_leg()->heading_hold);
            CHECK(sequence_start_stage()==0 && s_msel==15 && s_v==200.0f);
            CHECK(s_dist_target==(float)distances[rank-1u] && s_dist_heading_kp==1.2f && s_dist_align_enabled);
            CHECK(ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO == 0.02f);
            CHECK(ff_equal(s_dist_ff_ratio,-ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO) && last_x==200.0f &&
                  ff_equal(last_y,200.0f*ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO));
            CHECK(ff_equal(s_route_forward_ff_ratio,ratios[input]) && test_forward_ff_ratio()==0.0125f);
            CHECK(sequence_finish_stage()==0 && s_seq_state==SQ_DONE && s_seq_stage==ROUTE31_HOSTAGE_EXIT_STAGE);
            CHECK(!laser_state && !pulse_calls && !servo_calls && !s_go && last_x==0.0f && last_y==0.0f);
        }
    puts("route31 final15 executor: ranks1/2/3 ->1415/1315/1215 atv200 independently of QR shape, speed-scaled private HOSTAGE_EXIT_RIGHT_FF despite ordinaryFFF zero/negative/positive; ordinaryFFF RAM and routeykp/yawfix preserved, terminal STOP passed");
    return 0;
}

static int check_invalid_inputs_and_phase_locks(void)
{
    static const char *const invalid[] = {
        "fff0.05001", "fff-0.05001", "fff", "fffnan", "fffinf", "fff1e-3", "fff0.01x", "fff--0.01"
    };
    reset_fixture(); run_cmd("31"); run_cmd("fff0.01000");
    for (unsigned i = 0; i < sizeof invalid / sizeof invalid[0]; ++i) {
        run_cmd(invalid[i]);
        CHECK(strstr(last_message, "ERR ") != NULL);
        CHECK(ff_equal(s_route_forward_ff_ratio, 0.01f) && test_forward_ff_ratio() == 0.0125f);
    }
    static const char *const invalid_ykp[] = {
        "ykp5.001", "ykp-0.001", "ykp", "ykpnan", "ykpinf", "ykp1e-3", "ykp2x", "ykp--2"
    };
    run_cmd("ykp2");
    for (unsigned i = 0; i < sizeof invalid_ykp / sizeof invalid_ykp[0]; ++i) {
        run_cmd(invalid_ykp[i]);
        CHECK(strstr(last_message, "ERR ") != NULL);
        CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
    }
    /* Four ordinary phases at the nine prefix road nodes. */
    for (unsigned stage = 0; stage < 9u; ++stage) {
        for (unsigned phase = 0; phase < 4u; ++phase) {
            reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("ykp2"); run_cmd("g");
            s_seq_stage = (uint8_t)stage; route_seq_prepare();
            /* The new predeployment precedes stage8's original four road phases. */
            if (stage == ROUTE31_PREDEPLOY_STAGE)
                CHECK(fixture_complete_route31_predeploy() == 0);
            if (phase == 1u) {
                host_tick += T_DIST_STILL_MS; test_poll(); CHECK(s_seq_state == SQ_WAIT);
            }
            if (phase >= 2u) CHECK(sequence_start_stage() == 0);
            if (phase == 3u) {
                if (route31_owner() && s_seq_stage == 4u) {
                    CHECK(fixture_trigger_board_contact() == 0);
                } else if (dist_mode()) {
                    if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
                    else host_fore = s_dist_odo0 + s_dist_target;
                } else host_yaw = turn_target_deg();
                test_poll(); CHECK(s_round == R_BRAKE);
            }
            run_cmd("fff0.00400");
            CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
            CHECK(ff_equal(s_route_forward_ff_ratio, 0.01f) && test_forward_ff_ratio() == 0.0125f);
            run_cmd("ykp4");
            CHECK(strstr(last_message, "ERR ROUTE_SEQ_ACTIVE") != NULL);
            CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
            if (dist_mode() && phase >= 2u)
                CHECK(s_dist_heading_kp == (stage == 3u || stage == 4u ? 0.0f : stage == 6u ? 3.0f : 2.0f));
            CHECK(s_seq_stage == stage && route_seq_active());
            run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED);
        }
    }
    return 0;
}

static int check_direction_exclusion(void)
{
    static const int modes[] = {16,15,16};
    static const float targets[] = {-650.0f,0.0f,-190.0f};
    static const float velocities[] = {-300.0f,40.0f,-200.0f};
    /* Crossing/contact have no FF. Clearance uses the independent BFF,
     * never the caller's forward FFF. */
    for (unsigned i = 0u; i < 3u; ++i) {
        reset_fixture(); run_cmd("31"); run_cmd("fff0.01000"); run_cmd("g");
        s_seq_stage = (uint8_t)(3u + i); route_seq_prepare();
        CHECK(sequence_start_stage() == 0);
        CHECK(s_msel == modes[i] && s_dist_target == targets[i] && last_x == velocities[i]);
        CHECK(s_dist_ff_ratio == (i == 2u ? 0.00625f : 0.0f) &&
              s_dist_heading_kp == (i == 2u ? 0.3f : 0.0f) &&
              last_y == (i == 2u ? -200.0f * 0.00625f : 0.0f));
        CHECK(ff_equal(s_route_forward_ff_ratio, 0.01f) && test_forward_ff_ratio() == 0.0125f);
    }
    return 0;
}

static int check_heading_slot_direct_values_and_isolation(void)
{
    int manual_profile;
    reset_fixture(); run_cmd("17"); run_cmd("v100"); run_cmd("ykp5");
    CHECK(dist_heading_kp_get(&manual_profile) == 5.0f && manual_profile == 1);
    CHECK(step_heading_kp_deg() == 0.3f && s_route_heading_kp == 0.3f);
    /* Equal default numbers must not hide accidental ownership coupling.
     * Set a distinct global gain through the real command router, then prove
     * route31 still selects and dispatches its independent 0.3 RAM slot. */
    run_cmd("20"); run_cmd("ykp1");
    CHECK(step_heading_kp_deg() == 1.0f && s_route_heading_kp == 0.3f);
    run_cmd("31");
    CHECK(dist_heading_kp_get(&manual_profile) == 0.3f && manual_profile == 0);
    host_messages[0] = '\0'; run_cmd("param");
    CHECK(strstr(host_messages, "ykp=0.300 source=ROUTE31 global_ykp=1.000") != NULL);
    run_cmd("g"); CHECK(sequence_start_stage() == 0);
    host_yaw = 3.0f; tick();
    CHECK(s_dist_heading_kp == 0.3f && ff_equal(last_w, -0.3f * 3.0f * 0.0174533f));
    run_cmd("g"); run_cmd("20"); run_cmd("ykp0.3");
    CHECK(step_heading_kp_deg() == 0.3f && s_route_heading_kp == 0.3f);
    run_cmd("31"); run_cmd("ykp2");
    CHECK(s_route_heading_kp == 2.0f && step_heading_kp_deg() == 0.3f);
    CHECK(check_report("-0.00625", 1) == 0);
    run_cmd("g"); CHECK(sequence_start_stage() == 0);
    CHECK(s_dist_heading_kp == 2.0f);
    host_yaw = 3.0f; tick(); CHECK(ff_equal(last_w, -2.0f * 3.0f * 0.0174533f));
    host_yaw = -3.0f; tick(); CHECK(ff_equal(last_w, 2.0f * 3.0f * 0.0174533f));
    CHECK(host_precise_calls > 0u && host_integer_calls == 0u);
    run_cmd("g"); run_cmd("31"); run_cmd("ykp0");
    CHECK(s_route_heading_kp == 0.0f && step_heading_kp_deg() == 0.3f);
    run_cmd("g"); CHECK(sequence_start_stage() == 0);
    host_yaw = 3.0f; tick(); CHECK(s_dist_heading_kp == 0.0f && last_w == 0.0f);
    run_cmd("g"); run_cmd("31"); run_cmd("ykp5");
    CHECK(s_route_heading_kp == 5.0f && step_heading_kp_deg() == 0.3f);
    run_cmd("17"); run_cmd("v100");
    CHECK(dist_heading_kp_get(&manual_profile) == 5.0f && manual_profile == 1);
    run_cmd("param"); CHECK(strstr(last_message, "source=PROFILE global_ykp=0.300") != NULL);
    run_cmd("32"); run_cmd("param");
    CHECK(strstr(last_message, "ykp=0.300 source=GLOBAL global_ykp=0.300") != NULL);
    CHECK(step_heading_kp_deg() == 0.3f && s_route_heading_kp == 5.0f);
    run_cmd("31"); CHECK(s_route_heading_kp == 5.0f);
    CHECK(check_report("-0.00625", 1) == 0);
    run_cmd("0"); test_init();
    CHECK(s_route_heading_kp == 0.3f && step_heading_kp_deg() == 0.3f);
    run_cmd("17"); run_cmd("v100");
    CHECK(dist_heading_kp_get(&manual_profile) == 0.3f && manual_profile == 0);
    return 0;
}

static int check_exit_yaw_private_slot(void)
{
    reset_fixture(); run_cmd("31");
    CHECK(s_route31_exit_yaw_kp == 3.0f && s_route_heading_kp == 0.3f);
    const char *const invalid[] = {"xkp-1", "xkp5.1", "xkpnan", "xkpinf", "xkp2x"};
    for (unsigned i = 0u; i < sizeof invalid / sizeof invalid[0]; ++i) {
        run_cmd(invalid[i]); CHECK(s_route31_exit_yaw_kp == 3.0f && !last_w);
    }
    run_cmd("xkp0"); CHECK(s_route31_exit_yaw_kp == 0.0f);
    run_cmd("xkp5"); CHECK(s_route31_exit_yaw_kp == 5.0f);
    run_cmd("xkp2.5"); run_cmd("ykp1.2"); run_cmd("g");
    s_seq_stage = 6u; route_seq_prepare();
    CHECK(sequence_start_stage() == 0 && s_msel == 18 && s_dist_heading_kp == 2.5f);
    host_yaw = 3.0f; tick();
    CHECK(ff_equal(last_w, -2.5f * 3.0f * 0.0174533f));
    run_cmd("xkp4"); CHECK(s_route31_exit_yaw_kp == 2.5f && s_dist_heading_kp == 2.5f);
    CHECK(s_route_heading_kp == 1.2f && step_heading_kp_deg() == 0.3f);
    run_cmd("g"); run_cmd("43"); run_cmd("xkp1");
    CHECK(s_route43_tune.exit_yaw_kp == 1.0f && s_route31_exit_yaw_kp == 2.5f);
    run_cmd("31"); CHECK(s_route31_exit_yaw_kp == 2.5f);
    run_cmd("17"); run_cmd("xkp4");
    CHECK(s_route31_exit_yaw_kp == 2.5f && s_route43_tune.exit_yaw_kp == 1.0f);
    run_cmd("0"); test_init();
    CHECK(s_route31_exit_yaw_kp == 3.0f && s_route43_tune.exit_yaw_kp == 3.0f);
    puts("route31/43 xkp:0..5 private RIGHT800 gain; actual yaw command, malformed rejection, active lock, retention/reset and ordinary/manual isolation passed");
    return 0;
}

int main(void)
{
    CHECK(check_defaults_and_direct_values() == 0);
    CHECK(check_all_route_legs() == 0);
    CHECK(check_route_ff_sign_dispatch() == 0);
    CHECK(check_ram_retention_and_separation() == 0);
    CHECK(check_pair_turn_offset_scope() == 0);
    CHECK(check_new_task_tail_and_dynamic_motion() == 0);
    CHECK(check_route_workpoint_isolation() == 0);
    CHECK(check_hostage_exit_parameter_scope() == 0);
    CHECK(check_invalid_inputs_and_phase_locks() == 0);
    CHECK(check_direction_exclusion() == 0);
    CHECK(check_heading_slot_direct_values_and_isolation() == 0);
    CHECK(check_route_cross_heading_restart(31u) == 0);
    CHECK(check_exit_yaw_private_slot() == 0);
    puts("route31 tuning scope:16-stage task plan nine-road prefix/R2back630/tilt-contact-v40/back760 and direct ball180/bucket and return180/target handoffs withoutLEFT40/LEFT20;43 legacy offsets40/20 retained; fff-.00625/bff+.00625/ykp.3 trialseeds; five prefix heading-feedback/three BFFlegs; crossing/contact yaw/FF disabled; freshback190 restoresykp+BFF; overrides/mission32 isolation, RAM reset, reports, invalid/36road-phase locks and route-only90 turn targets passed");
    return 0;
}
