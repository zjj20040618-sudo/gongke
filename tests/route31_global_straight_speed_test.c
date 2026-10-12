/* Real BLE dispatcher/CRC parser and route/VAT executors, with injected host
 * time, odometry and images. Isolated stages do not prove complete route,
 * physical traction, grip, turn precision or speed calibration. */
#define ROUTE31_FAST_TIMING_FIXTURE_MAIN retained_global_speed_fast_main
#include "route31_fast_timing_test.c"
#undef ROUTE31_FAST_TIMING_FIXTURE_MAIN

static int global_prepare_with_pv(unsigned stage, unsigned requested, unsigned pv)
{
    char command[16]; int32_t qr[3];
    CHECK(wire_boot() == 0);
    host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
    run_cmd("31");
    CHECK(s_seq_state == SQ_READY && !s_route31_straight_v_set);
    if (requested) {
        snprintf(command,sizeof command,"v%u",requested); run_cmd(command);
        CHECK(s_route31_straight_v_set && s_route31_straight_v == (float)requested);
    }
    if (pv) {
        snprintf(command,sizeof command,"pv%u",pv); run_cmd(command);
        CHECK(s_route31_lateral_v == (float)pv);
    }
    wire_sync(); wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
    CHECK(proto_qr_get(qr)); run_cmd("g");
    CHECK(s_seq_state == SQ_STILL && fast_stopped());
    memcpy(s_seq_qr,qr,sizeof qr); step_vision_receive_end();
    if (stage == 4u) CHECK(fixture_prepare_route31_board() == 0);
    else {
        s_seq_stage = (uint8_t)(stage == ROUTE31_TARGET_CORNER_STAGE ?
                               ROUTE31_TARGET_STAGE : stage);
        /* Isolate the forward half of the new turn+forward road nodes here.
         * Full route replay separately executes both preceding RIGHT90 turns. */
        if (stage == 6u) s_route31_exit_turn_done = 1u;
        if (stage == 7u) s_route31_entry_turn_done = 1u;
        route_seq_prepare(); wire_sync();
        if (stage == ROUTE31_TARGET_CORNER_STAGE) {
            /* Real request-bound ACK/54 before closing RX; not aim/fire proof. */
            uint8_t order[] = {0x62u,(uint8_t)wire_request,(uint8_t)(wire_request>>8),
                              9u,0u,0x54u,1u,0u,8u,2u,2u,6u,8u,255u};
            wire_ack(wire_request,2u,0u); wire_feed(order,sizeof order,0);
            route31_target_rank_capture(); CHECK(s_route31_target_order.rank == 2u);
            step_vision_receive_end(); s_target35_route=0u; s_target35_phase=TA_OFF;
            s_seq_stage=(uint8_t)stage; route_seq_prepare();
        }
        if (stage == ROUTE31_HOSTAGE_EXIT_STAGE) {
            /* Rank admission only; this case does not simulate physical grab. */
            s_route31_hostage_rank=1u; route_seq_prepare();
        }
    }
    wire_sync();
    CHECK(s_seq_mode == ROUTE_TEST_MODE && fast_stopped());
    CHECK(s_route31_straight_v_set == (requested != 0u));
    return 0;
}

static int global_prepare(unsigned stage, unsigned requested)
{ return global_prepare_with_pv(stage,requested,0u); }

static float global_expected(unsigned stage, unsigned requested)
{ return requested ? (float)requested : stage >= ROUTE31_PAIR_STAGE ? ROUTE31_TASK_V_MMS : ROUTE31_STRAIGHT_V_MMS; }

static int global_ordinary(unsigned stage, unsigned requested)
{
    CHECK(global_prepare(stage,requested) == 0);
    CHECK(sequence_start_stage() == 0 && s_seq_state == SQ_RUN);
    if (stage == 6u) CHECK(s_msel == 15 && s_dist_target == (float)ROUTE31_EXIT_FORWARD_MM);
    if (stage == 7u) CHECK(s_msel == 15 && s_dist_target == (float)ROUTE31_TASK_ENTRY_FORWARD_MM);
    float expected=global_expected(stage,requested);
    CHECK(route_seq_speed_mms() == expected && s_v == expected);
    xy_advance(1200u);
    CHECK(s_seq_state == SQ_RUN && s_round == R_RUN);
    int backward=stage == 1u || stage == 5u;
    CHECK(last_x == (backward ? -expected : expected));
    float ff=stage >= ROUTE31_PAIR_STAGE ? 0.0f :
             backward ? s_route_backward_ff_ratio : s_route_forward_ff_ratio;
    CHECK(fabsf(last_y + ff*expected) < .00001f && !last_w && !laser_state);
    CHECK(s_dist_heading_kp == 10.0f && s_dist_ff_ratio == ff);
    float retained=s_route31_straight_v;
    run_cmd("v350"); CHECK(s_route31_straight_v == retained);
    CHECK(strstr(host_messages,"ERR ROUTE_SEQ_ACTIVE") != NULL);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
    CHECK(s_route31_straight_v == retained &&
          s_route31_straight_v_set == (requested != 0u));
    return 0;
}

static int global_vat_coarse(unsigned stage, unsigned requested)
{
    CHECK(global_prepare(stage,requested) == 0 && s_seq_state == SQ_TASK);
    CHECK(wire_task == (stage == ROUTE31_PAIR_STAGE ? PROTO_TASK_BALL : PROTO_TASK_HOSTAGE));
    wire_ack(wire_request,2u,0u); xy_advance(1300u); fast_snapshot();
    CHECK(fast_status.state == VAT_ROUTE_SEARCH &&
          s_route_search_speed == global_expected(stage,requested));
    CHECK(last_x == global_expected(stage,requested) && !last_y && !last_w);
    CHECK(s_route_search_kp == 10.0f && s_route_search_ff == 0.0f);
    run_cmd("0"); CHECK(fast_stopped());
    return 0;
}

static int global_bucket_coarse(unsigned requested, int direction)
{
    CHECK(global_prepare(ROUTE31_PAIR_STAGE,requested) == 0);
    wire_ack(wire_request,2u,0u);
    /* Enter the public post-turn boundary synthetically. This isolates the
     * NEW bucket request/search-speed inheritance, not grip/180 completion. */
    s_vat.ball_rank=(uint8_t)(direction < 0 ? 1u : 3u); s_vat.state=VAT_TURN_REQUEST;
    CHECK(vision_align_test_take_turn_request());
    vision_align_test_notify_turn_result(1); wire_sync();
    CHECK(wire_task == PROTO_TASK_BUCKET && s_route_bucket_eligible);
    wire_ack(wire_request,2u,0u); xy_advance(3500u); fast_snapshot();
    CHECK(fast_status.state == VAT_ROUTE_BUCKET_SEARCH);
    CHECK(s_route_search_speed == global_expected(ROUTE31_PAIR_STAGE,requested));
    CHECK(last_x == (float)direction * global_expected(ROUTE31_PAIR_STAGE,requested) &&
          !last_y && !last_w && !laser_state);
    run_cmd("0"); CHECK(fast_stopped());
    return 0;
}

static int global_target_coarse(unsigned requested)
{
    CHECK(global_prepare(ROUTE31_TARGET_STAGE,requested) == 0);
    CHECK(wire_task == PROTO_TASK_TARGET); wire_ack(wire_request,2u,0u);
    xy_advance(1300u);
    CHECK(s_target35_phase == TA_SEEK && !s_target31_fine);
    CHECK(last_x == global_expected(ROUTE31_TARGET_STAGE,requested) && !last_y && !last_w);
    /* Far selected observation stays coarse; it must not silently use50. */
    host_tick+=20u; xy_obj(wire_request,8,400,100);
    CHECK(!s_target31_fine && last_x == global_expected(ROUTE31_TARGET_STAGE,requested));
    run_cmd("0"); CHECK(fast_stopped());
    return 0;
}

static int check_global_speed_matrix(void)
{
    const unsigned speeds[]={0u,50u,100u,150u,200u,350u,600u};
    const unsigned ordinary[]={1u,5u,6u,7u,ROUTE31_TARGET_CORNER_STAGE,ROUTE31_HOSTAGE_EXIT_STAGE};
    for(unsigned n=0u;n<sizeof speeds/sizeof speeds[0];++n) {
        for(unsigned k=0u;k<sizeof ordinary/sizeof ordinary[0];++k)
            CHECK(global_ordinary(ordinary[k],speeds[n]) == 0);
        CHECK(global_vat_coarse(ROUTE31_PAIR_STAGE,speeds[n]) == 0);
        CHECK(global_vat_coarse(ROUTE31_HOSTAGE_STAGE,speeds[n]) == 0);
        CHECK(global_bucket_coarse(speeds[n],-1) == 0);
        CHECK(global_bucket_coarse(speeds[n],1) == 0);
        CHECK(global_target_coarse(speeds[n]) == 0);
    }
    puts("31 real v50/100/150/200/350/600 and default150: signed roads, ball/bucket/target/hostage coarse, corner/final outputs passed");
    return 0;
}

static int check_global_parameter_lifetime(void)
{
    CHECK(wire_boot() == 0); run_cmd("31");
    CHECK(s_route31_straight_v == 150.0f && !s_route31_straight_v_set &&
          route31_straight_speed() == 150.0f && route31_tail_straight_speed() == 150.0f);
    run_cmd("v0"); run_cmd("v601");
    CHECK(s_route31_straight_v == 150.0f && !s_route31_straight_v_set);
    run_cmd("v1"); CHECK(s_route31_straight_v == 1.0f && s_route31_straight_v_set);
    run_cmd("v200"); CHECK(s_route31_straight_v == 200.0f);
    run_cmd("v0"); run_cmd("v601"); CHECK(s_route31_straight_v == 200.0f);
    const char *stop[]={"g","a","0"};
    for(unsigned n=0u;n<sizeof stop/sizeof stop[0];++n) {
        run_cmd("g"); CHECK(route_seq_active());
        run_cmd(stop[n]);
        CHECK(s_seq_state == SQ_STOPPED &&
              s_route31_straight_v_set && s_route31_straight_v == 200.0f);
        run_cmd("31");
        CHECK(s_seq_state == SQ_READY && s_route31_straight_v_set &&
              route31_tail_straight_speed() == 200.0f && fast_stopped());
    }
    run_cmd("0"); run_cmd("15"); run_cmd("v350");
    CHECK(s_v == 350.0f && s_route31_straight_v == 200.0f && s_route31_straight_v_set);
    run_cmd("0"); run_cmd("31");
    CHECK(s_route31_straight_v == 200.0f && route31_tail_straight_speed() == 200.0f);
    run_cmd("0"); run_cmd("43"); run_cmd("v600");
    CHECK(s_route43_tune.straight_v == 600.0f && s_route31_straight_v == 200.0f);
    run_cmd("0"); run_cmd("31");
    CHECK(s_route31_straight_v_set && s_route31_straight_v == 200.0f);
    run_cmd("0"); test_init();
    CHECK(!s_route31_straight_v_set && s_route31_straight_v == 150.0f &&
          route31_tail_straight_speed() == 150.0f);
    /* Other-mode v must not create an explicit31 override after boot. */
    run_cmd("15"); run_cmd("v350"); run_cmd("0"); run_cmd("31");
    CHECK(!s_route31_straight_v_set && s_route31_straight_v == 150.0f &&
          route31_tail_straight_speed() == 150.0f);
    run_cmd("0"); CHECK(fast_stopped());
    puts("31 v range/RUN rejection and RAM retention through g/a/0/reselect; init reset and other-mode isolation passed");
    return 0;
}

static int check_global_fine_speed_isolation(void)
{
    const unsigned stages[]={ROUTE31_PAIR_STAGE,ROUTE31_HOSTAGE_STAGE,ROUTE31_PAIR_STAGE};
    for(unsigned kind=0u;kind<3u;++kind) for(int direction=-1;direction<=1;direction+=2) {
        CHECK(global_prepare(stages[kind],600u) == 0);
        wire_ack(wire_request,2u,0u);
        /* Suppress only unrelated parallel rack job in this isolated X case. */
        if(kind != 1u) s_route31_rack_deployed=1u;
        if(kind == 2u) {
            s_vat.ball_rank=3u; s_vat.state=VAT_TURN_REQUEST;
            CHECK(vision_align_test_take_turn_request());
            vision_align_test_notify_turn_result(1); wire_sync();
            wire_ack(wire_request,2u,0u);
        }
        xy_advance(260u);
        int model=kind == 0u ? 4 : kind == 1u ? 0 : 9;
        int goal=kind == 0u ? VAT_ROUTE31_BALL_X_PX : kind == 1u ?
            VAT_ROUTE31_HOSTAGE_X_PX : VAT_ROUTE31_BUCKET_X_PX;
        for(unsigned n=0u;n<40u && s_vat.state!=VAT_FINE_CONTINUOUS;++n) {
            host_tick+=20u; xy_obj(wire_request,model,goal+direction*12,220);
        }
        CHECK(s_vat.state == VAT_FINE_CONTINUOUS && last_x == (float)direction*20.0f &&
              !last_y && !last_w && !laser_state && s_route_search_speed == 600.0f);
        run_cmd("0"); CHECK(fast_stopped());
    }
    for(int direction=-1;direction<=1;direction+=2) {
        CHECK(global_prepare(ROUTE31_TARGET_STAGE,600u) == 0);
        wire_ack(wire_request,2u,0u); xy_advance(1300u);
        host_tick+=20u; xy_obj(wire_request,8,target35_point()+direction*20,100);
        CHECK(s_target31_fine && s_target35_phase == TA_SEEK &&
              last_x == (float)direction*30.0f && !last_y && !last_w && !laser_state);
        run_cmd("0"); CHECK(fast_stopped());
    }
    puts("explicit31 v600 cannot alter signed ball/bucket/hostage fine20 or target fine30 outputs");
    return 0;
}

static int check_global_fixed_motion_exceptions(void)
{
    CHECK(global_prepare(3u,600u) == 0); CHECK(sequence_start_stage() == 0);
    xy_advance(1200u);
    CHECK(s_v == 300.0f && last_x == -300.0f && !last_y && !last_w &&
          s_dist_heading_kp == 0.0f && s_dist_ff_ratio == 0.0f);
    run_cmd("0");
    CHECK(global_prepare(4u,600u) == 0); CHECK(sequence_start_stage() == 0);
    xy_advance(1200u);
    CHECK(s_v == 40.0f && last_x == 40.0f && !last_y && !last_w &&
          s_dist_heading_kp == 0.0f);
    run_cmd("0");
    CHECK(global_prepare(1u,600u) == 0);
    s_route31_corner_diag=1u; route_seq_prepare(); CHECK(sequence_start_stage() == 0);
    xy_advance(1200u);
    const float diagonal_cap=fminf(600.0f,ROUTE31_CORNER_DIAG_V_MMS);
    CHECK(s_v == diagonal_cap && route_seq_speed_mms() == diagonal_cap && last_x<0.0f && last_y<0.0f);
    CHECK(fabsf(sqrtf(last_x*last_x+last_y*last_y)-diagonal_cap) < .0001f);
    run_cmd("0");
    const unsigned sides[]={0u};
    for(unsigned n=0u;n<1u;++n) {
        CHECK(global_prepare_with_pv(sides[n],600u,170u) == 0);
        CHECK(sequence_start_stage() == 0); xy_advance(1200u);
        CHECK(s_v == 170.0f && route_seq_speed_mms() == 170.0f &&
              last_y == -170.0f && !last_w &&
              s_dist_heading_kp == 4.5f && s_dist_ff_ratio == .010f);
        CHECK(s_route31_straight_v == 600.0f);
        run_cmd("0");
    }
    CHECK(global_prepare(ROUTE31_PAIR_STAGE,600u) == 0);
    /* Both optional active31 offsets are retained at0 but MUST NOT dispatch.
     * Full task replay proves their actual aligned-action path starts the arm. */
    CHECK(ROUTE31_BALL_PREGRAB_LEFT_MM == 0u && ROUTE31_HOSTAGE_PREGRAB_LEFT_MM == 0u);
    s_route44_ball_offset=1u;
    CHECK(route_seq_leg()->mode == 17u && route_seq_leg()->distance_mm == 0u);
    s_route44_ball_offset=0u; s_seq_hostage_offset=1u;
    CHECK(route_seq_leg()->mode == 17u && route_seq_leg()->distance_mm == 0u &&
          route_seq_speed_mms() == 300.0f && route31_lateral_speed() == 300.0f);
    s_seq_hostage_offset=0u; run_cmd("0");
    /* New stage6/7 internal turns and final entry turn retain their turn
     * profile despite global v600 and arbitrary lateral RAM speed. */
    for (unsigned stage=6u; stage<=8u; ++stage) {
        CHECK(global_prepare_with_pv(stage,600u,170u) == 0);
        s_route31_exit_turn_done=s_route31_entry_turn_done=0u;
        route_seq_prepare();
        CHECK(route_seq_leg()->mode == 20u && route_seq_leg()->distance_mm == 0u);
        CHECK(sequence_start_stage() == 0); xy_advance(20u);
        CHECK(!last_x && !last_y && last_w == T_TURN_MAX_W && turn_target_deg() == 90.0f);
        run_cmd("0");
        CHECK(!s_route31_exit_turn_done && !s_route31_entry_turn_done);
    }
    CHECK(global_prepare(2u,600u) == 0); CHECK(sequence_start_stage() == 0);
    xy_advance(20u);
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == T_TURN_MAX_W &&
          turn_target_deg() == 90.0f); run_cmd("0");
    puts("31 global v leaves cross300/contact40/configured diagonal cap/departure pv/turn unchanged; new forward legs inherit v not pv, both active offsets0 retained but not run");
    return 0;
}

static int check_global_other_owner_isolation(void)
{
    CHECK(global_prepare(1u,600u) == 0); run_cmd("0");
    run_cmd("43"); wire_sync();
    wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123"); run_cmd("g");
    CHECK(s_route31_straight_v_set && s_route31_straight_v == 600.0f);
    s_seq_stage=1u; route_seq_prepare(); CHECK(sequence_start_stage() == 0);
    xy_advance(1200u); CHECK(s_v == 100.0f && last_x == -100.0f);
    run_cmd("0");
    run_cmd("44"); run_cmd("g"); CHECK(route44_owner() && s_seq_state == SQ_STILL);
    CHECK(sequence_start_stage() == 0); xy_advance(1200u);
    CHECK(s_v == 50.0f && last_x == 50.0f && s_route31_straight_v == 600.0f);
    run_cmd("0");
    run_cmd("35"); run_cmd("g"); wire_sync();
    wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123"); wire_sync();
    CHECK(wire_task == PROTO_TASK_TARGET); wire_ack(wire_request,2u,0u); xy_advance(1300u);
    CHECK(s_target35_phase == TA_SEEK && !s_target35_route && last_x == 50.0f &&
          target35_point() == 255 && s_route31_straight_v == 600.0f);
    host_tick+=20u; xy_obj(wire_request,8,target35_point()-20,100);
    CHECK(last_x == -50.0f && !laser_state); run_cmd("0"); CHECK(fast_stopped());
    puts("retained31 v600 does not change real43 ordinary100,44 ordinary50 or standalone35 signed50");
    return 0;
}

int main(void)
{
    CHECK(check_global_speed_matrix() == 0);
    CHECK(check_global_parameter_lifetime() == 0);
    CHECK(check_global_fine_speed_isolation() == 0);
    CHECK(check_global_fixed_motion_exceptions() == 0);
    CHECK(check_global_other_owner_isolation() == 0);
    puts("route31 global straight speed dispatch regression passed (host-only)");
    return 0;
}
