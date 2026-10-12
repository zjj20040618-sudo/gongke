/* Current31 automatic route through actual BLE/VAT/CRC/STEP and actual
 * namespaced motion.c. Sensor endpoints/pixels are HOST inputs, not a plant
 * simulation or proof of physical grasp, clearance or route success.
 */
static int r31_scaled_contract(float vx, float vy, float w, float scale);
#define ROUTE44_SCALED_FIXTURE_CONTRACT(vx,vy,w,scale) r31_scaled_contract(vx,vy,w,scale)
#define ROUTE44_CURRENT_FIXTURE_MAIN retained_r31_route44_main
#include "route44_current_test.c"
#undef ROUTE44_CURRENT_FIXTURE_MAIN
#undef ROUTE44_SCALED_FIXTURE_CONTRACT

static int r31_scaled_contract(float vx, float vy, float w, float scale)
{
    if(s_seq_state!=SQ_RUN || s_seq_stage!=3u || s_msel!=16 ||
       s_dist_heading_kp!=0.0f || vx>=0.0f || vy!=0.0f || w!=0.0f) return 0;
    if(route44_owner()) return s_route44_index==4u && scale==1.10f;
    return s_seq_mode==31u && scale==1.15f;
}

static int r31_revised_boot(void)
{
    CHECK(wire_boot()==0); host_wire_diag_hook=real_proto_wire_diag_get; xy_seq=1u;
    r44_scaled_calls=r44_scaled_contract_errors=0u; r44_scaled_value=0.0f;
    r31_board_calls=r31_board_contract_errors=0u;
    run_cmd("yfix1"); run_cmd("31"); wire_sync();
    CHECK(s_seq_state==SQ_READY && s_seq_mode==31u && r44_stopped());
    CHECK(ROUTE31_BALL_PREGRAB_LEFT_MM==0u &&
          s_route31_ball_pregrab_left_leg.mode==17u &&
          s_route31_ball_pregrab_left_leg.distance_mm==0u &&
          s_route31_ball_pregrab_left_leg.heading_hold &&
          s_route44_left_leg.distance_mm==30u && s_route43_tune.pair_left_mm==15u);
    CHECK(!s_route31_exit_turn_done && !s_route31_entry_turn_done &&
          ROUTE31_HOSTAGE_PREGRAB_LEFT_MM==0u);
    wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
    CHECK(proto_qr_get(NULL));
    run_cmd("g"); CHECK(s_seq_state==SQ_STILL && !route44_owner() && !s_route44_frozen);
    return 0;
}

/* The inherited44 fixture injects one axis.31's new internal diagonal needs
 * both encoder endpoints, without changing the independent44 test helper. */
static int r31_finish_road(void)
{
    if (!route31_diagonal_active()) return r44_finish_road();
    host_fore = -(float)ROUTE31_CORNER_DIAG_BACK_MM;
    host_lateral = -(float)ROUTE31_CORNER_DIAG_LEFT_MM;
    for (unsigned k=0u; k<180u && s_seq_state==SQ_RUN; ++k) {
        if (s_round==R_ALIGN) host_yaw=s_dist_heading0;
        CHECK(r44_service(20u)==0);
    }
    CHECK(s_seq_state!=SQ_STOPPED);
    return 0;
}

/* Replay the real first nine automatic route stages. No direct stage jump or
 * invented rack position: all three post-cross RIGHT90s finish without STEP. */
static int r31_to_ball_request(void)
{
    CHECK(r31_revised_boot()==0);
    for(unsigned k=0u;k<80u && s_seq_stage<ROUTE31_PAIR_STAGE;++k) {
        CHECK(!pulse_calls && !host_timer_active && !host_timer_start_calls &&
              !s_route31_rack_deployed && !s_route44_rack_started &&
              s_seq_state!=SQ_ARM_PREP);
        if(s_seq_state==SQ_QR_WAIT) {
            wire_sync(); wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
        } else {
            if(s_seq_state!=SQ_RUN) CHECK(r44_start_road()==0);
            CHECK(r31_finish_road()==0);
        }
    }
    wire_sync();
    CHECK(s_seq_stage==ROUTE31_PAIR_STAGE && s_seq_state==SQ_TASK &&
          s_seq_mode==31u && wire_opcode==0x63u && wire_task==1u && wire_digit==1u &&
          s_receiving && !s_ack && !pulse_calls && !host_timer_start_calls &&
          !s_route31_rack_deployed && !s_route44_rack_started);
    return 0;
}

static int r31_ball_search_ready(void)
{
    CHECK(r31_to_ball_request()==0);
    wire_ack(wire_request,2u,0u);
    CHECK(r44_service(T_DIST_STILL_MS+1000u)==0 && s_seq_state==SQ_TASK && last_x==ROUTE31_TASK_V_MMS &&
          !last_y && !last_w && !pulse_calls && !host_timer_active &&
          !s_route44_rack_started && !s_route31_rack_deployed);
    return 0;
}

static int check_r31_first_ball_parallel_rack_and_cancel(void)
{
    CHECK(r31_to_ball_request()==0);
    uint16_t request=wire_request;
    /* Neither an old ACK nor pre-ACK selected01 may authorize the extension. */
    wire_ack((uint16_t)(request-1u),2u,0u);
    xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !s_ack && !s_vat.ball_seen &&
          !s_route44_rack_started && !host_timer_active && !pulse_calls);
    wire_ack(request,2u,0u);
    CHECK(r44_service(T_DIST_STILL_MS+1000u)==0 && last_x==ROUTE31_TASK_V_MMS && !pulse_calls &&
          !host_timer_start_calls && !s_route44_rack_started && !s_vat.ball_seen);
    xy_qr(request,"123");
    r44_rank(4u,2u);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    xy_obj_poll(request,-1,0,0,0);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    /* A selected payload replaying that empty image's sequence is still old. */
    unsigned empty_sequence=xy_seq-1u;
    xy_seq=empty_sequence;
    xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    xy_obj_poll(request,5,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    xy_obj_poll((uint16_t)(request-1u),4,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    int brakes=brake_calls,servos=servo_calls;
    xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && s_route44_rack_started && host_timer_active &&
          host_timer_start_calls==1u && s_route31_lift_phase==R44_RACK_EXTEND &&
          s_seq_state==SQ_TASK && last_x==ROUTE31_TASK_V_MMS && !last_y && !last_w &&
          brake_calls==brakes && servo_calls==servos && !s_route_action_taken &&
          !s_route44_ball_offset && !host_jog_pulses[1][1]);
    unsigned starts=host_timer_start_calls;
    for(unsigned k=0u;k<5u;++k) {
        xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
        CHECK(r44_service(20u)==0 && s_seq_state==SQ_TASK && last_x==ROUTE31_TASK_V_MMS &&
              !last_y && !last_w && host_timer_start_calls==starts &&
              host_jog_pulses[0][0]>0u && host_jog_pulses[0][0]<ROUTE31_RACK_EXTEND_STEPS &&
              !host_jog_pulses[1][1] && servo_calls==servos && brake_calls==brakes &&
              !s_route_action_taken && !s_route44_ball_offset);
    }
    run_cmd("0"); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
    static const char *const keys[]={"g","a","0"};
    for(unsigned k=0u;k<3u;++k) {
        CHECK(r31_ball_search_ready()==0);
        request=wire_request;
        xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
        CHECK(r44_service(120u)==0 && host_timer_active && last_x==ROUTE31_TASK_V_MMS &&
              host_jog_pulses[0][0]>0u && host_jog_pulses[0][0]<ROUTE31_RACK_EXTEND_STEPS);
        int pulses=pulse_calls;
        servos=servo_calls; starts=host_timer_start_calls;
        run_cmd(keys[k]);
        CHECK(s_seq_state==SQ_STOPPED && r44_stopped() && !s_jog_clock.remaining);
        wire_ack(request,2u,0u); r44_rank(4u,2u);
        xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX,VAT_BALL_Y_PX,0);
        xy_obj_poll((uint16_t)(request-1u),4,VAT_ROUTE31_BALL_X_PX,VAT_BALL_Y_PX,0);
        CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED && r44_stopped() &&
              pulse_calls==pulses && servo_calls==servos && host_timer_start_calls==starts &&
              !s_receiving && !s_route_owned && !s_route_action_taken);
        run_cmd("g"); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
    }
    puts("31 zeroSTEP throughLEFT90;realQR/ACK/rank54/empty/replay/wrongtarget/wrongrequest cannotextend;firstNEWselectedBALL alone startsfiniteparallelrackonce;g/a/0 cancelchassis+STEP andlateACK/54/01 cannotrestart passed");
    return 0;
}

static int check_r31_scaled_actual_outputs_and_scope(void)
{
    float plain[4],changed[4];
    r44_real_motion_ik_precise(-300.0f,0.0f,0.0f,plain);
    r44_real_motion_vel_set_left_scaled(-300.0f,0.0f,0.0f,1.15f);
    memcpy(changed,r44_motion_rpm,sizeof changed);
    CHECK(fabsf(changed[0]-plain[0]*1.15f)<0.0001f &&
          fabsf(changed[3]-plain[3]*1.15f)<0.0001f &&
          changed[1]==plain[1] && changed[2]==plain[2]);
    r44_real_motion_vel_set_left_scaled(-300.0f,0.0f,0.0f,1.10f);
    CHECK(fabsf(r44_motion_rpm[0]-plain[0]*1.10f)<0.0001f &&
          fabsf(r44_motion_rpm[3]-plain[3]*1.10f)<0.0001f &&
          r44_motion_rpm[1]==plain[1] && r44_motion_rpm[2]==plain[2]);
    r44_real_motion_vel_set_precise(-300.0f,0.0f,0.0f);
    CHECK(!memcmp(plain,r44_motion_rpm,sizeof plain));
    CHECK(r31_revised_boot()==0);
    CHECK(s_route31_plan[0].distance_mm==545u && s_route31_plan[6].distance_mm==780u &&
          s_route31_plan[7].distance_mm==780u && ROUTE31_EXIT_RIGHT_MM==765u);
    CHECK(s_route43_tune.road_mm[0]==545u && s_route43_tune.road_mm[1]==610u &&
          s_route43_tune.road_mm[3]==620u && s_route43_tune.road_mm[6]==780u &&
          s_route43_tune.road_mm[7]==780u);
    CHECK(s_route44_plan[0].leg.mode==15u && s_route44_plan[0].leg.distance_mm==600u &&
          s_route44_plan[4].leg.distance_mm==620u && ROUTE44_CROSS_LEFT_SCALE==1.10f &&
          ROUTE44_STAGES==22u);
    run_cmd("0"); CHECK(r44_stopped());
    /* Deliberate stopped-slot dispatch verifies retained43 never opts into
     * either31's new1.15 or44's1.10 API; this is not a second full route. */
    run_cmd("43"); wire_sync();
    wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123"); run_cmd("g");
    s_seq_stage=3u; route_seq_prepare();
    unsigned calls=r44_scaled_calls;
    for(unsigned k=0;k<180u && s_seq_state!=SQ_RUN;++k) {
        if(s_seq_state==SQ_STEP_WAIT) run_cmd("g");
        CHECK(r44_service(20u)==0 && s_seq_state!=SQ_STOPPED);
    }
    CHECK(s_seq_state==SQ_RUN && s_seq_mode==43u && s_msel==16 &&
          s_dist_target==-620.0f && s_v==300.0f);
    CHECK(r44_service(20u)==0 && r44_scaled_calls==calls &&
          !r44_scaled_contract_errors && last_x==-300.0f && !last_y && !last_w);
    run_cmd("0"); CHECK(r44_stopped());
    puts("31 actualIK leftm0/m3 x1.15 versus44 x1.10/rightwheels unchanged/normalAPI no persistence;shared43baseline545/780 and44fixed22plan remain isolated passed");
    return 0;
}

/* Resolver slots below are isolated unit checks, not claimed route travel.
 * The full automatic replay separately checks real stage0/6 chassis commands. */
static int check_r31_mirrored_ff_ram_and_scope(void)
{
    CHECK(wire_boot()==0);
    run_cmd("31");
    CHECK(s_seq_state==SQ_READY && s_seq_stage==0u);
    CHECK(s_route31_left_ff_ratio==0.010f && s_route31_exit_right_ff_ratio==0.010f &&
          s_route31_left_yaw_kp==4.5f && s_route31_exit_yaw_kp==4.5f);
    float manual=s_right_ff_ratio;
    run_cmd("rff0.015");
    CHECK(s_route31_exit_right_ff_ratio==0.015f && s_route31_left_ff_ratio==0.010f &&
          s_route_right_ff_ratio==0.0f && s_right_ff_ratio==manual);
    run_cmd("lff0.02");
    CHECK(s_route31_left_ff_ratio==0.02f && s_route31_exit_right_ff_ratio==0.015f);
    run_cmd("param");
    CHECK(strstr(host_messages,"ROUTE31_EXIT_FF rff=0.01500")!=NULL);
    s_seq_stage=6u; s_msel=18;
    CHECK(route_right_ff_ratio()==0.015f);
    s_seq_contact_post=1u;
    CHECK(route_right_ff_ratio()==0.0f);
    s_seq_contact_post=0u; s_seq_stage=5u;
    CHECK(route_right_ff_ratio()==0.0f);
    s_seq_stage=ROUTE31_PAIR_STAGE;
    CHECK(route_right_ff_ratio()==0.0f && route_left_ff_ratio()==0.0f);
    s_seq_mode=ROUTE_STEP_MODE; s_seq_stage=6u;
    CHECK(s_route43_tune.left_yaw_kp==3.0f && s_route43_tune.exit_yaw_kp==3.0f);
    CHECK(route_right_ff_ratio()==s_route43_tune.right_ff && route_right_ff_ratio()==0.0f);
    s_route43_tune.right_ff=-0.012f;
    CHECK(route_right_ff_ratio()==-0.012f && s_route31_exit_right_ff_ratio==0.015f);
    s_seq_mode=ROUTE44_MODE;
    CHECK(route44_owner() && route_right_ff_ratio()==0.0f);
    s_seq_mode=ROUTE_NO_QR_MODE;
    CHECK(route_right_ff_ratio()==s_route_right_ff_ratio && route_right_ff_ratio()==0.0f);
    CHECK(wire_boot()==0 && s_route31_exit_right_ff_ratio==0.010f &&
          s_route31_left_ff_ratio==0.010f && s_route43_tune.right_ff==0.0f);
    puts("31 exitRIGHT +1% mirrors departureLEFT; private rff RAM/report independent of lff/manual/legacy43/44/tasktail; boot reset passed");
    return 0;
}

/* Isolated resolver slots; the full31 and inherited full44 replays below
 * independently traverse both real task turns and their request handoffs. */
static int check_r31_second_return_180_scope(void)
{
    CHECK(wire_boot()==0); run_cmd("31");
    CHECK(ROUTE31_ACTIVE_RETURN_TARGET_DEG==180.0f);
    s_seq_state=SQ_RUN; s_msel=22; s_seq_stage=ROUTE31_RETURN180_STAGE; s_seq_pair_turn=0u;
    CHECK(s_seq_mode==31u && turn_target_deg()==180.0f);
    s_seq_stage=ROUTE31_PAIR_STAGE; s_seq_pair_turn=1u;
    CHECK(turn_target_deg()==180.0f); /* First ball-to-bucket rotation stays180. */
    s_seq_stage=ROUTE31_RETURN180_STAGE;
    CHECK(turn_target_deg()==180.0f); /* Pair and second-return31 both remain180. */
    s_seq_mode=ROUTE44_MODE; s_route44_index=16u; s_seq_pair_turn=0u;
    CHECK(route44_owner() && turn_target_deg()==180.0f);
    s_route44_index=15u; s_seq_stage=ROUTE31_PAIR_STAGE; s_seq_pair_turn=1u;
    CHECK(turn_target_deg()==180.0f);
    s_seq_mode=ROUTE_STEP_MODE; s_seq_stage=ROUTE31_RETURN180_STAGE; s_seq_pair_turn=0u;
    CHECK(ROUTE31_RETURN_TARGET_DEG==182.0f &&
          s_route43_tune.turn_deg[ROUTE31_RETURN180_STAGE]==182.0f &&
          turn_target_deg()==182.0f);
    s_route43_tune.turn_deg[ROUTE31_RETURN180_STAGE]=176.0f;
    CHECK(turn_target_deg()==176.0f); /*43 still resolves its private RAM setting. */
    s_seq_stage=ROUTE31_PAIR_STAGE; s_seq_pair_turn=1u;
    CHECK(turn_target_deg()==180.0f);
    s_seq_mode=ROUTE_TEST_MODE; s_seq_state=SQ_OFF;
    s_seq_stage=ROUTE31_RETURN180_STAGE; s_seq_pair_turn=0u;
    CHECK(!route_seq_active() && turn_target_deg()==180.0f); /* Manual22 stays180. */
    CHECK(wire_boot()==0);
    puts("31 first+second180/44 first+second180/manual22 unchanged;43 historical182 seed and privateRAM resolve independently passed");
    return 0;
}

/* Deliberate executor entry tests stop/selection/init lifecycle only. The
 * complete automatic case below traverses both internal turns unseeded. */
static int check_r31_postcross_turn_flags_stop_reset(void)
{
    const char *const keys[]={"g","a","0"};
    for(unsigned stage=6u;stage<=8u;++stage)
        for(unsigned phase=0u;phase<(stage==8u?1u:2u);++phase)
            for(unsigned key=0u;key<sizeof keys/sizeof keys[0];++key) {
                CHECK(r31_revised_boot()==0);
                s_seq_stage=(uint8_t)stage;
                s_route31_exit_turn_done=stage>6u || phase;
                s_route31_entry_turn_done=stage>7u || (stage==7u && phase);
                route_seq_prepare(); CHECK(r44_start_road()==0);
                CHECK(s_msel==(stage==8u || !phase?20:15));
                CHECK(!pulse_calls && !host_timer_active && !laser_state);
                int pulses=pulse_calls;
                run_cmd(keys[key]);
                CHECK(s_seq_state==SQ_STOPPED && r44_stopped() &&
                      !s_route31_exit_turn_done && !s_route31_entry_turn_done);
                CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED &&
                      r44_stopped() && pulse_calls==pulses);
                run_cmd("g"); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
                /* Selection and a new run each independently clear flags. */
                s_route31_exit_turn_done=s_route31_entry_turn_done=1u;
                run_cmd("31"); CHECK(s_seq_state==SQ_READY &&
                      !s_route31_exit_turn_done && !s_route31_entry_turn_done);
                s_route31_exit_turn_done=s_route31_entry_turn_done=1u;
                run_cmd("g"); CHECK(s_seq_state==SQ_STILL &&
                      !s_route31_exit_turn_done && !s_route31_entry_turn_done);
                run_cmd("0");
            }
    s_route31_exit_turn_done=s_route31_entry_turn_done=1u;
    CHECK(wire_boot()==0 && !s_route31_exit_turn_done && !s_route31_entry_turn_done);
    puts("31 postcross internalRIGHT90/FWD flags: g/a/0 during either turn/forward and finalRIGHT90 cancel all output; latepoll cannot advance; select/newrun/init clear flags passed");
    return 0;
}

static int r31_to_board_push(void)
{
    CHECK(r31_revised_boot()==0);
    for(unsigned k=0u;k<35u && s_seq_stage<4u;++k) {
        if(s_seq_state!=SQ_RUN) CHECK(r44_start_road()==0);
        CHECK(r31_finish_road()==0 && !r31_board_calls && !r31_board_contract_errors);
    }
    CHECK(s_seq_stage==4u);
    host_messages[0]='\0'; /* Prior road traces can fill this bounded HOST log buffer. */
    CHECK(r44_start_road()==0 && s_board_contact.active && route31_contact_active());
    CHECK(r44_service(40u)==0 && r31_board_calls && !r31_board_contract_errors &&
          last_x>0.0f && !last_y && !last_w && r44_motion_rpm[0]>0.0f &&
          r44_motion_rpm[1]>0.0f && !r44_motion_rpm[2] && !r44_motion_rpm[3]);
    return 0;
}
static int check_r31_board_push_outputs_scope_and_stop(void)
{
    float plain[4];
    r44_real_motion_ik_precise(40.0f,0.0f,0.0f,plain);
    unsigned writes=r44_motion_precise_writes,masked=r44_motion_masked_writes;
    unsigned enters=r44_motion_irq_enters,restores=r44_motion_irq_restores;
    r44_motion_irq_mask=0u; r44_real_motion_vel_set_board_push(40.0f);
    CHECK(r44_motion_precise_writes==writes+4u && r44_motion_masked_writes==masked+4u &&
          r44_motion_irq_enters==enters+1u && r44_motion_irq_restores==restores+1u &&
          !r44_motion_irq_mask && r44_motion_rpm[0]==plain[0] && r44_motion_rpm[1]==plain[1] &&
          r44_motion_rpm[0]>0.0f && !r44_motion_rpm[2] && !r44_motion_rpm[3]);
    r44_motion_irq_mask=1u; r44_real_motion_vel_set_board_push(40.0f);
    CHECK(r44_motion_irq_mask==1u); r44_motion_irq_mask=0u;
    r44_real_motion_vel_set_precise(-100.0f,0.0f,0.0f);
    r44_real_motion_ik_precise(-100.0f,0.0f,0.0f,plain);
    CHECK(!memcmp(plain,r44_motion_rpm,sizeof plain) && r44_motion_rpm[2]<0.0f && r44_motion_rpm[3]<0.0f);
    r44_real_motion_vel_set_board_push(0.0f);
    CHECK(!r44_motion_rpm[0] && !r44_motion_rpm[1] && !r44_motion_rpm[2] && !r44_motion_rpm[3]);
    static const float invalid[]={-1.0f,NAN,INFINITY};
    for(unsigned k=0u;k<sizeof invalid/sizeof invalid[0];++k) {
        unsigned stops=r44_motion_stops,precise=r44_motion_precise_writes;
        r44_real_motion_vel_set_board_push(invalid[k]);
        CHECK(r44_motion_stops==stops+1u && r44_motion_precise_writes==precise &&
              !r44_motion_rpm[0] && !r44_motion_rpm[1] && !r44_motion_rpm[2] && !r44_motion_rpm[3]);
    }
    CHECK(r31_to_board_push()==0);
    CHECK(strstr(host_messages,"rear_only=1")!=NULL);
    CHECK(r31_finish_road()==0 && s_seq_stage==5u);
    unsigned calls=r31_board_calls;
    CHECK(r44_start_road()==0 && r44_service(20u)==0);
    CHECK(last_x<0.0f && isfinite(last_y) && !last_w &&
          !s_board_contact.active && r31_board_calls==calls && !r31_board_contract_errors);
    run_cmd("0"); CHECK(r44_stopped());
    static const char *const keys[]={"g","a","0"};
    for(unsigned k=0u;k<3u;++k) {
        CHECK(r31_to_board_push()==0); calls=r31_board_calls;
        run_cmd(keys[k]); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
        CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED && r44_stopped() &&
              r31_board_calls==calls && !r31_board_contract_errors);
    }
    CHECK(r31_to_board_push()==0); calls=r31_board_calls; host_imu_valid=0;
    CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED && r44_stopped() && r31_board_calls==calls);
    /* Stopped-slot scope checks: do not claim another physically traversed route. */
    CHECK(wire_boot()==0); host_wire_diag_hook=real_proto_wire_diag_get;
    run_cmd("43"); wire_sync(); wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123"); run_cmd("g");
    s_seq_stage=4u; s_route31_cross_heading=host_absolute_yaw;
    s_route31_cross_heading_valid=s_route31_cross_yaw_done=1u; route_seq_prepare();
    calls=r31_board_calls; CHECK(r44_start_road()==0 && r44_service(40u)==0 &&
                               s_board_contact.active && r31_board_calls==calls && !r31_board_contract_errors);
    run_cmd("0"); CHECK(r44_stopped());
    CHECK(r44_boot()==0); s_route44_index=5u; s_seq_stage=4u;
    s_route31_cross_heading=host_absolute_yaw; s_route31_cross_heading_valid=s_route31_cross_yaw_done=1u;
    route_seq_prepare(); calls=r31_board_calls;
    CHECK(r44_start_road()==0 && r44_service(40u)==0 && s_board_contact.active &&
          r31_board_calls==calls && !r31_board_contract_errors);
    run_cmd("0"); CHECK(r44_stopped());
    puts("31 BOARD push actualIK rear m0LR/m1RR positive,front m2RF/m3LF zeroRPM;atomic4write/PRIMASKrestore/invalidstop;nextBACK190 normal4wheel,no43/44scope;g/a/0/IMUstop cannot republish passed");
    return 0;
}

static int check_r31_automatic_complete_chain(void)
{
    static const uint8_t modes[16]={17,16,20,16,15,16,18,16,30,41,22,35,15,20,40,15};
    CHECK(r31_revised_boot()==0);
    unsigned visits=0u,last_stage=255u,road_tests=0u;
    unsigned ball_left=0u,grab_right=0u,hostage_left=0u,first180=0u,second180=0u;
    unsigned rack_parallel=0u,tail_ff_checks=0u,ball_grab_direct=0u;
    unsigned exit_turn=0u,exit_forward=0u,entry_turn=0u,entry_forward=0u,task_turn=0u,hostage_grab_direct=0u;
    unsigned ball_cruise_seen=0u,bucket_cruise_seen=0u,hostage_cruise_seen=0u;
    uint32_t entered_test=0u,laser_on_at=0u,laser_off_at=0u;
    uint16_t ranked_request=0u,ball_request=0u,bucket_request=0u;
    int saw_laser=0,contact_seen=0;
    for(unsigned tick=0u;tick<20000u && s_seq_state!=SQ_DONE;++tick) {
        CHECK(s_seq_state!=SQ_STOPPED && s_seq_state!=SQ_STEP_WAIT &&
              s_seq_state!=SQ_STEP_DRAIN && s_seq_mode==31u &&
              !route44_owner() && !s_route44_frozen);
        unsigned stage=s_seq_stage,scaled_before=r44_scaled_calls,board_before=r31_board_calls;
        CHECK(stage<16u);
        if(stage<=ROUTE31_PREDEPLOY_STAGE)
            CHECK(!pulse_calls && !host_timer_start_calls && !host_timer_active &&
                  !s_route31_rack_deployed && !s_route44_rack_started);
        if(stage!=last_stage) {
            CHECK(last_stage==255u || stage==last_stage+1u);
            last_stage=stage; ++visits;
            CHECK(s_route31_plan[stage].mode==modes[stage]);
        }
        if(s_seq_state==SQ_QR_WAIT) {
            CHECK(stage==0u); wire_sync();
            wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
        } else if(s_seq_state==SQ_CROSS_YAW) {
            float correction=s_route31_task_yaw_goal-host_yaw;
            host_absolute_yaw+=correction; host_yaw=s_route31_task_yaw_goal;
        } else if(s_seq_state==SQ_RUN) {
            if(s_active_test!=entered_test) {
                entered_test=s_active_test; ++road_tests;
                if(turn_closed_loop_mode()) {
                    float goal=turn_target_deg();
                    if(stage==9u) {
                        wire_sync();
                        CHECK(goal==180.0f && ball_left==0u && ball_grab_direct==1u && grab_right==0u &&
                              wire_task==1u && wire_request==ball_request &&
                              host_jog_pulses[1][1]==ROUTE31_LIFT_DOWN_STEPS &&
                              host_jog_pulses[1][0]==ROUTE31_LIFT_DOWN_STEPS);
                        ++first180;
                    } else if(stage==10u) {
                        CHECK(goal==180.0f && goal==ROUTE31_ACTIVE_RETURN_TARGET_DEG && bucket_request &&
                              host_jog_pulses[0][1]==ROUTE31_RACK_EXTEND_STEPS &&
                              host_jog_pulses[1][1]==ROUTE31_LIFT_DOWN_STEPS+ROUTE31_LIFT_BUCKET_DOWN_STEPS);
                        ++second180;
                    } else {
                        CHECK(goal==(stage==13u?ROUTE31_HOSTAGE_RIGHT_TARGET_DEG:90.0f));
                        if(stage==6u) {
                            CHECK(!s_route31_exit_turn_done && !s_route31_entry_turn_done && !exit_forward);
                            ++exit_turn;
                        } else if(stage==7u) {
                            CHECK(s_route31_exit_turn_done && !s_route31_entry_turn_done && exit_forward==1u);
                            ++entry_turn;
                        } else if(stage==8u) {
                            CHECK(s_route31_exit_turn_done && s_route31_entry_turn_done &&
                                  exit_forward==1u && entry_forward==1u && s_msel==20);
                            ++task_turn;
                        }
                    }
                    host_absolute_yaw+=goal-host_yaw; host_yaw=goal;
                } else {
                    CHECK(dist_mode());
                    if(stage>=9u) {
                        CHECK(s_v==ROUTE31_TASK_V_MMS && s_dist_ff_ratio==0.0f &&
                              s_dist_heading_kp>0.0f && s_dist_align_enabled);
                        ++tail_ff_checks;
                    }
                    if(stage==0u) CHECK(s_msel==17 && s_dist_target==-(float)ROUTE31_START_LEFT_MM && s_v==250.0f &&
                                        s_dist_ff_ratio==0.010f && s_dist_heading_kp==4.5f);
                    if(stage==1u) {
                        CHECK(s_msel==16);
                        if(s_route31_corner_diag) CHECK(fabsf(s_dist_target+route31_diagonal_length())<.001f &&
                            s_v==fminf(ROUTE31_STRAIGHT_V_MMS,ROUTE31_CORNER_DIAG_V_MMS));
                        else CHECK(s_dist_target==-(float)ROUTE31_CORNER_BACK_MM && s_v==150.0f);
                    }
                    if(stage==3u) CHECK(s_msel==16 && s_dist_target==-(float)ROUTE31_CORNER_CROSS_MM &&
                                         s_v==300.0f && s_dist_heading_kp==0.0f && !s_dist_ff_ratio);
                    if(stage==6u) {
                        CHECK(s_route31_exit_turn_done && !s_route31_entry_turn_done && exit_turn==1u &&
                              s_msel==15 && s_dist_target==(float)ROUTE31_EXIT_FORWARD_MM && s_v==150.0f &&
                              s_dist_ff_ratio==s_route_forward_ff_ratio && s_dist_heading_kp==10.0f);
                        ++exit_forward;
                    }
                    if(stage==7u) {
                        CHECK(s_route31_exit_turn_done && s_route31_entry_turn_done && entry_turn==1u &&
                              s_msel==15 && s_dist_target==(float)ROUTE31_TASK_ENTRY_FORWARD_MM && s_v==150.0f &&
                              s_dist_ff_ratio==s_route_forward_ff_ratio && s_dist_heading_kp==10.0f);
                        ++entry_forward;
                    }
                    if(stage==12u) CHECK(s_dist_target==635.0f && s_seq_qr[1]==2 &&
                                        s_route31_target_order.rank==1u && !s_receiving);
                    if(stage==9u) {
                        CHECK(s_route31_rack_deployed && !s_seq_pair_offset);
                        /* No zero-distance lateral leg may be dispatched.
                         * Real grab/lift is checked below;44 retains LEFT30. */
                        if(s_route44_ball_offset) ++ball_left;
                        else ++grab_right;
                        CHECK(ROUTE31_BALL_PREGRAB_LEFT_MM>0u);
                    }
                    if(stage==14u && s_msel==17) {
                        ++hostage_left;
                        CHECK(ROUTE31_HOSTAGE_PREGRAB_LEFT_MM>0u); /* No zero-distance dispatch. */
                    }
                    if(s_board_contact.active) {
                        CHECK(stage==4u && s_v==40.0f && s_dist_heading_kp==0.0f);
                        contact_seen=1;
                    } else {
                        host_yaw=s_dist_heading0+0.2f;
                        CHECK(r44_service(20u)==0 && s_seq_state==SQ_RUN && s_round==R_RUN);
                        if(stage==0u) {
                            CHECK(last_y<0.0f && last_x<0.0f);
                            CHECK(fabsf(last_x-0.010f*last_y)<0.0001f && last_w<0.0f);
                        }
                        if(stage==6u || stage==7u)
                            CHECK(last_x>0.0f && last_w<0.0f &&
                                  fabsf(last_y+s_route_forward_ff_ratio*fabsf(last_x))<.0001f);
                        if(stage>=9u) {
                            CHECK(dist_lateral()?last_x==0.0f:last_y==0.0f);
                            CHECK(last_w<0.0f);
                        }
                        host_yaw=s_dist_heading0;
                        if(route31_diagonal_active()) {
                            host_fore=-(float)ROUTE31_CORNER_DIAG_BACK_MM;
                            host_lateral=-(float)ROUTE31_CORNER_DIAG_LEFT_MM;
                        } else if(dist_lateral()) host_lateral=s_dist_odo0+s_dist_target;
                        else host_fore=s_dist_odo0+s_dist_target;
                    }
                }
            }
            if(stage==9u && s_seq_pair_turn) {
                wire_sync(); CHECK(wire_task==1u && wire_request==ball_request && !bucket_request);
            }
            if(s_board_contact.active) host_pitch=s_board_contact.pitch0+route_contact_tilt_deg()+0.1f;
            if(s_round==R_ALIGN) host_yaw=s_dist_heading0;
        } else if(s_seq_state==SQ_TASK) {
            wire_sync();
            if(s_receiving && !s_ack) wire_ack(wire_request,2u,0u);
            if(stage==11u) {
                CHECK(s_target35_route && target35_search_ff()==0.0f &&
                      route31_straight_speed()==ROUTE31_TASK_V_MMS && s_target35_kp>0.0f);
                if(ranked_request!=wire_request) { ranked_request=wire_request; r44_rank(8u,1u); }
                if(s_receiving) xy_obj_poll(wire_request,8,ROUTE31_TARGET_CX,100,0);
            } else {
                CHECK(stage==9u || stage==14u);
                CHECK(s_route_search_ff==0.0f);
                if(s_route_owned) CHECK(s_route_search_speed==ROUTE31_TASK_V_MMS && s_route_search_kp>0.0f);
                if(wire_task==1u) {
                    CHECK(stage==9u && !first180 && !s_seq_pair_offset);
                    CHECK(s_vat.x_goal==165 && s_vat.x_goal==VAT_ROUTE31_BALL_X_PX);
                    if(!ball_request) ball_request=wire_request;
                    CHECK(wire_request==ball_request);
                    if(ranked_request!=wire_request) { ranked_request=wire_request; r44_rank(4u,2u); }
                    if(s_receiving) xy_obj_poll(wire_request,4,
                        VAT_ROUTE31_BALL_X_PX+(ball_cruise_seen?0:100),VAT_BALL_Y_PX,0);
                } else if(wire_task==4u) {
                    CHECK(stage==9u && first180==1u && !s_seq_pair_turn &&
                          !s_seq_pair_offset && !s_route44_ball_offset && !s_route44_grab_return &&
                          wire_request!=ball_request && ball_left==0u && ball_grab_direct==1u && grab_right==0u);
                    CHECK(s_vat.x_goal==165 && s_vat.x_goal==VAT_ROUTE31_BUCKET_X_PX);
                    bucket_request=wire_request;
                    if(s_receiving) xy_obj_poll(wire_request,9,
                        VAT_ROUTE31_BUCKET_X_PX+(bucket_cruise_seen?0:100),VAT_BUCKET_Y_PX,0);
                } else {
                    CHECK(stage==14u && wire_task==3u);
                    CHECK(s_vat.x_goal==185 && s_vat.x_goal==VAT_ROUTE31_HOSTAGE_X_PX);
                    if(ranked_request!=wire_request) { ranked_request=wire_request; r44_rank(0u,1u); }
                    if(s_receiving) xy_obj_poll(wire_request,0,
                        VAT_ROUTE31_HOSTAGE_X_PX+(hostage_cruise_seen?0:100),VAT_HOSTAGE_Y_PX,0);
                }
            }
        }
        CHECK(r44_service(20u)==0 && !r44_scaled_contract_errors && !r31_board_contract_errors);
        CHECK(stage==3u || r44_scaled_calls==scaled_before);
        CHECK(stage==4u || r31_board_calls==board_before);
        if(stage==4u && s_seq_state==SQ_RUN && s_round==R_RUN && last_x>0.0f) {
            CHECK(r44_motion_rpm[0]>0.0f && r44_motion_rpm[1]>0.0f &&
                  !r44_motion_rpm[2] && !r44_motion_rpm[3]);
        }
        CHECK(s_seq_state!=SQ_ARM_PREP);
        if(stage<=ROUTE31_PREDEPLOY_STAGE)
            CHECK(!pulse_calls && !host_timer_start_calls && !host_timer_active);
        if(stage==ROUTE31_PAIR_STAGE && host_timer_active &&
           s_route31_lift_phase==R44_RACK_EXTEND) {
            CHECK(s_route44_rack_started && !host_jog_pulses[1][1] &&
                  !s_route44_ball_offset && !s_route_action_taken);
            /* At task150 the finite rack may finish during acceleration;
             * overlap means actual positive chassis command, not full cruise. */
            if(last_x>0.0f && last_x<=ROUTE31_TASK_V_MMS && !last_y && !last_w) ++rack_parallel;
        }
        if(stage==ROUTE31_PAIR_STAGE && host_timer_active &&
           s_route31_lift_phase==R31_LIFT_BALL_DOWN && !ball_grab_direct) {
            CHECK(ROUTE31_BALL_PREGRAB_LEFT_MM==0u && s_seq_state==SQ_TASK &&
                  !s_route44_ball_offset && !s_route44_grab_return && !s_seq_pair_offset &&
                  s_route31_rack_deployed && s_route_action_taken &&
                  host_jog_pulses[0][0]==ROUTE31_RACK_EXTEND_STEPS &&
                  !first180 && !last_x && !last_y && !last_w && !laser_state);
            ++ball_grab_direct;
        }
        if(stage==ROUTE31_HOSTAGE_STAGE && host_timer_active &&
           s_route31_lift_phase==R31_HOSTAGE_RACK_EXTEND && !hostage_grab_direct) {
            CHECK(ROUTE31_HOSTAGE_PREGRAB_LEFT_MM==0u && s_seq_state==SQ_TASK &&
                  !s_seq_hostage_offset && s_route_action_taken &&
                  !last_x && !last_y && !last_w && !laser_state &&
                  host_jog_pulses[0][0]>=ROUTE31_RACK_EXTEND_STEPS);
            ++hostage_grab_direct;
        }
        if(s_seq_state==SQ_TASK && (stage==9u || stage==14u) &&
           fabsf(last_x)==ROUTE31_TASK_V_MMS && !last_y && !last_w) {
            if(wire_task==1u) ball_cruise_seen=1u;
            else if(wire_task==4u) bucket_cruise_seen=1u;
            else if(wire_task==3u) hostage_cruise_seen=1u;
        }
        if(laser_state && !saw_laser) { saw_laser=1; laser_on_at=host_tick; }
        if(saw_laser && !laser_state && !laser_off_at) laser_off_at=host_tick;
    }
    if(s_seq_state!=SQ_DONE || visits!=16u || last_stage!=15u)
        fprintf(stderr,"31 revised chain state=%u visits=%u stage=%u ballL=%u ballR=%u hL=%u first180=%u second180=%u\n",
                s_seq_state,visits,last_stage,ball_left,grab_right,hostage_left,first180,second180);
    CHECK(s_seq_state==SQ_DONE && visits==16u && last_stage==15u && road_tests>=13u &&
          ball_left==0u && ball_grab_direct==1u && grab_right==0u && hostage_left==0u &&
          hostage_grab_direct==1u && first180==1u && second180==1u);
    CHECK(exit_turn==1u && exit_forward==1u && entry_turn==1u && entry_forward==1u && task_turn==1u &&
          !s_route31_exit_turn_done && !s_route31_entry_turn_done);
    CHECK(rack_parallel>0u && contact_seen && tail_ff_checks>=2u &&
          r44_scaled_calls>0u && r44_scaled_value==1.15f && !r44_scaled_contract_errors);
    CHECK(ball_cruise_seen && bucket_cruise_seen && hostage_cruise_seen);
    CHECK(saw_laser && laser_off_at-laser_on_at>=ROUTE31_LASER_MS-20u && !laser_state);
    CHECK(host_jog_pulses[0][0]==2u*ROUTE31_RACK_EXTEND_STEPS &&
          host_jog_pulses[0][1]==ROUTE31_RACK_EXTEND_STEPS+ROUTE31_HOSTAGE_RACK_RETRACT_STEPS &&
          host_jog_pulses[1][0]==ROUTE31_LIFT_DOWN_STEPS+ROUTE31_LIFT_BUCKET_DOWN_STEPS &&
          host_jog_pulses[1][1]==ROUTE31_LIFT_DOWN_STEPS+ROUTE31_LIFT_BUCKET_DOWN_STEPS);
    CHECK(host_servo==ROUTE31_HOSTAGE_GRIP_US && s_route31_hostage_rank==1u &&
          s_dist_target==(float)ROUTE31_HOSTAGE_RANK1_MM && r44_stopped());
    CHECK(s_route43_tune.road_mm[0]==545u && s_route43_tune.road_mm[7]==780u &&
          s_route43_tune.straight_v==100.0f && s_route43_tune.lateral_v==100.0f);
    printf("31 actualautomatic16stage:LEFT585/configuredBACK/DIAGONAL_back100_left100/BACK550_leftRPMx1.15/BACK190/RIGHT90/FWD775/RIGHT90/FWD775/RIGHT90;eachinternalturn/forwardonce,ordinaryFFF_retained_no_bff/rff/xkp,zeroSTEPthroughentry;firstNEWBALLparallelrackcompletebeforeDIRECTgrab,NOzeroDistanceLEFT;tail%.0fFF0_yawhold,BALL165_pregrabLEFT0_grab_NO_RIGHT30_first180_thenNEW_BUCKET165,release_retract_second180,laser2s,rankcorner635/465/390,HOSTAGE185_LEFT0_DIRECTgrab_retract_54_finalDONE;44LEFT30separatelyreplayed;noextraG passed\n",ROUTE31_TASK_V_MMS);
    return 0;
}

int main(void)
{
    CHECK(check_r31_scaled_actual_outputs_and_scope()==0);
    CHECK(check_r31_mirrored_ff_ram_and_scope()==0);
    CHECK(check_r31_second_return_180_scope()==0);
    CHECK(check_r31_postcross_turn_flags_stop_reset()==0);
    CHECK(check_r31_board_push_outputs_scope_and_stop()==0);
    CHECK(check_r31_first_ball_parallel_rack_and_cancel()==0);
    CHECK(check_r31_automatic_complete_chain()==0);
    CHECK(check_r44_complete_physical_plan()==0);
    puts("route31_revised_tail_test: actual software runner/IK only, no physical route or grasp acceptance");
    return 0;
}
