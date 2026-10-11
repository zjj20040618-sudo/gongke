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
    return s_seq_mode==31u && scale==1.30f;
}

static int r31_revised_boot(void)
{
    CHECK(wire_boot()==0); host_wire_diag_hook=real_proto_wire_diag_get; xy_seq=1u;
    r44_scaled_calls=r44_scaled_contract_errors=0u; r44_scaled_value=0.0f;
    run_cmd("yfix1"); run_cmd("31"); wire_sync();
    CHECK(s_seq_state==SQ_READY && s_seq_mode==31u && r44_stopped());
    wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
    CHECK(proto_qr_get(NULL));
    run_cmd("g"); CHECK(s_seq_state==SQ_STILL && !route44_owner() && !s_route44_frozen);
    return 0;
}

/* Replay the real first nine automatic route stages. No direct stage jump or
 * invented rack position: even LEFT90 must finish without one STEP emitted. */
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
            CHECK(r44_finish_road()==0);
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
    CHECK(r44_service(500u)==0 && s_seq_state==SQ_TASK && last_x==50.0f &&
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
    xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !s_ack && !s_vat.ball_seen &&
          !s_route44_rack_started && !host_timer_active && !pulse_calls);
    wire_ack(request,2u,0u);
    CHECK(r44_service(500u)==0 && last_x==50.0f && !pulse_calls &&
          !host_timer_start_calls && !s_route44_rack_started && !s_vat.ball_seen);
    xy_qr(request,"123");
    r44_rank(4u,2u);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    xy_obj_poll(request,-1,0,0,0);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    /* A selected payload replaying that empty image's sequence is still old. */
    unsigned empty_sequence=xy_seq-1u;
    xy_seq=empty_sequence;
    xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    xy_obj_poll(request,5,VAT_ROUTE_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    xy_obj_poll((uint16_t)(request-1u),4,VAT_ROUTE_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !pulse_calls && !s_route44_rack_started);
    int brakes=brake_calls,servos=servo_calls;
    xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && s_route44_rack_started && host_timer_active &&
          host_timer_start_calls==1u && s_route31_lift_phase==R44_RACK_EXTEND &&
          s_seq_state==SQ_TASK && last_x==50.0f && !last_y && !last_w &&
          brake_calls==brakes && servo_calls==servos && !s_route_action_taken &&
          !s_route44_ball_offset && !host_jog_pulses[1][1]);
    unsigned starts=host_timer_start_calls;
    for(unsigned k=0u;k<5u;++k) {
        xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX+100,VAT_BALL_Y_PX,0);
        CHECK(r44_service(20u)==0 && s_seq_state==SQ_TASK && last_x==50.0f &&
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
        xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX+100,VAT_BALL_Y_PX,0);
        CHECK(r44_service(120u)==0 && host_timer_active && last_x==50.0f &&
              host_jog_pulses[0][0]>0u && host_jog_pulses[0][0]<ROUTE31_RACK_EXTEND_STEPS);
        int pulses=pulse_calls;
        servos=servo_calls; starts=host_timer_start_calls;
        run_cmd(keys[k]);
        CHECK(s_seq_state==SQ_STOPPED && r44_stopped() && !s_jog_clock.remaining);
        wire_ack(request,2u,0u); r44_rank(4u,2u);
        xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
        xy_obj_poll((uint16_t)(request-1u),4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
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
    r44_real_motion_vel_set_left_scaled(-300.0f,0.0f,0.0f,1.30f);
    memcpy(changed,r44_motion_rpm,sizeof changed);
    CHECK(fabsf(changed[0]-plain[0]*1.30f)<0.0001f &&
          fabsf(changed[3]-plain[3]*1.30f)<0.0001f &&
          changed[1]==plain[1] && changed[2]==plain[2]);
    r44_real_motion_vel_set_left_scaled(-300.0f,0.0f,0.0f,1.10f);
    CHECK(fabsf(r44_motion_rpm[0]-plain[0]*1.10f)<0.0001f &&
          fabsf(r44_motion_rpm[3]-plain[3]*1.10f)<0.0001f &&
          r44_motion_rpm[1]==plain[1] && r44_motion_rpm[2]==plain[2]);
    r44_real_motion_vel_set_precise(-300.0f,0.0f,0.0f);
    CHECK(!memcmp(plain,r44_motion_rpm,sizeof plain));
    CHECK(r31_revised_boot()==0);
    CHECK(s_route31_plan[0].distance_mm==545u && s_route31_plan[7].distance_mm==780u);
    CHECK(s_route43_tune.road_mm[0]==545u && s_route43_tune.road_mm[1]==610u &&
          s_route43_tune.road_mm[3]==620u && s_route43_tune.road_mm[7]==780u);
    CHECK(s_route44_plan[0].leg.mode==15u && s_route44_plan[0].leg.distance_mm==600u &&
          s_route44_plan[4].leg.distance_mm==620u && ROUTE44_CROSS_LEFT_SCALE==1.10f &&
          ROUTE44_STAGES==22u);
    run_cmd("0"); CHECK(r44_stopped());
    /* Deliberate stopped-slot dispatch verifies retained43 never opts into
     * either31's new1.30 or44's1.10 API; this is not a second full route. */
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
    puts("31 actualIK leftm0/m3 x1.30 versus44 x1.10/rightwheels unchanged/normalAPI no persistence;shared43baseline545/780 and44fixed22plan remain isolated passed");
    return 0;
}

static int check_r31_automatic_complete_chain(void)
{
    static const uint8_t modes[16]={17,16,20,16,15,16,18,16,30,41,22,35,15,20,40,15};
    CHECK(r31_revised_boot()==0);
    unsigned visits=0u,last_stage=255u,road_tests=0u;
    unsigned ball_left=0u,grab_right=0u,hostage_left=0u,first180=0u,second180=0u;
    unsigned rack_parallel=0u,tail_ff_checks=0u;
    unsigned ball_cruise_seen=0u,bucket_cruise_seen=0u,hostage_cruise_seen=0u;
    uint32_t entered_test=0u,laser_on_at=0u,laser_off_at=0u;
    uint16_t ranked_request=0u,ball_request=0u,bucket_request=0u;
    int saw_laser=0,contact_seen=0;
    for(unsigned tick=0u;tick<20000u && s_seq_state!=SQ_DONE;++tick) {
        CHECK(s_seq_state!=SQ_STOPPED && s_seq_state!=SQ_STEP_WAIT &&
              s_seq_state!=SQ_STEP_DRAIN && s_seq_mode==31u &&
              !route44_owner() && !s_route44_frozen);
        unsigned stage=s_seq_stage,scaled_before=r44_scaled_calls;
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
                        CHECK(goal==180.0f && ball_left==1u && grab_right==1u &&
                              wire_task==1u && wire_request==ball_request &&
                              host_jog_pulses[1][1]==ROUTE31_LIFT_DOWN_STEPS &&
                              host_jog_pulses[1][0]==ROUTE31_LIFT_DOWN_STEPS);
                        ++first180;
                    } else if(stage==10u) {
                        CHECK(goal==180.0f && bucket_request &&
                              host_jog_pulses[0][1]==ROUTE31_RACK_EXTEND_STEPS &&
                              host_jog_pulses[1][1]==ROUTE31_LIFT_DOWN_STEPS+ROUTE31_LIFT_BUCKET_DOWN_STEPS);
                        ++second180;
                    } else CHECK(goal==(stage==8u?-90.0f:stage==13u?ROUTE31_HOSTAGE_RIGHT_TARGET_DEG:90.0f));
                    host_absolute_yaw+=goal-host_yaw; host_yaw=goal;
                } else {
                    CHECK(dist_mode());
                    if(stage>=9u) {
                        CHECK(s_v==50.0f && s_dist_ff_ratio==0.0f &&
                              s_dist_heading_kp>0.0f && s_dist_align_enabled);
                        ++tail_ff_checks;
                    }
                    if(stage==0u) CHECK(s_msel==17 && s_dist_target==-565.0f && s_v==100.0f);
                    if(stage==1u) CHECK(s_msel==16 && s_dist_target==-610.0f && s_v==100.0f);
                    if(stage==3u) CHECK(s_msel==16 && s_dist_target==-620.0f &&
                                         s_v==300.0f && s_dist_heading_kp==0.0f && !s_dist_ff_ratio);
                    if(stage==7u) CHECK(s_msel==16 && s_dist_target==-760.0f && s_v==100.0f);
                    if(stage==9u) {
                        CHECK(s_route31_rack_deployed && !s_seq_pair_offset);
                        if(s_msel==17) {
                            CHECK(s_route44_ball_offset && s_dist_target==-30.0f &&
                                  !host_jog_pulses[1][1] && !first180 &&
                                  host_jog_pulses[0][0]==ROUTE31_RACK_EXTEND_STEPS &&
                                  s_route31_lift_phase==R31_LIFT_OFF && !host_timer_active);
                            ++ball_left;
                        } else {
                            CHECK(s_msel==18 && s_route44_grab_return && s_dist_target==30.0f &&
                                  host_jog_pulses[1][1]==ROUTE31_LIFT_DOWN_STEPS &&
                                  host_jog_pulses[1][0]==ROUTE31_LIFT_DOWN_STEPS && !first180);
                            ++grab_right;
                        }
                    }
                    if(stage==14u && s_msel==17) {
                        CHECK(s_seq_hostage_offset && s_dist_target==-30.0f &&
                              host_jog_pulses[0][0]==ROUTE31_RACK_EXTEND_STEPS);
                        ++hostage_left;
                    }
                    if(s_board_contact.active) {
                        CHECK(stage==4u && s_v==40.0f && s_dist_heading_kp==0.0f);
                        contact_seen=1;
                    } else {
                        host_yaw=s_dist_heading0+0.2f;
                        CHECK(r44_service(20u)==0 && s_seq_state==SQ_RUN && s_round==R_RUN);
                        if(stage>=9u) {
                            CHECK(dist_lateral()?last_x==0.0f:last_y==0.0f);
                            CHECK(last_w<0.0f);
                        }
                        host_yaw=s_dist_heading0;
                        if(dist_lateral()) host_lateral=s_dist_odo0+s_dist_target;
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
                      route31_straight_speed()==50.0f && s_target35_kp>0.0f);
                if(s_receiving) xy_obj_poll(wire_request,8,ROUTE31_TARGET_CX,100,0);
            } else {
                CHECK(stage==9u || stage==14u);
                CHECK(s_route_search_ff==0.0f);
                if(s_route_owned) CHECK(s_route_search_speed==50.0f && s_route_search_kp>0.0f);
                if(wire_task==1u) {
                    CHECK(stage==9u && !first180 && !s_seq_pair_offset);
                    if(!ball_request) ball_request=wire_request;
                    CHECK(wire_request==ball_request);
                    if(ranked_request!=wire_request) { ranked_request=wire_request; r44_rank(4u,2u); }
                    if(s_receiving) xy_obj_poll(wire_request,4,
                        VAT_ROUTE_BALL_X_PX+(ball_cruise_seen?0:100),VAT_BALL_Y_PX,0);
                } else if(wire_task==4u) {
                    CHECK(stage==9u && first180==1u && !s_seq_pair_turn &&
                          !s_seq_pair_offset && !s_route44_ball_offset && !s_route44_grab_return &&
                          wire_request!=ball_request && ball_left==1u && grab_right==1u);
                    bucket_request=wire_request;
                    if(s_receiving) xy_obj_poll(wire_request,9,
                        VAT_ROUTE_BUCKET_X_PX+(bucket_cruise_seen?0:100),VAT_BUCKET_Y_PX,0);
                } else {
                    CHECK(stage==14u && wire_task==3u);
                    if(ranked_request!=wire_request) { ranked_request=wire_request; r44_rank(0u,1u); }
                    if(s_receiving) xy_obj_poll(wire_request,0,
                        VAT_ROUTE_HOSTAGE_X_PX+(hostage_cruise_seen?0:100),VAT_HOSTAGE_Y_PX,0);
                }
            }
        }
        CHECK(r44_service(20u)==0 && !r44_scaled_contract_errors);
        CHECK(stage==3u || r44_scaled_calls==scaled_before);
        CHECK(s_seq_state!=SQ_ARM_PREP);
        if(stage<=ROUTE31_PREDEPLOY_STAGE)
            CHECK(!pulse_calls && !host_timer_start_calls && !host_timer_active);
        if(stage==ROUTE31_PAIR_STAGE && host_timer_active &&
           s_route31_lift_phase==R44_RACK_EXTEND) {
            CHECK(s_route44_rack_started && !host_jog_pulses[1][1] &&
                  !s_route44_ball_offset && !s_route_action_taken);
            if(last_x==50.0f && !last_y && !last_w) ++rack_parallel;
        }
        if(s_seq_state==SQ_TASK && (stage==9u || stage==14u) &&
           fabsf(last_x)==50.0f && !last_y && !last_w) {
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
    CHECK(s_seq_state==SQ_DONE && visits==16u && last_stage==15u && road_tests>=14u &&
          ball_left==1u && grab_right==1u && hostage_left==1u && first180==1u && second180==1u);
    CHECK(rack_parallel>0u && contact_seen && tail_ff_checks>=5u &&
          r44_scaled_calls>0u && r44_scaled_value==1.30f && !r44_scaled_contract_errors);
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
    puts("31 actualautomatic16stage:LEFT565/BACK610/BACK620_leftRPMx1.30/BACK760;zeroSTEPthroughLEFT90,firstNEWBALLparallelrackcompletebeforeLEFT30/grab;tail50FF0_yawhold,LEFT30_grab_RIGHT30_first180_thenNEWbucket,release_retract_second180exact,laser2s,hostageLEFT30_NEWX_grab_retract_54_finalDONE;noextraG passed");
    return 0;
}

int main(void)
{
    CHECK(check_r31_scaled_actual_outputs_and_scope()==0);
    CHECK(check_r31_first_ball_parallel_rack_and_cancel()==0);
    CHECK(check_r31_automatic_complete_chain()==0);
    CHECK(check_r44_complete_physical_plan()==0);
    puts("route31_revised_tail_test: actual software runner/IK only, no physical route or grasp acceptance");
    return 0;
}
