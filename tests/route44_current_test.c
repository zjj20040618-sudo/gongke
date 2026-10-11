/* New44 per-physical-stage gated runner through the actual BLE/VAT/CRC/STEP
 * implementations. All pixels/yaw/encoder endpoints are explicit HOST inputs;
 * this verifies software sequencing, not actual travel or physical success. */
#define VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN route44_prior_xy_main
#include "vision_align_bluetooth_test.c"
#undef VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN

/* Keep the real motion implementation in a distinct host namespace. The route
 * fixture still observes chassis commands, while scaled API calls additionally
 * exercise actual IK/publication into a four-wheel capture, not an invented
 * wheel-RPM model. Only the new44 opt-in API delegates here. */
static float r44_motion_rpm[4];
static unsigned r44_motion_integer_writes, r44_motion_precise_writes, r44_motion_stops;
static unsigned r44_motion_masked_writes, r44_motion_irq_enters, r44_motion_irq_restores;
static uint32_t r44_motion_irq_mask;
static uint32_t r44_capture_irq_mask(void) { return r44_motion_irq_mask; }
static void r44_capture_irq_disable(void)
{ r44_motion_irq_mask=1u; ++r44_motion_irq_enters; }
static void r44_capture_irq_restore(uint32_t mask)
{ r44_motion_irq_mask=mask; ++r44_motion_irq_restores; }
static void r44_capture_integer_speed(int motor,int16_t rpm)
{ r44_motion_rpm[motor]=(float)rpm; ++r44_motion_integer_writes; }
static void r44_capture_precise_speed(int motor,float rpm)
{
    r44_motion_rpm[motor]=rpm; ++r44_motion_precise_writes;
    if(r44_motion_irq_mask) ++r44_motion_masked_writes;
}
static void r44_capture_stop(void)
{ memset(r44_motion_rpm,0,sizeof r44_motion_rpm); ++r44_motion_stops; }
void r44_real_motion_brake(void);
void r44_real_motion_ik_precise(float vx,float vy,float w,float rpm[4]);
#define motion_init r44_real_motion_init
#define motion_ik r44_real_motion_ik
#define motion_vel_set r44_real_motion_vel_set
#define motion_ik_precise r44_real_motion_ik_precise
#define motion_vel_set_precise r44_real_motion_vel_set_precise
#define motion_vel_set_creep r44_real_motion_vel_set_creep
#define motion_vel_set_left_scaled r44_real_motion_vel_set_left_scaled
#define motion_brake r44_real_motion_brake
#define motion_ramp_init r44_real_motion_ramp_init
#define motion_ramp_step r44_real_motion_ramp_step
#define motion_profile_get r44_real_motion_profile_get
#define motion_profile_set r44_real_motion_profile_set
#define motion_profile_config_missing r44_real_motion_profile_config_missing
#define motion_linear_ramp_init r44_real_motion_linear_ramp_init
#define motion_linear_profile_step r44_real_motion_linear_profile_step
#define motion_linear_ramp_step r44_real_motion_linear_ramp_step
#define motion_pose_update r44_real_motion_pose_update
#define motion_pose r44_real_motion_pose
#define motion_odo_mm r44_real_motion_odo_mm
#define motion_lateral_odo_mm r44_real_motion_lateral_odo_mm
#define ctrl_set_speed r44_capture_integer_speed
#define ctrl_set_speed_precise r44_capture_precise_speed
#define ctrl_set_speed_creep r44_capture_precise_speed
#define ctrl_stop_all r44_capture_stop
#define __get_PRIMASK r44_capture_irq_mask
#define __disable_irq r44_capture_irq_disable
#define __set_PRIMASK r44_capture_irq_restore
#include "../App/motion.c"
#undef motion_init
#undef motion_ik
#undef motion_vel_set
#undef motion_ik_precise
#undef motion_vel_set_precise
#undef motion_vel_set_creep
#undef motion_vel_set_left_scaled
#undef motion_brake
#undef motion_ramp_init
#undef motion_ramp_step
#undef motion_profile_get
#undef motion_profile_set
#undef motion_profile_config_missing
#undef motion_linear_ramp_init
#undef motion_linear_profile_step
#undef motion_linear_ramp_step
#undef motion_pose_update
#undef motion_pose
#undef motion_odo_mm
#undef motion_lateral_odo_mm
#undef ctrl_set_speed
#undef ctrl_set_speed_precise
#undef ctrl_set_speed_creep
#undef ctrl_stop_all
#undef __get_PRIMASK
#undef __disable_irq
#undef __set_PRIMASK

static unsigned r44_scaled_calls, r44_scaled_contract_errors;
static float r44_scaled_value;
#ifndef ROUTE44_SCALED_FIXTURE_CONTRACT
#define ROUTE44_SCALED_FIXTURE_CONTRACT(vx,vy,w,scale) \
    (route44_owner() && s_seq_state==SQ_RUN && s_route44_index==4u && \
     s_seq_stage==3u && s_msel==16 && s_dist_heading_kp==0.0f && \
     (vx)<0.0f && (vy)==0.0f && (w)==0.0f && (scale)==ROUTE44_CROSS_LEFT_SCALE)
#endif
void motion_vel_set_left_scaled(float vx,float vy,float w,float left_scale)
{
    ++r44_scaled_calls; r44_scaled_value=left_scale;
    if(!ROUTE44_SCALED_FIXTURE_CONTRACT(vx,vy,w,left_scale))
        ++r44_scaled_contract_errors;
    r44_real_motion_vel_set_left_scaled(vx,vy,w,left_scale);
    motion_vel_set(vx,vy,w); /* Existing route fixture's observed chassis command. */
}

static int r44_stopped(void)
{ return !last_x && !last_y && !last_w && !laser_state && !host_timer_active && !s_go; }

static int r44_service(unsigned ms)
{
    while(ms) {
        unsigned slice=ms<20u?ms:20u;
        host_timer_sync_hal();
        uint64_t until=host_timer_us+(uint64_t)slice*1000u;
        while(host_timer_active && host_timer_next_us<=until)
            CHECK(jog_host_timer_event()==0);
        host_timer_us=until; host_tick=(uint32_t)(until/1000u);
        wire_poll(); ms-=slice;
    }
    return 0;
}

static void r44_rank(uint8_t model,uint8_t rank)
{
    uint8_t p[14]={0x62u,(uint8_t)wire_request,(uint8_t)(wire_request>>8),9u,0u,0x54u,0u,0u,model,rank,3u,255u,255u,255u};
    uint8_t first=model>=3u?3u:0u,next=0u;
    p[10u+rank]=model;
    for(uint8_t i=0u;i<3u;++i) {
        if(i==rank-1u) continue;
        while(first+next==model) ++next;
        p[11u+i]=first+next++;
    }
    route_wire_le16(p+6u,xy_seq++); wire_feed(p,sizeof p,0);
}

static int r44_boot(void)
{
    CHECK(wire_boot()==0); host_wire_diag_hook=real_proto_wire_diag_get; xy_seq=1u;
    r44_scaled_calls=r44_scaled_contract_errors=0u; r44_scaled_value=0.0f;
    run_cmd("yfix0"); CHECK(!s_dist_align_on);
    run_cmd("31"); run_cmd("v219"); run_cmd("pv321");
    run_cmd("43"); run_cmd("v125"); run_cmd("pv135");
    run_cmd("44"); wire_sync();
    CHECK(s_seq_mode==44u && s_seq_state==SQ_READY && r44_stopped());
    return 0;
}

static int r44_start_road(void)
{
    for(unsigned k=0;k<180u && s_seq_state!=SQ_RUN;++k) {
        if(s_seq_state==SQ_STEP_WAIT) {
            CHECK(s_route44_frozen && s_route44_hold.valid && r44_stopped());
            run_cmd("g"); CHECK(!s_route44_frozen);
        }
        if(s_seq_state==SQ_CROSS_YAW) {
            float correction=s_route31_task_yaw_goal-host_yaw;
            host_absolute_yaw+=correction; host_yaw=s_route31_task_yaw_goal;
        }
        CHECK(r44_service(20u)==0);
        CHECK(s_seq_state!=SQ_STOPPED);
    }
    CHECK(s_seq_state==SQ_RUN && s_round==R_RUN);
    return 0;
}

/* Caller supplies the sensor-measured endpoint; no simulated traction plant. */
static int r44_finish_road(void)
{
    unsigned index=s_route44_index;
    CHECK(s_seq_state==SQ_RUN);
    if(turn_closed_loop_mode()) {
        host_absolute_yaw+=turn_target_deg()-host_yaw; host_yaw=turn_target_deg();
    } else if(s_board_contact.active) {
        host_pitch=s_board_contact.pitch0+route_contact_tilt_deg()+0.1f;
    } else if(dist_lateral()) host_lateral=s_dist_odo0+s_dist_target;
    else host_fore=s_dist_odo0+s_dist_target;
    for(unsigned k=0;k<180u && s_route44_index==index && s_seq_state==SQ_RUN;++k) {
        if(s_round==R_ALIGN) host_yaw=s_dist_heading0;
        CHECK(r44_service(20u)==0);
    }
    CHECK(s_seq_state!=SQ_STOPPED);
    return 0;
}

static int r44_prepare_ball_approach(void)
{
    CHECK(r44_boot()==0); run_cmd("g");
    for(unsigned k=0;k<80u && s_route44_index<ROUTE44_BALL_ENTRY_INDEX;++k) {
        if(s_seq_state==SQ_QR_WAIT) {
            wire_sync(); wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
        } else {
            if(s_seq_state!=SQ_RUN) CHECK(r44_start_road()==0);
            CHECK(r44_finish_road()==0);
        }
    }
    CHECK(s_route44_index==ROUTE44_BALL_ENTRY_INDEX && s_seq_stage==ROUTE44_ROAD_ROLE);
    if(s_seq_state==SQ_STEP_WAIT) run_cmd("g");
    wire_sync(); CHECK(wire_opcode==0x63u && wire_task==1u && wire_digit==1u);
    CHECK(s_route44_ball_early && !s_route44_rack_started && !host_timer_active);
    return 0;
}

static int r44_to_ball_approach(void)
{
    CHECK(r44_prepare_ball_approach()==0);
    wire_ack(wire_request,2u,0u); /* Actual camera ACK before stopped preparation elapses. */
    if(s_seq_state!=SQ_RUN) CHECK(r44_start_road()==0);
    CHECK(s_dist_target==200.0f && s_msel==15 && s_v==50.0f);
    return 0;
}

static int r44_trigger_parallel_rack(void)
{
    int brakes=brake_calls;
    wire_ack(wire_request,2u,0u);
    xy_obj_poll(wire_request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0);
    CHECK(s_route44_rack_started && host_timer_active && s_seq_state==SQ_RUN &&
          s_route44_index==ROUTE44_BALL_ENTRY_INDEX && last_x==50.0f && !last_y && !last_w &&
          brake_calls==brakes);
    return 0;
}

static int check_r44_parallel_rack_freshness_and_cancel(void)
{
    CHECK(r44_to_ball_approach()==0);
    uint16_t request=wire_request;
    wire_ack(request,2u,0u); CHECK(r44_service(20u)==0);
    CHECK(!s_route44_rack_started && !pulse_calls && last_x==50.0f);
    r44_rank(4u,2u); CHECK(r44_service(20u)==0);
    CHECK(!s_route44_rack_started && !pulse_calls && last_x==50.0f);
    xy_obj_poll(request,-1,0,0,0); CHECK(r44_service(20u)==0);
    CHECK(!s_route44_rack_started && !pulse_calls && last_x==50.0f);
    unsigned previous_sequence=xy_seq-1u;
    xy_seq=previous_sequence;
    xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !s_route44_rack_started && !pulse_calls);
    xy_obj_poll(request,5,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !s_route44_rack_started && !pulse_calls);
    xy_obj_poll((uint16_t)(request-1u),4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !s_route44_rack_started && !pulse_calls);
    CHECK(r44_trigger_parallel_rack()==0);
    unsigned starts=host_timer_start_calls;
    int servos=servo_calls,brakes=brake_calls;
    for(unsigned k=0;k<5u;++k) {
        xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
        CHECK(r44_service(20u)==0 && s_seq_state==SQ_RUN && last_x==50.0f && !last_y && !last_w);
        CHECK(host_timer_start_calls==starts && host_jog_pulses[0][0]>0u &&
              host_jog_pulses[0][0]<ROUTE31_RACK_EXTEND_STEPS && !host_jog_pulses[1][1] &&
              servo_calls==servos && brake_calls==brakes && !s_route44_ball_offset &&
              !s_route_action_taken && !s_vat.ball_fallback);
    }
    unsigned old_ball_sequence=xy_seq-1u;
    CHECK(r44_finish_road()==0 && s_route44_index==15u && s_seq_stage==ROUTE31_PAIR_STAGE);
    CHECK(r44_service(4000u)==0 && s_seq_state==SQ_STEP_WAIT && s_route44_frozen &&
          s_route31_rack_deployed && !host_timer_active &&
          host_jog_pulses[0][0]==ROUTE31_RACK_EXTEND_STEPS && !host_jog_pulses[1][1] &&
          !s_route44_ball_offset && !s_route_action_taken && !s_vat.ball_fallback &&
          wire_request==request && s_vat.state!=VAT_STOPPED);
    ProtoTargetRank rank;
    CHECK(proto_target_rank_get(&rank) && rank.rank==2u);
    unsigned next_sequence=xy_seq;
    xy_seq=old_ball_sequence;
    xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
    CHECK(r44_service(100u)==0 && !s_route44_ball_offset && !host_jog_pulses[1][1]);
    xy_seq=next_sequence;
    run_cmd("g"); CHECK(!s_route44_frozen && s_seq_state==SQ_TASK);
    for(unsigned k=0;k<150u && !s_route44_ball_offset;++k) {
        xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
        CHECK(r44_service(20u)==0 && s_seq_state!=SQ_STOPPED);
    }
    CHECK(s_route44_ball_offset && !host_jog_pulses[1][1] && r44_start_road()==0 &&
          s_msel==17 && s_dist_target==-30.0f && s_dist_align_enabled);
    CHECK(r44_finish_road()==0 && s_seq_state==SQ_TASK && host_timer_active);
    CHECK(r44_service(20u)==0 && host_jog_pulses[1][1]>0u);
    run_cmd("0"); CHECK(r44_stopped());
    for(unsigned k=0;k<3u;++k) {
        const char *keys[]={"g","a","0"};
        CHECK(r44_to_ball_approach()==0 && r44_trigger_parallel_rack()==0);
        CHECK(r44_service(100u)==0 && last_x==50.0f && host_timer_active);
        int pulses=pulse_calls;
        run_cmd(keys[k]); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
        wire_ack(wire_request,2u,0u);
        xy_obj_poll(wire_request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
        CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED && r44_stopped() && pulse_calls==pulses);
    }
    puts("44 earlyBALL realACK/01 selectedfresh-only triggers finite parallelrack3400;54/empty/replay/wrongtarget/wrongrequest cannot;roadcontinues50 whileIRQpulses;PAIR keepsrequest/rank but needsNEWpixels;g/a/0 stopsboth passed");
    return 0;
}

static int check_r44_ack_pending_and_new_after_entry_loss(void)
{
    CHECK(r44_prepare_ball_approach()==0);
    CHECK(s_receiving && !s_ack);
    xy_obj_poll(wire_request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0 && !s_route44_rack_started && !host_timer_active &&
          !s_vat.ball_seen && !s_route_action_taken);
    wire_ack(wire_request,2u,0u);
    CHECK(r44_service(20u)==0 && !s_route44_rack_started && !host_timer_active && !s_vat.ball_seen);
    CHECK(r44_start_road()==0 && !s_route44_rack_started && last_x==50.0f);
    CHECK(r44_trigger_parallel_rack()==0);
    r44_rank(4u,2u);
    CHECK(r44_finish_road()==0 && s_route44_index==15u);
    CHECK(r44_service(4000u)==0 && s_seq_state==SQ_STEP_WAIT && s_route44_frozen &&
          s_route31_rack_deployed && !s_route44_ball_offset &&
          !s_route_action_taken && !s_vat.ball_seen && !s_vat.ball_fallback && !host_jog_pulses[1][1]);
    run_cmd("g"); CHECK(s_seq_state==SQ_TASK && !s_route44_frozen);
    /* Only a NEW selected01 AFTER FWD200 may start the seen/lost300 clock. */
    xy_obj_poll(wire_request,4,VAT_ROUTE_BALL_X_PX+20,VAT_BALL_Y_PX,0);
    CHECK(r44_service(20u)==0);
    for(unsigned k=0;k<120u && !s_route44_ball_offset;++k)
        CHECK(r44_service(20u)==0 && s_seq_state!=SQ_STOPPED);
    CHECK(s_route44_ball_offset && s_vat.ball_fallback && s_route_action_taken &&
          s_route31_rack_deployed && !host_jog_pulses[1][1]);
    run_cmd("0"); CHECK(r44_stopped());
    puts("44 beforeACK01 cannot arm or surviveACK;NEW01 afterentry alone enablesexistinglost300 fallback,oldmovingpixels neverdo passed");
    return 0;
}

static int check_r44_first_turn_qr_gate(void)
{
    CHECK(r44_boot()==0);
    /* An old legal tuple may be present from an earlier handset scan. It
     * cannot authorize the NEW QR wait belonging to the completed first turn. */
    proto_qr_begin(); wire_sync(); wire_ack(wire_request,1u,0u);
    xy_qr(wire_request,"123"); CHECK(proto_qr_get(NULL));
    uint16_t old_request=wire_request;
    run_cmd("g"); CHECK(r44_start_road()==0);
    CHECK(s_route44_index==0u && s_seq_stage==0u && s_msel==15 &&
          s_dist_target==600.0f && s_v==50.0f && s_dist_ff_ratio==0.0f);
    CHECK(r44_finish_road()==0 && s_route44_index==1u && r44_start_road()==0);
    CHECK(s_msel==20 && turn_target_deg()==90.0f);
    CHECK(r44_finish_road()==0 && s_seq_state==SQ_QR_WAIT && r44_stopped());
    wire_sync(); CHECK(wire_request!=old_request && wire_mode==1u && !proto_qr_get(NULL));
    unsigned index=s_route44_index,stage=s_seq_stage;
    wire_ack(wire_request,1u,0u);
    CHECK(r44_service(5000u)==0 && s_seq_state==SQ_QR_WAIT && r44_stopped());
    xy_qr(old_request,"123");
    CHECK(s_seq_state==SQ_QR_WAIT && s_route44_index==index && s_seq_stage==stage && r44_stopped());
    xy_qr(wire_request,"000"); CHECK(s_seq_state==SQ_QR_WAIT && r44_stopped());
    run_cmd("g"); CHECK(s_seq_state==SQ_QR_WAIT && s_route44_frozen && r44_stopped());
    xy_qr(wire_request,"123");
    CHECK(s_route44_index==2u && s_seq_stage==1u && s_seq_state==SQ_STEP_WAIT &&
          s_route44_frozen && r44_stopped());
    CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STEP_WAIT && r44_stopped());
    CHECK(r44_start_road()==0 && s_msel==15 && s_dist_target==600.0f && s_v==50.0f);
    run_cmd("0"); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
    puts("44 first FWD600 then newgRIGHT90;onlyNEW legal QR entersfrozenWAIT_G beforeFWD600;old/invalid tuples,QRwaitg and5s polls cannot move passed");
    return 0;
}

static int check_r44_parameters_isolation_cancel(void)
{
    CHECK(r44_boot()==0);
    CHECK(route_seq_stage_count()==22u && route31_straight_speed()==50.0f &&
          route31_lateral_speed()==50.0f && s_route31_straight_v==219.0f &&
          s_route31_lateral_v==321.0f && s_route43_tune.straight_v==125.0f &&
          s_route43_tune.lateral_v==135.0f);
    for(unsigned k=0;k<3u;++k) {
        const char *keys[]={"g","a","0"};
        CHECK(r44_boot()==0); run_cmd("g"); CHECK(r44_start_road()==0);
        run_cmd(keys[k]); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
        CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED && r44_stopped());
        run_cmd("g"); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
    }
    run_cmd("31"); CHECK(s_seq_state==SQ_READY && s_seq_mode==31u && s_v==219.0f);
    run_cmd("43"); CHECK(s_seq_state==SQ_READY && s_seq_mode==43u && s_v==125.0f);
    run_cmd("35"); CHECK(s_seq_state==SQ_OFF && s_target35_phase==TA_READY &&
                         target35_fine_speed()==50.0f);
    run_cmd("15"); CHECK(s_seq_state==SQ_OFF && !s_dist_align_on);
    run_cmd("d100"); run_cmd("g"); CHECK(s_round==R_RUN && !s_dist_align_enabled);
    run_cmd("g"); CHECK(!last_x && !last_y && !last_w);
    puts("44 defaults50 not31/43 RAM knobs;g/a/0 immediate cancel+noautomaticresume;31/43/35 selection isolation;manualyfix0 retained passed");
    return 0;
}

static int check_r44_qr_wait_cancel_and_fixed_commands(void)
{
    static const char *const write_keys[]={"v100","pv100","sv100","d100","t95","tv2","tkp0.05",
        "ykp1","lkp1","xkp1","fff0.01","bff0.01","lff0.01","rff0.01","cff0.01","eff0.01",
        "sff0.01","e100","f1000","h100","lf1000","u1500","nl100","nr100","yfix1","kp1","cn0"};
    CHECK(r44_boot()==0);
    RouteStepTune tune=s_route43_tune;
    float straight=s_route31_straight_v,lateral=s_route31_lateral_v;
    float heading=s_route_heading_kp,forward=s_route_forward_ff_ratio;
    float distance=s_d,velocity=s_v;
    int pulses=pulse_calls,servos=servo_calls;
    for(unsigned i=0;i<sizeof write_keys/sizeof write_keys[0];++i) {
        run_cmd(write_keys[i]);
        CHECK(s_seq_mode==44u && s_seq_state==SQ_READY && r44_stopped());
        CHECK(!memcmp(&tune,&s_route43_tune,sizeof tune) &&
              s_route31_straight_v==straight && s_route31_lateral_v==lateral &&
              s_route_heading_kp==heading && s_route_forward_ff_ratio==forward &&
              s_d==distance && s_v==velocity && !s_dist_align_on &&
              pulse_calls==pulses && servo_calls==servos);
        CHECK(strstr(last_message,"R44_FIXED_PROFILE"));
    }
    for(unsigned k=1;k<3u;++k) {
        const char *keys[]={"g","a","0"};
        CHECK(r44_boot()==0); run_cmd("g"); CHECK(r44_start_road()==0);
        CHECK(r44_finish_road()==0 && r44_start_road()==0);
        CHECK(r44_finish_road()==0 && s_seq_state==SQ_QR_WAIT && r44_stopped());
        wire_sync(); uint16_t request=wire_request;
        unsigned index=s_route44_index;
        run_cmd(keys[k]); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
        wire_ack(request,1u,0u); xy_qr(request,"123");
        CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED && s_route44_index==index && r44_stopped());
        run_cmd("g"); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
    }
    puts("44 fixed READY rejects raw distance/speed/angle/PID/FF/arm edits;QR_WAIT a/0 closesRX so lateACK+legalQR cannot restart passed");
    return 0;
}

/* These stopped-stage snapshots test the production dynamic dispatcher. They
 * do not claim an additional measured or end-to-end route. */
static int check_r44_dynamic_tail_dispatch(void)
{
    static const uint16_t corners[]={ROUTE31_RED_TO_CORNER_MM,ROUTE31_GREEN_TO_CORNER_MM,ROUTE31_BLUE_TO_CORNER_MM};
    static const uint16_t exits[]={ROUTE31_HOSTAGE_RANK1_MM,ROUTE31_HOSTAGE_RANK2_MM,ROUTE31_HOSTAGE_RANK3_MM};
    for(unsigned k=0;k<3u;++k) {
        CHECK(r44_boot()==0);
        s_seq_qr[0]=1; s_seq_qr[1]=(int32_t)k+1; s_seq_qr[2]=3;
        s_route44_index=18u; s_seq_stage=ROUTE31_TARGET_CORNER_STAGE;
        route_seq_prepare(); CHECK(route_seq_leg()->distance_mm==corners[k]);
        CHECK(r44_start_road()==0 && s_msel==15 && s_dist_target==(float)corners[k] &&
              s_v==50.0f && !s_dist_ff_ratio && s_dist_heading_kp>0.0f && s_dist_align_enabled);
        run_cmd("0"); CHECK(r44_stopped());
        CHECK(r44_boot()==0);
        s_route44_index=21u; s_seq_stage=ROUTE31_HOSTAGE_EXIT_STAGE;
        s_route31_hostage_rank=(uint8_t)k+1u;
        route_seq_prepare(); CHECK(route_seq_leg()->distance_mm==exits[k]);
        CHECK(r44_start_road()==0 && s_msel==15 && s_dist_target==(float)exits[k] &&
              s_v==50.0f && !s_dist_ff_ratio && s_dist_heading_kp>0.0f && s_dist_align_enabled);
        run_cmd("0"); CHECK(r44_stopped());
    }
    puts("44 realstopped-stage dynamiccolor1/2/3 corner andrank1/2/3 exit dispatch preserve31 distances with50/FF0/yaw/endfix passed");
    return 0;
}

static int check_r44_scaled_motion_outputs(void)
{
    float plain[4],scaled[4],ordinary[4];
    MotionProfileTune profile_before,profile_after;
    r44_real_motion_profile_get(&profile_before);
    r44_real_motion_vel_set(-300.0f,0.0f,0.0f);
    memcpy(ordinary,r44_motion_rpm,sizeof ordinary);
    r44_real_motion_vel_set_precise(-300.0f,0.0f,0.0f);
    memcpy(plain,r44_motion_rpm,sizeof plain);
    unsigned writes=r44_motion_precise_writes,masked=r44_motion_masked_writes;
    unsigned enters=r44_motion_irq_enters,restores=r44_motion_irq_restores;
    r44_real_motion_vel_set_left_scaled(-300.0f,0.0f,0.0f,ROUTE44_CROSS_LEFT_SCALE);
    memcpy(scaled,r44_motion_rpm,sizeof scaled);
    CHECK(ROUTE44_CROSS_LEFT_SCALE==1.10f && plain[0]<0.0f && plain[3]<0.0f);
    CHECK(r44_motion_precise_writes==writes+4u);
    CHECK(r44_motion_masked_writes==masked+4u && r44_motion_irq_enters==enters+1u &&
          r44_motion_irq_restores==restores+1u && !r44_motion_irq_mask);
    CHECK(fabsf(scaled[0]-plain[0]*1.10f)<0.0001f && fabsf(scaled[3]-plain[3]*1.10f)<0.0001f &&
          fabsf(scaled[0])>fabsf(plain[0]) && fabsf(scaled[3])>fabsf(plain[3]));
    CHECK(scaled[1]==plain[1] && scaled[2]==plain[2]);
    r44_real_motion_vel_set(-300.0f,0.0f,0.0f);
    CHECK(!memcmp(ordinary,r44_motion_rpm,sizeof ordinary));
    r44_real_motion_profile_get(&profile_after);
    CHECK(!memcmp(&profile_before,&profile_after,sizeof profile_before));
    /* Verify scale application is downstream of real IK even for a combined
     * body command, and restoring PRIMASK does not unmask an existing caller. */
    r44_real_motion_ik_precise(750.0f,11.0f,-0.75f,plain);
    r44_motion_irq_mask=1u;
    r44_real_motion_vel_set_left_scaled(750.0f,11.0f,-0.75f,1.0f);
    CHECK(!memcmp(plain,r44_motion_rpm,sizeof plain) && r44_motion_irq_mask==1u);
    r44_real_motion_vel_set_left_scaled(750.0f,11.0f,-0.75f,1.10f);
    CHECK(fabsf(r44_motion_rpm[0]-plain[0]*1.10f)<0.0001f &&
          fabsf(r44_motion_rpm[3]-plain[3]*1.10f)<0.0001f &&
          r44_motion_rpm[1]==plain[1] && r44_motion_rpm[2]==plain[2] && r44_motion_irq_mask==1u);
    r44_motion_irq_mask=0u;
    r44_real_motion_brake();
    CHECK(!r44_motion_rpm[0] && !r44_motion_rpm[1] && !r44_motion_rpm[2] && !r44_motion_rpm[3]);
    static const float invalid[][4]={{NAN,0,0,1.10f},{0,INFINITY,0,1.10f},{0,0,NAN,1.10f},
        {-300,0,0,0},{-300,0,0,-1},{-300,0,0,NAN},{-300,0,0,INFINITY}};
    for(unsigned k=0;k<sizeof invalid/sizeof invalid[0];++k) {
        unsigned stops=r44_motion_stops,precise=r44_motion_precise_writes;
        r44_real_motion_vel_set_left_scaled(invalid[k][0],invalid[k][1],invalid[k][2],invalid[k][3]);
        CHECK(r44_motion_stops==stops+1u && r44_motion_precise_writes==precise &&
              !r44_motion_rpm[0] && !r44_motion_rpm[1] && !r44_motion_rpm[2] && !r44_motion_rpm[3]);
    }
    puts("44 newopt-in actualmotionIK atomicpublication scalesonlym0LR+m3LF by1.10,rightm1/m2 unchanged;PRIMASKrestored,nextnormalAPI unscaled/profileunchanged/brakezero passed");
    return 0;
}

static int check_r44_cross_scaled_cancel(void)
{
    for(unsigned k=0;k<3u;++k) {
        const char *keys[]={"g","a","0"};
        CHECK(r44_boot()==0); run_cmd("g");
        for(unsigned j=0;j<20u && s_route44_index<4u;++j) {
            if(s_seq_state==SQ_QR_WAIT) {
                wire_sync(); wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
            } else {
                if(s_seq_state!=SQ_RUN) CHECK(r44_start_road()==0);
                CHECK(r44_finish_road()==0);
            }
        }
        CHECK(s_route44_index==4u);
        if(s_seq_state!=SQ_RUN) CHECK(r44_start_road()==0);
        CHECK(r44_service(20u)==0 && r44_scaled_calls>0u && !r44_scaled_contract_errors &&
              r44_scaled_value==1.10f && last_x==-300.0f && !last_y && !last_w);
        unsigned calls=r44_scaled_calls;
        run_cmd(keys[k]); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
        CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED && r44_stopped() &&
              r44_scaled_calls==calls && !r44_scaled_contract_errors);
    }
    puts("44 leftscaledcross g/a/0 immediatelystop;laterpolls do notpublishscaledmotion passed");
    return 0;
}

/* Move the live host sensors while waiting, then query the actual BLE reports.
 * IRQ/raw sensor values must stay live: it is APPLICATION records, not the
 * hardware measurements, that are frozen and later excluded from the leg. */
static int r44_check_frozen_observation(unsigned completed,unsigned next)
{
    unsigned char hold[sizeof s_route44_hold];
    char frozen_report[1024];
    unsigned state=s_seq_state,stage=s_seq_stage,index=s_route44_index;
    uint32_t test=s_active_test,test_seq=s_test_seq,meas=s_meas_t0,node_t0=s_route44_node_t0;
    int pulses=pulse_calls,servos=servo_calls;
    CHECK(s_route44_frozen && s_route44_hold.valid &&
          s_route44_hold.completed_index==completed && index==next &&
          (state==SQ_STEP_WAIT || state==SQ_QR_WAIT) && r44_stopped());
    memcpy(hold,&s_route44_hold,sizeof hold);
    host_messages[0]='\0'; route44_hold_report();
    CHECK(strlen(host_messages)<sizeof frozen_report && strstr(host_messages,"R44 FROZEN") &&
          strstr(host_messages,"manual_motion_excluded=1 sensors_live=1"));
    memcpy(frozen_report,host_messages,strlen(host_messages)+1u);
    host_fore+=123.0f; host_lateral-=45.0f;
    host_absolute_yaw+=17.0f; host_yaw+=17.0f; host_pitch+=1.0f; host_roll-=0.7f;
    for(unsigned m=0;m<4u;++m) host_counts[m]+=101+(int32_t)m;
    CHECK(r44_service(3000u)==0);
    static const char *const reports[]={"route","param","vision","diag","imu"};
    for(unsigned k=0;k<sizeof reports/sizeof reports[0];++k) {
        run_cmd(reports[k]);
        CHECK(!memcmp(hold,&s_route44_hold,sizeof hold) && s_route44_frozen &&
              s_seq_state==state && s_seq_stage==stage && s_route44_index==index &&
              s_active_test==test && s_test_seq==test_seq && s_meas_t0==meas &&
              s_route44_node_t0==node_t0 &&
              pulse_calls==pulses && servo_calls==servos && r44_stopped());
    }
    host_messages[0]='\0'; route44_hold_report();
    CHECK(!strcmp(frozen_report,host_messages));
    CHECK(host_fore!=s_route44_hold.fore && host_absolute_yaw!=s_route44_hold.absolute &&
          host_counts[0]!=s_route44_hold.c[0]);
    return 0;
}

static int check_r44_road_qr_cross_freeze_gates(void)
{
    CHECK(r44_boot()==0); run_cmd("g"); CHECK(r44_start_road()==0);
    for(unsigned m=0;m<4u;++m) host_counts[m]=(int32_t)(1000u+m);
    CHECK(r44_finish_road()==0 && s_seq_state==SQ_STEP_WAIT && s_route44_index==1u);
    CHECK(s_route44_hold.fore==600.0f && s_route44_hold.yaw==0.0f &&
          s_route44_hold.test==s_active_test && s_route44_hold.run_ms>0u &&
          s_route44_hold.c[0]==1000 && s_route44_hold.c[3]==1003);
    CHECK(r44_check_frozen_observation(0u,1u)==0);
    unsigned char first_hold[sizeof s_route44_hold];
    memcpy(first_hold,&s_route44_hold,sizeof first_hold);
    uint32_t first_end=s_route44_hold.end_ms;
    run_cmd("g"); CHECK(!s_route44_frozen && s_seq_state==SQ_STILL &&
          !host_fore && !host_lateral && !host_yaw && !host_counts[0] &&
          s_route44_node_t0==host_tick && s_route44_node_t0>first_end &&
          !memcmp(first_hold,&s_route44_hold,sizeof first_hold));
    CHECK(r44_start_road()==0 && s_msel==20 && turn_target_deg()==90.0f);
    CHECK(r44_finish_road()==0 && s_seq_state==SQ_QR_WAIT && s_route44_index==1u);
    CHECK(r44_check_frozen_observation(1u,1u)==0);
    wire_sync(); uint16_t request=wire_request;
    run_cmd("g"); CHECK(s_seq_state==SQ_QR_WAIT && s_route44_frozen && r44_stopped());
    wire_ack(request,1u,0u); xy_qr(request,"123");
    CHECK(s_seq_state==SQ_STEP_WAIT && s_route44_index==2u && s_route44_frozen && r44_stopped());
    CHECK(r44_service(3000u)==0 && s_seq_state==SQ_STEP_WAIT && r44_stopped());
    run_cmd("g"); CHECK(!s_route44_frozen && !host_fore && !host_lateral && !host_yaw &&
          !host_counts[0] && s_seq_state==SQ_STILL);
    CHECK(r44_start_road()==0 && s_msel==15 && s_dist_target==600.0f);
    CHECK(r44_finish_road()==0 && r44_start_road()==0);
    CHECK(r44_finish_road()==0 && r44_start_road()==0 && s_route44_index==4u);
    host_yaw=s_dist_heading0+7.0f; host_absolute_yaw+=7.0f;
    CHECK(r44_finish_road()==0 && s_route44_index==5u && s_seq_state==SQ_CROSS_YAW &&
          !s_route44_frozen && !s_board_contact.zeroed);
    for(unsigned k=0;k<180u && s_seq_state!=SQ_STEP_WAIT;++k) {
        CHECK(s_seq_state==SQ_CROSS_YAW && !s_board_contact.zeroed);
        float delta=s_route31_task_yaw_goal-host_yaw;
        host_yaw+=delta; host_absolute_yaw+=delta;
        CHECK(r44_service(20u)==0);
    }
    CHECK(s_seq_state==SQ_STEP_WAIT && s_route31_cross_yaw_done && !s_board_contact.zeroed);
    CHECK(r44_check_frozen_observation(4u,5u)==0);
    CHECK(r44_start_road()==0 && s_board_contact.active && s_board_contact.zeroed &&
          s_board_contact.pitch0==host_pitch && s_board_contact.roll0==host_roll &&
          s_v==40.0f && !s_route44_frozen);
    run_cmd("0"); CHECK(r44_stopped());
    puts("44 ordinary/QR/postcrossyaw gates freezeexactreports whilemanualsensorslive;QRg notqueued,newg resetsencoder/localyaw andcontactzeroonlyafterg passed");
    return 0;
}

static int check_r44_after_clear_turn_before_forward_gate(void)
{
    CHECK(r44_boot()==0); run_cmd("g");
    for(unsigned k=0;k<80u && s_route44_index<6u;++k) {
        if(s_seq_state==SQ_QR_WAIT) {
            wire_sync(); wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
        } else {
            if(s_seq_state!=SQ_RUN) CHECK(r44_start_road()==0);
            CHECK(r44_finish_road()==0);
        }
    }
    CHECK(s_route44_index==6u && s_seq_stage==5u && r44_start_road()==0);
    CHECK(s_msel==16 && s_dist_target==-190.0f && s_v==50.0f);
    CHECK(r44_finish_road()==0 && s_seq_state==SQ_STEP_WAIT &&
          s_route44_index==7u && s_seq_stage==ROUTE44_ROAD_ROLE &&
          route_seq_leg()->mode==20u && s_route44_hold.completed_index==6u &&
          s_route44_frozen && r44_stopped());
    CHECK(r44_check_frozen_observation(6u,7u)==0);
    run_cmd("g"); CHECK(!s_route44_frozen && s_seq_state==SQ_STILL);
    CHECK(r44_start_road()==0 && s_route44_index==7u &&
          s_msel==20 && turn_target_deg()==90.0f && s_v==50.0f);
    CHECK(r44_service(20u)==0 && !last_x && !last_y && last_w>0.0f);
    CHECK(r44_finish_road()==0 && s_seq_state==SQ_STEP_WAIT &&
          s_route44_index==8u && s_seq_stage==6u &&
          route_seq_leg()->mode==15u && route_seq_leg()->distance_mm==780u &&
          s_route44_hold.completed_index==7u && s_route44_frozen && r44_stopped());
    CHECK(r44_check_frozen_observation(7u,8u)==0);
    run_cmd("g"); CHECK(!s_route44_frozen && s_seq_state==SQ_STILL);
    CHECK(r44_start_road()==0 && s_route44_index==8u && s_seq_stage==6u &&
          s_msel==15 && s_dist_target==780.0f && s_v==50.0f &&
          !s_dist_ff_ratio && s_dist_align_enabled);
    CHECK(r44_service(20u)==0 && last_x>0.0f && !last_y && !last_w);
    run_cmd("0"); CHECK(r44_stopped());
    puts("44 BACKclear190->WAITg->independentRIGHT90->WAITg->FWD780:manual/live/query polling cannot skip either insertedturn gate,precise90 plusnextforward50 verified passed");
    return 0;
}

static int check_r44_entry_drain_manual_resume(void)
{
    CHECK(r44_to_ball_approach()==0 && r44_trigger_parallel_rack()==0);
    uint16_t request=wire_request;
    r44_rank(4u,2u);
    unsigned starts=host_timer_start_calls;
    CHECK(r44_finish_road()==0);
    CHECK(s_seq_state==SQ_STEP_DRAIN && s_route44_index==15u &&
          !s_route44_frozen && (s_route31_lift_phase==R44_RACK_EXTEND ||
                              s_route31_lift_phase==R44_RACK_EXTEND_WAIT) &&
          !last_x && !last_y && !last_w &&
          !host_jog_pulses[1][1] && !s_route_action_taken);
    uint32_t road_end=s_route44_hold.end_ms;
    if(s_route31_lift_phase==R44_RACK_EXTEND_WAIT) {
        uint32_t elapsed=host_tick-s_route31_lift_wait_t0;
        CHECK(elapsed<ROUTE31_RACK_SETTLE_MS && !host_timer_active &&
              host_jog_pulses[0][0]==ROUTE31_RACK_EXTEND_STEPS);
        CHECK(r44_service(ROUTE31_RACK_SETTLE_MS-elapsed-1u)==0 &&
              s_seq_state==SQ_STEP_DRAIN && !s_route44_frozen && host_timer_start_calls==starts);
    }
    CHECK(r44_service(4000u)==0 && s_seq_state==SQ_STEP_WAIT && s_route44_frozen &&
          s_route31_rack_deployed && !host_timer_active && host_timer_start_calls==starts &&
          host_jog_pulses[0][0]==ROUTE31_RACK_EXTEND_STEPS && !host_jog_pulses[1][1] &&
          s_route44_hold.completed_index==14u && s_route44_hold.fore==200.0f &&
          s_route44_hold.end_ms>road_end &&
          s_route44_hold.run_ms==s_route44_hold.end_ms-s_route44_node_t0);
    CHECK(r44_check_frozen_observation(14u,15u)==0);
    float old_goal=s_vat.yaw_target,manual_delta=host_absolute_yaw-s_route44_hold.absolute;
    unsigned char frozen_sample[sizeof s_sample];
    memcpy(frozen_sample,(const void *)&s_sample,sizeof frozen_sample);
    wire_ack(request,2u,0u);
    xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
    unsigned frozen_sequence=xy_seq-1u;
    CHECK(r44_service(1000u)==0 && s_seq_state==SQ_STEP_WAIT && s_route44_frozen &&
          !memcmp(frozen_sample,(const void *)&s_sample,sizeof frozen_sample) &&
          !s_route44_ball_offset && !s_route_action_taken && !host_jog_pulses[1][1]);
    ProtoTargetRank rank;
    CHECK(proto_target_rank_get(&rank) && rank.rank==2u && wire_request==request);
    run_cmd("g");
    CHECK(s_seq_state==SQ_TASK && !s_route44_frozen && wire_request==request &&
          fabsf(s_vat.yaw_target-(old_goal+manual_delta))<0.0001f &&
          !host_fore && !host_lateral && !host_yaw && !host_counts[0] &&
          !s_vat.ball_seen && !s_vat.ball_fallback && !s_vat.cx && !s_vat.cy &&
          !s_route_action_taken && !s_route44_ball_offset && proto_target_rank_get(&rank) && rank.rank==2u);
    unsigned next_sequence=xy_seq;
    xy_seq=frozen_sequence;
    xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
    CHECK(r44_service(1500u)==0 && !s_route44_ball_offset && !s_route_action_taken &&
          !s_vat.ball_seen && !host_jog_pulses[1][1]);
    xy_seq=next_sequence;
    for(unsigned k=0;k<160u && !s_route44_ball_offset;++k) {
        xy_obj_poll(request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
        CHECK(r44_service(20u)==0 && s_seq_state!=SQ_STOPPED);
    }
    CHECK(s_route44_ball_offset && !host_jog_pulses[1][1] && wire_request==request);
    run_cmd("0"); CHECK(r44_stopped());
    for(unsigned k=0;k<3u;++k) {
        const char *keys[]={"g","a","0"};
        CHECK(r44_to_ball_approach()==0 && r44_trigger_parallel_rack()==0 && r44_finish_road()==0 &&
              s_seq_state==SQ_STEP_DRAIN);
        int pulses=pulse_calls;
        run_cmd(keys[k]); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
        CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED && pulse_calls==pulses && r44_stopped());
    }
    puts("44 FWD200 drain completesfiniteSTEP+settle beforefrozenWAIT;manualheading rebases sameBALLrequest/rank whilewait/replayedpixels cannotact;NEWpixels reauthorize;drain g/a/0 stopsboth passed");
    return 0;
}

static int check_r44_complete_physical_plan(void)
{
    static const uint8_t want_role[22]={0,16,1,2,3,4,5,16,6,16,16,16,7,16,16,9,10,11,12,13,14,15};
    static const uint8_t want_mode[22]={15,20,15,20,16,15,16,20,15,20,16,20,16,20,15,41,22,35,15,20,40,15};
    static const uint16_t want_mm[22]={600,0,600,0,620,0,190,0,780,0,800,0,780,0,200,0,0,0,0,0,0,0};
    CHECK(r44_boot()==0);
    for(unsigned i=0;i<22u;++i) {
        CHECK(s_route44_plan[i].role==want_role[i] && s_route44_plan[i].leg.mode==want_mode[i] &&
              s_route44_plan[i].leg.distance_mm==want_mm[i]);
    }
    run_cmd("g");
    unsigned visits=0u, last_index=255u, preball_left=0u, postgrab_right=0u;
    unsigned hostage_left=0u, qr_sent=0u, gated_steps=0u, drain_seen=0u;
    unsigned gates_by_node[22]={0};
    unsigned pair_turns=0u, release_return_turns=0u, road_tests=0u, road_projection=0u;
    uint32_t entered_test=0u;
    uint16_t ranked_request=0u, ball_request=0u, bucket_request=0u;
    int contact_zero_checked=0, saw_parallel_rack=0, saw_laser=0;
    for(unsigned tick=0u;tick<20000u && s_seq_state!=SQ_DONE;++tick) {
        CHECK(s_seq_state!=SQ_STOPPED);
        unsigned index=s_route44_index;
        unsigned scaled_before=r44_scaled_calls;
        CHECK(index<22u && s_seq_stage==want_role[index]);
        if(index!=last_index) {
            CHECK(last_index==255u || index==last_index+1u);
            last_index=index; ++visits;
            CHECK(route_seq_leg()->mode==want_mode[index] &&
                  route_seq_speed_mms()==(index==4u?300.0f:index==5u?40.0f:50.0f));
        }
        if(s_seq_state==SQ_STEP_WAIT) {
            CHECK(s_route44_frozen && s_route44_hold.valid &&
                  s_route44_hold.completed_index+1u==index && r44_stopped());
            ++gated_steps; ++gates_by_node[index];
            if(gates_by_node[index]>1u)
                fprintf(stderr,"44 repeatedWAIT nextphysical=%u completed=%u priorTEST=%lu rack=%u\n",index,
                        (unsigned)s_route44_hold.completed_index,(unsigned long)s_active_test,
                        (unsigned)s_route31_lift_phase);
            run_cmd("g"); CHECK(!s_route44_frozen);
        }
        if(s_seq_state==SQ_STEP_DRAIN) {
            CHECK(index==15u && !s_route44_frozen && !last_x && !last_y && !last_w &&
                  !laser_state && (s_route31_lift_phase==R44_RACK_EXTEND ||
                                  s_route31_lift_phase==R44_RACK_EXTEND_WAIT));
            ++drain_seen;
        }
        if(index==ROUTE44_BALL_ENTRY_INDEX) {
            wire_sync(); CHECK(wire_task==1u && s_route44_ball_early);
            if(s_receiving && !s_ack) wire_ack(wire_request,2u,0u);
            if(!ball_request) ball_request=wire_request;
            CHECK(ball_request==wire_request);
            if(ranked_request!=wire_request) { ranked_request=wire_request; r44_rank(4u,2u); }
            if(s_receiving && s_seq_state==SQ_RUN)
                xy_obj_poll(wire_request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
            CHECK(!host_jog_pulses[1][1] && !s_route44_ball_offset && !s_route_action_taken);
        }
        if(s_seq_state==SQ_QR_WAIT) {
            CHECK(index==1u && r44_stopped() && !qr_sent);
            wire_sync(); wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
            ++qr_sent;
        } else if(s_seq_state==SQ_CROSS_YAW) {
            float correction=s_route31_task_yaw_goal-host_yaw;
            host_absolute_yaw+=correction; host_yaw=s_route31_task_yaw_goal;
        } else if(s_seq_state==SQ_RUN) {
            if(s_active_test!=entered_test) {
                entered_test=s_active_test; ++road_tests;
                if(turn_closed_loop_mode()) {
                    float target=turn_target_deg();
                    CHECK(turn_speed_scale(90.0f)==1.0f && turn_velocity(90.0f)==T_TURN_MAX_W);
                    CHECK(fabsf(turn_velocity(0.5f)-T_TURN_MIN_W)<0.00001f);
                    CHECK(s_v==50.0f && !last_x && !last_y);
                    if(index==15u) {
                        CHECK(target==180.0f && preball_left==1u && postgrab_right==1u &&
                              wire_request==ball_request && host_jog_pulses[1][1]==ROUTE31_LIFT_DOWN_STEPS);
                        ++pair_turns;
                    } else if(index==16u) {
                        CHECK(target==180.0f && bucket_request &&
                              host_jog_pulses[0][1]==ROUTE31_RACK_EXTEND_STEPS);
                        ++release_return_turns;
                    } else CHECK(target==(index==19u?ROUTE31_HOSTAGE_RIGHT_TARGET_DEG:90.0f));
                    host_absolute_yaw+=target-host_yaw; host_yaw=target;
                } else {
                    CHECK(dist_mode() && s_dist_ff_ratio==0.0f);
                    if(!s_board_contact.active) {
                        CHECK(s_dist_heading_kp>0.0f || index==4u);
                        CHECK(s_dist_align_enabled==dist_ordinary_leg() && !s_dist_align_on);
                        if(index==15u) {
                            CHECK(s_v==50.0f && s_route31_rack_deployed);
                            if(s_msel==17) {
                                CHECK(s_dist_target==-30.0f && !host_jog_pulses[1][1] &&
                                      s_route44_ball_offset && !pair_turns); ++preball_left;
                            } else {
                                CHECK(s_msel==18 && s_dist_target==30.0f && s_route44_grab_return &&
                                      host_jog_pulses[1][1]==ROUTE31_LIFT_DOWN_STEPS &&
                                      host_jog_pulses[1][0]==ROUTE31_LIFT_DOWN_STEPS && !pair_turns);
                                ++postgrab_right;
                            }
                        }
                        if(index==20u && s_msel==17) { CHECK(s_dist_target==-(float)ROUTE31_HOSTAGE_PREGRAB_LEFT_MM); ++hostage_left; }
                        /* Observe the actual motion command before supplying an
                         * endpoint: FF0 removes orthogonal translation, while
                         * an injected small yaw deviation still drives hold. */
                        host_yaw=s_dist_heading0+0.2f;
                        CHECK(r44_service(20u)==0 && s_seq_state==SQ_RUN && s_round==R_RUN);
                        CHECK(dist_lateral() ? last_x==0.0f && last_y*s_dist_target>0.0f : last_y==0.0f && last_x*s_dist_target>0.0f);
                        if(s_dist_heading_kp>0.0f) CHECK(last_w<0.0f);
                        if(index==ROUTE44_BALL_ENTRY_INDEX && host_timer_active && s_route44_rack_started)
                            saw_parallel_rack=1;
                        host_yaw=s_dist_heading0; ++road_projection;
                        if(dist_lateral()) host_lateral=s_dist_odo0+s_dist_target;
                        else host_fore=s_dist_odo0+s_dist_target;
                    } else {
                        CHECK(index==5u && s_v==40.0f && s_dist_heading_kp==0.0f &&
                              route_contact_tilt_deg()==2.0f && route_contact_zero_wait_ms()==300u);
                        contact_zero_checked=1;
                    }
                }
            }
            if(s_board_contact.active) host_pitch=s_board_contact.pitch0+2.1f;
            if(s_round==R_ALIGN) host_yaw=s_dist_heading0;
        } else if(s_seq_state==SQ_TASK) {
            wire_sync();
            if(s_receiving && !s_ack) wire_ack(wire_request,2u,0u);
            if(index==17u) {
                CHECK(s_target35_route && target35_search_ff()==0.0f && s_target35_kp>0.0f);
                if(laser_state) saw_laser=1;
                if(s_receiving) xy_obj_poll(wire_request,8,ROUTE31_TARGET_CX,100,0);
            } else {
                CHECK(index==15u || index==20u);
                CHECK(s_route_search_ff==0.0f);
                if(s_route_owned) CHECK(s_route_search_kp>0.0f && s_route_search_speed==50.0f);
                else CHECK(index==20u && s_vat.state==VAT_DONE && s_route31_hostage_hold);
                if(wire_task==1u) {
                    CHECK(index==15u && wire_request==ball_request);
                    if(ranked_request!=wire_request) { ranked_request=wire_request; r44_rank(4u,2u); }
                    if(s_receiving) xy_obj_poll(wire_request,4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX,0);
                } else if(wire_task==4u) {
                    CHECK(index==15u && pair_turns==1u && wire_request!=ball_request &&
                          !s_seq_pair_offset && !s_route44_grab_return && !s_route44_ball_offset);
                    bucket_request=wire_request;
                    if(s_receiving) xy_obj_poll(wire_request,9,VAT_ROUTE_BUCKET_X_PX,VAT_BUCKET_Y_PX,0);
                } else {
                    CHECK(wire_task==3u);
                    if(ranked_request!=wire_request) { ranked_request=wire_request; r44_rank(0u,1u); }
                    if(s_receiving) xy_obj_poll(wire_request,0,VAT_ROUTE_HOSTAGE_X_PX,VAT_HOSTAGE_Y_PX,0);
                }
            }
        }
        CHECK(r44_service(20u)==0);
        CHECK(!r44_scaled_contract_errors && (index==4u || r44_scaled_calls==scaled_before));
    }
    if(s_seq_state!=SQ_DONE || visits!=22u || last_index!=21u || qr_sent!=1u || gated_steps!=21u)
        fprintf(stderr,"44 chain state=%u visits=%u index=%u QR=%u gates=%u drain=%u\n",
                (unsigned)s_seq_state,visits,last_index,qr_sent,gated_steps,drain_seen);
    CHECK(s_seq_state==SQ_DONE && visits==22u && last_index==21u && qr_sent==1u && gated_steps==21u);
    for(unsigned i=1u;i<22u;++i) CHECK(gates_by_node[i]==1u);
    CHECK(drain_seen>0u);
    CHECK(r44_scaled_calls>0u && r44_scaled_value==1.10f && !r44_scaled_contract_errors);
    CHECK(preball_left==1u && postgrab_right==1u && hostage_left==1u);
    CHECK(pair_turns==1u && release_return_turns==1u && road_tests>=20u &&
          road_projection>=11u && contact_zero_checked && saw_parallel_rack && saw_laser);
    CHECK(host_jog_pulses[0][0]==2u*ROUTE31_RACK_EXTEND_STEPS &&
          host_jog_pulses[0][1]==ROUTE31_RACK_EXTEND_STEPS+ROUTE31_HOSTAGE_RACK_RETRACT_STEPS);
    CHECK(host_jog_pulses[1][0]==ROUTE31_LIFT_DOWN_STEPS+ROUTE31_LIFT_BUCKET_DOWN_STEPS &&
          host_jog_pulses[1][1]==ROUTE31_LIFT_DOWN_STEPS+ROUTE31_LIFT_BUCKET_DOWN_STEPS);
    CHECK(host_servo==ROUTE31_HOSTAGE_GRIP_US && s_route31_hostage_rank==1u &&
          s_dist_target==(float)ROUTE31_HOSTAGE_RANK1_MM && r44_stopped());
    CHECK(s_route31_straight_v==219.0f && s_route31_lateral_v==321.0f &&
          s_route43_tune.straight_v==125.0f && s_route43_tune.lateral_v==135.0f);
    puts("44 full22physical/31logical gatedchain:21newg gates,QR-after-firstturn,zeroFF50/cross300/contact40,afterclearRIGHT90_beforeFWD780,20-profileturns,earlyBALL_parallelrack_drain,continuousLEFT30_grab_RIGHT30_180_NEWbucket_release_retract,second180,54 ranks,laser2s,hostage offset,actualarm counts,finalDONE passed");
    return 0;
}

#ifndef ROUTE44_CURRENT_FIXTURE_MAIN
#define ROUTE44_CURRENT_FIXTURE_MAIN main
#endif
int ROUTE44_CURRENT_FIXTURE_MAIN(void)
{
    CHECK(check_r44_first_turn_qr_gate()==0);
    CHECK(check_r44_parameters_isolation_cancel()==0);
    CHECK(check_r44_qr_wait_cancel_and_fixed_commands()==0);
    CHECK(check_r44_dynamic_tail_dispatch()==0);
    CHECK(check_r44_scaled_motion_outputs()==0);
    CHECK(check_r44_cross_scaled_cancel()==0);
    CHECK(check_r44_road_qr_cross_freeze_gates()==0);
    CHECK(check_r44_after_clear_turn_before_forward_gate()==0);
    CHECK(check_r44_entry_drain_manual_resume()==0);
    CHECK(check_r44_parallel_rack_freshness_and_cancel()==0);
    CHECK(check_r44_ack_pending_and_new_after_entry_loss()==0);
    CHECK(check_r44_complete_physical_plan()==0);
    puts("route44_current_test: actual software runner replay only, no hardware acceptance");
    return 0;
}
