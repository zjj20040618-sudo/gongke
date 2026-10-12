/* Current43 stepped31 replay. Real BLE/VAT/CRC/STEP state machines; injected
 * pixels, yaw, encoder endpoints and IRQs are NOT physical acceptance. */
#define VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN current43_prior_xy_main
#include "vision_align_bluetooth_test.c"
#undef VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN

static int c43_stopped(void)
{ return !last_x && !last_y && !last_w && !laser_state && !host_timer_active && !s_go; }

static int c43_service(unsigned ms)
{
    while (ms) {
        unsigned slice=ms<20u?ms:20u;
        host_timer_sync_hal();
        uint64_t until=host_timer_us+(uint64_t)slice*1000u;
        while (host_timer_active && host_timer_next_us<=until)
            CHECK(jog_host_timer_event()==0);
        host_timer_us=until; host_tick=(uint32_t)(until/1000u);
        wire_poll(); ms-=slice;
    }
    return 0;
}

static int c43_emit(unsigned count)
{
    unsigned budget=count*2u+T_JOG_MAX_PPS/500u+4u;
    int want=pulse_calls+(int)count;
    CHECK(host_timer_active);
    while (pulse_calls<want && budget--) CHECK(jog_host_timer_event()==0);
    CHECK(pulse_calls==want && !last_x && !last_y && !last_w);
    return 0;
}

static int c43_boot(int early_qr)
{
    CHECK(wire_boot()==0); host_wire_diag_hook=real_proto_wire_diag_get; xy_seq=1u;
    run_cmd("31"); run_cmd("v219"); run_cmd("pv321");
    run_cmd("43"); wire_sync();
    CHECK(s_seq_mode==43u && s_seq_state==SQ_READY && c43_stopped());
    if (early_qr) { wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123"); }
    return 0;
}

static int c43_no_write(const char *key)
{
    RouteStepTune before=s_route43_tune;
    unsigned stage=s_seq_stage,state=s_seq_state,unit=s_route43_unit;
    int pulses=pulse_calls,servos=servo_calls;
    uint16_t u=host_servo;
    run_cmd(key);
    CHECK(!memcmp(&before,&s_route43_tune,sizeof before) && stage==s_seq_stage &&
          state==s_seq_state && unit==s_route43_unit && pulses==pulse_calls &&
          servos==servo_calls && u==host_servo && !laser_state);
    return 0;
}

/* Deliberately selected stopped slots test parser scope, not a physical route. */
static void c43_slot(unsigned stage, uint8_t pending)
{
    s_seq_stage=(uint8_t)stage; s_seq_state=SQ_STEP_WAIT; s_route43_pending=pending;
    s_seq_pair_turn=s_seq_pair_offset=s_seq_return_offset=s_seq_hostage_offset=0u;
    s_msel=43; s_round=R_READY; motion_brake(); bp_laser_set(0);
    s_seq_qr[0]=1; s_seq_qr[1]=2; s_seq_qr[2]=3;
}

static int c43_wait(void)
{
    CHECK(s_seq_state==SQ_STEP_WAIT && c43_stopped());
    unsigned stage=s_seq_stage,pending=s_route43_pending,unit=s_route43_unit;
    int pulses=pulse_calls,servos=servo_calls;
    uint16_t held=host_servo,request=wire_request;
    if (s_receiving) wire_ack(request,wire_mode,0u);
    for (unsigned k=0;k<5u;++k) { host_tick+=20u; xy_obj(request,9,105,VAT_BUCKET_Y_PX); }
    CHECK(c43_service(5000u)==0);
    run_cmd("?"); run_cmd("diag"); run_cmd("param"); run_cmd("route");
    CHECK(s_seq_state==SQ_STEP_WAIT && s_seq_stage==stage && s_route43_pending==pending &&
          s_route43_unit==unit && pulse_calls==pulses && servo_calls==servos &&
          host_servo==held && wire_request==request && c43_stopped());
    return 0;
}

static int c43_utf8(const unsigned char *p)
{
    while (*p) {
        unsigned c=*p++,need=0,min=0,value=0;
        if (c<0x80u) continue;
        if (c>=0xc2u && c<=0xdfu) { need=1; min=0x80; value=c&31u; }
        else if (c>=0xe0u && c<=0xefu) { need=2; min=0x800; value=c&15u; }
        else if (c>=0xf0u && c<=0xf4u) { need=3; min=0x10000; value=c&7u; }
        else return 0;
        while (need--) { if ((*p&0xc0u)!=0x80u) return 0; value=(value<<6)|(*p++&63u); }
        if (value<min || value>0x10ffffu || (value>=0xd800u && value<=0xdfffu)) return 0;
    }
    return 1;
}

static int c43_report_check(void)
{
    host_messages[0]='\0'; route43_report("WAIT_G");
    CHECK(c43_utf8((const unsigned char*)host_messages));
    CHECK(strstr(host_messages,"next=") && strstr(host_messages,"MID1.5 END0.4 STABLE400"));
    CHECK(strstr(host_messages,"\xe4\xb8\x8b\xe4\xb8\x80\xe6\xad\xa5"));
    const char *p=host_messages;
    unsigned lines=0;
    while (*p) {
        const char *end=strstr(p,"\r\n"); CHECK(end && end-p<512);
        if ((unsigned char)*p>=0x80u) CHECK(end-p>=3 && !memcmp(end-3,"\xe3\x80\x82",3));
        ++lines; p=end+2;
    }
    CHECK(lines==3u);
    return 0;
}

static int check_current_defaults_and_ui(void)
{
    CHECK(c43_boot(1)==0);
    CHECK(s_route43_tune.straight_v==100.0f && s_route43_tune.lateral_v==100.0f);
    CHECK(s_route43_tune.road_mm[0]==545u && s_route43_tune.road_mm[1]==610u &&
          s_route43_tune.road_mm[3]==620u && s_route43_tune.road_mm[6]==780u &&
          s_route43_tune.road_mm[7]==780u);
    CHECK(s_route43_tune.turn_deg[2]==90.0f && s_route43_tune.turn_deg[8]==90.0f &&
          s_route43_tune.turn_deg[10]==ROUTE31_RETURN_TARGET_DEG &&
          s_route43_tune.turn_deg[13]==ROUTE31_HOSTAGE_RIGHT_TARGET_DEG &&
          s_route43_tune.pair_turn_deg==180.0f);
    CHECK(s_route43_tune.pair_left_mm==15u && !s_route43_tune.return_left_mm &&
          s_route43_tune.hostage_left_mm==30u && s_route43_tune.rack_steps==3400u &&
          s_route43_tune.hostage_rack_steps==3400u && s_route43_tune.rack_pps==5000u &&
          s_route43_tune.ball_down_steps==32000u && s_route43_tune.bucket_down_steps==20000u);
    CHECK(s_route43_tune.corner_mm[0]==ROUTE31_RED_TO_CORNER_MM &&
          s_route43_tune.corner_mm[1]==ROUTE31_GREEN_TO_CORNER_MM &&
          s_route43_tune.corner_mm[2]==ROUTE31_BLUE_TO_CORNER_MM);
    CHECK(s_route31_straight_v==219.0f && s_route31_lateral_v==321.0f);
    for (unsigned stage=0;stage<ROUTE31_STAGES;++stage) {
        c43_slot(stage,R43_PENDING_PREP); s_route31_rack_deployed=stage!=8u;
        CHECK(c43_report_check()==0);
        if (stage==3u) CHECK(route_seq_speed_mms()==300.0f);
        if (stage==4u) CHECK(route_seq_speed_mms()==40.0f);
    }
    c43_slot(9u,R43_PENDING_BUCKET); CHECK(c43_report_check()==0);
    c43_slot(14u,R43_PENDING_HOSTAGE); CHECK(c43_report_check()==0);
    s_seq_hostage_offset=1u; CHECK(c43_report_check()==0);
    s_seq_hostage_offset=0u; run_cmd("cn0");
    host_messages[0]='\0'; route43_report("WAIT_G");
    CHECK(!strstr(host_messages,"\xe4\xb8\x8b\xe4\xb8\x80\xe6\xad\xa5"));
    puts("43 defaults/current31 distance-speed-turn recipes; UTF8 next-action plus applicable knobs untruncated across16 stages/internal gates passed");
    return 0;
}

static int check_knobs_and_scope(void)
{
    CHECK(c43_boot(1)==0);
    float old_heading=s_route_heading_kp,old_left=s_route31_left_yaw_kp;
    float old_fff=s_route_forward_ff_ratio,old_xkp=s_route31_exit_yaw_kp;
    run_cmd("d600"); run_cmd("sv120"); run_cmd("v125"); run_cmd("pv135");
    run_cmd("ykp1.2"); run_cmd("lkp2.3"); run_cmd("xkp3.4");
    run_cmd("fff0.0125"); run_cmd("bff-0.02"); run_cmd("lff0.01"); run_cmd("rff-0.01");
    run_cmd("cff0.1"); run_cmd("eff0.08"); run_cmd("tv1.25"); run_cmd("tkp0.05");
    run_cmd("e7"); run_cmd("f1234");
    CHECK(s_route43_tune.road_mm[0]==600u && s_route43_tune.leg_v[0]==120.0f &&
          s_route43_tune.straight_v==125.0f && s_route43_tune.lateral_v==135.0f &&
          s_route43_tune.heading_kp==1.2f && s_route43_tune.left_yaw_kp==2.3f &&
          s_route43_tune.exit_yaw_kp==3.4f && s_route43_tune.forward_ff==0.0125f &&
          s_route43_tune.backward_ff==-0.02f && s_route43_tune.corner_ff==0.1f &&
          s_route43_tune.exit_ff==0.08f && s_route43_tune.turn_max_w==1.25f &&
          s_route43_tune.turn_kp==0.05f && s_route43_tune.rack_steps==7u && s_route43_tune.rack_pps==1234u);
    CHECK(s_route31_plan[0].distance_mm==545u && s_route31_straight_v==219.0f &&
          s_route31_lateral_v==321.0f && s_route_heading_kp==old_heading &&
          s_route31_left_yaw_kp==old_left && s_route_forward_ff_ratio==old_fff && s_route31_exit_yaw_kp==old_xkp);
    const char *const invalid[]={"d0","d20001","d1.5","dnan","sv0","sv601","v-1","pv601",
        "ykp5.1","lkp-1","xkpnan","tv0.29","tv4.1","tkp0","tkp0.51",
        "fff0.051","bff-0.051","lffnan","rff0.06","cff-0.01","eff0.151",
        "e0","e100001","f0","f20001","t90","h4","lf1234","u2000","cn2"};
    for(unsigned k=0;k<sizeof invalid/sizeof invalid[0];++k) CHECK(c43_no_write(invalid[k])==0);
    c43_slot(2u,R43_PENDING_PREP); run_cmd("t92.5");
    CHECK(s_route43_tune.turn_deg[2]==92.5f);
    CHECK(c43_no_write("t0")==0 && c43_no_write("t200.1")==0 && c43_no_write("t-90")==0 && c43_no_write("d100")==0);
    s_seq_state=SQ_RUN; s_msel=20; s_round=R_RUN;
    const char *const running[]={"d500","sv150","v200","pv250","t95","tv2","tkp0.1","ykp2","lkp2","xkp2","fff0","cff0","eff0","e9","f500","h9","lf500","u1900","cn0"};
    for(unsigned k=0;k<sizeof running/sizeof running[0];++k) CHECK(c43_no_write(running[k])==0);
    CHECK(turn_target_deg()==92.5f && fabsf(turn_velocity(92.5f)-1.25f)<0.0001f);
    c43_slot(9u,R43_PENDING_PREP); s_route31_rack_deployed=1u;
    run_cmd("h4"); run_cmd("lf1000"); run_cmd("u2200"); run_cmd("sff0.02");
    CHECK(s_route43_tune.ball_down_steps==4u && s_route43_tune.lift_pps==1000u &&
          s_route43_tune.ball_grip_us==2200u && s_route43_tune.ball_ff==0.02f);
    CHECK(c43_no_write("e8")==0 && c43_no_write("f1000")==0 && c43_no_write("u2501")==0 && c43_no_write("lf20001")==0);
    c43_slot(9u,R43_PENDING_BUCKET); run_cmd("h6"); run_cmd("lf1200");
    CHECK(s_route43_tune.bucket_down_steps==6u && s_route43_tune.lift_pps==1200u);
    CHECK(c43_no_write("u1900")==0);
    c43_slot(14u,R43_PENDING_HOSTAGE); s_route31_rack_deployed=0u;
    s_route43_rack_issued=7u; s_route43_rack_pps_issued=1234u;
    run_cmd("e3001"); run_cmd("f800"); run_cmd("u2300");
    CHECK(s_route43_tune.hostage_rack_steps==3001u && s_route43_tune.hostage_rack_pps==800u &&
          s_route43_tune.hostage_grip_us==2300u && s_route43_rack_issued==7u &&
          s_route43_rack_pps_issued==1234u && route31_rack_return_steps()==7u &&
          route31_rack_return_pps()==1234u && route31_hostage_rack_steps()==3001u);
    CHECK(c43_no_write("e2999")==0 && c43_no_write("h10")==0);
    CHECK(c43_wait()==0);
    run_cmd("0"); run_cmd("43");
    CHECK(s_route43_tune.road_mm[0]==600u && s_route43_tune.hostage_rack_steps==3001u);
    puts("43 d/sv/v/pv/t/tv/tkp/ykp/lkp/xkp/FF/e/f/h/lf/u scopes,ranges,RUN lock,private31 isolation,reselect retention,rack snapshot independence passed");
    return 0;
}

static int c43_start_road(void)
{
    run_cmd("g");
    for(unsigned k=0;k<150u && s_seq_state!=SQ_RUN;++k) {
        if(s_seq_state==SQ_CROSS_YAW) {
            float correction=s_route31_task_yaw_goal-host_yaw;
            host_absolute_yaw+=correction; host_yaw=s_route31_task_yaw_goal;
        }
        CHECK(c43_service(20u)==0);
        CHECK(s_seq_state!=SQ_STOPPED);
    }
    CHECK(s_seq_state==SQ_RUN && s_round==R_RUN);
    return 0;
}

static int c43_finish_road(void)
{
    CHECK(s_seq_state==SQ_RUN);
    if(turn_closed_loop_mode()) {
        host_absolute_yaw+=turn_target_deg()-host_yaw; host_yaw=turn_target_deg();
    } else if(s_board_contact.active) {
        host_pitch=s_board_contact.pitch0+route_contact_tilt_deg()+0.1f;
    } else if(dist_lateral()) host_lateral=s_dist_odo0+s_dist_target;
    else host_fore=s_dist_odo0+s_dist_target;
    for(unsigned k=0;k<150u && s_seq_state==SQ_RUN;++k) {
        if(s_round==R_ALIGN) host_yaw=s_dist_heading0;
        CHECK(c43_service(20u)==0);
    }
    if(s_seq_state==SQ_QR_WAIT) CHECK(c43_service(20u)==0);
    CHECK(s_seq_state==SQ_STEP_WAIT || s_seq_state==SQ_DONE);
    CHECK(c43_stopped());
    return 0;
}

static int c43_aim(int model,int goal)
{
    CHECK(s_vat.x_goal==(model==9?105:model==4?135:215));
    wire_sync(); wire_ack(wire_request,2u,0u); CHECK(c43_service(1100u)==0);
    for(unsigned k=0;k<120u && !host_timer_active && s_seq_state==SQ_TASK;++k) {
        host_tick+=20u; xy_obj(wire_request,model,goal,220);
    }
    CHECK(s_seq_state!=SQ_STOPPED);
    return 0;
}

static void c43_rank(uint8_t model,uint8_t rank)
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

static int check_private_knobs_reach_executors(void)
{
    CHECK(c43_boot(1)==0);
    run_cmd("sv120"); run_cmd("lkp2.3"); run_cmd("lff0.02");
    CHECK(c43_start_road()==0 && s_v==120.0f && s_dist_heading_kp==2.3f && s_dist_ff_ratio==0.02f);
    run_cmd("0"); CHECK(c43_boot(1)==0);
    c43_slot(7u,R43_PENDING_PREP); run_cmd("sv130"); run_cmd("ykp1.2"); run_cmd("bff-0.02");
    CHECK(c43_start_road()==0 && s_v==130.0f && s_dist_heading_kp==1.2f && s_dist_ff_ratio==-0.02f);
    run_cmd("0"); CHECK(c43_boot(1)==0);
    c43_slot(6u,R43_PENDING_PREP); run_cmd("sv140"); run_cmd("xkp3.4"); run_cmd("rff-0.01");
    CHECK(c43_start_road()==0 && s_v==140.0f && s_dist_heading_kp==3.4f && s_dist_ff_ratio==-0.01f);
    run_cmd("0"); CHECK(c43_boot(1)==0);
    c43_slot(12u,R43_PENDING_PREP); run_cmd("sv150"); run_cmd("cff0.12");
    CHECK(c43_start_road()==0 && s_v==150.0f && s_dist_ff_ratio==-0.12f);
    run_cmd("0"); CHECK(c43_boot(1)==0);
    c43_slot(15u,R43_PENDING_PREP); s_route31_hostage_rank=2u;
    run_cmd("sv160"); run_cmd("eff0.11");
    CHECK(c43_start_road()==0 && s_v==160.0f && s_dist_ff_ratio==-0.11f &&
          s_dist_target==(float)ROUTE31_HOSTAGE_RANK2_MM);
    run_cmd("0"); CHECK(c43_boot(1)==0);
    c43_slot(11u,R43_PENDING_PREP); run_cmd("v140"); run_cmd("ykp1.4"); run_cmd("sff0.04");
    run_cmd("g"); wire_sync(); wire_ack(wire_request,2u,0u);
    CHECK(c43_service(1100u)==0 && s_target35_kp==1.4f && last_x==140.0f &&
          fabsf(last_y-5.6f)<0.0001f);
    run_cmd("0"); CHECK(c43_boot(1)==0);
    c43_slot(14u,R43_PENDING_PREP); s_route31_rack_deployed=0u;
    run_cmd("v145"); run_cmd("ykp1.5"); run_cmd("sff0.045"); run_cmd("g");
    CHECK(s_seq_state==SQ_TASK && s_route_search_kp==1.5f &&
          s_route_search_speed==145.0f && s_route_search_ff==0.045f);
    puts("43 private perleg speed/yaw gain/FF actually reach distance,target and hostage search executors;turn angle/cap/gain checked above passed");
    return 0;
}

static int check_full_current_route(void)
{
    CHECK(c43_boot(1)==0); run_cmd("e7"); run_cmd("f1234");
    for(unsigned stage=0;stage<8u;++stage) {
        CHECK(s_seq_stage==stage && c43_start_road()==0 && c43_finish_road()==0);
        CHECK(c43_wait()==0);
    }
    CHECK(s_seq_stage==8u && !s_route31_rack_deployed);
    run_cmd("g"); CHECK(s_seq_state==SQ_ARM_PREP && host_timer_active);
    CHECK(s_route43_rack_issued==7u && host_timer_pps==1234u && c43_emit(7u)==0);
    wire_poll(); CHECK(c43_service(250u)==0 && s_route31_rack_deployed && c43_wait()==0);
    CHECK(c43_start_road()==0 && c43_finish_road()==0 && s_seq_stage==9u);
    run_cmd("h4"); run_cmd("lf1000"); run_cmd("u2200");
    run_cmd("g"); CHECK(s_seq_state==SQ_TASK && c43_aim(4,VAT_ROUTE_BALL_X_PX)==0);
    c43_rank(4u,2u);
    CHECK(host_timer_active && s_route31_lift_phase==R31_LIFT_BALL_DOWN && host_timer_pps==1000u);
    CHECK(c43_emit(4u)==0); wire_poll(); CHECK(host_servo==2200u);
    CHECK(c43_service(2000u)==0 && s_route31_lift_phase==R31_LIFT_BALL_UP && c43_emit(4u)==0);
    wire_poll(); CHECK(c43_service(20u)==0 && s_seq_pair_turn && c43_wait()==0);
    CHECK(c43_start_road()==0 && c43_finish_road()==0 && s_seq_pair_offset && c43_wait()==0);
    CHECK(route_seq_speed_mms()==80.0f);
    CHECK(c43_start_road()==0 && c43_finish_road()==0 && s_route43_pending==R43_PENDING_BUCKET && c43_wait()==0);
    CHECK(c43_no_write("e9")==0 && c43_no_write("f1600")==0);
    run_cmd("h6"); run_cmd("g"); CHECK(c43_aim(9,VAT_ROUTE_BUCKET_X_PX)==0);
    CHECK(host_timer_active && s_route31_lift_phase==R31_LIFT_BUCKET_DOWN && c43_emit(6u)==0);
    wire_poll(); CHECK(host_servo==ARM_SERVO_START_US);
    CHECK(c43_service(2000u)==0 && s_route31_lift_phase==R31_LIFT_BUCKET_UP && c43_emit(6u)==0);
    wire_poll(); CHECK(s_route31_lift_phase==R31_RACK_RETRACT && host_timer_pps==1234u && s_jog_clock.remaining==7u);
    CHECK(c43_emit(7u)==0); wire_poll();
    CHECK(c43_service(270u)==0 && !s_route31_rack_deployed && s_seq_stage==10u && c43_wait()==0);
    CHECK(host_jog_pulses[0][0]==7u && host_jog_pulses[0][1]==7u);
    CHECK(c43_start_road()==0 && c43_finish_road()==0 && s_seq_stage==11u && !s_seq_return_offset);
    run_cmd("g"); wire_sync(); wire_ack(wire_request,2u,0u); CHECK(c43_service(1100u)==0);
    CHECK(target35_point()==250 && target35_low()==247 && target35_high()==253 && target35_fine_enter()==360);
    for(unsigned k=0;k<80u && !laser_state;++k) { host_tick+=20u; xy_obj(wire_request,8,target35_point(),100); }
    CHECK(laser_state && c43_service(2000u)==0 && s_seq_stage==12u && c43_wait()==0);
    CHECK(c43_start_road()==0 && c43_finish_road()==0 && s_seq_stage==13u);
    CHECK(c43_start_road()==0 && c43_finish_road()==0 && s_seq_stage==14u);
    run_cmd("e3001"); run_cmd("f800"); run_cmd("u2300");
    CHECK(s_route43_rack_issued==7u && s_route43_rack_pps_issued==1234u);
    run_cmd("g"); wire_sync(); wire_ack(wire_request,2u,0u); CHECK(c43_service(1100u)==0);
    c43_rank(0u,1u);
    for(unsigned k=0;k<120u && s_seq_state==SQ_TASK;++k) { host_tick+=20u; xy_obj(wire_request,0,VAT_ROUTE_HOSTAGE_X_PX,220); }
    CHECK(s_seq_state==SQ_STEP_WAIT && s_seq_hostage_offset && !host_timer_active && c43_wait()==0);
    run_cmd("d31"); run_cmd("sv90"); CHECK(s_route43_tune.hostage_left_mm==31u && route_seq_speed_mms()==90.0f);
    CHECK(c43_start_road()==0 && c43_finish_road()==0 && s_route43_pending==R43_PENDING_HOSTAGE && !s_seq_hostage_offset && c43_wait()==0);
    CHECK(!host_timer_active && s_route31_lift_phase==R31_LIFT_OFF);
    run_cmd("g"); CHECK(s_seq_state==SQ_TASK && c43_aim(0,VAT_ROUTE_HOSTAGE_X_PX)==0);
    CHECK(host_timer_active && host_jog_axis==0 && host_timer_pps==800u &&
          s_route31_lift_phase==R31_HOSTAGE_RACK_EXTEND &&
          s_jog_clock.remaining+s_jog_clock.completed==3001u);
    /* The real receive/recheck poll can commit the previously-seen300ms loss
     * rule after this fresh g; service() also clocks every live STEP event. */
    CHECK(c43_emit(s_jog_clock.remaining)==0); wire_poll();
    CHECK(host_jog_pulses[0][0]==7u+3001u);
    CHECK(c43_service(250u)==0 && host_servo==2300u);
    CHECK(c43_service(250u)==0 && host_timer_active && s_jog_clock.remaining==3000u);
    CHECK(c43_emit(3000u)==0); wire_poll(); CHECK(c43_service(270u)==0);
    CHECK(s_seq_stage==15u && s_route31_hostage_rank==1u && c43_wait()==0);
    CHECK(s_route43_rack_issued==7u && s_route43_rack_pps_issued==1234u && host_servo==2300u);
    CHECK(c43_start_road()==0 && c43_finish_road()==0 && s_seq_state==SQ_DONE);
    CHECK(c43_service(5000u)==0 && c43_stopped() && host_servo==2300u);
    puts("43 full current16stage chain:allnext-g gates incl cross->contact,rack,ball180/15,bucket exactissued return,no returnoffset,targetfire2s,hostageoffset->g recheck/grab/rank final DONE passed");
    return 0;
}

int main(void)
{
    CHECK(check_current_defaults_and_ui()==0);
    CHECK(check_knobs_and_scope()==0);
    CHECK(check_private_knobs_reach_executors()==0);
    CHECK(check_full_current_route()==0);
    puts("route43_current_step_test: host software replay only; no vehicle/camera/laser/arm physical acceptance");
    return 0;
}
