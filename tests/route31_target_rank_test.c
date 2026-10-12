/* Current31 target54 lifecycle through the real request/CRC/parser and route
 * dispatcher. The stopped-stage entry below isolates this task; injected
 * pixels/IMU are HOST inputs, not a physical route or laser acceptance test. */
#define ROUTE44_CURRENT_FIXTURE_MAIN retained_r31_target_rank_route44_main
#include "route44_current_test.c"
#undef ROUTE44_CURRENT_FIXTURE_MAIN

static uint8_t target_rank_model(unsigned digit)
{
    static const uint8_t models[3]={6u,8u,7u};
    return models[digit-1u];
}

static uint16_t target_rank_distance(unsigned rank)
{
    static const uint16_t distances[3]={ROUTE31_TARGET_RANK1_MM,
        ROUTE31_TARGET_RANK2_MM,ROUTE31_TARGET_RANK3_MM};
    return distances[rank-1u];
}

static uint16_t target_color_distance(unsigned digit)
{
    static const uint16_t distances[3]={ROUTE31_RED_TO_CORNER_MM,
        ROUTE31_GREEN_TO_CORNER_MM,ROUTE31_BLUE_TO_CORNER_MM};
    return distances[digit-1u];
}

static void target_rank_packet(uint16_t request,uint8_t model,uint8_t rank)
{
    uint8_t p[14]={0x62u,(uint8_t)request,(uint8_t)(request>>8),9u,0u,
        0x54u,0u,0u,model,rank,rank,255u,255u,255u};
    uint8_t other=6u;
    route_wire_le16(p+6u,xy_seq++);
    for(uint8_t i=0u;i<rank;++i) {
        if(i==rank-1u) p[11u+i]=model;
        else { while(other==model) ++other; p[11u+i]=other++; }
    }
    wire_feed(p,sizeof p,0);
}

/* Only the route entry is a stopped-slot snapshot. From the NEW63 onward all
 * acknowledgements,54/01 packets, firing, stopping and next-leg dispatch use
 * production code; no assignment to the new private rank/fire_done flags. */
static int target_rank_enter(unsigned mode,unsigned digit,int fresh_boot)
{
    static const int32_t tuples[3][3]={{2,1,3},{1,2,3},{1,3,2}};
    char command[8];
    CHECK(digit>=1u && digit<=3u);
    if(fresh_boot) CHECK(wire_boot()==0);
    host_wire_diag_hook=real_proto_wire_diag_get;
    xy_seq=1u;
    snprintf(command,sizeof command,"%u",mode); run_cmd(command); wire_sync();
    CHECK(s_seq_state==SQ_READY && r44_stopped());
    run_cmd("g"); CHECK(s_seq_state==SQ_STILL && r44_stopped());
    memcpy(s_seq_qr,tuples[digit-1u],sizeof s_seq_qr);
    s_seq_stage=ROUTE31_TARGET_STAGE;
    if(mode==ROUTE44_MODE) {
        unsigned index=0u;
        while(index<ROUTE44_STAGES && s_route44_plan[index].role!=ROUTE31_TARGET_STAGE) ++index;
        CHECK(index<ROUTE44_STAGES); s_route44_index=(uint8_t)index;
    }
    unsigned commands=wire_commands;
    route_seq_prepare(); wire_sync();
    CHECK(s_seq_state==SQ_TASK && s_seq_stage==ROUTE31_TARGET_STAGE &&
          s_seq_mode==mode && s_target35_route && r44_stopped() &&
          wire_commands==commands+1u && wire_opcode==0x63u &&
          wire_task==PROTO_TASK_TARGET && wire_digit==digit &&
          wire_request==s_target31_request && s_receiving && !s_ack && !wire_bad);
    return 0;
}

static int target_rank_fire(void)
{
    unsigned mode=s_seq_mode,digit=wire_digit;
    uint16_t request=wire_request;
    for(unsigned k=0u;k<500u && !laser_state;++k) {
        CHECK(s_seq_state==SQ_TASK && s_seq_stage==ROUTE31_TARGET_STAGE);
        CHECK(target35_point()==(mode==ROUTE_TEST_MODE?265:250) &&
              target35_low()==(mode==ROUTE_TEST_MODE?262:247) &&
              target35_high()==(mode==ROUTE_TEST_MODE?268:253) &&
              target35_fine_enter()==(mode==ROUTE_TEST_MODE?375:360));
        if(s_receiving) xy_obj_poll(request,target_rank_model(digit),target35_point(),100,0);
        CHECK(r44_service(20u)==0);
    }
    CHECK(s_seq_state==SQ_TASK && s_seq_stage==ROUTE31_TARGET_STAGE &&
          s_target35_phase==TA_FIRE && laser_state && host_laser_on_calls==1 &&
          !last_x && !last_y && !last_w && !host_timer_active && !s_go &&
          wire_request==request);
    CHECK(mode==ROUTE_TEST_MODE ? s_receiving && s_ack : !s_receiving);
    return 0;
}

static int target_rank_wait(void)
{
    CHECK(r44_service(ROUTE31_LASER_MS+100u)==0);
    CHECK(s_seq_mode==ROUTE_TEST_MODE && s_seq_state==SQ_TASK &&
          s_seq_stage==ROUTE31_TARGET_STAGE && s_receiving && s_ack &&
          !laser_state && r44_stopped() && host_laser_on_calls==1);
    unsigned commands=wire_commands;
    uint16_t request=wire_request;
    CHECK(r44_service(3000u)==0 && s_seq_state==SQ_TASK &&
          s_seq_stage==ROUTE31_TARGET_STAGE && s_receiving &&
          wire_request==request && wire_commands==commands &&
          host_laser_on_calls==1 && r44_stopped());
    return 0;
}

static int target_rank_corner(unsigned expected)
{
    ProtoTargetRank rank;
    CHECK(s_seq_mode==ROUTE_TEST_MODE && s_seq_stage==ROUTE31_TARGET_CORNER_STAGE &&
          s_seq_state==SQ_STILL && !s_receiving && !real_proto_target_rank_get(&rank) &&
          !laser_state && host_laser_on_calls==1 && r44_stopped() &&
          route_seq_leg()->distance_mm==expected);
    CHECK(r44_start_road()==0 && s_seq_stage==ROUTE31_TARGET_CORNER_STAGE &&
          s_msel==15 && s_dist_target==(float)expected && !laser_state);
    run_cmd("0"); CHECK(s_seq_state==SQ_STOPPED && r44_stopped());
    return 0;
}

static int check_pre_fire_ranks(void)
{
    for(unsigned digit=1u;digit<=3u;++digit) for(unsigned rank=1u;rank<=3u;++rank) {
        CHECK(target_rank_enter(ROUTE_TEST_MODE,digit,1)==0);
        wire_ack(wire_request,2u,0u);
        /* Arrives before preparation clears its pixel sample, not at FIRE. */
        target_rank_packet(wire_request,target_rank_model(digit),(uint8_t)rank);
        CHECK(target_rank_fire()==0);
        CHECK(r44_service(ROUTE31_LASER_MS)==0);
        CHECK(target_rank_corner(target_rank_distance(rank))==0);
    }
    puts("31 target54 pre-fire rank1/2/3 across all QR colors survives preparation; configured distances depend on rank, not color passed");
    return 0;
}

static int check_during_and_after_fire_ranks(void)
{
    for(unsigned rank=1u;rank<=3u;++rank) {
        CHECK(target_rank_enter(ROUTE_TEST_MODE,2u,1)==0);
        wire_ack(wire_request,2u,0u); CHECK(target_rank_fire()==0);
        CHECK(r44_service(600u)==0 && laser_state && s_receiving);
        target_rank_packet(wire_request,8u,(uint8_t)rank);
        CHECK(laser_state && s_seq_stage==ROUTE31_TARGET_STAGE && s_receiving);
        CHECK(r44_service(ROUTE31_LASER_MS-600u)==0);
        CHECK(target_rank_corner(target_rank_distance(rank))==0);

        CHECK(target_rank_enter(ROUTE_TEST_MODE,2u,1)==0);
        wire_ack(wire_request,2u,0u); CHECK(target_rank_fire()==0);
        CHECK(target_rank_wait()==0);
        unsigned commands=wire_commands;
        target_rank_packet(wire_request,8u,(uint8_t)rank);
        CHECK(wire_commands==commands && host_laser_on_calls==1);
        CHECK(target_rank_corner(target_rank_distance(rank))==0);
    }
    puts("31 same request remains open through FIRE and laserOFF rank wait; late54 advances exactly once without new63/refire passed");
    return 0;
}

static int check_unknown_and_wrong_session_cannot_advance(void)
{
    CHECK(target_rank_enter(ROUTE_TEST_MODE,2u,1)==0);
    uint16_t request=wire_request;
    target_rank_packet(request,8u,3u); /* pre-ACK54 cannot authorize anything */
    wire_ack((uint16_t)(request-1u),2u,0u);
    CHECK(!s_ack);
    wire_ack(request,2u,0u); CHECK(target_rank_fire()==0);
    CHECK(target_rank_wait()==0);
    target_rank_packet(request,8u,0u);
    target_rank_packet((uint16_t)(request-1u),8u,1u);
    target_rank_packet(request,6u,1u); /* wrong QR-selected target */
    target_rank_packet(request,4u,1u); /* wrong ball category */
    CHECK(r44_service(1000u)==0 && s_seq_state==SQ_TASK &&
          s_seq_stage==ROUTE31_TARGET_STAGE && s_receiving &&
          !laser_state && host_laser_on_calls==1 && r44_stopped());
    target_rank_packet(request,8u,3u);
    CHECK(target_rank_corner(ROUTE31_TARGET_RANK3_MM)==0);

    /* Defensive dispatcher contract: impossible/unknown private31 rank must
     * never silently fall back to the legal selected color distance. */
    CHECK(target_rank_enter(ROUTE_TEST_MODE,2u,1)==0);
    s_seq_stage=ROUTE31_TARGET_CORNER_STAGE; route_seq_prepare();
    CHECK(s_seq_state==SQ_STOPPED && r44_stopped() && !s_receiving);
    puts("31 unknown/preACK/oldrequest/wrongtarget/wrongcategory rank cannot reach corner; invalid directcorner snapshot stops instead of guessing color passed");
    return 0;
}

static int check_wait_stops_and_new_run(void)
{
    static const char *const keys[]={"g","a","0"};
    for(unsigned k=0u;k<3u;++k) {
        CHECK(target_rank_enter(ROUTE_TEST_MODE,2u,1)==0);
        wire_ack(wire_request,2u,0u); CHECK(target_rank_fire()==0);
        CHECK(target_rank_wait()==0);
        uint16_t request=wire_request;
        unsigned commands=wire_commands,starts=host_timer_start_calls;
        int pulses=pulse_calls,servos=servo_calls;
        run_cmd(keys[k]); CHECK(s_seq_state==SQ_STOPPED && r44_stopped() && !s_receiving);
        wire_ack(request,2u,0u); target_rank_packet(request,8u,1u);
        xy_obj_poll(request,8,ROUTE31_TARGET_CX,100,0);
        CHECK(r44_service(5000u)==0 && s_seq_state==SQ_STOPPED &&
              s_seq_stage==ROUTE31_TARGET_STAGE && r44_stopped() && !s_receiving &&
              host_laser_on_calls==1 && wire_commands==commands &&
              host_timer_start_calls==starts && pulse_calls==pulses && servo_calls==servos);
    }
    CHECK(target_rank_enter(ROUTE_TEST_MODE,2u,1)==0);
    wire_ack(wire_request,2u,0u);
    target_rank_packet(wire_request,8u,1u);
    CHECK(target_rank_fire()==0 && r44_service(ROUTE31_LASER_MS)==0);
    uint16_t old_request=wire_request;
    CHECK(target_rank_corner(ROUTE31_TARGET_RANK1_MM)==0);
    /* Same initialized application/parser: mode select + newg, not reboot. */
    host_laser_on_calls=0;
    CHECK(target_rank_enter(ROUTE_TEST_MODE,2u,0)==0 && wire_request!=old_request);
    target_rank_packet(old_request,8u,1u);
    wire_ack(wire_request,2u,0u); CHECK(target_rank_fire()==0);
    CHECK(target_rank_wait()==0); /* old rank/fire_done cannot close the new task */
    target_rank_packet(wire_request,8u,3u);
    CHECK(target_rank_corner(ROUTE31_TARGET_RANK3_MM)==0);
    puts("31 rank wait g/a/0 stops chassis/laser/STEP and late packets; same-app newg/newrequest clears prior rank/fire_done passed");
    return 0;
}

static int check_rank_wait_faults(void)
{
    for(unsigned fault=0u;fault<2u;++fault) {
        CHECK(target_rank_enter(ROUTE_TEST_MODE,2u,1)==0);
        wire_ack(wire_request,2u,0u); CHECK(target_rank_fire()==0);
        CHECK(target_rank_wait()==0);
        if(fault) host_imu_valid=0;
        else wire_ack(wire_request,2u,1u);
        CHECK(r44_service(20u)==0 && s_seq_state==SQ_STOPPED &&
              r44_stopped() && !s_receiving && host_laser_on_calls==1);
    }
    CHECK(target_rank_enter(ROUTE_TEST_MODE,2u,1)==0);
    wire_ack(wire_request,2u,0u); CHECK(target_rank_fire()==0);
    wire_ack(wire_request,2u,1u);
    CHECK(r44_service(20u)==0 && s_seq_state==SQ_STOPPED &&
          r44_stopped() && !s_receiving && host_laser_on_calls==1);
    puts("31 open target RX NACK during FIRE/wait and IMU failure remain STOP, not timeout continuation passed");
    return 0;
}

static int check_43_44_keep_color_and_local_close(void)
{
    static const unsigned modes[]={ROUTE_STEP_MODE,ROUTE44_MODE};
    for(unsigned k=0u;k<2u;++k) for(unsigned digit=1u;digit<=3u;++digit)
    for(unsigned have_rank=0u;have_rank<2u;++have_rank) {
        CHECK(target_rank_enter(modes[k],digit,1)==0);
        wire_ack(wire_request,2u,0u);
        unsigned rank=digit==1u?3u:1u;
        if(have_rank) target_rank_packet(wire_request,target_rank_model(digit),(uint8_t)rank);
        CHECK(target_rank_fire()==0 && !s_receiving);
        target_rank_packet(wire_request,target_rank_model(digit),(uint8_t)rank);
        CHECK(r44_service(ROUTE31_LASER_MS)==0 && !laser_state &&
              s_seq_state==SQ_STEP_WAIT && s_seq_stage==ROUTE31_TARGET_CORNER_STAGE);
        run_cmd("g"); CHECK(s_seq_state==SQ_STILL);
        CHECK(route_seq_leg()->distance_mm==target_color_distance(digit));
        CHECK(r44_start_road()==0 && s_msel==15 &&
              s_dist_target==(float)target_color_distance(digit) && !laser_state);
        run_cmd("0"); CHECK(r44_stopped());
    }
    puts("43/44 with or without rank still close target RX at FIRE and retain original color distances/stepg; target54 does not alter either route passed");
    return 0;
}

static int check_return_rotation_scope(void)
{
    CHECK(wire_boot()==0); run_cmd("31");
    s_seq_state=SQ_RUN; s_msel=22; s_seq_stage=ROUTE31_RETURN180_STAGE;
    s_seq_pair_turn=0u;
    CHECK(turn_target_deg()==ROUTE31_ACTIVE_RETURN_TARGET_DEG);
    s_seq_stage=ROUTE31_PAIR_STAGE; s_seq_pair_turn=1u;
    CHECK(turn_target_deg()==180.0f);
    s_seq_mode=ROUTE44_MODE; s_seq_pair_turn=0u;
    s_seq_stage=ROUTE31_RETURN180_STAGE;
    CHECK(turn_target_deg()==180.0f);
    CHECK(wire_boot()==0);
    return 0;
}

int main(void)
{
    CHECK(check_pre_fire_ranks()==0);
    CHECK(check_during_and_after_fire_ranks()==0);
    CHECK(check_unknown_and_wrong_session_cannot_advance()==0);
    CHECK(check_wait_stops_and_new_run()==0);
    CHECK(check_rank_wait_faults()==0);
    CHECK(check_43_44_keep_color_and_local_close()==0);
    CHECK(check_return_rotation_scope()==0);
    puts("route31_target_rank_test: software lifecycle/CRC/dispatcher only; no hardware/visual first-appearance order or route acceptance");
    return 0;
}
