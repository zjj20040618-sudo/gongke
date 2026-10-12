/* Real VAT/CRC/request fixture. Time, pixels, yaw and wheel counts are HOST
 * inputs: this proves timeout ownership/barriers, not physical convergence. */
#define ROUTE44_CURRENT_FIXTURE_MAIN retained_r31_timeout_route44_main
#include "route44_current_test.c"
#undef ROUTE44_CURRENT_FIXTURE_MAIN

static void timeout_frame(const ProtoFrame *frame)
{
    real_proto_stats_get(&host_proto_stats);
    vision_align_test_feed_frame(frame);
}
static void timeout_poll(unsigned ms)
{
    host_tick += ms; wire_sync(); vision_align_test_poll(); wire_sync();
}
static void timeout_ack(unsigned result)
{
    uint8_t p[5]={0x61u,(uint8_t)wire_request,(uint8_t)(wire_request>>8),2u,(uint8_t)result};
    wire_feed_raw(p,sizeof p,0); wire_sync();
}
static int timeout_start(unsigned owner)
{
    const int32_t qr[3]={1,2,3};
    CHECK(wire_boot()==0); host_wire_diag_hook=real_proto_wire_diag_get;
    proto_set_on_frame(timeout_frame); xy_seq=1u;
    CHECK(vision_align_test_start_route_scoped(41u,qr,(uint8_t)owner));
    wire_sync(); CHECK(wire_opcode==0x63u && wire_task==1u && wire_digit==1u);
    timeout_ack(0u); CHECK(vat_ack_ready());
    return 0;
}
static int timeout_search(unsigned owner)
{
    CHECK(timeout_start(owner)==0);
    CHECK(vision_align_test_route_search_speed_set(50.0f));
    timeout_poll(260u); timeout_poll(20u); timeout_poll(400u);
    CHECK(s_vat.state==VAT_ROUTE_SEARCH && last_x==50.0f);
    host_absolute_yaw=3.0f; timeout_poll(20u);
    CHECK(s_search_mid.phase==1u && s_vat.state==VAT_BRAKE && !last_x && !last_w);
    timeout_poll(260u);
    CHECK(s_search_mid.phase==2u && s_vat.state==VAT_YAW_FIX && last_w<0.0f);
    return 0;
}
static int mid_timeout_continue(void)
{
    CHECK(timeout_search(31u)==0);
    uint16_t request=wire_request;
    xy_seq=77u; xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX,VAT_BALL_Y_PX,0);
    timeout_poll(ROUTE31_POST_YAW_MAX_MS);
    CHECK(s_search_mid.phase==3u && s_search_mid.timeout_latched &&
          s_vat.state!=VAT_STOPPED && !last_x && !last_w && !s_vat.good);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    timeout_poll(240u); CHECK(s_search_mid.phase==3u);
    host_counts[0]++; timeout_poll(20u); /* Actual motion restarts the stop clock. */
    timeout_poll(240u); CHECK(s_search_mid.phase==3u);
    timeout_poll(20u);
    CHECK(!s_search_mid.phase && s_search_mid.need_new && s_vat.state==VAT_ROUTE_SEARCH &&
          !s_route31_target.seen && !s_vat.ball_seen && !s_vat.alignment_confirmed &&
          wire_request==request && s_vat.yaw_target==0.0f);
    xy_seq=77u; xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX,VAT_BALL_Y_PX,0);
    timeout_poll(399u);
    CHECK(s_search_mid.need_new && !s_route31_target.seen &&
          !s_search_mid.phase && s_vat.state==VAT_ROUTE_SEARCH && last_x==50.0f && last_w<0.0f);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    timeout_poll(1u);
    CHECK(s_search_mid.phase==1u && s_vat.state==VAT_BRAKE && !last_x && !last_w &&
          wire_request==request && s_vat.yaw_target==0.0f && !s_vat.alignment_confirmed);
    /* A further rotating image is only a barrier, not arrival evidence. */
    xy_seq=78u; xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    host_absolute_yaw=.2f; timeout_poll(260u); timeout_poll(400u);
    CHECK(!s_search_mid.phase && s_search_mid.need_new && s_vat.state==VAT_ROUTE_SEARCH &&
          s_vat.yaw_target==0.0f && wire_request==request && !s_vat.good);
    xy_seq=78u; xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    timeout_poll(20u); CHECK(s_search_mid.need_new && !s_route31_target.seen);
    xy_seq=79u; xy_obj_poll(request,4,VAT_ROUTE31_BALL_X_PX+100,VAT_BALL_Y_PX,0);
    timeout_poll(20u);
    CHECK(!s_search_mid.need_new && !s_search_mid.phase && s_route31_target.seen &&
          s_vat.state==VAT_ROUTE_SEARCH && last_x>0.0f && last_x<=50.0f);
    /* Stop/restart resets the bounded ramp; the first resumed20ms command
     * is not promised to jump to cruise speed. It then reaches the same50. */
    timeout_poll(100u); CHECK(s_vat.state==VAT_ROUTE_SEARCH && last_x==50.0f);
    vision_align_test_cancel();
    CHECK(timeout_search(31u)==0 && !s_search_mid.timeout_latched); /* New task resets latch. */
    return 0;
}
static int ordinary_yaw_begin(unsigned owner)
{
    CHECK(timeout_start(owner)==0);
    /* Narrow yaw-fix boundary, not a simulated successful arm/180 operation:
     * actual VAT creates and ACKs the bucket request, then enters normal yaw. */
    CHECK(vat_request(PROTO_TASK_BUCKET,0u,host_tick)); wire_sync(); timeout_ack(0u);
    s_route_bucket_eligible=s_route_bucket_seen=1u; /* Existing bucket fine-phase ownership. */
    s_initial_search=0u; s_need_new=1u; s_vat.yaw_dirty=s_vat.yaw_ever=1u;
    host_absolute_yaw=3.0f; timeout_poll(260u);
    CHECK(s_vat.state==VAT_YAW_FIX && !s_search_mid.phase);
    timeout_poll(20u); CHECK(last_w<0.0f);
    return 0;
}
static int ordinary_timeout_continue(void)
{
    CHECK(ordinary_yaw_begin(31u)==0);
    uint16_t request=wire_request;
    timeout_poll(T_DIST_ALIGN_MAX_MS);
    CHECK(s_search_mid.phase==4u && !last_w && !last_x && !s_vat.alignment_confirmed &&
          s_vat.state==VAT_YAW_FIX && !s_route31_target.seen);
    xy_seq=91u; xy_obj_poll(request,9,VAT_ROUTE31_BUCKET_X_PX,VAT_BUCKET_Y_PX,0);
    timeout_poll(240u); CHECK(s_search_mid.phase==4u);
    host_counts[2]++; timeout_poll(20u); timeout_poll(240u);
    CHECK(s_search_mid.phase==4u); timeout_poll(20u);
    CHECK(s_vat.state==VAT_RECHECK && s_search_mid.need_new && !s_search_mid.phase &&
          !s_vat.good && !s_route31_target.seen && !s_vat.bucket_seen &&
          !s_vat.alignment_confirmed && s_vat.yaw_target==0.0f && wire_request==request);
    timeout_poll(1000u);
    CHECK(s_vat.state==VAT_RECHECK && !last_x && !last_w &&
          vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    xy_seq=91u; xy_obj_poll(request,9,VAT_ROUTE31_BUCKET_X_PX,VAT_BUCKET_Y_PX,0);
    timeout_poll(20u); CHECK(s_search_mid.need_new && s_vat.state==VAT_RECHECK);
    xy_seq=92u; xy_obj_poll(request,9,VAT_ROUTE31_BUCKET_X_PX+30,VAT_BUCKET_Y_PX,0);
    timeout_poll(20u);
    CHECK(!s_search_mid.need_new && s_vat.state==VAT_FINE_CONTINUOUS && last_x>0.0f &&
          !s_vat.alignment_confirmed && !s_vat.good);
    return 0;
}
static int isolation_and_gain(void)
{
    const unsigned owners[2]={43u,44u};
    CHECK(timeout_start(31u)==0);
    CHECK(vision_align_test_route_search_kp_set(10.0f) && s_route_search_kp==10.0f);
    CHECK(!vision_align_test_route_search_kp_set(10.01f) &&
          !vision_align_test_route_search_kp_set(-0.01f) &&
          !vision_align_test_route_search_kp_set(NAN));
    CHECK(!step_heading_kp_set(10.0f) && step_heading_kp_deg()==0.3f);
    timeout_poll(260u); host_absolute_yaw=1.0f; timeout_poll(20u); timeout_poll(400u);
    CHECK(s_vat.state==VAT_ROUTE_SEARCH && fabsf(last_w+0.174533f)<0.00001f);
    for(unsigned i=0u;i<2u;++i) {
        CHECK(timeout_start(owners[i])==0);
        CHECK(vision_align_test_route_search_kp_set(5.0f) &&
              !vision_align_test_route_search_kp_set(5.01f) &&
              !vision_align_test_route_search_kp_set(10.0f));
        CHECK(timeout_search(owners[i])==0); timeout_poll(ROUTE31_POST_YAW_MAX_MS);
        CHECK(s_vat.state==VAT_STOPPED && !strcmp(s_vat.reason,"SEARCH_MID_YAW_TIMEOUT_GT1P5"));
        CHECK(ordinary_yaw_begin(owners[i])==0); timeout_poll(T_DIST_ALIGN_MAX_MS);
        CHECK(s_vat.state==VAT_STOPPED && !strcmp(s_vat.reason,"YAW_TIMEOUT"));
    }
    CHECK(xy_start(39u)==0); s_vat.state=VAT_YAW_FIX; s_yaw_from=host_tick;
    timeout_poll(T_DIST_ALIGN_MAX_MS);
    CHECK(s_vat.state==VAT_STOPPED && !strcmp(s_vat.reason,"YAW_TIMEOUT"));
    return 0;
}
static int timeout_fault_and_cancel(void)
{
    const char *commands[3]={"g","a","0"};
    for(unsigned phase=0u;phase<2u;++phase) for(unsigned i=0u;i<3u;++i) {
        if(phase) { CHECK(ordinary_yaw_begin(31u)==0); timeout_poll(T_DIST_ALIGN_MAX_MS); }
        else { CHECK(timeout_search(31u)==0); timeout_poll(ROUTE31_POST_YAW_MAX_MS); }
        uint16_t request=wire_request;
        /* Supply the real route owner so g has its active-route stop meaning. */
        s_seq_mode=31u; s_seq_state=SQ_TASK; s_seq_stage=9u; s_msel=31;
        run_cmd(commands[i]); CHECK(s_vat.state==VAT_STOPPED && !s_search_mid.phase &&
                                   !s_receiving && !last_x && !last_w && !host_timer_active);
        xy_obj_poll(request,phase?9:4,phase?VAT_ROUTE31_BUCKET_X_PX:VAT_ROUTE31_BALL_X_PX,250,0);
        timeout_poll(1000u); CHECK(s_vat.state==VAT_STOPPED && !last_x && !last_w && !pulse_calls);
    }
    CHECK(timeout_search(31u)==0); timeout_ack(1u); timeout_poll(20u);
    CHECK(s_vat.state==VAT_STOPPED && !strcmp(s_vat.reason,"NACK"));
    CHECK(timeout_search(31u)==0); host_imu_valid=0; timeout_poll(20u);
    CHECK(s_vat.state==VAT_STOPPED && !strcmp(s_vat.reason,"IMUERR"));
    CHECK(timeout_search(31u)==0); host_absolute_yaw=NAN; timeout_poll(20u);
    CHECK(s_vat.state==VAT_STOPPED && !strcmp(s_vat.reason,"IMUERR"));
    CHECK(timeout_search(31u)==0); host_abort=1; timeout_poll(20u);
    CHECK(s_vat.state==VAT_STOPPED && !strcmp(s_vat.reason,"ABORT"));
    CHECK(timeout_search(31u)==0);
    ProtoFrame bad={0}; bad.type=PF_OBJ; bad.cls=CLS_BALL; bad.label=0;
    bad.sequence=123u; bad.img_w=100u; bad.img_h=480u; bad.cx=50; bad.cy=250;
    bad.w=bad.h=24; bad.conf=90;
    vision_align_test_feed_frame(&bad); timeout_poll(20u);
    CHECK(s_vat.state==VAT_STOPPED && !strcmp(s_vat.reason,"IMAGE_GEOMETRY"));
    return 0;
}
static int workpoint_owner_isolation(void)
{
    const unsigned owners[3]={31u,43u,44u};
    const ProtoTask tasks[3]={PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    const uint8_t digits[3]={1u,0u,3u};
    const int current_goals[3]={165,165,185};
    const int legacy_goals[3]={135,105,215};
    CHECK(VAT_TOL_PX==10 && VAT_ROUTE_BALL_X_PX==135 &&
          VAT_ROUTE_BUCKET_X_PX==105 && VAT_ROUTE_HOSTAGE_X_PX==215);
    for(unsigned owner=0u;owner<3u;++owner) {
        CHECK(timeout_start(owners[owner])==0);
        for(unsigned task=0u;task<3u;++task) {
            CHECK(vat_request(tasks[task],digits[task],host_tick));
            wire_sync(); timeout_ack(0u);
            CHECK(s_route_owner_mode==owners[owner] && s_vat.x_goal==
                  (owners[owner]==31u?current_goals[task]:legacy_goals[task]));
            if(owners[owner]==31u) {
                CHECK(s_vat.x_goal-VAT_TOL_PX==(tasks[task]==PROTO_TASK_HOSTAGE?175:155));
                CHECK(s_vat.x_goal+VAT_TOL_PX==(tasks[task]==PROTO_TASK_HOSTAGE?195:175));
            }
        }
        vision_align_test_cancel();
    }
    for(unsigned mode=38u;mode<=41u;++mode) {
        CHECK(xy_start(mode)==0 && !s_route_owned && s_vat.x_goal==190);
        vision_align_test_cancel();
    }
    CHECK(wire_boot()==0); run_cmd("35");
    CHECK(s_seq_state==SQ_OFF && target35_point()==255 &&
          target35_low()==250 && target35_high()==260);
    puts("31 BALL165/BUCKET165/HOSTAGE185 +/-10;43/44 original135/105/215;standalone38..41 X190 and35 X255 isolation passed");
    return 0;
}
int main(void)
{
    CHECK(workpoint_owner_isolation()==0);
    CHECK(mid_timeout_continue()==0);
    CHECK(ordinary_timeout_continue()==0);
    CHECK(isolation_and_gain()==0);
    CHECK(timeout_fault_and_cancel()==0);
    puts("route31 vision timeout continue tests PASS"); return 0;
}
