/* Actual VAT engine + AA55/CRC/request parser. Coordinates, yaw and elapsed
 * time are explicit host inputs; this is not physical straightness evidence. */
#define ROUTE44_CURRENT_FIXTURE_MAIN retained_fine_guard_route44_main
#include "route44_current_test.c"
#undef ROUTE44_CURRENT_FIXTURE_MAIN

static void fine_frame(const ProtoFrame *frame)
{
    real_proto_stats_get(&host_proto_stats);
    vision_align_test_feed_frame(frame);
}
static void fine_poll(unsigned ms)
{
    host_tick += ms; wire_sync(); vision_align_test_poll(); wire_sync();
}
static void fine_ack(unsigned result)
{
    uint8_t p[5]={0x61u,(uint8_t)wire_request,(uint8_t)(wire_request>>8),2u,(uint8_t)result};
    wire_feed_raw(p,sizeof p,0); wire_sync();
}
static int fine_model(void)
{ return s_vat.task==PROTO_TASK_BUCKET ? 9 : s_vat.task==PROTO_TASK_HOSTAGE ? 0 : 4; }
static int fine_y(void)
{ return s_vat.task==PROTO_TASK_BUCKET ? VAT_BUCKET_Y_PX : s_vat.task==PROTO_TASK_HOSTAGE ? VAT_HOSTAGE_Y_PX : VAT_BALL_Y_PX; }
static void fine_obj(int cx,unsigned ms)
{
    xy_obj_poll(wire_request,fine_model(),cx,fine_y(),0); fine_poll(ms);
}
static void fine_rank(void)
{
    uint8_t model=(uint8_t)fine_model();
    uint8_t p[14]={0x62u,(uint8_t)wire_request,(uint8_t)(wire_request>>8),9u,0u,
                   0x54u,0u,0u,model,2u,3u,3u,4u,5u};
    if(model==0u) {p[11]=1u;p[12]=0u;p[13]=2u;}
    route_wire_le16(p+6u,xy_seq++); wire_feed_raw(p,sizeof p,0); wire_sync();
}
static int fine_start(unsigned owner,unsigned task,int direction)
{
    const int32_t qr[3]={1,2,3};
    CHECK(wire_boot()==0); host_wire_diag_hook=real_proto_wire_diag_get;
    proto_set_on_frame(fine_frame); xy_seq=1u;
    if(owner) CHECK(vision_align_test_start_route_scoped(task==PROTO_TASK_HOSTAGE?40u:41u,qr,(uint8_t)owner));
    else CHECK(vision_align_test_start(39u,NULL));
    if(owner && task==PROTO_TASK_BUCKET) {
        /* Same paired-bucket request entry; mechanical/180 legs are outside
         * this fine-only test. Preserve the genuine observed BALL54 rank. */
        wire_sync(); fine_ack(0u); fine_rank(); fine_poll(20u);
        CHECK(s_vat.ball_rank==2u);
        s_vat.state=VAT_TURN_REQUEST;
        CHECK(vision_align_test_take_turn_request());
        vision_align_test_notify_turn_result(1);
        CHECK(s_vat.task==PROTO_TASK_BUCKET && s_route_bucket_eligible);
    }
    wire_sync(); fine_ack(0u);
    if(task!=PROTO_TASK_BUCKET) { fine_rank(); fine_poll(20u); }
    fine_poll(260u);
    for(unsigned i=0u;i<40u && s_vat.state!=VAT_FINE_CONTINUOUS;++i)
        fine_obj(s_vat.x_goal+direction*11,20u);
    if(owner) {
        CHECK(s_vat.state==VAT_FINE_CONTINUOUS && s_vat.axis==1u);
        CHECK(last_x==(float)direction*VAT_X_SPEED_MMS && last_y==0.0f);
        CHECK(!s_initial_search && !s_search_mid.phase && s_route_search_ff==0.0f);
    }
    return 0;
}
static int fine_repair(unsigned task,int sign)
{
    CHECK(fine_start(31u,task,sign)==0);
    uint16_t request=s_vat.request; int zeros=zero_calls;
    float goal=s_vat.yaw_target; int xgoal=s_vat.x_goal;
    uint8_t ballrank=s_vat.ball_rank,rank=s_vat.target_rank;
    /* This path must not depend on having previously driven coarse search. */
    s_route_search_started=0u;
    host_absolute_yaw=host_yaw=(float)sign*1.0f; fine_poll(20u);
    CHECK(!s_search_mid.phase && s_vat.state==VAT_FINE_CONTINUOUS);
    host_absolute_yaw=host_yaw=(float)sign*1.001f; fine_poll(20u);
    CHECK(s_search_mid.phase==1u && s_search_mid.fine_recheck && s_search_mid.need_new);
    CHECK(s_vat.state==VAT_BRAKE && last_x==0.0f && last_y==0.0f && last_w==0.0f);
    fine_obj(xgoal,20u); /* Rotating/moving image can advance only the barrier. */
    CHECK(!s_vat.good && vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    fine_poll(260u);
    CHECK(s_search_mid.phase==2u && s_vat.state==VAT_YAW_FIX);
    CHECK(last_x==0.0f && last_y==0.0f && last_w==(float)-sign*0.40f);
    host_absolute_yaw=host_yaw=(float)sign*0.2f; fine_poll(20u); fine_poll(400u);
    CHECK(!s_search_mid.phase && s_search_mid.need_new && s_vat.state==VAT_RECHECK);
    CHECK(last_x==0.0f && last_y==0.0f && last_w==0.0f);
    CHECK(s_vat.yaw_target==goal && s_vat.x_goal==xgoal && s_vat.request==request);
    CHECK(s_vat.ball_rank==ballrank && s_vat.target_rank==rank && zero_calls==zeros);
    CHECK(!s_initial_search && !s_route_search_started && !s_vat.good);
    /* Empty, wrong request/class, duplicate, and delayed images cannot restart
     * or trigger the existing seen-loss grab while waiting for NEW pixels. */
    unsigned next=xy_seq;
    xy_obj_poll(request,-1,0,0,0); fine_poll(20u);
    xy_obj_poll((uint16_t)(request-1u),fine_model(),xgoal,fine_y(),0); fine_poll(20u);
    xy_obj_poll(request,fine_model()==4?5:4,xgoal,fine_y(),0); fine_poll(20u);
    xy_seq=s_search_mid.sequence;
    xy_obj_poll(request,fine_model(),xgoal,fine_y(),0); fine_poll(20u);
    xy_seq=(uint16_t)(s_search_mid.sequence-1u);
    xy_obj_poll(request,fine_model(),xgoal,fine_y(),0); fine_poll(20u);
    xy_seq=next+10u; fine_poll(2200u);
    CHECK(s_vat.state==VAT_RECHECK && s_search_mid.need_new && !s_vat.good);
    CHECK(!s_vat.ball_fallback && !s_vat.bucket_fallback && !s_vat.hostage_fallback);
    CHECK(last_x==0.0f && last_y==0.0f && last_w==0.0f && !laser_state);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    /* Even a far NEW coordinate resumes latched fine v20, never coarse. */
    fine_obj(xgoal+sign*100,20u);
    CHECK(!s_search_mid.need_new && s_vat.state==VAT_FINE_CONTINUOUS && !s_initial_search);
    CHECK(last_x==(float)sign*VAT_X_SPEED_MMS && last_y==0.0f);
    CHECK(s_vat.yaw_target==goal && s_vat.request==request && zero_calls==zeros);
    return 0;
}
static int fine_timeout_retry(void)
{
    CHECK(fine_start(31u,PROTO_TASK_BALL,1)==0);
    uint16_t request=s_vat.request; float goal=s_vat.yaw_target;
    host_absolute_yaw=host_yaw=3.0f; fine_poll(20u); fine_poll(260u);
    CHECK(s_search_mid.phase==2u); fine_poll(2000u);
    CHECK(s_search_mid.phase==3u && s_search_mid.timeout_latched);
    CHECK(!s_vat.alignment_confirmed && !s_vat.good && !s_vat.ball_fallback);
    fine_poll(260u);
    CHECK(s_vat.state==VAT_RECHECK && s_search_mid.need_new && s_search_mid.retry_wait_motion);
    fine_poll(5000u);
    CHECK(s_vat.state==VAT_RECHECK && s_search_mid.retry_wait_motion && s_search_mid.timeout_latched);
    CHECK(last_x==0.0f && last_y==0.0f && last_w==0.0f);
    fine_obj(s_vat.x_goal+100,20u);
    uint32_t restart=host_tick;
    CHECK(s_vat.state==VAT_FINE_CONTINUOUS && !s_search_mid.retry_wait_motion);
    CHECK(s_search_mid.retry_from==restart && s_search_mid.timeout_latched);
    for(unsigned i=0;i<3u;++i) fine_obj(s_vat.x_goal+100,100u);
    fine_obj(s_vat.x_goal+100,99u);
    CHECK(host_tick-restart==399u && !s_search_mid.phase && s_search_mid.timeout_latched);
    CHECK(last_x==VAT_X_SPEED_MMS && last_y==0.0f);
    fine_poll(1u); /* Guard is intentionally before the 20ms image poll. */
    CHECK(host_tick-restart==400u && s_search_mid.phase==1u && !s_search_mid.timeout_latched);
    CHECK(last_x==0.0f && last_y==0.0f && last_w==0.0f);
    CHECK(s_vat.request==request && s_vat.yaw_target==goal && !s_initial_search);
    return 0;
}
static int fine_cancel_and_faults(void)
{
    const char *keys[]={"g","a","0"};
    for(unsigned i=0u;i<3u;++i) {
        CHECK(fine_start(31u,PROTO_TASK_BALL,1)==0);
        host_absolute_yaw=host_yaw=2.0f; fine_poll(20u);
        if(i) fine_poll(260u);
        if(i==2u) {fine_poll(2000u);fine_poll(260u);}
        s_seq_mode=31u;s_seq_state=SQ_TASK;s_seq_stage=9u;s_msel=31;
        run_cmd(keys[i]);wire_sync();
        CHECK(s_vat.state==VAT_STOPPED && !s_search_mid.phase && !s_search_mid.need_new);
        CHECK(!s_search_mid.retry_wait_motion && last_x==0.0f && last_y==0.0f && last_w==0.0f);
        xy_obj_poll(wire_request,4,250,VAT_BALL_Y_PX,0);fine_poll(1000u);
        CHECK(s_vat.state==VAT_STOPPED && !laser_state);
    }
    for(unsigned fault=0u;fault<4u;++fault) {
        CHECK(fine_start(31u,PROTO_TASK_BALL,1)==0);
        host_absolute_yaw=host_yaw=2.0f; fine_poll(20u);fine_poll(260u);
        if(fault==0u) fine_ack(1u);
        if(fault==1u) host_imu_valid=0;
        if(fault==2u) host_absolute_yaw=NAN;
        if(fault==3u) host_abort=1;
        fine_poll(20u);
        CHECK(s_vat.state==VAT_STOPPED && !s_search_mid.phase && !s_search_mid.need_new);
        CHECK(last_x==0.0f && last_y==0.0f && last_w==0.0f && !laser_state);
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    }
    return 0;
}
static int fine_old_owner_isolation(void)
{
    for(unsigned owner=43u;owner<=44u;++owner) {
        CHECK(fine_start(owner,PROTO_TASK_BALL,1)==0);
        host_absolute_yaw=host_yaw=3.0f; fine_poll(20u);
        CHECK(s_vat.state==VAT_FINE_CONTINUOUS && !s_search_mid.phase);
        CHECK(!s_search_mid.fine_recheck && !s_search_mid.retry_wait_motion);
        CHECK(last_x==VAT_X_SPEED_MMS && s_vat.x_goal==VAT_ROUTE_BALL_X_PX);
    }
    CHECK(fine_start(0u,PROTO_TASK_BUCKET,1)==0);
    host_absolute_yaw=host_yaw=3.0f;fine_poll(20u);
    CHECK(!s_search_mid.phase && !s_search_mid.fine_recheck && !s_route_owned);
    CHECK(s_vat.x_goal==VAT_X_PX && s_vat.state!=VAT_FINE_CONTINUOUS);
    /* Do not turn this moving-only repair into normal endpoint yaw gating. */
    for(unsigned task=PROTO_TASK_BALL;task<=PROTO_TASK_HOSTAGE;++task) {
        if(task!=PROTO_TASK_BALL && task!=PROTO_TASK_HOSTAGE) continue;
        CHECK(fine_start(31u,task,1)==0);
        host_absolute_yaw=host_yaw=0.8f;
        fine_obj(s_vat.x_goal,20u); fine_poll(260u);
        for(unsigned i=0u;i<10u && s_vat.good<VAT_GOOD_FRAMES;++i)
            fine_obj(s_vat.x_goal,20u);
        CHECK(!s_search_mid.phase && s_vat.state!=VAT_YAW_FIX && !vat_stopped_yaw_enabled());
        CHECK(s_vat.good==VAT_GOOD_FRAMES && last_x==0.0f && last_y==0.0f && last_w==0.0f);
    }
    return 0;
}
int main(void)
{
    const unsigned tasks[]={PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    for(unsigned i=0u;i<3u;++i)
        if(fine_repair(tasks[i],1)||fine_repair(tasks[i],-1)) return 1;
    if(fine_timeout_retry()||fine_cancel_and_faults()||fine_old_owner_isolation()) return 1;
    puts("PASS:31 moving fine yaw guard/new-frame barrier/same-task retry400/cancel/owner isolation");
    return 0;
}
