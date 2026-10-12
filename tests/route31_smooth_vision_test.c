/* Current mode31 X-only continuous vision: real AA55/CRC/request/ACK/VAT.
 * Host time, camera pixels, yaw and encoder totals are explicit inputs.
 * Command continuity is software evidence, not physical wheel synchronization. */
#define ROUTE31_FAST_TIMING_FIXTURE_MAIN retained_smooth_timing_main
#include "route31_fast_timing_test.c"
#undef ROUTE31_FAST_TIMING_FIXTURE_MAIN

static void smooth_frame(const ProtoFrame *frame)
{
    real_proto_stats_get(&host_proto_stats);
    vision_align_test_feed_frame(frame);
}
static void smooth_poll(unsigned ms)
{
    host_tick += ms; wire_sync(); vision_align_test_poll(); wire_sync();
}
static void smooth_at(uint32_t at)
{
    host_tick = at; wire_sync(); vision_align_test_poll(); wire_sync();
}
static int smooth_model(void)
{ return s_vat.task == PROTO_TASK_BUCKET ? 9 : s_vat.task == PROTO_TASK_HOSTAGE ? 0 : 4; }
static int smooth_y(void)
{ return s_vat.task == PROTO_TASK_BUCKET ? VAT_BUCKET_Y_PX : s_vat.task == PROTO_TASK_HOSTAGE ? VAT_HOSTAGE_Y_PX : VAT_BALL_Y_PX; }
static int smooth_near_error(void)
{ return s_vat.task == PROTO_TASK_HOSTAGE ? 19 : 14; }
static int smooth_gate_error(void)
{ return s_vat.task == PROTO_TASK_HOSTAGE ? 20 : 15; }
static void smooth_obj(int x,unsigned ms)
{
    host_tick += ms;
    xy_obj_poll(wire_request,smooth_model(),x,smooth_y(),0); smooth_poll(0u);
}
static void smooth_ack(unsigned result)
{
    uint8_t p[5] = {0x61u,(uint8_t)wire_request,(uint8_t)(wire_request >> 8),2u,(uint8_t)result};
    wire_feed_raw(p,sizeof p,0); wire_sync();
}
static void smooth_rank(void)
{
    uint8_t model=(uint8_t)smooth_model();
    uint8_t p[14] = {0x62u,(uint8_t)wire_request,(uint8_t)(wire_request >> 8),9u,0u,
                    0x54u,0u,0u,model,2u,3u,3u,4u,5u};
    if(model == 0u) {p[11] = 1u; p[12] = 0u; p[13] = 2u;}
    route_wire_le16(p+6u,xy_seq++); wire_feed_raw(p,sizeof p,0); wire_sync();
}
static void smooth_geometry(unsigned width,unsigned height,unsigned x,unsigned sequence)
{
    uint8_t p[30]={0x62u,(uint8_t)wire_request,(uint8_t)(wire_request >> 8),25u,0u,0x01u};
    route_wire_le16(p+6u,sequence); p[8]=1u;
    route_wire_le16(p+9u,width); route_wire_le16(p+11u,height);
    p[19]=(uint8_t)smooth_model(); route_wire_le16(p+20u,900u);
    route_wire_le16(p+22u,x); route_wire_le16(p+24u,(unsigned)smooth_y());
    route_wire_le16(p+26u,24u); route_wire_le16(p+28u,24u);
    host_tick+=VAT_POLL_MS;
    wire_feed_raw(p,sizeof p,0); smooth_poll(0u);
}
static int smooth_begin(unsigned owner,ProtoTask task,int ack)
{
    const int32_t qr[3] = {1,2,3};
    CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get;
    proto_set_on_frame(smooth_frame); xy_seq = 1u;
    CHECK(vision_align_test_start_route_scoped(task == PROTO_TASK_HOSTAGE ? 40u : 41u,
                                              qr,(uint8_t)owner));
    wire_sync();
    if(task == PROTO_TASK_BUCKET) {
        /* Isolate a legitimately completed paired turn at its existing request
         * boundary; do not fabricate arm or road completion evidence. */
        CHECK(vat_request(PROTO_TASK_BUCKET,0u,host_tick)); wire_sync();
        s_route_bucket_eligible = 1u;
    }
    CHECK(wire_opcode == 0x63u && wire_task == (unsigned)task && !wire_bad);
    if(ack) smooth_ack(0u);
    smooth_poll(260u); smooth_poll(20u); smooth_poll(100u);
    return 0;
}
static int smooth_moving(void)
{ return last_x != 0.0f && last_y == 0.0f && !laser_state && !host_timer_active; }
static int smooth_stopped(void)
{ return last_x == 0.0f && last_y == 0.0f && last_w == 0.0f && !laser_state; }
static int smooth_no_action(void)
{
    return !s_vat.ball_fallback && !s_vat.bucket_fallback && !s_vat.hostage_fallback &&
           !s_vat.alignment_confirmed && !host_timer_active && !pulse_calls &&
           vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE;
}

static int check_smooth_ack_far_and_never_seen(void)
{
    const ProtoTask tasks[] = {PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    for(unsigned t=0u;t<3u;++t) {
        CHECK(smooth_begin(31u,tasks[t],0) == 0);
        smooth_obj(s_vat.x_goal+smooth_near_error(),20u);
        CHECK(smooth_stopped() && !s_route31_target.seen && smooth_no_action());
        smooth_ack(0u); smooth_poll(260u); smooth_poll(100u);
        CHECK(smooth_moving() && !s_route31_target.seen && smooth_no_action());
        smooth_poll(5000u);
        CHECK(smooth_moving() && fabsf(last_x) == 30.0f && smooth_no_action());
        uint16_t request=wire_request;
        smooth_obj(619,20u); uint32_t flash=host_tick;
        CHECK(s_route31_target.seen && !s_route31_smooth_near && smooth_moving());
        xy_obj_poll(request,-1,0,0,0); smooth_poll(20u);
        xy_obj_poll((uint16_t)(request-1u),smooth_model(),s_vat.x_goal+smooth_near_error(),smooth_y(),0);
        smooth_poll(20u); smooth_rank(); smooth_poll(20u);
        CHECK(!s_route31_smooth_near && smooth_no_action());
        smooth_at(flash+5000u);
        CHECK(smooth_moving() && fabsf(last_x) == 30.0f && smooth_no_action());
        vision_align_test_cancel(); CHECK(smooth_stopped());
    }
    puts("PASS smooth31: preACK blocked; never-seen/far619/empty/oldreq/rank54 continue slow search beyond5s without action");
    return 0;
}

static int check_smooth_reverse_yaw_and_dropouts(void)
{
    const ProtoTask tasks[] = {PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    for(unsigned t=0u;t<3u;++t) {
        CHECK(smooth_begin(31u,tasks[t],1) == 0);
        for(unsigned i=0u;i<20u;++i) smooth_obj(s_vat.x_goal+smooth_near_error(),20u);
        CHECK(s_route31_smooth_near && last_x == 30.0f && s_vat.state == VAT_FINE_CONTINUOUS);
        host_absolute_yaw=host_yaw=2.0f;
        smooth_obj(s_vat.x_goal+smooth_near_error(),20u);
        CHECK(!s_search_mid.phase && s_vat.state != VAT_BRAKE && smooth_moving());
        CHECK(fabsf(last_w) <= 0.100001f);
        host_absolute_yaw=host_yaw=0.5f;
        smooth_obj(s_vat.x_goal+smooth_near_error(),20u);
        CHECK(fabsf(last_w + 3.0f*0.5f*0.0174533f) < 0.00001f);
        host_absolute_yaw=host_yaw=2.0f;
        float prior=last_x; unsigned crossed=0u;
        for(unsigned i=0u;i<60u;++i) {
            smooth_obj(s_vat.x_goal-smooth_near_error(),20u);
            CHECK(s_vat.state != VAT_BRAKE && s_vat.state != VAT_YAW_FIX && !s_search_mid.phase);
            CHECK(last_x <= prior+0.00001f && fabsf(last_x) <= 30.00001f);
            CHECK(fabsf(last_w) <= fminf(0.10f,0.005f*fabsf(last_x))+0.00001f);
            if(last_x <= 0.0f) crossed=1u;
            prior=last_x;
            if(last_x == -30.0f) break;
        }
        CHECK(crossed && last_x == -30.0f && !s_route31_loss_braking && smooth_no_action());
        uint32_t seen=host_tick;
        xy_obj_poll(wire_request,-1,0,0,0); smooth_poll(20u);
        CHECK(last_x < 0.0f && s_vat.state == VAT_FINE_CONTINUOUS && !s_route31_loss_braking);
        smooth_at(seen+300u);
        CHECK(last_x < 0.0f && !s_route31_loss_braking && smooth_no_action());
        if(tasks[t] == PROTO_TASK_HOSTAGE) {
            smooth_at(seen+1999u);
            CHECK(last_x == -30.0f && !s_route31_loss_braking && smooth_no_action());
        }
        vision_align_test_cancel(); CHECK(smooth_stopped());
    }
    puts("PASS smooth31: near latch, private capped heading,2deg yaw no parked guard; continuous slew reversal;empty/stale retain last direction");
    return 0;
}

static int smooth_expected_action(ProtoTask task)
{ return task == PROTO_TASK_BALL ? VAT_ROUTE_ACTION_BALL : task == PROTO_TASK_BUCKET ? VAT_ROUTE_ACTION_BUCKET : VAT_ROUTE_ACTION_HOSTAGE; }

static int check_smooth_loss_gate_and_deadlines(void)
{
    const ProtoTask tasks[] = {PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    for(unsigned t=0u;t<3u;++t) {
        CHECK(smooth_begin(31u,tasks[t],1) == 0);
        smooth_obj(s_vat.x_goal+smooth_gate_error(),20u); uint32_t far=host_tick;
        CHECK(!s_route31_smooth_near);
        smooth_at(far+5000u); smooth_poll(100u);
        CHECK(smooth_moving() && smooth_no_action());
        smooth_obj(s_vat.x_goal+smooth_near_error(),20u); uint32_t near=host_tick;
        CHECK(s_route31_smooth_near && smooth_moving() && !s_route31_loss_braking);
        unsigned deadline=tasks[t] == PROTO_TASK_HOSTAGE ? 2000u : 301u;
        smooth_at(near+deadline-1u);
        CHECK(!s_route31_loss_braking && smooth_moving() && smooth_no_action());
        smooth_at(near+deadline);
        CHECK(s_route31_loss_braking && smooth_stopped() && !s_vat.alignment_confirmed);
        smooth_at(near+deadline+T_DIST_STILL_MS-1u);
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        smooth_at(near+deadline+T_DIST_STILL_MS);
        CHECK(vision_align_test_take_route_action() == smooth_expected_action(tasks[t]));
        CHECK(!s_vat.alignment_confirmed && smooth_stopped() && !host_timer_active && !pulse_calls);
        vision_align_test_cancel();
    }
    puts("PASS smooth31:ball/bucket abs15 excluded/14 arms;hostage abs20 excluded/19 arms;301/2000 brake,still250 then exact action without fake alignment");
    return 0;
}

static int check_smooth_new_coordinate_renews_pending_loss(void)
{
    CHECK(smooth_begin(31u,PROTO_TASK_HOSTAGE,1) == 0);
    smooth_obj(s_vat.x_goal+smooth_near_error(),20u); uint32_t seen=host_tick;
    smooth_at(seen+2000u); CHECK(s_route31_loss_braking && smooth_stopped());
    smooth_at(host_tick+100u);
    unsigned accepted=s_route31_target.frame.sequence, next=xy_seq;
    xy_seq=accepted; smooth_obj(s_vat.x_goal+100,20u); xy_seq=next;
    CHECK(s_route31_target.at == seen && s_route31_loss_braking);
    smooth_obj(s_vat.x_goal+100,20u); seen=s_route31_target.at;
    CHECK(!s_vat.hostage_fallback && !s_route31_loss_braking && smooth_no_action());
    /* Action braking had already begun; its existing wheel-still barrier is
     * retained before a renewed target resumes. Ordinary dropout never enters
     * this brake path (covered independently above). */
    smooth_poll(T_DIST_STILL_MS); smooth_poll(20u); CHECK(smooth_moving());
    smooth_at(seen+1999u); CHECK(!s_route31_loss_braking && smooth_no_action());
    smooth_at(seen+2000u); CHECK(s_route31_loss_braking && smooth_stopped());
    smooth_at(host_tick+T_DIST_STILL_MS);
    CHECK(s_vat.state == VAT_WAIT_HOSTAGE_ACTION && s_vat.hostage_fallback);
    smooth_obj(s_vat.x_goal+100,20u);
    CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE && !s_vat.hostage_fallback);
    vision_align_test_cancel();
    puts("PASS smooth31:duplicate cannot renew;NEW01 cancels loss brake and pending take;fresh2s deadline restarts");
    return 0;
}

static int check_smooth_five_fresh_arrival(void)
{
    const ProtoTask tasks[] = {PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    for(unsigned t=0u;t<3u;++t) {
        CHECK(smooth_begin(31u,tasks[t],1) == 0);
        smooth_obj(s_vat.x_goal,20u); uint32_t stopped=host_tick;
        CHECK(smooth_stopped() && s_vat.state == VAT_BRAKE && !s_vat.good);
        smooth_at(stopped+T_DIST_STILL_MS-1u);
        CHECK(!s_vat.good && vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        smooth_at(stopped+T_DIST_STILL_MS);
        /* General VAT cadence may defer the exact250 deadline after a249 poll;
         * no shorter hold is accepted. Resume its next eligible20ms poll. */
        smooth_poll(VAT_POLL_MS);
        CHECK(s_vat.state == VAT_RECHECK && !s_vat.good);
        for(unsigned frame=1u;frame<=VAT_GOOD_FRAMES;++frame) {
            smooth_obj(s_vat.x_goal,20u);
            CHECK(smooth_stopped());
            if(frame<VAT_GOOD_FRAMES)
                CHECK(s_vat.good == frame && vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        }
        CHECK((tasks[t] != PROTO_TASK_HOSTAGE || s_vat.alignment_confirmed) &&
              s_vat.good == VAT_GOOD_FRAMES && !s_vat.ball_fallback &&
              !s_vat.bucket_fallback && !s_vat.hostage_fallback);
        CHECK(vision_align_test_take_route_action() == smooth_expected_action(tasks[t]));
        vision_align_test_cancel();
    }
    puts("PASS smooth31:in-band moving frame only brakes;still250 and five NEW stopped images retain real alignment action");
    return 0;
}

static int check_smooth_private_hostage_boundaries(void)
{
    const ProtoTask tasks[]={PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    for(int side=-1;side<=1;side+=2) {
        CHECK(smooth_begin(31u,PROTO_TASK_HOSTAGE,1)==0);
        smooth_obj(s_vat.x_goal+side*20,20u);
        CHECK(!s_route31_smooth_near && !s_route31_hostage_loss_armed);
        smooth_poll(5000u); smooth_poll(100u);
        CHECK(s_vat.state==VAT_FINE_CONTINUOUS && !s_route31_smooth_near && smooth_no_action());
        smooth_obj(s_vat.x_goal+side*19,20u);
        CHECK(s_route31_smooth_near && s_route31_hostage_loss_armed &&
              s_vat.state==VAT_FINE_CONTINUOUS && !s_route31_loss_braking && smooth_no_action());
        vision_align_test_cancel();
    }
    for(unsigned t=0u;t<3u;++t) for(int side=-1;side<=1;side+=2) {
        const int tolerance=tasks[t]==PROTO_TASK_HOSTAGE?15:10;
        CHECK(smooth_begin(31u,tasks[t],1)==0);
        CHECK(vat_x_tolerance_px()==tolerance &&
              vat_fine_error_px()==(tasks[t]==PROTO_TASK_HOSTAGE?20:15));
        if(tasks[t]==PROTO_TASK_HOSTAGE) CHECK(s_vat.x_goal==210);
        const int inside=s_vat.x_goal+side*tolerance;
        if(tasks[t]==PROTO_TASK_HOSTAGE) CHECK(inside==(side<0?195:225));
        smooth_obj(inside,20u); uint32_t stopped=host_tick;
        CHECK(smooth_stopped() && s_vat.state==VAT_BRAKE && !s_vat.good);
        smooth_at(stopped+T_DIST_STILL_MS);
        CHECK(s_vat.state==VAT_RECHECK && !s_vat.good);
        for(unsigned frame=1u;frame<=VAT_GOOD_FRAMES;++frame) {
            smooth_obj(inside,20u);
            CHECK(smooth_stopped());
            if(frame<VAT_GOOD_FRAMES)
                CHECK(s_vat.good==frame && vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        }
        CHECK(s_vat.good==VAT_GOOD_FRAMES && !s_vat.ball_fallback &&
              !s_vat.bucket_fallback && !s_vat.hostage_fallback);
        CHECK(vision_align_test_take_route_action()==smooth_expected_action(tasks[t]));
        vision_align_test_cancel();

        CHECK(smooth_begin(31u,tasks[t],1)==0);
        const int outside=s_vat.x_goal+side*(tolerance+1);
        if(tasks[t]==PROTO_TASK_HOSTAGE) CHECK(outside==(side<0?194:226));
        for(unsigned frame=0u;frame<VAT_GOOD_FRAMES+2u;++frame) {
            smooth_obj(outside,20u);
            CHECK(s_vat.state==VAT_FINE_CONTINUOUS && s_route31_smooth_near &&
                  !s_vat.good && !s_vat.alignment_confirmed && smooth_no_action());
        }
        vision_align_test_cancel();
    }
    for(unsigned owner=43u;owner<=44u;++owner) {
        CHECK(smooth_begin(owner,PROTO_TASK_HOSTAGE,1)==0);
        CHECK(s_vat.x_goal==215 && vat_x_tolerance_px()==10 && vat_fine_error_px()==15 &&
              !vat_route31_smooth_enabled());
        vision_align_test_cancel();
    }
    puts("PASS smooth31:hostage strict+-20 gate excluded/+-19 arms;x210195/225 inclusive stopped5frame arrival,194/226 onlyfine;ball/bucket+-10 and43/44 hostage215/+-10/fine15 retained");
    return 0;
}

static int check_smooth_geometry_freshness(void)
{
    const ProtoTask tasks[]={PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    for(unsigned t=0u;t<3u;++t) {
        CHECK(smooth_begin(31u,tasks[t],1)==0);
        smooth_obj(619,20u);
        unsigned accepted=s_route31_target.frame.sequence; uint32_t at=s_route31_target.at;
        smooth_geometry(800u,480u,(unsigned)(s_vat.x_goal+smooth_near_error()),accepted);
        CHECK(s_vat.state != VAT_STOPPED && !s_route31_smooth_near &&
              s_route31_target.at==at && s_vat.state==VAT_FINE_CONTINUOUS && smooth_no_action());
        smooth_geometry(800u,480u,(unsigned)(s_vat.x_goal+smooth_near_error()),(uint16_t)(accepted-1u));
        CHECK(s_vat.state != VAT_STOPPED && !s_route31_smooth_near &&
              s_route31_target.at==at && s_vat.state==VAT_FINE_CONTINUOUS && smooth_no_action());
        /* Out-of-image coordinates fail the real CRC parser's field contract;
         * absence must not retarget, arm loss or create geometry panic. */
        smooth_geometry(640u,480u,640u,accepted+1u);
        CHECK(s_vat.state != VAT_STOPPED && s_route31_target.at==at &&
              !s_route31_smooth_near && s_vat.state==VAT_FINE_CONTINUOUS && smooth_no_action());
        smooth_geometry(800u,480u,(unsigned)(s_vat.x_goal+smooth_near_error()),accepted+2u);
        CHECK(s_vat.state==VAT_STOPPED && smooth_stopped() && !s_receiving &&
              !strcmp(s_vat.reason,"IMAGE_GEOMETRY") &&
              vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        smooth_obj(s_vat.x_goal,20u); smooth_poll(4000u);
        CHECK(s_vat.state==VAT_STOPPED && smooth_stopped() && !host_timer_active && !pulse_calls);
    }
    puts("PASS smooth31:old-sequence changed geometry and parser-invalid cx are absence/no arm;NEW selected changed geometry aborts;late frame inert");
    return 0;
}

static int check_smooth_stop_and_scope(void)
{
    const char *keys[] = {"g","a","0"};
    for(unsigned fault=0u;fault<7u;++fault) {
        CHECK(smooth_begin(31u,PROTO_TASK_HOSTAGE,1) == 0);
        smooth_obj(s_vat.x_goal+smooth_near_error(),20u);
        if(fault<3u) {
            s_seq_mode=31u; s_seq_state=SQ_TASK; s_seq_stage=ROUTE31_HOSTAGE_STAGE; s_msel=31;
            run_cmd(keys[fault]); wire_sync();
        } else if(fault == 3u) smooth_ack(1u);
        else if(fault == 4u) host_imu_valid=0;
        else if(fault == 5u) host_absolute_yaw=NAN;
        else host_abort=1;
        smooth_poll(20u);
        CHECK(s_vat.state == VAT_STOPPED && smooth_stopped() && !s_receiving);
        uint16_t old=wire_request;
        xy_obj_poll(old,0,VAT_ROUTE31_HOSTAGE_X_PX,VAT_HOSTAGE_Y_PX,0);
        smooth_poll(4000u);
        CHECK(s_vat.state == VAT_STOPPED && smooth_stopped() &&
              vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
    }
    for(unsigned owner=43u;owner<=44u;++owner) {
        CHECK(smooth_begin(owner,PROTO_TASK_BALL,1) == 0);
        CHECK(!vat_route31_smooth_enabled());
        smooth_obj(s_vat.x_goal+smooth_near_error(),20u);
        CHECK(s_vat.state == VAT_BRAKE && smooth_stopped());
        vision_align_test_cancel();
    }
    CHECK(wire_boot() == 0); host_wire_diag_hook=real_proto_wire_diag_get; xy_seq=1u;
    run_cmd("35"); run_cmd("g"); wire_sync();
    wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123"); wire_sync();
    CHECK(wire_task == 2u && !vat_route31_smooth_enabled());
    wire_ack(wire_request,2u,0u);
    for(unsigned i=0u;i<100u && s_target35_phase != TA_SEEK;++i) {host_tick+=20u;wire_poll();}
    CHECK(s_target35_phase == TA_SEEK && target35_point() == 255);
    for(unsigned i=0u;i<5u;++i) {host_tick+=20u;xy_obj(wire_request,8,target35_point(),160);}
    CHECK(s_target35_phase == TA_SETTLE && !laser_state);
    run_cmd("0"); CHECK(!laser_state && !last_x && !last_y && !last_w);
    puts("PASS smooth31:g/a/0/NACK/invalidIMU/NaN/abort stop and lateframes inert;43/44 original gate and35 x255/1s retained");
    return 0;
}

int main(void)
{
    if(check_smooth_ack_far_and_never_seen() || check_smooth_reverse_yaw_and_dropouts() ||
       check_smooth_loss_gate_and_deadlines() || check_smooth_new_coordinate_renews_pending_loss() ||
       check_smooth_five_fresh_arrival() || check_smooth_private_hostage_boundaries() ||
       check_smooth_geometry_freshness() ||
       check_smooth_stop_and_scope()) return 1;
    puts("route31_smooth_vision_test:host software only;no physical trajectory/grasp acceptance");
    return 0;
}
