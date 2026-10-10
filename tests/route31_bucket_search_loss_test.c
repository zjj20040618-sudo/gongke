/* Owner43 legacy bucket contract: real CRC/request parser + real VAT module.
 * Current31 continuousX/strict300ms coverage is in the dedicated31 fixtures;
 * Reuse the existing module mocks, not its older bucket assertions.
 * Pixels, yaw and encoders are injected; this is not hardware acceptance. */
#define main inherited_vat_module_fixture_main
#include "vision_align_test_test.c"
#undef main

static int bs_bucket_unacked_rank(unsigned position)
{
    CHECK(reset_route_scoped(41u,43u));
    route_auto_ball_rank=(int)position;
    CHECK(vision_align_test_route_search_speed_set(200.0f));
    CHECK(vision_align_test_route_search_kp_set(3.0f));
    CHECK(vision_align_test_route_search_ff_set(0.03f));
    CHECK(ready()); CHECK(finish(4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX));
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
    vision_align_test_notify_route_action_result(1); proto_service();
    CHECK(last_task==PROTO_TASK_BUCKET && st.state==VAT_WAIT_BALL_ACTION);
    vision_align_test_status(&st);
    CHECK(st.state==VAT_TURN_REQUEST && st.x_goal==125 && !st.bucket_seen);
    uint16_t preturn=request();
    ack(preturn,2u,0u);
    object(preturn,9,125,VAT_BUCKET_Y_PX,640u,480u,seq++); tick(1000u);
    CHECK(st.state==VAT_TURN_REQUEST && !st.bucket_seen && stopped());
    CHECK(vision_align_test_take_turn_request());
    w=1.2f; tick(5000u); CHECK(st.state==VAT_TURN_ACTIVE && w==1.2f);
    yaw=217.0f; vision_align_test_notify_turn_result(1); proto_service();
    vision_align_test_status(&st);
    CHECK(st.state==VAT_BRAKE && st.x_goal==125 && st.task==PROTO_TASK_BUCKET);
    CHECK(request()!=preturn && !st.bucket_seen && !st.bucket_fallback && stopped());
    return 1;
}
static int bs_bucket_unacked(void) { return bs_bucket_unacked_rank(1u); }
static int bs_bucket_ready(void)
{
    CHECK(bs_bucket_unacked()); ack(request(),2u,0u); tick(260u);
    CHECK(st.state==VAT_RECHECK && stopped()); return 1;
}
static int bs_start_backward(void)
{
    CHECK(bs_bucket_ready()); tick(2000u);
    CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH && vx==0.0f && vy==0.0f);
    tick(20u); CHECK(fabsf(vx+14.0f)<0.0001f && vy==0.0f);
    tick(280u); CHECK(vx==-200.0f && vy==0.0f && w==0.0f);
    CHECK(!st.bucket_seen && !st.bucket_fallback && !st.step);
    return 1;
}
static int bs_pending_loss(int x)
{
    CHECK(bs_start_backward());
    object(request(),9,x,VAT_BUCKET_Y_PX,640u,480u,seq++); tick(20u);
    CHECK(st.bucket_seen && st.bucket_age_ms==20u && !st.bucket_fallback);
    tick(1980u); CHECK(st.state==VAT_BRAKE && stopped() && !st.good);
    tick(250u);
    CHECK(st.state==VAT_WAIT_BUCKET_ACTION && st.bucket_fallback && !st.alignment_confirmed && !st.good);
    return 1;
}

static int bs_ack_never_seen_and_far_gate(void)
{
    CHECK(bs_bucket_unacked()); uint16_t current=request();
    object(current,9,125,VAT_BUCKET_Y_PX,640u,480u,seq++);
    ack((uint16_t)(current-1u),2u,0u); tick(1000u);
    object((uint16_t)(current-1u),9,125,VAT_BUCKET_Y_PX,640u,480u,seq++);
    tick(5000u); CHECK(st.state==VAT_RECHECK && !st.bucket_seen && stopped());
    ack(current,2u,0u); tick(20u);
    object(current,-1,0,0,640u,480u,seq++);
    object(current,0,215,VAT_HOSTAGE_Y_PX,640u,480u,seq++);
    tick(1999u); CHECK(st.state==VAT_RECHECK && stopped());
    tick(1u); CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH && vx==0.0f);
    tick(300u); CHECK(vx==-200.0f && vy==0.0f && !st.bucket_seen);
    tick(5000u); CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH && !st.bucket_fallback);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);

    object(current,9,500,VAT_BUCKET_Y_PX,640u,480u,400u); tick(20u);
    CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH && vx==-200.0f && st.bucket_seen && !st.good);
    /* Duplicate/backward crossings cannot latch fine or refresh loss. */
    object(current,9,154,VAT_BUCKET_Y_PX,640u,480u,400u); tick(20u);
    object(current,9,154,VAT_BUCKET_Y_PX,640u,480u,399u); tick(20u);
    CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH && st.bucket_age_ms==60u && vx==-200.0f);
    yaw=217.5f;leg_yaw=5.0f; tick(20u);
    CHECK(w<0.0f && heading_hold_kp==3.0f && fabsf(heading_hold_target-4.5f)<0.0001f);
    object(current,9,154,VAT_BUCKET_Y_PX,640u,480u,401u);
    object(current,-1,0,0,640u,480u,402u); tick(1u);
    CHECK(st.state==VAT_BRAKE && stopped() && !st.good && st.yaw_dirty && st.yaw_ever);
    CHECK(stop_recheck()); CHECK(st.yaw_target==217.0f);
    object(current,9,500,VAT_BUCKET_Y_PX,640u,480u,403u); tick(20u);
    CHECK(st.state==VAT_STEP_MOVE && vx==20.0f && vy==0.0f && w==0.0f);
    CHECK(end_step_by_odometry()); CHECK(stop_recheck());
    /* Once fine latched, old far coordinates cannot reopen coarse movement. */
    CHECK(st.state==VAT_RECHECK && stopped() && !st.good);
    seq=404u; CHECK(finish(9,125,VAT_BUCKET_Y_PX));
    CHECK(st.state==VAT_WAIT_BUCKET_ACTION && !st.bucket_fallback && !st.alignment_confirmed);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BUCKET);
    vision_align_test_notify_route_action_result(1); tick(20u);
    CHECK(st.state==VAT_DONE && st.good==VAT_GOOD_FRAMES && !st.bucket_fallback && stopped());
    return 1;
}
static int bs_loss_wrap_recovery_and_commit(void)
{
    CHECK(bs_start_backward()); now_ms=UINT32_MAX-1000u;
    object(request(),9,154,VAT_BUCKET_Y_PX,640u,480u,65534u); tick(20u);
    object(request(),9,154,VAT_BUCKET_Y_PX,640u,480u,65535u); tick(20u);
    object(request(),9,154,VAT_BUCKET_Y_PX,640u,480u,0u); tick(20u);
    CHECK(st.bucket_seen && st.bucket_age_ms==20u && stopped() && !st.alignment_confirmed);
    object(request(),9,125,VAT_BUCKET_Y_PX,640u,480u,65535u);
    object(request(),9,125,VAT_BUCKET_Y_PX,640u,480u,0u);
    object(request(),-1,0,0,640u,480u,1u);
    rank_frame(request(),2u,9u,1u,1u,9u,255u,255u);
    tick(1979u); CHECK(st.bucket_age_ms==1999u && st.state==VAT_RECHECK);
    tick(1u); CHECK(st.bucket_age_ms==2000u && st.state==VAT_BRAKE && stopped());
    tick(249u); CHECK(st.state==VAT_BRAKE && vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    tick(1u); CHECK(st.state==VAT_WAIT_BUCKET_ACTION && st.bucket_fallback && !st.good && !st.alignment_confirmed);
    vision_align_test_notify_route_action_result(1); vision_align_test_status(&st);
    CHECK(st.state==VAT_WAIT_BUCKET_ACTION); /* No owner take: no release completion. */
    object(request(),9,500,VAT_BUCKET_Y_PX,640u,480u,3u);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    vision_align_test_status(&st);
    CHECK(st.state==VAT_BRAKE && !st.bucket_fallback && st.bucket_age_ms==0u && stopped());
    tick(1999u); CHECK(st.state==VAT_RECHECK && stopped());
    tick(1u); CHECK(st.state==VAT_BRAKE && st.bucket_age_ms==2000u);
    counts[2]++; tick(249u); CHECK(st.state==VAT_BRAKE);
    tick(249u); CHECK(st.state==VAT_BRAKE && vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    tick(1u); CHECK(st.state==VAT_WAIT_BUCKET_ACTION && st.bucket_fallback);
    /* Movement between pending and take also resets the250ms gate. */
    counts[0]++; CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    tick(249u); CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    tick(1u); CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BUCKET);
    object(request(),9,125,VAT_BUCKET_Y_PX,640u,480u,4u); tick(20u);
    CHECK(st.state==VAT_WAIT_BUCKET_ACTION && st.bucket_fallback && !st.alignment_confirmed);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    vision_align_test_notify_route_action_result(1); tick(20u);
    CHECK(st.state==VAT_DONE && st.bucket_seen && st.bucket_fallback && !st.alignment_confirmed && !st.good && stopped());
    CHECK(strcmp(st.reason,"BUCKET_LOST2S_RELEASED")==0);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    return 1;
}
static int bs_recovery_brake_and_faults(void)
{
    CHECK(bs_pending_loss(154));
    vision_align_test_status(&st); uint32_t age=st.bucket_age_ms;
    object(request(),9,154,VAT_BUCKET_Y_PX,640u,480u,(uint16_t)(seq-1u));
    tick(20u); CHECK(st.bucket_fallback && st.bucket_age_ms==age+20u);
    /* A NEW crossing received during uncommitted loss brake cancels it. */
    object(request(),9,125,VAT_BUCKET_Y_PX,640u,480u,seq++); tick(20u);
    CHECK(st.state==VAT_BRAKE && !st.bucket_fallback && stopped());
    tick(260u); CHECK(st.state==VAT_RECHECK && !st.good);
    CHECK(finish(9,125,VAT_BUCKET_Y_PX));
    CHECK(st.state==VAT_WAIT_BUCKET_ACTION && !st.bucket_fallback);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BUCKET);
    vision_align_test_notify_route_action_result(1); tick(20u); CHECK(st.state==VAT_DONE);
    for(unsigned f=0u;f<7u;f++) {
        CHECK(bs_pending_loss(154));
        if(f==0u)vision_align_test_cancel();
        else if(f==1u)aborted=1;
        else if(f==2u)imu_valid=0;
        else if(f==3u)yaw=NAN;
        else if(f==4u)ack(request(),2u,1u);
        else if(f==5u)object(request(),9,100,VAT_BUCKET_Y_PX,115u,480u,seq++);
        else {CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BUCKET);ack(request(),2u,1u);vision_align_test_notify_route_action_result(1);}
        tick(1u); CHECK(st.state==VAT_STOPPED && stopped() && !st.good);
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    }
    return 1;
}
static int bs_independent_isolation(void)
{
    CHECK(reset(39u)); CHECK(st.x_goal==190); CHECK(ready()); tick(10000u);
    CHECK(st.state==VAT_ALIGN && !st.bucket_seen && !st.bucket_fallback && stopped());
    frame(9,500,VAT_BUCKET_Y_PX); CHECK(st.state==VAT_STEP_MOVE && vx==20.0f);
    CHECK(end_step_by_odometry()); CHECK(stop_recheck()); tick(2500u);
    CHECK(st.state==VAT_RECHECK && !st.bucket_fallback && stopped());
    vision_align_test_cancel();
    CHECK(reset(41u)); CHECK(ready()); CHECK(finish(4,190,VAT_BALL_Y_PX));
    tick(5000u); CHECK(vision_align_test_take_turn_request());yaw=217.0f;
    vision_align_test_notify_turn_result(1);proto_service();ack(request(),2u,0u);tick(260u);
    CHECK(st.x_goal==190);tick(10000u);
    CHECK(st.state==VAT_RECHECK && !st.bucket_seen && !st.bucket_fallback && stopped());
    vision_align_test_cancel();return 1;
}
static int bs_rank_direction_far_loss_and_fine_boundary(void)
{
    for(unsigned position=1u;position<=3u;position++) {
        CHECK(bs_bucket_unacked_rank(position));
        CHECK(st.ball_rank==position); ack(request(),2u,0u); tick(260u);
        CHECK(st.state==VAT_RECHECK&&stopped());tick(2000u);
        if(position==2u) {
            CHECK(st.state==VAT_RECHECK&&stopped());tick(10000u);
            CHECK(st.state==VAT_RECHECK&&stopped()&&!st.bucket_fallback);
            object(request(),9,500,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
            CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==0.0f);
            tick(20u);CHECK(fabsf(vx-14.0f)<0.001f&&vy==0.0f);
            tick(301u);CHECK(stopped()&&!st.bucket_fallback);
        } else {
            CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==0.0f);tick(20u);
            CHECK(fabsf(vx-(position==1u?-14.0f:14.0f))<0.001f&&vy==0.0f);
            tick(280u);CHECK(vx==(position==1u?-200.0f:200.0f));
            object(request(),9,500,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
            CHECK(vx==(position==1u?-200.0f:200.0f)&&st.state==VAT_ROUTE_BUCKET_SEARCH);
            tick(301u);CHECK(stopped()&&!st.bucket_fallback);
            tick(1679u);CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==0.0f&&!st.bucket_fallback);
            tick(300u);
            CHECK(vx==(position==1u?-200.0f:200.0f));
        }
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        vision_align_test_cancel();
    }
    for(unsigned edge=0u;edge<2u;edge++) {
        CHECK(bs_start_backward());int far=edge?155:95;
        object(request(),9,far,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
        CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&!st.bucket_fallback&&vx==-200.0f);
        object(request(),9,edge?154:96,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
        CHECK(st.state==VAT_BRAKE&&stopped());CHECK(stop_recheck());
        object(request(),9,500,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
        CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);vision_align_test_cancel();
    }
    return 1;
}
static int bs_coarse_drop_wait2s_exact_boundaries_and_recovery(void)
{
    for(unsigned position=1u;position<=3u;position++) {
        CHECK(bs_bucket_unacked_rank(position));ack(request(),2u,0u);tick(260u);tick(2000u);tick(300u);
        uint16_t far_sequence=seq++;object(request(),9,500,VAT_BUCKET_Y_PX,640u,480u,far_sequence);tick(20u);
        tick(20u);tick(260u);
        CHECK(st.bucket_age_ms==300u&&st.state==VAT_ROUTE_BUCKET_SEARCH&&fabsf(vx)>0.0f&&!st.bucket_fallback);
        CHECK(position==1u?vx<0.0f:vx>0.0f);
        tick(1u);CHECK(st.bucket_age_ms==301u&&stopped()&&!st.bucket_fallback);
        CHECK(!strcmp(st.reason,"BUCKET_WAIT2_NO_NEW_FRAME"));
        /* Replay, empty01 and inappropriate54 are not fresh bucket coordinates. */
        object(request(),9,500,VAT_BUCKET_Y_PX,640u,480u,far_sequence);
        object(request(),-1,0,0,640u,480u,seq++);
        rank_frame(request(),seq++,9u,1u,1u,9u,255u,255u);
        tick(1698u);CHECK(st.bucket_age_ms==1999u&&stopped()&&!st.bucket_fallback);
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        tick(1u);CHECK(st.bucket_age_ms==2000u&&stopped()&&!st.bucket_fallback);
        if(position==2u) {
            CHECK(!strcmp(st.reason,"BUCKET_MIDDLE_WAIT_FRAME"));tick(5000u);CHECK(stopped());
        } else {
            tick(20u);CHECK(fabsf(vx-(position==1u?-14.0f:14.0f))<0.001f&&vy==0.0f);
            tick(280u);CHECK(vx==(position==1u?-200.0f:200.0f));
        }
        vision_align_test_cancel();
    }
    /* A NEW far frame during the paused interval restarts at0, not stale full cruise. */
    CHECK(bs_start_backward());frame(9,500,VAT_BUCKET_Y_PX);tick(301u);CHECK(stopped());
    frame(9,95,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==0.0f);
    tick(20u);CHECK(fabsf(vx+14.0f)<0.001f&&vy==0.0f);
    frame(9,154,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_BRAKE&&stopped());CHECK(stop_recheck());
    tick(1720u);CHECK(st.state==VAT_BRAKE&&st.bucket_age_ms==2000u);
    tick(250u);CHECK(st.state==VAT_WAIT_BUCKET_ACTION&&st.bucket_fallback);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BUCKET);
    vision_align_test_cancel();return 1;
}
static int bs_rank1_and_rank3_far_either_side_keep_direction(void)
{
    for(unsigned position=1u;position<=3u;position+=2u) {
        for(unsigned edge=0u;edge<2u;edge++) {
            CHECK(bs_bucket_unacked_rank(position));ack(request(),2u,0u);tick(260u);tick(2000u);tick(300u);
            CHECK(vx==(position==1u?-200.0f:200.0f));
            object(request(),9,edge?155:95,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
            CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==(position==1u?-200.0f:200.0f)&&!st.good);
            object(request(),9,edge?500:40,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
            CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==(position==1u?-200.0f:200.0f));
            object(request(),9,edge?154:96,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
            CHECK(st.state==VAT_BRAKE&&stopped());CHECK(stop_recheck());
            frame(9,edge?154:96,VAT_BUCKET_Y_PX);
            CHECK(st.state==VAT_STEP_MOVE&&vx==(edge?20.0f:-20.0f)&&vy==0.0f);
            vision_align_test_cancel();
        }
    }
    return 1;
}
int main(void)
{
    if(!bs_ack_never_seen_and_far_gate() || !bs_loss_wrap_recovery_and_commit()
       || !bs_recovery_brake_and_faults() || !bs_independent_isolation()
       || !bs_rank_direction_far_loss_and_fine_boundary()
       || !bs_coarse_drop_wait2s_exact_boundaries_and_recovery()
       || !bs_rank1_and_rank3_far_either_side_keep_direction()
       || !check_route31_gate15_scoped43_gate30_isolation()) return 1;
    puts("bucket43 legacy host: X125 strict gate95/155 coarse,96/154 fine;rank1 alwaysback/rank3 alwaysforward whilefar/rank2 freshcx-or-wait; ACK2s,owner200 ramp/noFF;fresh300/301..1999 stops/2000 rank-resumes-ramp;strict abs(error)<30 sticky fine20;coarse loss no release/fine seenlost2s release/still250/recovery+take;seq+tickwrap/replay/empty/54/ACK/abort/IMU/geometry,standalone39/41 isolated passed; current31 continuousX/lost300 covered separately; no hardware acceptance");
    return 0;
}
