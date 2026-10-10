/* Actual VAT + CRC/request parser; injected wheels/yaw/images, not a robot
 * plant. Compile with App/vision_align_test.c App/proto.c, ordinary and -O2. */
#define main mid_yaw_prior_fixture_main
#define motion_vel_set_precise mid_yaw_fixture_precise
#include "vision_align_test_test.c"
#undef motion_vel_set_precise
#undef main
#include "route_test_plan.h"

static unsigned mid_drive_calls;
void motion_vel_set_precise(float x, float y, float turn)
{ ++mid_drive_calls; mid_yaw_fixture_precise(x,y,turn); }

static int mid_model(unsigned task)
{ return task==PROTO_TASK_BUCKET?9:task==PROTO_TASK_HOSTAGE?0:4; }
static int mid_goal(unsigned task)
{ return task==PROTO_TASK_BUCKET?105:task==PROTO_TASK_HOSTAGE?215:135; }
static int mid_y(unsigned task)
{ return task==PROTO_TASK_BUCKET?VAT_BUCKET_Y_PX:task==PROTO_TASK_HOSTAGE?VAT_HOSTAGE_Y_PX:VAT_BALL_Y_PX; }

static int mid_start_rank(unsigned task,int direction,unsigned owner,unsigned ball_rank)
{
    CHECK(reset_route_scoped(task==PROTO_TASK_HOSTAGE?40u:41u,owner));
    CHECK(vision_align_test_route_search_kp_set(3.0f));
    CHECK(vision_align_test_route_search_speed_set(100.0f));
    CHECK(vision_align_test_route_yaw_settle_ms_set(owner==31u?400u:700u));
    if(task!=PROTO_TASK_BUCKET) CHECK(vision_align_test_route_search_ff_set(0.03f));
    if(task==PROTO_TASK_BUCKET) route_auto_ball_rank=(int)ball_rank;
    CHECK(ready());
    if(task==PROTO_TASK_BUCKET){
        CHECK(finish(4,135,VAT_BALL_Y_PX));
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
        vision_align_test_notify_route_action_result(1);
        CHECK(vision_align_test_take_turn_request());
        yaw=217.0f; vision_align_test_notify_turn_result(1); proto_service();
        ack(request(),2u,0u); tick(260u); CHECK(st.state==VAT_RECHECK&&stopped());
    }
    int goal=owner==43u&&task==PROTO_TASK_BUCKET?125:mid_goal(task);
    frame(mid_model(task),goal+40*direction,mid_y(task));
    CHECK(st.state==(task==PROTO_TASK_BUCKET?VAT_ROUTE_BUCKET_SEARCH:VAT_ROUTE_SEARCH));
    tick(20u); CHECK(vx*(float)direction>0.0f&&!st.good&&!imu_zero);
    return 1;
}
static int mid_start(unsigned task,int direction,unsigned owner)
{ return mid_start_rank(task,direction,owner,direction<0?1u:3u); }
static int mid_trigger(float error)
{
    yaw=st.yaw_target+error; tick(20u);
    CHECK(st.state==VAT_BRAKE&&stopped()&&strcmp(st.reason,"SEARCH_MID_YAW_BRAKE")==0);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    return 1;
}
static int mid_begin_fix(void)
{
    tick(249u); CHECK(st.state==VAT_BRAKE&&stopped());
    tick(1u); CHECK(st.state==VAT_YAW_FIX&&vx==0.0f&&vy==0.0f&&fabsf(w)>=0.30f);
    return 1;
}
static int mid_restore(void)
{
    yaw=st.yaw_target; tick(20u); CHECK(st.state==VAT_YAW_FIX&&stopped());
    tick(399u); CHECK(st.state==VAT_YAW_FIX&&stopped());
    tick(1u); CHECK(st.state==VAT_ROUTE_SEARCH||st.state==VAT_ROUTE_BUCKET_SEARCH);
    CHECK(stopped()&&!st.good&&!st.latest&&!imu_zero);
    CHECK(strcmp(st.reason,"SEARCH_MID_YAW_RESUME_WAIT_NEW")==0);
    return 1;
}

static int check_all_tasks_directions_and_new_frames(void)
{
    const unsigned tasks[]={PROTO_TASK_BALL,PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE};
    for(unsigned k=0u;k<3u;k++)for(int direction=-1;direction<=1;direction+=2){
        unsigned task=tasks[k]; CHECK(mid_start(task,direction,31u));
        float original=st.yaw_target; uint16_t req=request(); unsigned tx_before=request_count;
        yaw=original+1.5f; tick(20u);
        CHECK(st.state==(task==PROTO_TASK_BUCKET?VAT_ROUTE_BUCKET_SEARCH:VAT_ROUTE_SEARCH));
        CHECK(vx*(float)direction>0.0f); /* Strictly >, not >=. */
        CHECK(mid_trigger(direction>0?1.51f:-1.51f));
        /* Wheel coasting restarts the initial250ms brake gate. */
        counts[0]++; tick(20u); CHECK(st.state==VAT_BRAKE&&stopped());
        CHECK(mid_begin_fix()); CHECK(w*(float)direction<0.0f);
        uint16_t rotating_seq=seq++;
        object(req,mid_model(task),mid_goal(task),mid_y(task),640u,480u,rotating_seq); tick(20u);
        CHECK(st.state==VAT_YAW_FIX&&!st.good&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        tick(301u); CHECK(st.state==VAT_YAW_FIX&&!st.good&&vx==0.0f&&vy==0.0f);
        CHECK(mid_restore()); CHECK(st.yaw_target==original&&request()==req&&request_count==tx_before);
        tick(20u); CHECK(vx*(float)direction>0.0f&&!st.good&&!st.latest);
        /* A replay of the rotating in-band frame cannot brake/align/grab. */
        object(req,mid_model(task),mid_goal(task),mid_y(task),640u,480u,rotating_seq); tick(20u);
        CHECK(st.state==(task==PROTO_TASK_BUCKET?VAT_ROUTE_BUCKET_SEARCH:VAT_ROUTE_SEARCH));
        CHECK(vx*(float)direction>0.0f&&!st.good&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        object(req,mid_model(task),mid_goal(task),mid_y(task),640u,480u,(uint16_t)(rotating_seq-1u));tick(20u);
        CHECK(vx*(float)direction>0.0f&&!st.good);
        tick(301u); CHECK(vx*(float)direction>0.0f&&!st.good&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        /* Truly NEW selected pixels latch fine, but do not replace five
         * stopped NEW frames. The original task/request/rank survives. */
        frame(mid_model(task),mid_goal(task),mid_y(task));
        CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good);
        CHECK(stop_recheck()); CHECK(st.yaw_target==original);
        CHECK(finish(mid_model(task),mid_goal(task),mid_y(task)));
        CHECK(st.good==VAT_GOOD_FRAMES&&stopped());
        CHECK(st.alignment_confirmed==(task==PROTO_TASK_HOSTAGE?1u:0u));
        CHECK(st.state==(task==PROTO_TASK_BALL?VAT_WAIT_BALL_ACTION:task==PROTO_TASK_BUCKET?VAT_WAIT_BUCKET_ACTION:VAT_WAIT_HOSTAGE_ACTION));
        vision_align_test_cancel(); CHECK(stopped());
    }
    puts("mid-search31: three tasks x forward/back; >1.5 brake/still/original-yaw400, no LOST300/action while fixing, original direction blind resume, rotating/replayed pixels rejected, NEW stopped five-frame arrival passed");
    return 1;
}
static int check_timeout_residual_and_hysteresis(void)
{
    CHECK(mid_start(PROTO_TASK_BALL,1,31u)); CHECK(mid_trigger(2.0f)); CHECK(mid_begin_fix());
    tick(1999u); CHECK(st.state==VAT_YAW_FIX&&vx==0.0f&&vy==0.0f);
    tick(1u); CHECK(st.state==VAT_STOPPED&&stopped()&&strcmp(st.reason,"SEARCH_MID_YAW_TIMEOUT_GT1P5")==0);
    unsigned calls=mid_drive_calls; tick(10000u); CHECK(mid_drive_calls==calls&&st.state==VAT_STOPPED&&stopped());

    CHECK(mid_start(PROTO_TASK_HOSTAGE,-1,31u)); CHECK(mid_trigger(-2.0f)); CHECK(mid_begin_fix());
    yaw=st.yaw_target-1.5f; tick(2000u);
    CHECK(st.state==VAT_YAW_FIX&&stopped()&&strcmp(st.reason,"SEARCH_MID_YAW_ACCEPT_STILL")==0);
    tick(249u); CHECK(stopped()&&st.state==VAT_YAW_FIX); tick(1u);
    CHECK(st.state==VAT_ROUTE_SEARCH&&stopped()&&strcmp(st.reason,"SEARCH_MID_YAW_ACCEPT_WAIT_NEW")==0);
    tick(20u); CHECK(vx<0.0f&&st.state==VAT_ROUTE_SEARCH); /* No immediate same-error re-trigger at1.5. */
    yaw=st.yaw_target-1.51f;tick(20u);CHECK(st.state==VAT_BRAKE&&stopped());
    vision_align_test_cancel();
    puts("mid-search31: exact1999/2000 timeout, residual>1.5 terminal/no restart, <=1.5 accepted only after250ms wheel stop, strict threshold avoids same-error instant looping passed");
    return 1;
}
static int check_middle_bucket_and_geometry_barriers(void)
{
    for(int direction=-1;direction<=1;direction+=2){
        CHECK(mid_start_rank(PROTO_TASK_BUCKET,direction,31u,2u));
        CHECK(st.ball_rank==2u); CHECK(mid_trigger(2.0f)); CHECK(mid_begin_fix());
        CHECK(mid_restore());tick(20u);
        CHECK(st.ball_rank==2u&&vx*(float)direction>0.0f);
        /* Clearing turn pixels must not turn an already-running middle search
         * into its INITIAL no-frame wait, or let empty/wrongtarget arm it. */
        object(request(),-1,0,0,640u,480u,seq++);tick(20u);
        object(request(),4,135,VAT_BALL_Y_PX,640u,480u,seq++);tick(20u);
        tick(301u); CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx*(float)direction>0.0f);
        CHECK(!st.good&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        frame(9,105,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_BRAKE&&stopped());
        CHECK(stop_recheck());CHECK(finish(9,105,VAT_BUCKET_Y_PX));
        CHECK(st.state==VAT_WAIT_BUCKET_ACTION&&st.good==VAT_GOOD_FRAMES&&st.ball_rank==2u);
        vision_align_test_cancel();
    }
    for(unsigned resumed=0u;resumed<2u;resumed++){
        CHECK(mid_start(PROTO_TASK_BALL,1,31u));CHECK(mid_trigger(2.0f));CHECK(mid_begin_fix());
        if(resumed)CHECK(mid_restore());
        object(request(),4,135,VAT_BALL_Y_PX,641u,480u,seq++);tick(20u);
        CHECK(st.state==VAT_STOPPED&&stopped()&&strcmp(st.reason,"IMAGE_GEOMETRY")==0);
        unsigned before=mid_drive_calls;tick(10000u);CHECK(mid_drive_calls==before&&stopped());
    }
    puts("mid-search31: middle-bucket rank2 retains original signed blind resume, empty/wrongtarget cannot arm; changed geometry during repair/resume terminal and cannot restart passed");
    return 1;
}
static int check_scopes_and_faults(void)
{
    CHECK(mid_start(PROTO_TASK_BALL,1,43u));yaw=st.yaw_target+2.0f;tick(20u);
    CHECK(st.state==VAT_ROUTE_SEARCH&&vx>0.0f&&w<0.0f);vision_align_test_cancel();
    CHECK(reset(38u));CHECK(ready());tick(20u);CHECK(st.state==VAT_STEP_MOVE);
    yaw=st.yaw_target+2.0f;tick(20u);CHECK(st.state==VAT_STEP_MOVE&&vx>0.0f&&w==0.0f);vision_align_test_cancel();
    /* Already-latched fine X retains its moving heading controller; this is
     * not permission to restore stopped BALL/HOSTAGE task-end hard yaw. */
    CHECK(reset_route(40u));CHECK(ready());frame(0,229,VAT_HOSTAGE_Y_PX);CHECK(stop_recheck());
    frame(0,226,VAT_HOSTAGE_Y_PX);CHECK(st.state==VAT_FINE_CONTINUOUS&&vx>0.0f);
    yaw=st.yaw_target+2.0f;frame(0,226,VAT_HOSTAGE_Y_PX);
    CHECK(st.state==VAT_FINE_CONTINUOUS&&vx>0.0f&&w<0.0f);vision_align_test_cancel();

    for(unsigned phase=0u;phase<3u;phase++)for(unsigned fault=0u;fault<5u;fault++){
        CHECK(mid_start(PROTO_TASK_HOSTAGE,1,31u));CHECK(mid_trigger(2.0f));
        if(phase){CHECK(mid_begin_fix());if(phase==2u)CHECK(mid_restore());}
        if(fault==0u)vision_align_test_cancel();
        else if(fault==1u)aborted=1;
        else if(fault==2u)imu_valid=0;
        else if(fault==3u)yaw=NAN;
        else ack(request(),2u,1u);
        tick(20u); CHECK(st.state==VAT_STOPPED&&stopped());
        unsigned before=mid_drive_calls; tick(10000u);
        CHECK(mid_drive_calls==before&&st.state==VAT_STOPPED&&stopped());
    }
    puts("mid-search31:43/standalone/fine scope unchanged; cancel/abort/IMU/NaN/NACK in brake/correct/resumed-coordinate-barrier cannot issue later movement passed");
    return 1;
}
int main(void)
{
    if(!check_all_tasks_directions_and_new_frames()||!check_timeout_residual_and_hysteresis()||
       !check_middle_bucket_and_geometry_barriers()||!check_scopes_and_faults())return 1;
    puts("route31_search_mid_yaw_test: host-only actual VAT/parser regression passed; no physical road acceptance");
    return 0;
}
