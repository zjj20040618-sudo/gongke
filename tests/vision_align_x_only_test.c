/* Default candidate: real VAT and CRC/ACK/request parser, X-only alignment.
 * Pixels, odometry and heading are injected host evidence, not a robot plant. */
#define main vat_legacy_xy_fixture_main
#include "vision_align_test_test.c"
#undef main

_Static_assert(VAT_Y_ALIGN_ENABLE == 0, "This case must test the default X-only build");

static void xonly_frame(int model, int x, int y, unsigned height)
{
    object(request(), model, x, y, 640u, height, seq++);
    tick(20u);
}

static int xonly_finish(int model, int y, unsigned height, VisionAlignTestState expected)
{
    CHECK(st.state == VAT_ALIGN || st.state == VAT_RECHECK);
    for (unsigned i = 1u; i <= VAT_GOOD_FRAMES; ++i) {
        xonly_frame(model, 190, y, height);
        CHECK(vy == 0.0f && stopped() && !st.yaw_dirty && !st.yaw_ever && !st.step);
        if (i < VAT_GOOD_FRAMES) CHECK(st.state == VAT_ALIGN && st.good == i);
    }
    CHECK(st.state == expected && st.cy == y && st.img_h == height);
    return 1;
}

static int check_y_error_and_dormant_target_outside_image(void)
{
    const unsigned modes[] = {38u, 39u, 40u, 41u};
    const int models[] = {4, 9, 0, 4};
    const VisionAlignTestState ends[] = {VAT_DONE, VAT_DONE, VAT_DONE, VAT_HOLD_BALL};
    /* All dormant Y workpoints exceed180; actual observed cy remains legal.
     * Both X boundary values must still be accepted without any Y step. */
    for (unsigned mode = 0u; mode < 4u; ++mode) for (unsigned boundary = 0u; boundary < 2u; ++boundary) {
        CHECK(reset(modes[mode])); CHECK(ready());
        for (unsigned i = 1u; i <= VAT_GOOD_FRAMES; ++i) {
            int y = i & 1u ? 20 : 150;
            xonly_frame(models[mode], boundary ? 200 : 180, y, 180u);
            CHECK(stopped() && vy == 0.0f && !st.step && !st.yaw_dirty && !st.yaw_ever);
            if (i < VAT_GOOD_FRAMES) CHECK(st.state == VAT_ALIGN && st.good == i);
        }
        CHECK(st.state == ends[mode] && st.good == VAT_GOOD_FRAMES && st.cy == 20 && st.img_h == 180u);
        vision_align_test_cancel(); CHECK(stopped());
    }
    puts("defaultX: ball/bucket/hostage/41 ignore Y error and dormantY beyond180px; X180/200 inclusive,5 fresh,cy retained,no lateral/Yfix passed");
    return 1;
}

static int check_x_steps_recheck_and_y_never_moves(void)
{
    CHECK(reset(39u)); CHECK(ready());
    xonly_frame(9, 201, 20, 180u);
    CHECK(st.state == VAT_STEP_MOVE && vx == 20.0f && vy == 0.0f && w == 0.0f && st.axis == 1u);
    xonly_frame(9, 179, 150, 180u); /* No reversal while a latched3mm step is active. */
    CHECK(st.state == VAT_STEP_MOVE && vx == 20.0f && vy == 0.0f);
    CHECK(end_step_by_odometry()); CHECK(stop_recheck());
    tick(1000u); CHECK(st.state == VAT_RECHECK && stopped() && !st.good);
    xonly_frame(9, 179, 150, 180u);
    CHECK(st.state == VAT_STEP_MOVE && vx == -20.0f && vy == 0.0f && w == 0.0f && st.axis == 1u);
    CHECK(end_step_by_odometry()); CHECK(stop_recheck());
    for (unsigned i = 0u; i < VAT_GOOD_FRAMES; ++i) {
        xonly_frame(9, 190, 20, 180u);
        CHECK(stopped() && !st.yaw_dirty && !st.yaw_ever && st.step == 2u);
    }
    CHECK(st.state == VAT_DONE && st.cy == 20);
    /* Real short-step age and software time caps remain effective. */
    CHECK(reset(39u)); CHECK(ready()); xonly_frame(9, 201, 20, 180u);
    tick(240u); CHECK(st.state == VAT_STEP_MOVE && vx == 20.0f && vy == 0.0f);
    tick(20u); CHECK(st.state == VAT_BRAKE && stopped() && st.step_capped);
    CHECK(stop_recheck()); vision_align_test_cancel(); CHECK(stopped());
    puts("defaultX: only +/-X20/3mm or250ms cap,latched direction,250ms brake,newX/lostwait; Y variation creates no lateral or Y-specific yaw passed");
    return 1;
}

static int check_ack_fresh_invalid_cy_and_cancel(void)
{
    CHECK(reset(39u));
    xonly_frame(9, 190, 20, 180u); CHECK(stopped() && !st.good && !st.latest);
    ack((uint16_t)(request() - 1u), 2u, 0u); tick(20u); CHECK(stopped() && !st.good);
    ack(request(), 2u, 0u); tick(260u); CHECK(st.state == VAT_ALIGN && stopped());
    object((uint16_t)(request()-1u), 9, 190, 20, 640u, 180u, seq++); tick(20u);
    CHECK(stopped() && !st.good && !st.latest);
    xonly_frame(9, 190, 20, 180u); CHECK(st.good == 1u && stopped());
    for (unsigned i = 0u; i < 5u; ++i) {
        object(request(), 9, 190, 150, 640u, 180u, (uint16_t)(seq - 1u)); tick(20u);
        CHECK(st.good == 1u && st.state == VAT_ALIGN && stopped());
    }
    tick(301u); CHECK(!st.good && !st.latest && stopped());
    xonly_frame(-1, 0, 0, 180u); CHECK(!st.good && !st.latest && stopped());
    xonly_frame(4, 190, 20, 180u); CHECK(!st.good && !st.latest && stopped());

    ProtoStats before, after;
    proto_stats_get(&before);
    const int bad_y[] = {180, 181, -1};
    for (unsigned i = 0u; i < 3u; ++i) {
        object(request(), 9, 190, bad_y[i], 640u, 180u, seq++); tick(20u);
        CHECK(st.state != VAT_DONE && !st.good && stopped());
    }
    proto_stats_get(&after);
    CHECK(after.binary_bad == before.binary_bad + 3u && after.obj == before.obj);
    /* Valid coordinates in a changed image size still cannot pass geometry. */
    CHECK(reset(39u)); CHECK(ready()); xonly_frame(9, 190, 20, 180u);
    xonly_frame(9, 190, 20, 181u);
    CHECK(st.state == VAT_STOPPED && strcmp(st.reason, "IMAGE_GEOMETRY") == 0 && stopped());
    for (unsigned phase = 0u; phase < 4u; ++phase) {
        CHECK(reset(39u)); if (phase) CHECK(ready());
        if (phase == 2u || phase == 3u) { xonly_frame(9, 201, 20, 180u); CHECK(st.state == VAT_STEP_MOVE); }
        if (phase == 3u) { CHECK(end_step_by_odometry()); CHECK(stop_recheck()); }
        vision_align_test_cancel(); xonly_frame(9, 190, 20, 180u); tick(10000u);
        CHECK(st.state == VAT_STOPPED && stopped());
    }
    CHECK(reset(39u)); CHECK(ready()); xonly_frame(9, 201, 20, 180u);
    ack(request(), 2u, 1u); tick(1u); CHECK(st.state == VAT_STOPPED && stopped());
    CHECK(reset(39u)); CHECK(ready()); xonly_frame(9, 201, 20, 180u);
    aborted=1; tick(1u); CHECK(st.state == VAT_STOPPED && stopped());
    CHECK(reset(39u)); CHECK(ready()); seq=65533u;
    CHECK(xonly_finish(9, 20, 180u, VAT_DONE)); CHECK(seq < 20u);
    puts("defaultX: ACK/request/class/fresh/duplicate/empty/seqwrap gates retained; actualcy outsideimage rejected,geometry change stops,cancel/NACK/abort cannot resume passed");
    return 1;
}

static int check_standalone41_hold_and_turn_no_y(void)
{
    CHECK(reset(41u)); CHECK(ready()); CHECK(xonly_finish(4,20,180u,VAT_HOLD_BALL));
    unsigned before=request_count;
    tick(4980u); CHECK(st.state==VAT_HOLD_BALL&&stopped()&&request_count==before);
    tick(20u); proto_service(); /* VAT queued the new bucket request after this tick's TX service. */
    CHECK(st.state==VAT_TURN_REQUEST&&stopped()&&request_count==before+1u);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    uint16_t old=request(); CHECK(vision_align_test_take_turn_request());
    w=1.2f; tick(20u); CHECK(st.state==VAT_TURN_ACTIVE&&w==1.2f);
    yaw=217.0f; vision_align_test_notify_turn_result(1); proto_service(); vision_align_test_status(&st);
    CHECK(request()!=old&&st.state==VAT_BRAKE&&st.yaw_target==217.0f&&stopped());
    object(old,9,190,20,640u,180u,seq++); tick(20u); CHECK(!st.good&&stopped());
    ack(request(),2u,0u); object(request(),9,190,20,640u,180u,seq++); tick(20u);
    tick(260u); CHECK(st.state==VAT_RECHECK&&stopped()&&!st.latest&&!st.good);
    CHECK(xonly_finish(9,20,180u,VAT_HOLD_BUCKET));
    tick(4980u); CHECK(st.state==VAT_HOLD_BUCKET&&stopped());
    tick(20u); CHECK(st.state==VAT_DONE&&stopped()&&!arm_calls&&!laser_on&&!imu_zero);
    puts("defaultX: standalone41 keeps ball5s/+180/newbucketrequest/newX/bucket5s; no arm or laser,no Y-specific correction passed");
    return 1;
}

static int check_route_search_first_brake_new_x_and_no_y(void)
{
    const unsigned modes[]={41u,40u}; const int models[]={4,0};
    const int x_goals[]={135,215}; /* Private31/43 goals; standalone38..41 stay190. */
    for(unsigned kind=0u;kind<2u;kind++)for(unsigned sticky=0u;sticky<2u;sticky++){
        CHECK(reset_route(modes[kind])); CHECK(vision_align_test_route_search_kp_set(3.0f));
        if(kind&&sticky)route_auto_hostage_rank=0;
        xonly_frame(models[kind],x_goals[kind],20,180u); tick(1000u); CHECK(stopped()&&!st.good);
        ack(request(),2u,0u); tick(20u); CHECK(st.state==VAT_ROUTE_SEARCH&&vx==0.0f&&vy==0.0f);
        tick(20u); CHECK(fabsf(vx-14.0f)<0.00001f&&vy==0.0f);
        tick(20u); CHECK(fabsf(vx-28.0f)<0.00001f&&vy==0.0f);
        tick(1000u); CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&!st.step);
        object(request(),models[kind],x_goals[kind],20,640u,180u,seq++);
        if(sticky)object(request(),-1,0,0,640u,180u,seq++);
        tick(20u); CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good);
        tick(240u); CHECK(st.state==VAT_BRAKE&&stopped());
        tick(20u); CHECK(st.state==VAT_RECHECK&&stopped()&&!st.good&&!st.latest);
        tick(1000u); CHECK(st.state==VAT_RECHECK&&stopped());
        xonly_frame(models[kind],x_goals[kind]+11,150,180u);
        CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f&&st.axis==1u);
        CHECK(end_step_by_odometry()); CHECK(stop_recheck());
        for(unsigned i=0u;i<VAT_GOOD_FRAMES;i++){
            xonly_frame(models[kind],x_goals[kind],20,180u);
            CHECK(stopped()&&vy==0.0f&&!st.yaw_dirty&&st.step==1u);
        }
        CHECK(st.state==(kind?VAT_WAIT_HOSTAGE_ACTION:VAT_WAIT_BALL_ACTION)&&st.cy==20);
        if(kind){
            CHECK(st.alignment_confirmed&&!st.hostage_fallback&&st.hostage_seen);
            vision_align_test_notify_route_action_result(1);tick(20u);
            CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&stopped()); /* No hidden auto-action in finish. */
            CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_HOSTAGE);
            CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
            tick(1000u);CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&stopped());
            vision_align_test_notify_route_action_result(1);tick(20u);
            if(sticky){
                CHECK(st.state==VAT_WAIT_HOSTAGE_RANK&&!st.target_rank&&stopped());
                CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
                tick(5000u);CHECK(st.state==VAT_WAIT_HOSTAGE_RANK&&stopped());
                rank_frame(request(),seq++,0u,2u,3u,1u,0u,2u);tick(20u);
                CHECK(st.state==VAT_DONE&&st.target_rank==2u&&st.alignment_confirmed);
            }else CHECK(st.state==VAT_DONE&&st.target_rank==1u&&st.alignment_confirmed);
        }
        vision_align_test_cancel(); CHECK(stopped());
    }
    /* SEARCH-induced late heading restoration is independent of Y enable. */
    CHECK(check_route_search_gain_and_late_yaw_recheck());
    CHECK(check_route_action_failure_and_cancel());
    puts("defaultX route:matchingACK acc700 search0/14/28/clamp100,stickyfirstframe brake250/newX,X20microsteps/noY;explicit HOSTAGE takeonce/result thenDONE or rankWAIT/real54;privategain/postsearchyaw/freshX beforelift retained passed");
    return 1;
}

static int check_route_relative30_search_direction_and_sticky_fine(void)
{
    const unsigned modes[]={41u,40u}; const int models[]={4,0};
    const int goals[]={135,215};
    CHECK(VAT_ROUTE_FINE_ERROR_PX==30&&VAT_ROUTE_BUCKET_FINE_ERROR_PX==30);
    CHECK(VAT_ROUTE_BALL_X_PX==135&&VAT_ROUTE_BUCKET_X_PX==125&&VAT_ROUTE_HOSTAGE_X_PX==215);
    for(unsigned kind=0u;kind<2u;kind++)for(unsigned side=0u;side<2u;side++){
        int direction=side?1:-1;
        CHECK(reset_route(modes[kind]));CHECK(vision_align_test_route_search_ff_set(0.03f));
        CHECK(ready());
        /* Exact +/-30 is still coarse; the observed error controls direction.
         * Restarted coarse ramps begin at0, not the prior full-speed command. */
        xonly_frame(models[kind],goals[kind]+direction*30,20,180u);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==0.0f&&vy==0.0f&&!st.good);
        tick(20u);CHECK(fabsf(vx-direction*14.0f)<0.0001f);
        CHECK(fabsf(vy-(direction>0?14.0f*0.03f:0.0f))<0.00001f);
        tick(20u);CHECK(fabsf(vx-direction*28.0f)<0.0001f);
        CHECK(fabsf(vy-(direction>0?28.0f*0.03f:0.0f))<0.00001f);
        tick(200u);CHECK(vx==direction*100.0f&&st.state==VAT_ROUTE_SEARCH);
        CHECK(fabsf(vy-(direction>0?3.0f:0.0f))<0.00001f);
        uint16_t accepted=(uint16_t)(seq-1u);
        object(request(),models[kind],goals[kind]-direction*30,20,640u,180u,accepted);
        tick(20u);CHECK(st.state==VAT_ROUTE_SEARCH&&vx==direction*100.0f);
        object(request(),models[kind],goals[kind]-direction*30,20,640u,180u,(uint16_t)(accepted-1u));
        tick(20u);CHECK(st.state==VAT_ROUTE_SEARCH&&vx==direction*100.0f);
        /* A fresh far frame on the other side resets the acceleration clock. */
        direction=-direction;
        xonly_frame(models[kind],goals[kind]+direction*30,150,180u);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==0.0f&&vy==0.0f);
        tick(20u);CHECK(fabsf(vx-direction*14.0f)<0.0001f);
        CHECK(fabsf(vy-(direction>0?14.0f*0.03f:0.0f))<0.00001f);
        /* +/-29 latches fine even if an empty packet follows before polling. */
        object(request(),models[kind],goals[kind]+direction*29,20,640u,180u,seq++);
        object(request(),-1,0,0,640u,180u,seq++);tick(20u);
        CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good);
        CHECK(stop_recheck());
        xonly_frame(models[kind],goals[kind]+direction*40,150,180u);
        CHECK(st.state==VAT_STEP_MOVE&&vx==direction*20.0f&&vy==0.0f&&st.axis==1u);
        CHECK(end_step_by_odometry());CHECK(stop_recheck());
        /* Arrival stays +/-10 inclusive, not the new30px coarse/fine gate. */
        for(unsigned i=0u;i<VAT_GOOD_FRAMES;i++){
            xonly_frame(models[kind],goals[kind]+(i&1u?10:-10),20,180u);
            CHECK(stopped()&&vy==0.0f);
            if(i+1u<VAT_GOOD_FRAMES)CHECK(st.state==VAT_ALIGN&&st.good==i+1u);
        }
        CHECK(st.state==(kind?VAT_WAIT_HOSTAGE_ACTION:VAT_WAIT_BALL_ACTION));
        vision_align_test_cancel();CHECK(stopped());
    }
    /* No selected image retains forward search; reverse motion has no FF. */
    CHECK(reset_route(41u));CHECK(vision_align_test_route_search_ff_set(0.03f));CHECK(ready());
    xonly_frame(4,105,20,180u);tick(20u);CHECK(vx<0.0f&&vy==0.0f);
    xonly_frame(-1,0,0,180u);CHECK(st.state==VAT_ROUTE_SEARCH&&vx==0.0f&&vy==0.0f);
    tick(20u);CHECK(fabsf(vx-14.0f)<0.0001f&&fabsf(vy-0.42f)<0.00001f);
    vision_align_test_cancel();CHECK(stopped());
    puts("defaultX route: relative +/-30 remains signedcoarse, +/-29 latchesstickyfine20, arrival +/-10/5fresh; reverseacc700 restarts0/14/28, forwardFFonly, missingimageforward passed");
    return 1;
}

int main(void)
{
    if(!check_y_error_and_dormant_target_outside_image()
       ||!check_x_steps_recheck_and_y_never_moves()
       ||!check_ack_fresh_invalid_cy_and_cancel()
       ||!check_standalone41_hold_and_turn_no_y()
       ||!check_route_search_first_brake_new_x_and_no_y()
       ||!check_route_relative30_search_direction_and_sticky_fine()
       ||!check_route_search_threshold_right_ff_and_isolation()
       ||!check_route_task_x_goals_and_standalone_isolation()
       ||!check_route_bucket_wait_search_sticky_and_loss()||!check_route_bucket_faults_and_cancel()
       ||!check_route_rank_wait_capture_and_independent())return 1;
    puts("vision_align_x_only_test: defaultYoff host checks passed; no physical UART, camera, chassis or mechanism acceptance");
    return 0;
}
