/* Real VAT + CRC/request/ACK parser, synthetic heading/pixels/time only.
 * These angular commands do not prove motor torque or loaded physical yaw.
 * Compile this file with App/vision_align_test.c and App/proto.c. */
#define main ball_yaw_inherited_fixture_main
#include "vision_align_test_test.c"
#undef main

static int power_model(void)
{ return st.task == PROTO_TASK_BUCKET ? 9 : st.task == PROTO_TASK_HOSTAGE ? 0 : 4; }
static int power_y(void)
{ return st.task == PROTO_TASK_BUCKET ? VAT_BUCKET_Y_PX : st.task == PROTO_TASK_HOSTAGE ? VAT_HOSTAGE_Y_PX : VAT_BALL_Y_PX; }

/* Real NEW selected01 keeps the >300ms fallback out of yaw-only cases.
 * Frames seen while braking/correcting must never contribute to arrival. */
static void power_tick(unsigned ms)
{
    while (ms) {
        unsigned slice = ms > 100u ? 100u : ms;
        object(request(),power_model(),st.x_goal,power_y(),640u,480u,seq++);
        tick(slice); ms -= slice;
    }
}
static int power_route_yaw(unsigned task, unsigned owner, float error)
{
    CHECK(reset_route_scoped(task == PROTO_TASK_HOSTAGE ? 40u : 41u,owner));
    if (owner == 31u) CHECK(vision_align_test_route_yaw_settle_ms_set(400u));
    CHECK(ready());
    if (task == PROTO_TASK_BUCKET) {
        CHECK(finish(4,VAT_ROUTE_BALL_X_PX,VAT_BALL_Y_PX));
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_BALL);
        vision_align_test_notify_route_action_result(1); proto_service();
        CHECK(vision_align_test_take_turn_request());
        yaw = 217.0f; vision_align_test_notify_turn_result(1); proto_service();
        ack(request(),2u,0u); tick(260u);
        CHECK(st.state == VAT_RECHECK && !st.bucket_seen && stopped());
        tick(VAT_ROUTE_BUCKET_MISSING_MS); tick(20u);
        CHECK(st.state == VAT_ROUTE_BUCKET_SEARCH);
    } else {
        tick(20u); tick(160u);
        CHECK(st.state == VAT_ROUTE_SEARCH && vx == 100.0f && vy == 0.0f);
    }
    float original = st.yaw_target;
    /* This explicit legacy-enabled fixture audits STOPPED task-end power,
     * not31's new >1.5deg coarse mid-search repair. Latch the fine gate and
     * old VAT_YAW_FIX at0.6deg first; inject the requested large error only
     * after that controller owns the motors. Small-error timing is unchanged. */
    float admission_error = fabsf(error) > 1.5f ? (error > 0.0f ? 0.6f : -0.6f) : error;
    yaw = original - admission_error;
    frame(power_model(),st.x_goal + 14,power_y());
    CHECK(st.state == VAT_BRAKE && st.yaw_dirty && st.yaw_ever && stopped());
    power_tick(260u);
    CHECK(st.state == VAT_YAW_FIX && stopped() && !st.good && !st.alignment_confirmed);
    yaw = original - error;
    power_tick(20u);
    CHECK(st.state == VAT_YAW_FIX && st.yaw_target == original && vx == 0.0f && vy == 0.0f);
    return 1;
}
static int power_signed(float expected)
{
    CHECK(st.state == VAT_YAW_FIX && vx == 0.0f && vy == 0.0f &&
          fabsf(w - expected) < 0.00001f && !st.good && !st.alignment_confirmed &&
          !st.ball_fallback && !st.bucket_fallback && !st.hostage_fallback &&
          !imu_zero && !arm_calls && !laser_on);
    return 1;
}
static int check_ball_floor_cap_boost_and_progress(void)
{
    CHECK(VAT_ROUTE31_BALL_YAW_MIN_W == 0.30f && VAT_ROUTE31_BALL_YAW_MAX_W == 0.60f &&
          VAT_YAW_MIN_W == 0.18f && T_DIST_ALIGN_MAX_W == 0.30f);
    for (int sign=-1;sign<=1;sign+=2) {
        for (unsigned edge=0u;edge<2u;++edge) {
            CHECK(power_route_yaw(PROTO_TASK_BALL,31u,sign * 0.6f));
            CHECK(power_signed(sign * 0.30f));
            power_tick(edge ? 400u : 399u);
            CHECK(power_signed(sign * (edge ? 0.60f : 0.30f)));
            vision_align_test_cancel(); CHECK(stopped());
        }
        CHECK(power_route_yaw(PROTO_TASK_BALL,31u,sign * 8.0f));
        CHECK(power_signed(sign * 0.60f)); /* proportional output capped, even before boost. */
        vision_align_test_cancel();
        CHECK(power_route_yaw(PROTO_TASK_BALL,31u,sign * 1.0f));
        power_tick(400u); CHECK(power_signed(sign * 0.60f));
        yaw = st.yaw_target - sign * 0.95f;
        power_tick(20u); CHECK(power_signed(sign * 0.30f)); /* exactly0.05 improvement resets. */
        power_tick(400u); CHECK(power_signed(sign * 0.60f));
        yaw = st.yaw_target + sign * 0.6f;
        power_tick(20u); CHECK(power_signed(-sign * 0.30f)); /* overshoot resets boost/direction. */
        power_tick(400u); CHECK(power_signed(-sign * 0.60f));
        vision_align_test_cancel(); tick(10000u);
        CHECK(st.state == VAT_STOPPED && stopped() && vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
    }
    puts("31 BALL stoppedyaw:both signs .30 floor/.60 cap,399 no boost400 boost,0.05 improvement resets,overshoot reverses .30 then .60;NEW01 keepalive prevents false fallback/alignment passed");
    return 1;
}
static int check_owner_task_isolation(void)
{
    const unsigned tasks[] = {PROTO_TASK_BUCKET,PROTO_TASK_HOSTAGE,PROTO_TASK_BALL};
    const unsigned owners[] = {31u,31u,43u};
    for (unsigned n=0u;n<3u;++n) for (int sign=-1;sign<=1;sign+=2) {
        CHECK(power_route_yaw(tasks[n],owners[n],sign * 0.6f));
        CHECK(power_signed(sign * 0.18f));
        power_tick(400u);
        CHECK(power_signed(sign * (owners[n] == 31u ? 0.30f : 0.18f)));
        vision_align_test_cancel();
        CHECK(power_route_yaw(tasks[n],owners[n],sign * 8.0f));
        CHECK(power_signed(sign * 0.30f));
        power_tick(400u); CHECK(power_signed(sign * 0.30f));
        vision_align_test_cancel();
    }
    for (unsigned mode=38u;mode<=41u;mode+=3u) {
        CHECK(reset(mode)); CHECK(ready());
#if VAT_Y_ALIGN_ENABLE
        frame(4,VAT_X_PX,VAT_BALL_Y_PX - 11);
        yaw = st.yaw_target + 0.6f;
        CHECK(end_step_by_odometry()); tick(260u); tick(20u);
        CHECK(power_signed(-0.18f)); power_tick(400u); CHECK(power_signed(-0.18f));
        yaw = st.yaw_target + 8.0f; power_tick(20u); CHECK(power_signed(-0.30f));
#else
        frame(4,VAT_X_PX + 12,VAT_BALL_Y_PX);
        CHECK(st.state == VAT_STEP_MOVE && vx == 20.0f && vy == 0.0f && w == 0.0f);
        yaw = st.yaw_target + 0.6f;
        CHECK(end_step_by_odometry()); CHECK(stop_recheck());
        CHECK(!st.yaw_ever && !st.yaw_dirty && stopped());
#endif
        vision_align_test_cancel(); CHECK(stopped());
    }
    puts("31 BUCKET/HOSTAGE retain .18/.30;43 BALL retains .18/.30;independent38/41 retain their original X-only/XY path,never inherit BALL .30/.60 passed");
    return 1;
}
static int check_stability_fresh_recheck_and_faults(void)
{
    for (unsigned edge=0u;edge<2u;++edge) {
        CHECK(power_route_yaw(PROTO_TASK_BALL,31u,0.6f));
        yaw = st.yaw_target + 0.31f; power_tick(20u); CHECK(power_signed(-0.30f));
        yaw = st.yaw_target; power_tick(20u);
        CHECK(st.state == VAT_YAW_FIX && stopped());
        power_tick(edge ? 400u : 399u);
        CHECK(st.state == (edge ? VAT_RECHECK : VAT_YAW_FIX) && stopped() &&
              !st.good && !st.latest && !st.alignment_confirmed && !st.ball_fallback);
        if (!edge) { vision_align_test_cancel(); continue; }
        uint16_t old_sequence = (uint16_t)(seq - 1u);
        object(request(),4,st.x_goal,VAT_BALL_Y_PX,640u,480u,old_sequence); tick(20u);
        CHECK(st.state == VAT_RECHECK && !st.good && stopped());
        for (unsigned fresh=1u;fresh<=VAT_GOOD_FRAMES;++fresh) {
            frame(4,st.x_goal,VAT_BALL_Y_PX);
            CHECK(stopped() && !st.ball_fallback);
            if (fresh < VAT_GOOD_FRAMES) CHECK(st.state == VAT_ALIGN && st.good == fresh);
        }
        CHECK(st.state == VAT_WAIT_BALL_ACTION && st.good == 5u && !st.ball_fallback &&
              !strcmp(st.reason,"LIFT_WAIT")); /* Ball uses state/good; alignment_confirmed is hostage metadata. */
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_BALL);
        vision_align_test_cancel();
    }
    for (unsigned fault=0u;fault<3u;++fault) {
        CHECK(power_route_yaw(PROTO_TASK_BALL,31u,0.6f));
        power_tick(400u); CHECK(power_signed(0.60f));
        if (!fault) vision_align_test_cancel(); /* g dispatcher delegates to this module cancellation. */
        else if (fault == 1u) imu_valid = 0;
        else yaw = NAN;
        tick(1u);
        CHECK(st.state == VAT_STOPPED && stopped() && !st.alignment_confirmed &&
              !st.ball_fallback && vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        tick(10000u); CHECK(stopped() && st.state == VAT_STOPPED);
    }
    puts("31 BALL yaw:0.31 rejects,399ms waits400ms ->RECHECK;moving/duplicate pixels cannot count,requires five NEW stoppedframes;cancel API(g delegate),invalid IMU/NaN stop boosted correction without grab passed");
    return 1;
}
int main(void)
{
    if (!check_ball_floor_cap_boost_and_progress() || !check_owner_task_isolation() ||
        !check_stability_fresh_recheck_and_faults()) return 1;
    puts("route31_ball_yaw_power_test:host VAT/PROTO checks only,no torque/physical loaded-yaw acceptance");
    return 0;
}
