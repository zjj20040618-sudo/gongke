/* Route31 continuous X / strict seen-lost300ms contract against real VAT and
 * CRC/request/ACK parser. Injected pixels/IMU/encoders are not hardware proof. */
#define main vat_continuous_inherited_fixture_main
#include "vision_align_test_test.c"
#undef main

_Static_assert(VAT_Y_ALIGN_ENABLE == 0, "Continuous31 requires the production X-only build");

static int cx_start(unsigned task, unsigned owner)
{
    CHECK(reset_route_scoped(task == PROTO_TASK_HOSTAGE ? 40u : 41u, owner));
    CHECK(ready());
    if (task == PROTO_TASK_BUCKET) {
        CHECK(finish(4, VAT_ROUTE_BALL_X_PX, VAT_BALL_Y_PX));
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_BALL);
        vision_align_test_notify_route_action_result(1); proto_service();
        CHECK(vision_align_test_take_turn_request());
        yaw = 217.0f; vision_align_test_notify_turn_result(1); proto_service();
        CHECK(!st.ball_fallback);
        ack(request(), 2u, 0u); tick(260u);
        CHECK(st.state == VAT_RECHECK && !st.bucket_seen && stopped());
    }
    return 1;
}
static int cx_model(unsigned task)
{ return task == PROTO_TASK_BALL ? 4 : task == PROTO_TASK_BUCKET ? 9 : 0; }
static int cx_y(unsigned task)
{ return task == PROTO_TASK_BALL ? VAT_BALL_Y_PX : task == PROTO_TASK_BUCKET ? VAT_BUCKET_Y_PX : VAT_HOSTAGE_Y_PX; }
static unsigned cx_fallback(unsigned task)
{ return task == PROTO_TASK_BALL ? st.ball_fallback : task == PROTO_TASK_BUCKET ? st.bucket_fallback : st.hostage_fallback; }
static unsigned cx_seen(unsigned task)
{ return task == PROTO_TASK_BALL ? st.ball_seen : task == PROTO_TASK_BUCKET ? st.bucket_seen : st.hostage_seen; }
static uint32_t cx_age(unsigned task)
{ return task == PROTO_TASK_BALL ? st.ball_age_ms : task == PROTO_TASK_BUCKET ? st.bucket_age_ms : st.hostage_age_ms; }
static VisionAlignTestState cx_pending(unsigned task)
{ return task == PROTO_TASK_BALL ? VAT_WAIT_BALL_ACTION : task == PROTO_TASK_BUCKET ? VAT_WAIT_BUCKET_ACTION : VAT_WAIT_HOSTAGE_ACTION; }
static int cx_action(unsigned task)
{ return task == PROTO_TASK_BALL ? VAT_ROUTE_ACTION_BALL : task == PROTO_TASK_BUCKET ? VAT_ROUTE_ACTION_BUCKET : VAT_ROUTE_ACTION_HOSTAGE; }
static int cx_stop_keepalive(unsigned task, int x)
{
    CHECK(st.state == VAT_BRAKE && stopped());
    for (unsigned i = 0u; i < 3u; ++i) {
        object(request(), cx_model(task), x, cx_y(task), 640u, 480u, seq++);
        tick(100u);
        CHECK(stopped() && !cx_fallback(task) && !st.good);
    }
    CHECK(st.state == VAT_RECHECK && !st.latest && !st.rx_fresh);
    return 1;
}
static int cx_fine(unsigned task)
{
    CHECK(cx_start(task, 31u));
    unsigned creep_before = creep_drive_calls;
    frame(cx_model(task), st.x_goal + 14, cx_y(task));
    CHECK(st.state == VAT_BRAKE && stopped());
    CHECK(cx_stop_keepalive(task, st.x_goal + 14));
    CHECK(creep_drive_calls == creep_before); /* Coarse search/brake is unchanged. */
    frame(cx_model(task), st.x_goal + 12, cx_y(task));
    CHECK(st.state == VAT_FINE_CONTINUOUS && vx == 20.0f && vy == 0.0f);
    CHECK(creep_drive_calls > creep_before);
    return 1;
}
static int check_continuous_heading_reverse_arrival(void)
{
    for (unsigned task = PROTO_TASK_BALL; task <= PROTO_TASK_BUCKET; ++task) {
        if (task == PROTO_TASK_TARGET) continue;
        CHECK(cx_fine(task));
        unsigned begins = st.step, holds = heading_hold_calls;
        fore += 2000.0f; yaw = st.yaw_target + 0.2f; leg_yaw = 5.0f;
        tick(100u);
        CHECK(st.state == VAT_FINE_CONTINUOUS && vx == 20.0f && vy == 0.0f && w < 0.0f);
        CHECK(st.step == begins && !st.step_capped && !st.good && heading_hold_calls > holds);
        CHECK(fabsf(heading_hold_target - 4.8f) < 0.0001f);
        frame(cx_model(task), st.x_goal - 12, cx_y(task));
        CHECK(st.state == VAT_BRAKE && stopped() && strcmp(st.reason, "FINE_REVERSE_BRAKE") == 0);
        CHECK(cx_stop_keepalive(task, st.x_goal - 12));
        frame(cx_model(task), st.x_goal - 12, cx_y(task));
        CHECK(st.state == VAT_FINE_CONTINUOUS && vx == -20.0f && vy == 0.0f);
        frame(cx_model(task), st.x_goal, cx_y(task));
        CHECK(st.state == VAT_BRAKE && stopped() && !st.good);
        CHECK(cx_stop_keepalive(task, st.x_goal));
        for (unsigned i = 1u; i <= VAT_GOOD_FRAMES; ++i) {
            frame(cx_model(task), st.x_goal, cx_y(task));
            CHECK(stopped() && !cx_fallback(task));
            if (i < VAT_GOOD_FRAMES) CHECK(st.state == VAT_ALIGN && st.good == i);
        }
        CHECK(st.state == cx_pending(task) && st.good == VAT_GOOD_FRAMES);
        CHECK(vision_align_test_take_route_action() == cx_action(task));
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        vision_align_test_cancel(); CHECK(stopped());
    }
    return 1;
}
static int check_strict_loss_all_tasks_and_recovery(void)
{
    for (unsigned task = PROTO_TASK_BALL; task <= PROTO_TASK_BUCKET; ++task) {
        if (task == PROTO_TASK_TARGET) continue;
        CHECK(cx_start(task, 31u));
        int goal = st.x_goal;
        uint16_t current = request();
        object(current, cx_model(task), goal + 100, cx_y(task), 640u, 480u, 100u);
        tick(300u);
        CHECK(cx_seen(task) && cx_age(task) == 300u && !cx_fallback(task));
        CHECK(st.state != cx_pending(task));
        /* Same/older01,54,empty and old request never refresh NEW-coordinate age. */
        object(current, cx_model(task), goal, cx_y(task), 640u, 480u, 100u);
        object(current, cx_model(task), goal, cx_y(task), 640u, 480u, 99u);
        object(current, -1, 0, 0, 640u, 480u, 101u);
        object((uint16_t)(current-1u), cx_model(task), goal, cx_y(task), 640u, 480u, 102u);
        tick(1u);
        CHECK(cx_age(task) == 301u && st.state == VAT_BRAKE && stopped() && !st.good);
        CHECK(!st.alignment_confirmed && !cx_fallback(task));
        counts[2]++; tick(200u); CHECK(st.state == VAT_BRAKE);
        tick(249u); CHECK(st.state == VAT_BRAKE && !cx_fallback(task));
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        tick(1u);
        CHECK(st.state == cx_pending(task) && cx_fallback(task) && !st.good && !st.alignment_confirmed);
        /* Recovery between poll and owner take must cancel the pending action. */
        object(current, cx_model(task), goal + 12, cx_y(task), 640u, 480u, 103u);
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        vision_align_test_status(&st);
        CHECK(st.state == VAT_BRAKE && !cx_fallback(task) && cx_age(task) == 0u && stopped());
        tick(300u); CHECK(!cx_fallback(task) && st.state != cx_pending(task));
        tick(1u); CHECK(st.state == VAT_BRAKE && !cx_fallback(task));
        tick(250u);
        CHECK(st.state == cx_pending(task) && cx_fallback(task) && !st.good && !st.alignment_confirmed);
        CHECK(vision_align_test_take_route_action() == cx_action(task));
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        vision_align_test_cancel(); CHECK(stopped());
    }
    return 1;
}
static int check_never_seen_ack_gate_new_request_and_abort(void)
{
    for (unsigned mode = 40u; mode <= 41u; ++mode) {
        CHECK(reset_route_scoped(mode, 31u));
        int model = mode == 40u ? 0 : 4;
        object(request(), model, st.x_goal, mode == 40u ? VAT_HOSTAGE_Y_PX : VAT_BALL_Y_PX, 640u, 480u, seq++);
        tick(5000u); CHECK(!st.ball_seen && !st.hostage_seen && stopped());
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        ack(request(), 2u, 0u);
        object(request(), -1, 0, 0, 640u, 480u, seq++);
        tick(5000u);
        CHECK(!st.ball_seen && !st.hostage_seen && !st.bucket_seen);
        CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
        vision_align_test_cancel(); tick(10000u); CHECK(st.state == VAT_STOPPED && stopped());
    }
    CHECK(cx_fine(PROTO_TASK_BALL));
    tick(280u); CHECK(st.ball_age_ms == 300u && st.state == VAT_FINE_CONTINUOUS && vx == 20.0f);
    aborted = 1; tick(1u); CHECK(st.state == VAT_STOPPED && stopped() && !st.ball_fallback);
    CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);

    CHECK(cx_start(PROTO_TASK_BUCKET, 31u));
    CHECK(!st.ball_seen && !st.ball_fallback && !st.bucket_seen);
    object((uint16_t)(request()-1u), 9, st.x_goal, VAT_BUCKET_Y_PX, 640u, 480u, seq++);
    tick(3000u); CHECK(!st.bucket_seen && !st.bucket_fallback);
    CHECK(vision_align_test_take_route_action() == VAT_ROUTE_ACTION_NONE);
    vision_align_test_cancel(); CHECK(stopped());
    return 1;
}
static int check_replay_empty_fine_isolation(void)
{
    CHECK(cx_fine(PROTO_TASK_BALL));
    uint16_t last = st.sequence;
    object(request(), 4, st.x_goal - 30, VAT_BALL_Y_PX, 640u, 480u, last); tick(20u);
    CHECK(st.state == VAT_FINE_CONTINUOUS && vx == 20.0f && !st.good);
    object(request(), -1, 0, 0, 640u, 480u, (uint16_t)(last+1u)); tick(20u);
    CHECK(st.state == VAT_FINE_CONTINUOUS && vx == 20.0f && !st.good && !st.latest);
    object(request(), 0, st.x_goal - 30, VAT_HOSTAGE_Y_PX, 640u, 480u, (uint16_t)(last+2u)); tick(20u);
    CHECK(st.state == VAT_FINE_CONTINUOUS && vx == 20.0f && !st.good && !st.latest);
    object(request(), 4, st.x_goal - 30, VAT_BALL_Y_PX, 640u, 480u, last); tick(20u);
    CHECK(st.state == VAT_FINE_CONTINUOUS && vx == 20.0f && !st.good && !st.ball_fallback);
    tick(200u); CHECK(st.ball_age_ms == 300u && vx == 20.0f && !st.ball_fallback);
    tick(1u); CHECK(st.state == VAT_BRAKE && stopped() && !st.ball_fallback);
    vision_align_test_cancel(); CHECK(stopped());
    /*43 retains its bounded3mm/250ms path and2s loss, not31's fallback. */
    unsigned creep_before = creep_drive_calls;
    CHECK(cx_start(PROTO_TASK_HOSTAGE, 43u));
    frame(0, st.x_goal + 14, VAT_HOSTAGE_Y_PX); CHECK(st.state == VAT_BRAKE);
    CHECK(stop_recheck());
    frame(0, st.x_goal + 12, VAT_HOSTAGE_Y_PX);
    CHECK(st.state == VAT_STEP_MOVE && vx == 20.0f && w == 0.0f);
    tick(301u); CHECK(st.state == VAT_BRAKE && !st.hostage_fallback);
    vision_align_test_cancel(); CHECK(stopped());
    CHECK(creep_drive_calls == creep_before);
    CHECK(reset(38u)); CHECK(ready()); frame(4, VAT_X_PX + 12, VAT_BALL_Y_PX);
    CHECK(st.state == VAT_STEP_MOVE && !st.ball_seen);
    tick(301u); CHECK(st.state == VAT_BRAKE && !st.ball_fallback);
    vision_align_test_cancel(); CHECK(stopped());
    CHECK(creep_drive_calls == creep_before);
    return 1;
}
int main(void)
{
    if (!check_continuous_heading_reverse_arrival() || !check_strict_loss_all_tasks_and_recovery() ||
        !check_never_seen_ack_gate_new_request_and_abort() || !check_replay_empty_fine_isolation()) return 1;
    puts("route31 continuousX20/creep-only-fine-dispatch/original-heading/reversal-brake/post-stop5new; three-task strict300/301+still250 loss source, recovery/abort/newrequest,43/standalone isolation passed");
    return 0;
}
