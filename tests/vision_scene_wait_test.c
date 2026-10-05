/* Real steps.c handshake: old slots cleared, no motion while waiting, abort remains effective. */
#include "steps.h"
#define main unused_alignment_main
#define osDelay unused_alignment_delay
#define proto_send_scene unused_alignment_scene
#define proto_scene_status unused_alignment_status
#define proto_target_filter unused_alignment_filter
#define proto_send_target unused_alignment_target
#define proto_receive_end unused_alignment_receive_end
#include "align_timeout_test.c"
#undef main
#undef osDelay
#undef proto_send_scene
#undef proto_scene_status
#undef proto_target_filter
#undef proto_send_target
#undef proto_receive_end
#include "../App/steps.c"

static int status, outcome;
static unsigned requests, delays;
static unsigned receive_end_calls;
static ProtoScene last_scene;
static ProtoTask last_task;
static uint8_t last_digit;
static int queue_ok;
void proto_send_scene(ProtoScene scene) { requests++; last_scene = scene; }
void proto_receive_end(void) { receive_end_calls++; }
int proto_target_filter(ProtoTask task, uint8_t digit, int *cls, int *label)
{
    /* Mapping itself is tested against real proto.c in vision_task_select_test. */
    if (task == PROTO_TASK_BUCKET && digit == 0u) { *cls = CLS_BUCKET; *label = -1; return 1; }
    if (digit < 1u || digit > 3u || task < PROTO_TASK_BALL || task > PROTO_TASK_HOSTAGE) return 0;
    *cls = (int)task - 1;
    *label = (int)digit + (task == PROTO_TASK_HOSTAGE ? 2 : -1);
    return 1;
}
int proto_send_target(ProtoTask task, uint8_t digit)
{
    requests++; last_task = task; last_digit = digit;
    if (s_obj_pending || s_qr_pending || s_obj_want_cls != (int)task - 1) abort();
    return queue_ok;
}
int proto_scene_status(void) { return status; }
void osDelay(uint32_t ms)
{
    now_ms += ms;
    if (++delays == 2u) {
        if (outcome == 2) run_abort();
        else status = outcome;
    }
}
static void reset_fixture(int result)
{
    run_reset(); status = 0; outcome = result; requests = delays = brake_calls = 0u;
    receive_end_calls = 0u;
    queue_ok = 1; last_task = (ProtoTask)0; last_digit = 99u;
    s_qr_pending = s_obj_pending = 1; /* old phase, must not survive new request */
}
int main(void)
{
    reset_fixture(1);
    if (!step_vision_scene(SCENE_QR) || requests != 1u || delays != 2u
        || brake_calls != 1u || s_qr_pending || s_obj_pending) return 1;
    reset_fixture(-1);
    if (step_vision_scene(SCENE_EOD) || !run_aborted() || requests != 1u
        || last_scene != SCENE_EOD || !receive_end_calls || !s_vision_receive_closed) return 1;
    reset_fixture(2);
    if (step_vision_scene(SCENE_ANTI) || !run_aborted() || requests != 1u
        || last_scene != SCENE_ANTI || !receive_end_calls || !s_vision_receive_closed) return 1;
    reset_fixture(1); run_abort(); requests = 0u;
    if (step_vision_scene(SCENE_RESCUE) || requests || delays) return 1;
    for (unsigned task = 1u; task <= 4u; task++) {
        ProtoFrame wrong = {0}, wanted = {0}, out;
        uint8_t digit = task == 4u ? 0u : 2u;
        reset_fixture(1);
        if (!step_vision_target((ProtoTask)task, digit) || requests != 1u
            || delays != 2u || brake_calls != 1u || s_obj_pending || s_qr_pending
            || last_task != (ProtoTask)task || last_digit != digit) return 1;
        wrong.type = wanted.type = PF_OBJ;
        wrong.cls = ((int)task) % 4; wrong.label = 0;
        wanted.cls = s_obj_want_cls; wanted.label = s_obj_want_label < 0 ? 0 : s_obj_want_label;
        steps_feed_frame(&wanted); steps_feed_frame(&wrong);
        if (!step_object_take(&out) || out.cls != wanted.cls || out.label != wanted.label) return 1;
        steps_feed_frame(&wanted);
        step_vision_receive_end();
        steps_feed_frame(&wanted);
        if (step_object_take(&out) || s_qr_pending || s_obj_pending || !s_vision_receive_closed
            || requests != 1u || receive_end_calls != 1u) return 1;
    }
    reset_fixture(1);
    if (step_vision_target(PROTO_TASK_BALL, 0u) || requests || delays || brake_calls
        || !s_obj_pending || !s_qr_pending) return 1; /* no destructive queue on typo */
    reset_fixture(1); run_abort();
    if (step_vision_target(PROTO_TASK_BALL, 1u) || requests || delays || brake_calls) return 1;
    reset_fixture(-1);
    if (step_vision_target(PROTO_TASK_TARGET, 3u) || !run_aborted() || requests != 1u
        || !receive_end_calls || !s_vision_receive_closed) return 1;
    reset_fixture(2);
    if (step_vision_target(PROTO_TASK_BUCKET, 0u) || !run_aborted() || requests != 1u
        || !receive_end_calls || !s_vision_receive_closed) return 1;
    reset_fixture(1); queue_ok = 0;
    if (step_vision_target(PROTO_TASK_HOSTAGE, 1u) || !run_aborted() || requests != 1u
        || !receive_end_calls || !s_vision_receive_closed || delays) return 1;
    puts("vision stage wait: generic/4 selected tasks, atomic slot filter, local RX close without camera IDLE, invalid selector, failure and abort passed");
    return 0;
}
