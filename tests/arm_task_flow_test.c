/* Host-only flow check. Compile with task_eod.c/task_rescue.c and these mocks;
 * no firmware values are changed and no hardware movement is simulated. */
#include "robot_tasks.h"
#include "steps.h"
#include "proto.h"
#include "motion.h"
#include <stdio.h>
#include <string.h>

static const char *events[32];
static unsigned event_count;
static const char *fail_stage;

static void event(const char *name)
{
    if (event_count < sizeof events / sizeof events[0]) events[event_count++] = name;
}

static int check(const char *case_name, const char *const *expected, unsigned count)
{
    unsigned i;
    if (event_count != count) {
        fprintf(stderr, "%s: got %u events, expected %u\n", case_name, event_count, count);
        return 0;
    }
    for (i = 0; i < count; ++i) {
        if (strcmp(events[i], expected[i]) != 0) {
            fprintf(stderr, "%s: event %u is %s, expected %s\n",
                    case_name, i, events[i], expected[i]);
            return 0;
        }
    }
    return 1;
}

static void reset(const char *failure)
{
    event_count = 0;
    fail_stage = failure;
}

int run_aborted(void) { return 0; }
void proto_send_scene(ProtoScene scene)
{
    event(scene == SCENE_EOD ? "SCENE_EOD" : "SCENE_RESCUE");
}
int step_sweep(int want, int cls, int label, int32_t d[3], uint32_t to)
{
    (void)want; (void)label; (void)d; (void)to;
    event(cls == CLS_BALL ? "SWEEP_BALL" : cls == CLS_BUCKET ? "SWEEP_BUCKET" : "SWEEP_HOSTAGE");
    return 1;
}
int step_align(int cls, int label, uint32_t to)
{
    (void)label; (void)to;
    event(cls == CLS_BALL ? "ALIGN_BALL" : cls == CLS_BUCKET ? "ALIGN_BUCKET" : "ALIGN_HOSTAGE");
    return 1;
}
int step_arm_prepare(const char *task, const char *stage)
{
    (void)task;
    event(stage);
    return !fail_stage || strcmp(stage, fail_stage) != 0;
}
int step_arm_run(const char *task, const char *stage, uint32_t steps,
                 int (*action)(uint32_t))
{
    (void)task; (void)steps; (void)action;
    event(stage);
    return !fail_stage || strcmp(stage, fail_stage) != 0;
}
int step_arm_release(const char *task, const char *stage)
{
    (void)task;
    event(stage);
    return !fail_stage || strcmp(stage, fail_stage) != 0;
}
int step_arm_lower(uint32_t steps) { (void)steps; return 1; }
int step_arm_lift(uint32_t steps) { (void)steps; return 1; }
int step_grasp(uint32_t steps) { (void)steps; return 1; }
int step_rack_retract(uint32_t steps) { (void)steps; return 1; }
int step_rotate_deg(int deg, uint32_t to)
{
    (void)to;
    event(deg == 180 ? "TURN_180" : "WRONG_TURN");
    return deg == 180;
}
float motion_lateral_odo_mm(void) { return 0.0f; }
int step_return_lateral_odo(float target, uint32_t to)
{
    (void)target; (void)to;
    event("RETURN_LATERAL");
    return 1;
}

int main(void)
{
    static const char *const eod[] = {
        "SCENE_EOD", "SWEEP_BALL", "ALIGN_BALL", "BALL_STILL",
        "BALL_PRELOWER", "BALL_GRASP", "BALL_LIFT", "TURN_180",
        "SWEEP_BUCKET", "ALIGN_BUCKET", "BUCKET_STILL", "BUCKET_LOWER",
        "BUCKET_RELEASE", "BUCKET_LIFT_OUT", "RACK_RETRACT",
        "RETURN_LATERAL", "TURN_180"
    };
    static const char *const rescue[] = {
        "SCENE_RESCUE", "SWEEP_HOSTAGE", "ALIGN_HOSTAGE", "HOSTAGE_STILL",
        "HOSTAGE_PRELOWER", "HOSTAGE_GRASP", "HOSTAGE_LIFT"
    };
    static const char *const eod_stop_at_still[] = {
        "SCENE_EOD", "SWEEP_BALL", "ALIGN_BALL", "BALL_STILL"
    };
    static const char *const eod_stop_at_release[] = {
        "SCENE_EOD", "SWEEP_BALL", "ALIGN_BALL", "BALL_STILL",
        "BALL_PRELOWER", "BALL_GRASP", "BALL_LIFT", "TURN_180",
        "SWEEP_BUCKET", "ALIGN_BUCKET", "BUCKET_STILL", "BUCKET_LOWER",
        "BUCKET_RELEASE"
    };
    if (!task_eod_config_missing() || !task_rescue_config_missing()) {
        fputs("Expected uncalibrated task gates to remain closed\n", stderr);
        return 1;
    }
    reset(0);
    if (task_eod_run(LAB_R) != TASK_OK || !check("EOD", eod, sizeof eod / sizeof eod[0])) return 1;
    reset(0);
    if (task_rescue_run(LAB_CYL) != TASK_OK ||
        !check("RESCUE", rescue, sizeof rescue / sizeof rescue[0])) return 1;
    reset("BALL_STILL");
    if (task_eod_run(LAB_R) != TASK_ABORT ||
        !check("EOD_STOP_STILL", eod_stop_at_still, sizeof eod_stop_at_still / sizeof eod_stop_at_still[0])) return 1;
    reset("BUCKET_RELEASE");
    if (task_eod_run(LAB_R) != TASK_ABORT ||
        !check("EOD_STOP_RELEASE", eod_stop_at_release, sizeof eod_stop_at_release / sizeof eod_stop_at_release[0])) return 1;
    puts("arm task flow: 4 cases passed");
    return 0;
}
