/* Host-only flow check. Compile with task_eod.c/task_rescue.c and these mocks;
 * no firmware values are changed and no hardware movement is simulated. */
#include "robot_tasks.h"
#include "steps.h"
#include "proto.h"
#include "motion.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

static const char *events[32];
static unsigned event_count;
static const char *fail_stage;
static int aborted;
static unsigned selected_count;
static ProtoTask selected_tasks[2];
static uint8_t selected_digits[2];

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
    aborted = 0;
    selected_count = 0;
    memset(selected_tasks, 0, sizeof selected_tasks);
    memset(selected_digits, 0, sizeof selected_digits);
}

int run_aborted(void) { return aborted; }
int step_vision_target(ProtoTask task, uint8_t digit)
{
    const char *name = task == PROTO_TASK_BALL ? "VISION_BALL" :
                       task == PROTO_TASK_BUCKET ? "VISION_BUCKET" :
                       task == PROTO_TASK_HOSTAGE ? "VISION_HOSTAGE" : "WRONG_TARGET";
    event(name);
    if (selected_count < 2u) {
        selected_tasks[selected_count] = task;
        selected_digits[selected_count] = digit;
    }
    selected_count++;
    if (fail_stage && strcmp(name, fail_stage) == 0) { aborted = 1; return 0; }
    return !aborted;
}
static int target_matches(int cls, int label)
{
    unsigned i = selected_count - 1u;
    if (!selected_count || selected_count > 2u) return 0;
    if (cls == CLS_BALL)
        return selected_tasks[i] == PROTO_TASK_BALL && label == (int)selected_digits[i] - 1;
    if (cls == CLS_BUCKET)
        return selected_tasks[i] == PROTO_TASK_BUCKET && selected_digits[i] == 0u && label == -1;
    return cls == CLS_HOSTAGE && selected_tasks[i] == PROTO_TASK_HOSTAGE &&
           label == (int)selected_digits[i] + 2;
}
int step_sweep(int want, int cls, int label, int32_t d[3], uint32_t to)
{
    event(cls == CLS_BALL ? "SWEEP_BALL" : cls == CLS_BUCKET ? "SWEEP_BUCKET" : "SWEEP_HOSTAGE");
    return !aborted && want == PF_OBJ && d == 0 && to == 0u && target_matches(cls, label);
}
int step_align(int cls, int label, uint32_t to)
{
    event(cls == CLS_BALL ? "ALIGN_BALL" : cls == CLS_BUCKET ? "ALIGN_BUCKET" : "ALIGN_HOSTAGE");
    return !aborted && to == 0u && target_matches(cls, label);
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
    return deg == 180 && (!fail_stage || strcmp("TURN_180", fail_stage) != 0);
}
float motion_odo_mm(void) { return 0.0f; }
int step_return_forward_odo(float target, uint32_t to)
{
    (void)target; (void)to;
    event("RETURN_FORWARD");
    return 1;
}

int main(void)
{
    static const char *const eod[] = {
        "VISION_BALL", "SWEEP_BALL", "ALIGN_BALL", "BALL_STILL",
        "BALL_PRELOWER", "BALL_GRASP", "BALL_LIFT", "TURN_180",
        "VISION_BUCKET", "SWEEP_BUCKET", "ALIGN_BUCKET", "BUCKET_STILL", "BUCKET_LOWER",
        "BUCKET_RELEASE", "BUCKET_LIFT_OUT", "RACK_RETRACT",
        "RETURN_FORWARD", "TURN_180"
    };
    static const char *const rescue[] = {
        "VISION_HOSTAGE", "SWEEP_HOSTAGE", "ALIGN_HOSTAGE", "HOSTAGE_STILL",
        "HOSTAGE_PRELOWER", "HOSTAGE_GRASP", "HOSTAGE_LIFT"
    };
    static const char *const eod_stop_at_still[] = {
        "VISION_BALL", "SWEEP_BALL", "ALIGN_BALL", "BALL_STILL"
    };
    static const char *const eod_stop_at_release[] = {
        "VISION_BALL", "SWEEP_BALL", "ALIGN_BALL", "BALL_STILL",
        "BALL_PRELOWER", "BALL_GRASP", "BALL_LIFT", "TURN_180",
        "VISION_BUCKET", "SWEEP_BUCKET", "ALIGN_BUCKET", "BUCKET_STILL", "BUCKET_LOWER",
        "BUCKET_RELEASE"
    };
    static const int invalid_colors[] = { INT_MIN, -257, -256, -1, LAB_CYL, 255, 256, INT_MAX };
    static const int invalid_shapes[] = { INT_MIN, -253, -2, LAB_R, LAB_G, LAB_B,
                                          LAB_WAIST + 1, 255, 259, 260, INT_MAX };
    unsigned i;
    if (PROTO_TASK_BALL != 1 || PROTO_TASK_TARGET != 2 ||
        PROTO_TASK_HOSTAGE != 3 || PROTO_TASK_BUCKET != 4) return 1;
    if (!task_eod_config_missing() || !task_rescue_config_missing()) {
        fputs("Expected uncalibrated task gates to remain closed\n", stderr);
        return 1;
    }
    for (int color = LAB_R; color <= LAB_B; ++color) {
        reset(0);
        if (task_eod_run(color) != TASK_OK || !check("EOD", eod, sizeof eod / sizeof eod[0]) ||
            selected_count != 2u || selected_tasks[0] != PROTO_TASK_BALL ||
            selected_digits[0] != (uint8_t)(color + 1) || selected_tasks[1] != PROTO_TASK_BUCKET ||
            selected_digits[1] != 0u) return 1;
    }
    for (int shape = LAB_CYL; shape <= LAB_WAIST; ++shape) {
        reset(0);
        if (task_rescue_run(shape) != TASK_OK || !check("RESCUE", rescue, sizeof rescue / sizeof rescue[0]) ||
            selected_count != 1u || selected_tasks[0] != PROTO_TASK_HOSTAGE ||
            selected_digits[0] != (uint8_t)(shape - 2)) return 1;
    }
    reset("BALL_STILL");
    if (task_eod_run(LAB_R) != TASK_ABORT ||
        !check("EOD_STOP_STILL", eod_stop_at_still, sizeof eod_stop_at_still / sizeof eod_stop_at_still[0])) return 1;
    reset("BUCKET_RELEASE");
    if (task_eod_run(LAB_R) != TASK_ABORT ||
        !check("EOD_STOP_RELEASE", eod_stop_at_release, sizeof eod_stop_at_release / sizeof eod_stop_at_release[0])) return 1;
    reset("VISION_BALL");
    if (task_eod_run(LAB_R) != TASK_ABORT || !check("EOD_BALL_ACK_STOP", eod, 1u)) return 1;
    reset("VISION_BUCKET");
    if (task_eod_run(LAB_R) != TASK_ABORT || !check("EOD_BUCKET_ACK_STOP", eod, 9u)) return 1;
    reset("VISION_HOSTAGE");
    if (task_rescue_run(LAB_CYL) != TASK_ABORT || !check("RESCUE_ACK_STOP", rescue, 1u)) return 1;
    reset("TURN_180");
    if (task_eod_run(LAB_R) != TASK_ABORT || !check("EOD_TURN_STOP", eod, 8u) || selected_count != 1u) return 1;
    reset(0); aborted = 1;
    if (task_eod_run(LAB_R) != TASK_ABORT || event_count || selected_count) return 1;
    reset(0); aborted = 1;
    if (task_rescue_run(LAB_CYL) != TASK_ABORT || event_count || selected_count) return 1;
    for (i = 0u; i < sizeof invalid_colors / sizeof invalid_colors[0]; ++i) {
        reset(0);
        if (task_eod_run(invalid_colors[i]) != TASK_ABORT || event_count || selected_count) return 1;
    }
    for (i = 0u; i < sizeof invalid_shapes / sizeof invalid_shapes[0]; ++i) {
        reset(0);
        if (task_rescue_run(invalid_shapes[i]) != TASK_ABORT || event_count || selected_count) return 1;
    }
    puts("arm task flow: selected BALL/HOSTAGE digits1..3, TURN180 -> BUCKET0 handshake, unchanged arm order/gates, invalid labels and entry/handshake/turn stops passed (host mocks)");
    return 0;
}
