/* Host-only call-order check for the real ANTI task with fake dependencies. */
#include "robot_tasks.h"
#include "steps.h"
#include "proto.h"
#include <stdio.h>
#include <string.h>

static const char *events[8];
static unsigned count;
static int still_ok;

static void record(const char *event)
{
    if (count < sizeof events / sizeof events[0]) events[count++] = event;
}

int run_aborted(void) { return 0; }
void proto_send_scene(ProtoScene scene)
{
    record(scene == SCENE_ANTI ? "SCENE_ANTI" : "WRONG_SCENE");
}
int step_sweep(int want, int cls, int label, int32_t d[3], uint32_t to)
{
    (void)label; (void)d; (void)to;
    record(want == PF_OBJ && cls == CLS_TARGET ? "SWEEP_TARGET" : "WRONG_SWEEP");
    return 1;
}
int step_align(int cls, int label, uint32_t to)
{
    (void)label; (void)to;
    record(cls == CLS_TARGET ? "ALIGN_TARGET" : "WRONG_ALIGN");
    return 1;
}
int step_prepare_leg(void)
{
    record("STILL_CHECK");
    return still_ok;
}
int step_fire(uint32_t hold_ms)
{
    record(hold_ms == 2000u ? "FIRE" : "WRONG_FIRE_TIME");
    return 1;
}

static int check(const char *name, const char *const *want, unsigned expected)
{
    unsigned i;
    if (count != expected) {
        fprintf(stderr, "%s: got %u events, expected %u\n", name, count, expected);
        return 0;
    }
    for (i = 0; i < expected; ++i) {
        if (strcmp(events[i], want[i]) != 0) {
            fprintf(stderr, "%s: event %u is %s, expected %s\n", name, i, events[i], want[i]);
            return 0;
        }
    }
    return 1;
}

int main(void)
{
    static const char *const success[] = {
        "SCENE_ANTI", "SWEEP_TARGET", "ALIGN_TARGET", "STILL_CHECK", "FIRE"
    };
    static const char *const failed_still[] = {
        "SCENE_ANTI", "SWEEP_TARGET", "ALIGN_TARGET", "STILL_CHECK"
    };
    still_ok = 1;
    count = 0;
    if (task_anti_run(LAB_R) != TASK_OK ||
        !check("anti_success", success, sizeof success / sizeof success[0])) return 1;
    still_ok = 0;
    count = 0;
    if (task_anti_run(LAB_R) != TASK_ABORT ||
        !check("anti_still_fail", failed_still, sizeof failed_still / sizeof failed_still[0])) return 1;
    puts("anti task flow: 2 cases passed");
    return 0;
}
