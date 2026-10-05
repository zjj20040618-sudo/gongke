/* Host-only call-order check for the real ANTI task with fake dependencies. */
#include "robot_tasks.h"
#include "steps.h"
#include "proto.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>

static const char *events[8];
static unsigned count;
static unsigned now_ms;
static int target_ok, settle_ok, fire_ok, aborted;
static uint8_t selected_digit;

static void record(const char *event)
{
    if (count < sizeof events / sizeof events[0]) events[count++] = event;
}

int run_aborted(void) { return aborted; }
int step_vision_target(ProtoTask task, uint8_t digit)
{
    selected_digit = digit;
    record(task == PROTO_TASK_TARGET ? "VISION_TARGET" : "WRONG_TARGET");
    if (!target_ok) { aborted = 1; return 0; }
    return !aborted;
}
int step_sweep(int want, int cls, int label, int32_t d[3], uint32_t to)
{
    record(want == PF_OBJ && cls == CLS_TARGET && label == (int)selected_digit - 1 &&
           d == 0 && to == 0u && !aborted ? "SWEEP_TARGET" : "WRONG_SWEEP");
    return 1;
}
int step_align(int cls, int label, uint32_t to)
{
    record(cls == CLS_TARGET && label == (int)selected_digit - 1 && to == 0u && !aborted ?
           "ALIGN_TARGET" : "WRONG_ALIGN");
    return 1;
}
int step_target_settle(void)
{
    record("SETTLE_1S");
    if (!settle_ok) { aborted = 1; return 0; }
    now_ms += TARGET_AIM_SETTLE_MS;
    return 1;
}
int step_fire(uint32_t hold_ms)
{
    record(hold_ms == 2000u && now_ms == 1000u && !aborted ? "FIRE_2S" : "WRONG_FIRE_TIME");
    if (!fire_ok) { aborted = 1; return 0; }
    now_ms += hold_ms;
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
        "VISION_TARGET", "SWEEP_TARGET", "ALIGN_TARGET", "SETTLE_1S", "FIRE_2S"
    };
    static const char *const failed_still[] = {
        "VISION_TARGET", "SWEEP_TARGET", "ALIGN_TARGET", "SETTLE_1S"
    };
    static const int invalid_colors[] = { INT_MIN, -257, -256, -1, LAB_CYL, 255, 256, INT_MAX };
    if (TARGET_AIM_SETTLE_MS != 1000u || TARGET_LASER_ON_MS != 2000u) return 1;
    target_ok = settle_ok = fire_ok = 1;
    for (int color = LAB_R; color <= LAB_B; ++color) {
        aborted = 0; count = now_ms = 0u;
        if (task_anti_run(color) != TASK_OK ||
            !check("anti_success", success, sizeof success / sizeof success[0]) ||
            selected_digit != (uint8_t)(color + 1) || now_ms != 3000u) return 1;
    }
    settle_ok = 0;
    aborted = 0; count = now_ms = 0u;
    if (task_anti_run(LAB_R) != TASK_ABORT ||
        !check("anti_settle_abort", failed_still, sizeof failed_still / sizeof failed_still[0]) || now_ms != 0u) return 1;
    settle_ok = 1; fire_ok = 0;
    aborted = 0; count = now_ms = 0u;
    if (task_anti_run(LAB_R) != TASK_ABORT ||
        !check("anti_laser_abort", success, sizeof success / sizeof success[0]) || now_ms != 1000u) return 1;
    aborted = 1; count = now_ms = 0u;
    if (task_anti_run(LAB_R) != TASK_ABORT || count != 0u || now_ms != 0u) return 1;
    target_ok = 0; settle_ok = fire_ok = 1;
    aborted = 0; count = now_ms = 0u;
    if (task_anti_run(LAB_R) != TASK_ABORT || !check("anti_ack_stop", success, 1u) || now_ms != 0u) return 1;
    target_ok = 1;
    for (unsigned i = 0u; i < sizeof invalid_colors / sizeof invalid_colors[0]; ++i) {
        aborted = 0; count = now_ms = 0u;
        if (task_anti_run(invalid_colors[i]) != TASK_ABORT || count != 0u || now_ms != 0u) return 1;
    }
    puts("anti task flow: selected TARGET digits1..3, stopped1s then laser2s, invalid labels and handshake/settle/laser/entry stops passed (host mocks)");
    return 0;
}
