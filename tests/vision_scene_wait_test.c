/* Real steps.c handshake: old slots cleared, no motion while waiting, abort remains effective. */
#include "steps.h"
#define main unused_alignment_main
#define osDelay unused_alignment_delay
#define proto_send_scene unused_alignment_scene
#define proto_scene_status unused_alignment_status
#include "align_timeout_test.c"
#undef main
#undef osDelay
#undef proto_send_scene
#undef proto_scene_status
#include "../App/steps.c"

static int status, outcome;
static unsigned requests, delays;
static ProtoScene last_scene;
void proto_send_scene(ProtoScene scene) { requests++; last_scene = scene; }
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
    s_qr_pending = s_obj_pending = 1; /* old phase, must not survive new request */
}
int main(void)
{
    reset_fixture(1);
    if (!step_vision_scene(SCENE_QR) || requests != 1u || delays != 2u
        || brake_calls != 1u || s_qr_pending || s_obj_pending) return 1;
    reset_fixture(-1);
    if (step_vision_scene(SCENE_EOD) || !run_aborted() || requests != 2u
        || last_scene != SCENE_IDLE) return 1;
    reset_fixture(2);
    if (step_vision_scene(SCENE_ANTI) || !run_aborted() || requests != 2u
        || last_scene != SCENE_IDLE) return 1;
    reset_fixture(1); run_abort(); requests = 0u;
    if (step_vision_scene(SCENE_RESCUE) || requests || delays) return 1;
    puts("vision stage wait: ACK/fresh readiness, clear old slots, failure, during/before abort passed");
    return 0;
}
