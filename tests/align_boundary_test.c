/* Real step_align with synthetic OBJ frames and inert hardware.
 * Checks command application, not physical signs, friction or accuracy. */
#include <string.h>
#define main unused_timeout_fixture_main
#define osDelay unused_fixture_delay
#define motion_vel_set unused_fixture_drive
#define motion_brake unused_fixture_brake
#define motion_odo_mm unused_fixture_fore
#define motion_lateral_odo_mm unused_fixture_lateral
#include "align_timeout_test.c"
#undef main
#undef osDelay
#undef motion_vel_set
#undef motion_brake
#undef motion_odo_mm
#undef motion_lateral_odo_mm

static int pixel_error;
static unsigned drive_calls, frame_count;
static float drive_x, drive_y, drive_w;
static float fore_mm, lateral_mm, current_x, current_y;
static int feed_frames = 1, stop_on_command = 1;
static unsigned forward_calls, backward_calls;

float motion_odo_mm(void) { return fore_mm; }
float motion_lateral_odo_mm(void) { return lateral_mm; }

void osDelay(uint32_t ms)
{
    ProtoFrame frame = {0};
    now_ms += ms;
    fore_mm += current_x * (float)ms / 1000.0f;
    lateral_mm += current_y * (float)ms / 1000.0f;
    if (!feed_frames || (feed_frames == 2 && current_x >= 0.0f)) return;
    frame.type = PF_OBJ; frame.cls = CLS_BALL; frame.label = LAB_R;
    frame.cx = 160 + pixel_error; frame.h = 10;
    steps_feed_frame(&frame);
    frame_count++;
}

void motion_brake(void) { brake_calls++; current_x = current_y = 0.0f; }
void motion_vel_set(float x, float y, float w)
{
    drive_calls++; drive_x = x; drive_y = y; drive_w = w;
    current_x = x; current_y = y;
    if (x > 0.0f) forward_calls++;
    if (x < 0.0f) backward_calls++;
    if (stop_on_command) run_abort(); /* inspect first command, not physical motion */
}

#include "../App/steps.c"

static void reset_fixture(void)
{
    now_ms = 0u; brake_calls = drive_calls = frame_count = 0u;
    forward_calls = backward_calls = 0u;
    drive_x = drive_y = drive_w = current_x = current_y = 0.0f;
    fore_mm = 500.0f; lateral_mm = 200.0f;
    feed_frames = stop_on_command = 1;
    run_reset();
}

static int check(int error, int expected_done, float expected_speed)
{
    int done;
    reset_fixture(); pixel_error = error;
    done = step_align(CLS_BALL, LAB_R, 5000u);
    if (done != expected_done || drive_calls != (expected_done ? 0u : 1u) ||
        drive_x != (float)VISION_CX_FWD_SIGN * expected_speed || drive_y != 0.0f || drive_w != 0.0f ||
        brake_calls == 0u || frame_count < X_ALIGN_N) {
        fprintf(stderr, "error=%d done=%d drives=%u v=(%.2f,%.2f,%.2f) frames=%u\n",
                error, done, drive_calls, drive_x, drive_y, drive_w, frame_count);
        return 0;
    }
    return 1;
}

int main(void)
{
    reset_fixture();
    if (VISION_CX_FWD_SIGN == 0) {
        if (step_align(CLS_BALL, LAB_R, 5000u) || drive_calls || now_ms ||
            strcmp(steps_config_missing(), "VISION_CX_FWD_SIGN") != 0) return 1;
        puts("left-camera gate: uncalibrated pixel sign rejects alignment without drive");
        return 0;
    }
    if (!check(0, 1, 0.0f)) return 1;
    if (!check(8, 1, 0.0f) || !check(-8, 1, 0.0f)) return 1;
    if (!check(9, 0, 12.0f) || !check(-9, 0, -12.0f)) return 1;
    if (!check(19, 0, 12.0f) || !check(-19, 0, -12.0f)) return 1;
    if (!check(20, 0, 12.0f) || !check(-20, 0, -12.0f)) return 1;
    if (!check(140, 0, 80.0f) || !check(-140, 0, -80.0f)) return 1;
    reset_fixture(); feed_frames = 0;
    if (align_depth_move(50.0f, 5000u) || drive_calls != 1u ||
        drive_x != 0.0f || drive_y != -X_DEPTH_V) return 1;
    reset_fixture(); feed_frames = 0;
    if (align_depth_move(-50.0f, 5000u) || drive_calls != 1u ||
        drive_x != 0.0f || drive_y != X_DEPTH_V) return 1;
    reset_fixture(); feed_frames = 0;
    if (step_return_forward_odo(510.0f, 5000u) || drive_calls != 1u ||
        drive_x != SWEEP_FWD_MMS || drive_y != 0.0f) return 1;
    reset_fixture(); feed_frames = 0;
    if (step_return_forward_odo(490.0f, 5000u) || drive_calls != 1u ||
        drive_x != -SWEEP_FWD_MMS || drive_y != 0.0f) return 1;
    reset_fixture(); feed_frames = 2; stop_on_command = 0;
    if (!step_sweep(PF_OBJ, CLS_BALL, LAB_R, 0, 5000u) ||
        forward_calls == 0u || backward_calls == 0u || lateral_mm != 200.0f ||
        fore_mm <= 500.0f || fore_mm > 510.0f || current_x || current_y) return 1;
    puts("left-camera axes: 11 alignment boundaries + depth2 + return2 + bounded scan1 passed");
    return 0;
}
