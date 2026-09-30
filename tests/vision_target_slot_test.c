/* Reuse existing fake hardware; include real private slot implementation. */
#define main alignment_fixture_main
#include "align_timeout_test.c"
#undef main
#include "../App/steps.c"

int main(void)
{
    ProtoFrame red = {0}, blue = {0}, qr = {0}, out;
    red.type = blue.type = PF_OBJ;
    red.cls = blue.cls = CLS_BALL;
    red.label = LAB_R; blue.label = LAB_B;
    qr.type = PF_QR; qr.a = qr.b = qr.c = 1;
    run_reset();
    select_object(CLS_BALL, LAB_R);
    steps_feed_frame(&red); steps_feed_frame(&blue); steps_feed_frame(&qr);
    if (!take_frame(PF_OBJ, &out) || out.label != LAB_R) return 1;
    if (!take_frame(PF_QR, &out) || out.a != 1) return 1;
    steps_feed_frame(&red);
    select_object(CLS_BUCKET, -1);
    if (take_frame(PF_OBJ, &out)) return 1;
    steps_feed_frame(&red);
    if (take_frame(PF_OBJ, &out)) return 1;
    run_reset(); steps_feed_frame(&blue);
    if (!take_frame(PF_OBJ, &out) || out.label != LAB_B) return 1;
    puts("vision target slot: filter / QR separation / phase reset / run reset passed");
    return 0;
}
