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
    steps_feed_frame(&blue); steps_feed_frame(&qr);
    step_vision_receive_end();
    if (!s_vision_receive_closed || s_obj_pending || s_qr_pending
        || alignment_receive_end_calls != 1u) return 1;
    step_object_select(CLS_BALL, LAB_B); /* selection alone cannot reopen a closed stage */
    steps_feed_frame(&blue); steps_feed_frame(&qr);
    if (take_frame(PF_OBJ, &out) || take_frame(PF_QR, &out)) return 1;
    run_reset(); /* clears abort/slots, but must not cancel a boot/R1 QR in proto */
    if (alignment_receive_end_calls != 1u || s_vision_receive_closed) return 1;
    steps_feed_frame(&blue); steps_feed_frame(&qr);
    run_abort();
    steps_feed_frame(&blue); steps_feed_frame(&qr);
    if (!run_aborted() || !s_vision_receive_closed || take_frame(PF_OBJ, &out)
        || take_frame(PF_QR, &out) || alignment_receive_end_calls != 2u) return 1;
    puts("vision target slot: filter / QR separation / phase reset / local-close late-frame drop / abort / boot-QR-preserving reset passed");
    return 0;
}
