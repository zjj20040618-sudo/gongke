/* Host-only test of real ASCII vision parser and diagnostic counters. */
#include "proto.h"
#include <stdio.h>
#include <string.h>

static ProtoFrame last_frame;
static unsigned callbacks;
static unsigned transmissions;
uint32_t HAL_GetTick(void) { return 0u; }

static void on_tx(const char *message) { (void)message; transmissions++; }

static void on_frame(const ProtoFrame *frame)
{
    last_frame = *frame;
    callbacks++;
}

static void feed(const char *line)
{
    while (*line) proto_feed_byte((uint8_t)*line++);
}

int main(void)
{
    ProtoStats stats;
    proto_init();
    proto_set_on_frame(on_frame);
    proto_set_tx(on_tx);
    proto_send_scene(SCENE_QR);
    proto_send_ping();
    if (transmissions != 2u) return 1;
    proto_set_binary_mode(1);
    proto_send_scene(SCENE_QR);
    proto_send_ping();
    if (transmissions != 2u) return 1; /* no invented binary commands */
    proto_set_binary_mode(0);

    feed("QR,1,2,3\r\n");
    if (callbacks != 1u || last_frame.type != PF_QR ||
        last_frame.a != 1 || last_frame.b != 2 || last_frame.c != 3) return 1;

    feed("QR,1,2,x\n");        /* correct field count, bad numeric token */
    feed("QR,1,2\n");          /* missing task selection */
    feed("QR,1,2,3,4\n");      /* extra task selection */
    feed("OBJ,0,1,10,20,x,40,90\n");
    feed("QR,1,2,2147483648\n"); /* integer overflow */
    if (callbacks != 1u) {
        fputs("Malformed frames were delivered to the callback\n", stderr);
        return 1;
    }

    feed("OBJ,0,1,10,20,30,40,90\n");
    if (callbacks != 2u || last_frame.type != PF_OBJ ||
        last_frame.cls != 0 || last_frame.label != 1 ||
        last_frame.cx != 10 || last_frame.cy != 20 ||
        last_frame.w != 30 || last_frame.h != 40 || last_frame.conf != 90) return 1;

    feed("READY\n");
    if (callbacks != 3u || last_frame.type != PF_READY) return 1;

    feed("QR,1,2,3,xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx\n");
    if (callbacks != 3u) return 1; /* overlong line must drop in full */

    proto_stats_get(&stats);
    if (stats.lines != 8u || stats.accepted != 3u || stats.rejected != 5u ||
        stats.qr != 1u || stats.obj != 1u || stats.overflow != 1u) {
        fprintf(stderr, "Bad stats: lines=%lu accepted=%lu rejected=%lu qr=%lu obj=%lu overflow=%lu\n",
                (unsigned long)stats.lines, (unsigned long)stats.accepted,
                (unsigned long)stats.rejected, (unsigned long)stats.qr,
                (unsigned long)stats.obj, (unsigned long)stats.overflow);
        return 1;
    }
    puts("vision ASCII parser: 9 frames checked, including overflow drop");
    return 0;
}
