/* Host-only synthetic byte replay; timestamps permit incomplete-frame gap tests. */
#include "proto.h"
#include <stdio.h>
#include <string.h>

static uint32_t now_ms;
uint32_t HAL_GetTick(void) { return now_ms; }
static void received(const ProtoFrame *f)
{
    if (f->type == PF_QR) printf("QR,%d,%d,%d,%u\n", f->a, f->b, f->c, f->sequence);
    else if (f->type == PF_OBJ) printf("OBJ,%d,%d,%d,%d,%d,%d,%d,%u,%u,%u\n",
        f->cls, f->label, f->cx, f->cy, f->w, f->h, f->conf, f->sequence, f->img_w, f->img_h);
}
static int nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
int main(void)
{
    char hex[1024];
    unsigned long tick;
    ProtoStats stats;
    proto_init(); proto_set_binary_mode(1); proto_set_on_frame(received);
    while (scanf("%lu %1023s", &tick, hex) == 2) {
        const size_t n = strlen(hex);
        now_ms = (uint32_t)tick;
        if (n % 2u) return 1;
        for (size_t i = 0; i < n; i += 2u) {
            int high = nibble(hex[i]), low = nibble(hex[i + 1u]);
            if (high < 0 || low < 0) return 1;
            proto_feed_byte((uint8_t)((high << 4) | low));
        }
    }
    proto_stats_get(&stats);
    printf("STATS,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu\n",
        (unsigned long)stats.accepted, (unsigned long)stats.qr, (unsigned long)stats.obj,
        (unsigned long)stats.crc_bad, (unsigned long)stats.binary_bad,
        (unsigned long)stats.binary_gap, (unsigned long)stats.binary_unmapped,
        (unsigned long)stats.duplicate);
    return 0;
}
