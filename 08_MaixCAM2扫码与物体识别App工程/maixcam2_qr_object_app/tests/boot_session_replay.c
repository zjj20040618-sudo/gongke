/* Host bridge to the unchanged MCU public API. No GPIO/RNG/motion. */
#include "proto.h"
#include <stdio.h>
#include <string.h>
static uint32_t tick;
static unsigned objects, qrs;
uint32_t HAL_GetTick(void) { return tick; }
static void tx(const uint8_t *data, uint16_t length) {
    printf("TX,");
    for (unsigned i = 0; i < length; ++i) printf("%02x", data[i]);
    puts("");
}
static void rx(const ProtoFrame *frame) {
    if (frame->type == PF_OBJ) ++objects;
    if (frame->type == PF_QR) ++qrs;
}
static int bytes(const char *text, uint8_t *data, unsigned capacity) {
    unsigned n = (unsigned)strlen(text);
    if (n % 2 || n / 2 > capacity) return -1;
    for (unsigned i = 0; i < n / 2; ++i) {
        unsigned value;
        if (sscanf(text + i * 2, "%2x", &value) != 1) return -1;
        data[i] = (uint8_t)value;
    }
    return (int)(n / 2);
}
int main(void) {
    char line[1024];
    uint8_t data[500];
    while (scanf("%1023s", line) == 1) {
        ++tick;
        if (line[0] == 'B') {
            if (bytes(line + 1, data, 8) != 8) return 2;
            proto_init(); proto_set_binary_mode(1);
            proto_set_binary_tx(tx); proto_set_on_frame(rx); objects = qrs = 0;
            if (!proto_session_begin(data)) return 3;
            proto_qr_begin();
        } else if (line[0] == 'T' && strlen(line) == 3) {
            if (!proto_send_target((ProtoTask)(line[1] - '0'), (uint8_t)(line[2] - '0'))) return 4;
        } else if (!strcmp(line, "#")) proto_receive_end();
        else if (strcmp(line, "?") && strcmp(line, "~")) {
            int n = bytes(line, data, sizeof data);
            if (n < 0) return 5;
            for (int i = 0; i < n; ++i) proto_feed_byte(data[i]);
        }
        proto_service();
        ProtoSessionDiag session; ProtoWireDiag wire; ProtoTargetRank rank;
        int32_t qr[3] = {0, 0, 0};
        proto_session_diag_get(&session); proto_wire_diag_get(&wire);
        int ranked = proto_target_rank_get(&rank), decoded = proto_qr_get(qr);
        printf("STATE,%u,%u,%u,%u,%u,%u,%d,%u,%d,%ld,%ld,%ld,%u,%u\n",
            session.state, wire.request, wire.ack, wire.fresh, wire.task, wire.selection,
            ranked, ranked ? rank.rank : 0, decoded,
            (long)qr[0], (long)qr[1], (long)qr[2], objects, qrs);
        puts("END"); fflush(stdout);
    }
    return 0;
}
