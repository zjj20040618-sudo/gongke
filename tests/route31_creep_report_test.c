/* Actual report formatter plus inherited peripheral mocks. Not UART or wheel
 * hardware acceptance. No additional polling/transmit stream is introduced. */
#define main creep_report_inherited_main
#include "g_command_stop_test.c"
#undef main

static void fine_wire(ProtoWireDiag *out)
{
    memset(out,0,sizeof *out);
    out->receiving=out->ack=1u;out->mode=2u;
    out->request=s_vat.request;out->task=s_vat.task;out->selection=s_vat.digit;
}

static void prepare_report(unsigned owner)
{
    reset_fixture();s_seq_mode=(uint8_t)owner;s_seq_stage=ROUTE31_PAIR_STAGE;
    s_seq_state=SQ_TASK;s_route31_lift_phase=R31_LIFT_OFF;
    s_vat.state=VAT_FINE_CONTINUOUS;s_vat.reason="CONTINUOUS_X";
    s_vat.request=9u;s_vat.task=PROTO_TASK_BALL;s_vat.digit=1u;
    host_wire_diag_hook=fine_wire;
    host_counts[0]=-10;host_counts[1]=20;host_counts[2]=-30;host_counts[3]=40;
    host_uart_calls=0u;host_messages[0]='\0';
}

int main(void)
{
    prepare_report(31u);route31_task_report();
    CHECK(host_uart_calls==2u);
    CHECK(strstr(host_messages,"fine_ctrl=creep wheels=LR,RR,RF,LF c=-10,20,-30,40 rpm=0.0,0.0,0.0,0.0\r\n"));
    CHECK(strlen(host_messages)<896u);
    prepare_report(43u);route31_task_report();
    CHECK(host_uart_calls==2u && !strstr(host_messages,"fine_ctrl=creep"));
    prepare_report(31u);s_vat.state=VAT_RECHECK;route31_task_report();
    CHECK(host_uart_calls==2u && !strstr(host_messages,"fine_ctrl=creep"));
    prepare_report(31u);host_wire_diag_hook=NULL;route31_task_report();
    CHECK(host_uart_calls==2u && strstr(host_messages,"WAIT_TASK_ACK") &&
          !strstr(host_messages,"fine_ctrl=creep"));
    puts("31 fine wheel evidence: actual report appends LR/RR/RF/LF encoder and RPM to one existing record only;43/coarse/ACK-wait unchanged passed");
    return 0;
}
