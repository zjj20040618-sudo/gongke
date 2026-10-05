/* Real robot.c callbacks and DefaultTask recovery, with a HAL-order fixture.
 * Models this repository's one-byte UART_Receive_IT -> RxCplt -> error branch
 * ordering. Host-only: no motors, serial hardware, firmware flash or timing
 * acceptance. Compile with tests/uart_stubs and App plus section GC. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../App/robot.c"

#define CHECK(c) do { if (!(c)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(1); } } while (0)

UART_HandleTypeDef huart2, huart3, huart4;
static unsigned vision_bytes, imu_bytes, bt_bytes;
static uint8_t vision_last, imu_last, bt_last;
static unsigned service_calls, poll_calls;

uint32_t HAL_UART_GetError(UART_HandleTypeDef *h) { return h->ErrorCode; }
uint32_t HAL_GetTick(void) { return 0u; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *rx, uint16_t n)
{
    CHECK(n == 1u && rx && h->RxState == HAL_UART_STATE_READY);
    h->arm_calls++;
    if (h->fail_next_arm) { h->fail_next_arm--; return HAL_BUSY; }
    h->rx = rx;
    h->ErrorCode = HAL_UART_ERROR_NONE; /* Real UART_Start_Receive_IT behavior. */
    h->RxState = HAL_UART_STATE_BUSY_RX;
    h->rx_enabled = 1u;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t)
{ (void)h; (void)p; (void)n; (void)t; return HAL_OK; }
void host_uart_clear_ore(UART_HandleTypeDef *h)
{
    h->clear_calls++;
    h->dr_reads++;
    h->ore_flag = h->rxne_flag = 0u; /* SR/DR clears ORE and consumes DR. */
}
void proto_feed_byte(uint8_t c) { vision_bytes++; vision_last = c; }
void imu_feed(uint8_t c) { imu_bytes++; imu_last = c; }
void test_feed(uint8_t c) { bt_bytes++; bt_last = c; }
void proto_service(void) { service_calls++; }
int proto_qr_take_notice(int32_t out[3]) { (void)out; return 0; }
void test_poll(void) { poll_calls++; }
void bp_debug_send(const char *s) { (void)s; }

static unsigned consumed(unsigned ix)
{
    if (ix == 0u) return vision_bytes;
    if (ix == 2u) return imu_bytes;
    return (unsigned)((s_bt_wr - s_bt_rd) & (BT_RX_N - 1u)) + bt_bytes;
}

static void reset_fixture(void)
{
    memset(&huart2, 0, sizeof huart2); memset(&huart3, 0, sizeof huart3);
    memset(&huart4, 0, sizeof huart4);
    huart2.RxState = huart3.RxState = huart4.RxState = HAL_UART_STATE_READY;
    s_bt_wr = s_bt_rd = 0u; s_bt_drop = 0u;
    for (unsigned i = 0u; i < 3u; i++) s_uart_err[i] = s_uart_last_err[i] = s_uart_arm_fail[i] = 0u;
    vision_bytes = imu_bytes = bt_bytes = service_calls = poll_calls = 0u;
    vision_last = imu_last = bt_last = 0u;
    uart_rx_ensure_all();
}

/* Actual HAL one-byte receive ordering: DR read drains RXNE/ORE, completion
 * marks READY and disables RX before invoking the real application callback. */
static void receive_complete(UART_HandleTypeDef *h)
{
    CHECK(h->rx_enabled && h->RxState == HAL_UART_STATE_BUSY_RX && h->rxne_flag);
    *h->rx = h->dr; h->dr_reads++;
    h->rxne_flag = h->ore_flag = 0u;
    h->RxState = HAL_UART_STATE_READY; h->rx_enabled = 0u;
    HAL_UART_RxCpltCallback(h);
}

static void good_irq(UART_HandleTypeDef *h, uint8_t c)
{
    h->dr = c; h->rxne_flag = 1u;
    CHECK(h->ErrorCode == HAL_UART_ERROR_NONE);
    receive_complete(h);
    CHECK(h->RxState == HAL_UART_STATE_BUSY_RX && h->rx_enabled);
}

static void error_irq(UART_HandleTypeDef *h, unsigned ix, uint32_t error,
                      int has_rxne, int next_byte_before_callback)
{
    unsigned before = consumed(ix), arms = h->arm_calls, clears = h->clear_calls;
    h->ErrorCode = error;
    h->ore_flag = (error & HAL_UART_ERROR_ORE) != 0u;
    if (has_rxne) {
        h->dr = 0xAAu; h->rxne_flag = 1u;
        receive_complete(h);
        CHECK(h->ErrorCode == error); /* Do not erase HAL's error classification. */
        CHECK(h->arm_calls == arms && !h->rx_enabled);
    }
    CHECK(consumed(ix) == before); /* Bad bytes never enter any parser/ring. */
    if (error & HAL_UART_ERROR_ORE) {
        /* UART_EndRxTransfer occurs AFTER RxCplt and BEFORE ErrorCallback. */
        h->RxState = HAL_UART_STATE_READY; h->rx_enabled = 0u;
    }
    if (next_byte_before_callback) {
        CHECK(has_rxne && !h->ore_flag);
        h->dr = 0x42u; h->rxne_flag = 1u;
    }
    HAL_UART_ErrorCallback(h);
    CHECK(s_uart_last_err[ix] == error && s_uart_err[ix] == 1u);
    if (!(error & HAL_UART_ERROR_ORE)) h->ErrorCode = HAL_UART_ERROR_NONE;
    CHECK(consumed(ix) == before);
    if (has_rxne) CHECK(h->clear_calls == clears); /* No second DR read. */
    if (!has_rxne && (error & HAL_UART_ERROR_ORE)) CHECK(h->clear_calls == clears + 1u);
    if (next_byte_before_callback) {
        CHECK(h->rxne_flag && h->dr == 0x42u);
        receive_complete(h);
        CHECK(consumed(ix) == before + 1u);
    }
}

static void check_normal_three_ports(void)
{
    reset_fixture();
    good_irq(&huart2, 0x12u); good_irq(&huart4, 0x34u); good_irq(&huart3, 0x56u);
    CHECK(vision_bytes == 1u && vision_last == 0x12u);
    CHECK(imu_bytes == 1u && imu_last == 0x34u);
    CHECK(bt_bytes == 0u && s_bt_wr != s_bt_rd); /* BT parsing stays outside ISR. */
    robot_bt_service(); CHECK(bt_bytes == 1u && bt_last == 0x56u);
    CHECK(service_calls == 1u && poll_calls == 1u);
    CHECK(huart2.arm_calls == 2u && huart3.arm_calls == 2u && huart4.arm_calls == 2u);
    CHECK(s_uart_err[0] == 0u && s_uart_err[1] == 0u && s_uart_err[2] == 0u);
    puts("UART normal: three real RX callbacks rearm; Bluetooth parses only in DefaultTask passed");
}

static void check_error_classes(void)
{
    const uint32_t errors[] = {HAL_UART_ERROR_FE, HAL_UART_ERROR_NE, HAL_UART_ERROR_PE,
        HAL_UART_ERROR_ORE, HAL_UART_ERROR_FE | HAL_UART_ERROR_NE | HAL_UART_ERROR_PE | HAL_UART_ERROR_ORE};
    for (unsigned ix = 0u; ix < 3u; ix++) for (unsigned e = 0u; e < sizeof errors / sizeof errors[0]; e++) {
        reset_fixture();
        UART_HandleTypeDef *h = ix == 0u ? &huart2 : ix == 1u ? &huart3 : &huart4;
        error_irq(h, ix, errors[e], 1, 0);
        CHECK(h->arm_calls == 2u && h->RxState == HAL_UART_STATE_BUSY_RX && h->rx_enabled);
        CHECK(h->ErrorCode == HAL_UART_ERROR_NONE);
        good_irq(h, 0x77u); CHECK(consumed(ix) == 1u);
        robot_bt_service(); CHECK(s_uart_err[ix] == 1u && s_uart_last_err[ix] == errors[e]);
    }
    puts("UART errors: FE/NE/PE/ORE/mixed on all three ports; bad byte discarded; bits preserved; next good byte passed");
}

static void check_ore_drain_and_next_byte(void)
{
    reset_fixture(); error_irq(&huart2, 0u, HAL_UART_ERROR_ORE, 1, 1);
    CHECK(vision_bytes == 1u && vision_last == 0x42u && huart2.clear_calls == 0u);
    reset_fixture(); error_irq(&huart2, 0u, HAL_UART_ERROR_ORE, 0, 0);
    CHECK(huart2.clear_calls == 1u && !huart2.ore_flag);
    CHECK(huart2.RxState == HAL_UART_STATE_BUSY_RX && huart2.rx_enabled);
    good_irq(&huart2, 0x66u); CHECK(vision_bytes == 1u && vision_last == 0x66u);
    puts("UART ORE: no double DR read when RXNE drained error; preserves following byte; still-set ORE drained once passed");
}

static void check_default_task_recovery(void)
{
    for (unsigned ix = 0u; ix < 3u; ix++) for (unsigned ore = 0u; ore < 2u; ore++) {
        reset_fixture();
        UART_HandleTypeDef *h = ix == 0u ? &huart2 : ix == 1u ? &huart3 : &huart4;
        h->fail_next_arm = 1u;
        error_irq(h, ix, ore ? HAL_UART_ERROR_ORE : HAL_UART_ERROR_FE, 1, 0);
        CHECK(h->RxState == HAL_UART_STATE_READY && !h->rx_enabled && s_uart_arm_fail[ix] == 1u);
        robot_bt_service();
        CHECK(h->RxState == HAL_UART_STATE_BUSY_RX && h->rx_enabled && h->arm_calls == 3u);
        CHECK(s_uart_last_err[ix] == (ore ? HAL_UART_ERROR_ORE : HAL_UART_ERROR_FE));
        good_irq(h, 0x88u); CHECK(consumed(ix) == 1u);
    }
    reset_fixture();
    huart2.RxState = HAL_UART_STATE_READY; huart2.rx_enabled = 0u; huart2.fail_next_arm = 1u;
    uart_rx_ensure_all(); CHECK(s_uart_arm_fail[0] == 1u && !huart2.rx_enabled);
    robot_bt_service(); CHECK(huart2.rx_enabled && huart2.RxState == HAL_UART_STATE_BUSY_RX);
    puts("UART recovery: error rearm failure on all ports FE/ORE plus initial-arm failure recovered by real DefaultTask service passed");
}

static void check_busy_receive_and_normal_rearm_failure(void)
{
    reset_fixture();
    /* Nonblocking notification without a completed one-byte transfer must
     * leave an already-BUSY RX armed, not register a second receive. */
    error_irq(&huart2, 0u, HAL_UART_ERROR_FE, 0, 0);
    CHECK(huart2.arm_calls == 1u && huart2.rx_enabled);
    good_irq(&huart2, 0x91u); CHECK(vision_bytes == 1u && vision_last == 0x91u);
    reset_fixture();
    huart2.fail_next_arm = 1u;
    huart2.dr = 0x92u; huart2.rxne_flag = 1u; receive_complete(&huart2);
    CHECK(vision_bytes == 1u && huart2.RxState == HAL_UART_STATE_READY);
    CHECK(s_uart_arm_fail[0] == 1u && s_uart_err[0] == 0u);
    robot_bt_service(); CHECK(huart2.rx_enabled && huart2.RxState == HAL_UART_STATE_BUSY_RX);
    good_irq(&huart2, 0x93u); CHECK(vision_bytes == 2u && vision_last == 0x93u);
    puts("UART BUSY/rearm: nonblocking active transfer not double-armed; normal callback rearm failure recovers passed");
}

int main(void)
{
    check_normal_three_ports(); check_error_classes(); check_ore_drain_and_next_byte();
    check_default_task_recovery(); check_busy_receive_and_normal_rearm_failure();
    puts("uart_error_rearm_test: all host checks passed; physical QR cause is not established");
    return 0;
}
