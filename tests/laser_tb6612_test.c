/* Host regression of the actual board adapter, not a substitute implementation.
 * Direct BSRR writes are checked as register writes; this mock is not an analog
 * bridge, timer-waveform, voltage, current or physical laser-safety validation. */
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "laser_stubs/main.h"
#include "laser_stubs/tim.h"
#include "laser_stubs/usart.h"

GPIO_TypeDef host_gpio_a, host_gpio_b, host_gpio_c, host_gpio_e;
TIM_TypeDef host_tim11;
static TIM_TypeDef host_tim1, host_tim2, host_tim3, host_tim4, host_tim8, host_tim12;
TIM_HandleTypeDef htim1, htim2, htim3, htim4, htim8, htim11, htim12;
UART_HandleTypeDef huart2 = {2u}, huart3 = {3u}, huart4 = {4u};
unsigned host_clock_enable_mask;
static uint32_t host_primask;
static unsigned host_errors, host_fail_laser_start, host_hal_calls;
static unsigned host_encoder_starts, host_motor_pwm_starts, host_laser_pwm_starts;
typedef struct {
    char kind;
    const void *object;
    uint32_t pin_or_channel, value;
} host_event_t;
static host_event_t host_events[256];
static unsigned host_event_count;
static void host_event(char kind, const void *object, uint32_t pin, uint32_t value)
{
    assert(host_event_count < sizeof host_events / sizeof host_events[0]);
    host_events[host_event_count++] = (host_event_t){kind, object, pin, value};
}
uint32_t __get_PRIMASK(void) { return host_primask; }
void __disable_irq(void) { host_primask = 1u; }
void __enable_irq(void) { host_primask = 0u; }
void __set_PRIMASK(uint32_t mask) { host_primask = mask; }
void Error_Handler(void) { ++host_errors; }
void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pins, GPIO_PinState state)
{
    ++host_hal_calls;
    host_event('G', port, pins, (uint32_t)state);
    if (port == GPIOC && (pins & GPIO_PIN_12) && state == GPIO_PIN_SET) {
        /* Check the bridge is safe at the enable edge, not just at function end. */
        assert((GPIOB->ODR & GPIO_PIN_0) != 0u);
        assert((GPIOB->ODR & GPIO_PIN_1) == 0u);
        assert(TIM11->CCR1 == TIM11->ARR + 1u);
        assert(TIM11->EGR == TIM_EGR_UG);
    }
    if (state == GPIO_PIN_SET) port->ODR |= pins;
    else port->ODR &= ~pins;
}
void host_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{
    assert(timer->Instance != NULL);
    host_event('C', timer, channel, value);
    switch (channel) {
        case TIM_CHANNEL_1: timer->Instance->CCR1 = value; break;
        case TIM_CHANNEL_2: timer->Instance->CCR2 = value; break;
        case TIM_CHANNEL_3: timer->Instance->CCR3 = value; break;
        case TIM_CHANNEL_4: timer->Instance->CCR4 = value; break;
        default: assert(!"unknown PWM channel");
    }
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    ++host_hal_calls;
    host_event('P', timer, channel, 0u);
    if (timer == &htim11) {
        ++host_laser_pwm_starts;
        if (host_fail_laser_start) return HAL_ERROR;
    } else {
        assert(timer == &htim1);
        ++host_motor_pwm_starts;
    }
    timer->Instance->CR1 |= TIM_CR1_CEN;
    timer->Instance->CCER |= 1u << channel;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_TIM_Encoder_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    ++host_hal_calls;
    assert(channel == TIM_CHANNEL_ALL);
    assert(timer == &htim2 || timer == &htim3 || timer == &htim4 || timer == &htim8);
    ++host_encoder_starts;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef *uart, uint8_t *data, uint16_t size)
{
    (void)uart; (void)data; (void)size;
    ++host_hal_calls;
    return HAL_OK;
}

#include "../App/board_pins.c"

static void fixture_reset(void)
{
    memset(&host_gpio_a, 0, sizeof host_gpio_a);
    memset(&host_gpio_b, 0, sizeof host_gpio_b);
    memset(&host_gpio_c, 0, sizeof host_gpio_c);
    memset(&host_gpio_e, 0, sizeof host_gpio_e);
    memset(&host_tim1, 0, sizeof host_tim1);
    memset(&host_tim2, 0, sizeof host_tim2);
    memset(&host_tim3, 0, sizeof host_tim3);
    memset(&host_tim4, 0, sizeof host_tim4);
    memset(&host_tim8, 0, sizeof host_tim8);
    memset(&host_tim11, 0, sizeof host_tim11);
    memset(&host_tim12, 0, sizeof host_tim12);
    htim1 = (TIM_HandleTypeDef){&host_tim1, {199u}};
    htim2 = (TIM_HandleTypeDef){&host_tim2, {0xffffffffu}};
    htim3 = (TIM_HandleTypeDef){&host_tim3, {65535u}};
    htim4 = (TIM_HandleTypeDef){&host_tim4, {65535u}};
    htim8 = (TIM_HandleTypeDef){&host_tim8, {65535u}};
    htim11 = (TIM_HandleTypeDef){&host_tim11, {199u}};
    htim12 = (TIM_HandleTypeDef){&host_tim12, {19999u}};
    host_tim11.ARR = 199u;
    host_primask = host_errors = host_fail_laser_start = host_hal_calls = 0u;
    host_encoder_starts = host_motor_pwm_starts = host_laser_pwm_starts = 0u;
    host_clock_enable_mask = host_event_count = 0u;
    s_laser_ready = 0u;
}
static void assert_laser_off(void)
{
    assert((GPIOC->ODR & GPIO_PIN_12) == 0u);
    assert((GPIOB->ODR & (GPIO_PIN_0 | GPIO_PIN_1)) == 0u);
    assert(TIM11->CCR1 == 0u);
}
static void assert_laser_on(void)
{
    assert((GPIOC->ODR & GPIO_PIN_12) != 0u);
    assert((GPIOB->ODR & GPIO_PIN_0) != 0u);
    assert((GPIOB->ODR & GPIO_PIN_1) == 0u);
    assert(TIM11->CCR1 == 200u && TIM11->CCR1 == TIM11->ARR + 1u);
    assert(TIM11->EGR == TIM_EGR_UG);
}
static void assert_no_pa15_wheel_stby_touches(void)
{
    for (unsigned i = 0; i < host_event_count; ++i) {
        const host_event_t *event = &host_events[i];
        if (event->kind != 'G') continue;
        assert(event->object != GPIOA);
        assert(!(event->object == GPIOC && (event->pin_or_channel & GPIO_PIN_8)));
    }
}
static void test_init_and_early_on(void)
{
    fixture_reset();
    htim11.Instance = NULL; /* ON must remain OFF before HAL timer initialization. */
    bp_laser_set(1);
    assert_laser_off();
    assert(s_laser_ready == 0u);
    htim11.Instance = TIM11;
    host_event_count = 0u;
    bp_init();
    assert_laser_off();
    assert(s_laser_ready == 1u);
    assert(host_encoder_starts == 4u && host_motor_pwm_starts == 4u);
    assert(host_laser_pwm_starts == 1u);
    assert((GPIOC->ODR & GPIO_PIN_8) != 0u);
    for (unsigned m = 0; m < 4u; ++m) {
        assert(s_enc[m]->Instance->CNT == 0u);
        assert(s_enc_last[m] == 0 && s_enc_raw_total[m] == 0);
    }
    /* Laser STBY is never enabled by initialization. */
    for (unsigned i = 0; i < host_event_count; ++i)
        assert(!(host_events[i].kind == 'G' && host_events[i].object == GPIOC &&
                 host_events[i].pin_or_channel == GPIO_PIN_12 && host_events[i].value));
}
static void test_switch_order_and_isolation(void)
{
    fixture_reset();
    bp_init();
    host_tim1.CCR1 = 11u; host_tim1.CCR2 = 22u;
    host_tim1.CCR3 = 33u; host_tim1.CCR4 = 44u;
    TIM_TypeDef wheel_before = host_tim1;
    GPIOA->MODER = GPIOA->ODR = 0xa55a1234u;
    GPIOE->MODER = GPIOE->ODR = 0x5aa51234u;
    GPIOB->ODR |= GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    uint32_t wheel_direction_before = GPIOB->ODR & ~(GPIO_PIN_0 | GPIO_PIN_1);
    host_event_count = 0u;
    host_primask = 1u;
    for (unsigned repeat = 0; repeat < 3u; ++repeat) {
        unsigned start = host_event_count;
        bp_laser_set(1);
        assert_laser_on();
        assert(host_primask == 1u); /* Preserve caller's IRQ state. */
        assert(host_event_count == start + 4u);
        assert(host_events[start].object == GPIOB && host_events[start].pin_or_channel == GPIO_PIN_1 && host_events[start].value == 0u);
        assert(host_events[start + 1u].object == GPIOB && host_events[start + 1u].pin_or_channel == GPIO_PIN_0 && host_events[start + 1u].value == 1u);
        assert(host_events[start + 2u].kind == 'C' && host_events[start + 2u].object == &htim11 && host_events[start + 2u].value == 200u);
        assert(host_events[start + 3u].object == GPIOC && host_events[start + 3u].pin_or_channel == GPIO_PIN_12 && host_events[start + 3u].value == 1u);
        start = host_event_count;
        bp_laser_set(0);
        assert_laser_off();
        assert(host_events[start].kind == 'G' && host_events[start].object == GPIOC && host_events[start].pin_or_channel == GPIO_PIN_12 && host_events[start].value == 0u);
        assert(host_events[start + 1u].kind == 'C' && host_events[start + 1u].value == 0u);
        assert(host_primask == 1u);
    }
    bp_laser_set(0); /* Repeated OFF is safe. */
    assert_laser_off();
    host_primask = 0u;
    bp_laser_set(1);
    assert(host_primask == 0u);
    bp_laser_set(0);
    assert_no_pa15_wheel_stby_touches();
    assert(memcmp(&wheel_before, &host_tim1, sizeof wheel_before) == 0);
    assert(GPIOA->MODER == 0xa55a1234u && GPIOA->ODR == 0xa55a1234u);
    assert(GPIOE->MODER == 0x5aa51234u && GPIOE->ODR == 0x5aa51234u);
    assert((GPIOB->ODR & ~(GPIO_PIN_0 | GPIO_PIN_1)) == wheel_direction_before);
    assert((GPIOC->ODR & GPIO_PIN_8) != 0u);
}
static void test_emergency_without_hal_or_handle(void)
{
    fixture_reset();
    bp_init();
    bp_laser_set(1);
    htim11.Instance = NULL;
    GPIOB->MODER = 0xabcdef98u;
    GPIOC->MODER = 0x76543210u;
    TIM11->CCER = 0xa55b;
    TIM11->CR1 = 0x5aa5;
    unsigned hal_calls = host_hal_calls;
    uint32_t old_b_mode = GPIOB->MODER, old_c_mode = GPIOC->MODER;
    uint32_t old_ccer = TIM11->CCER, old_cr1 = TIM11->CR1;
    host_event_count = 0u;
    bp_laser_emergency_off();
    assert(host_hal_calls == hal_calls && host_event_count == 0u);
    assert(s_laser_ready == 0u && host_clock_enable_mask == 7u);
    assert(GPIOC->BSRR == GPIO_PIN_12 << 16u);
    assert(GPIOB->BSRR == (GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_9) << 16u);
    assert(TIM11->CCR1 == 0u);
    assert(TIM11->CCER == (old_ccer & ~TIM_CCER_CC1E));
    assert(TIM11->CR1 == (old_cr1 & ~TIM_CR1_CEN));
    const uint32_t b_mode_mask = (3u << 0u) | (3u << 2u) | (3u << 18u);
    const uint32_t c_mode_mask = 3u << 24u;
    assert((GPIOB->MODER & ~b_mode_mask) == (old_b_mode & ~b_mode_mask));
    assert((GPIOC->MODER & ~c_mode_mask) == (old_c_mode & ~c_mode_mask));
    assert((GPIOB->MODER & b_mode_mask) == ((1u << 0u) | (1u << 2u) | (1u << 18u)));
    assert((GPIOC->MODER & c_mode_mask) == 1u << 24u);
    bp_laser_set(1); /* Stale ON cannot reopen the bridge after fatal off. */
    assert((GPIOC->ODR & GPIO_PIN_12) == 0u);
    assert((GPIOB->ODR & (GPIO_PIN_0 | GPIO_PIN_1)) == 0u);
    assert(TIM11->CCR1 == 0u);
    bp_laser_emergency_off(); /* Idempotent, still without a handle. */
    assert(s_laser_ready == 0u);
}
static void test_pwm_start_failure(void)
{
    fixture_reset();
    host_fail_laser_start = 1u;
    bp_init();
    assert(host_errors == 1u && s_laser_ready == 0u);
    assert(TIM11->CCR1 == 0u && !(TIM11->CCER & TIM_CCER_CC1E));
    assert(!(TIM11->CR1 & TIM_CR1_CEN));
    bp_laser_set(1);
    assert_laser_off();
}
static void test_existing_motor_and_encoder_adapter(void)
{
    static const int expected_inv[4] = {1, -1, -1, 1};
    fixture_reset();
    bp_init();
    bp_laser_set(1);
    volatile uint32_t *wheel_compare[4] = {
        &host_tim1.CCR1, &host_tim1.CCR2, &host_tim1.CCR3, &host_tim1.CCR4
    };
    for (unsigned m = 0; m < 4u; ++m) {
        bp_motor_set((int)m, BP_DIR_FWD, 123);
        assert(*wheel_compare[m] == 123u);
        assert(((s_in1p[m]->ODR & s_in1[m]) != 0u) == (expected_inv[m] > 0));
        assert(((s_in2p[m]->ODR & s_in2[m]) != 0u) == (expected_inv[m] < 0));
        s_enc[m]->Instance->CNT = 7u;
        assert(bp_enc_delta((int)m) == expected_inv[m] * 7);
        assert(bp_enc_raw_total((int)m) == 7);
        bp_motor_set((int)m, BP_DIR_REV, 999);
        assert(*wheel_compare[m] == MOTOR_PWM_PERIOD);
        assert(((s_in1p[m]->ODR & s_in1[m]) != 0u) == (expected_inv[m] < 0));
        bp_motor_brake((int)m);
        assert(*wheel_compare[m] == 0u);
        assert((s_in1p[m]->ODR & s_in1[m]) != 0u);
        assert((s_in2p[m]->ODR & s_in2[m]) != 0u);
        bp_motor_stop((int)m);
        assert(*wheel_compare[m] == 0u);
        assert_laser_on();
    }
    unsigned events_before = host_event_count;
    bp_motor_set(-1, 1, 100); bp_motor_set(4, 1, 100);
    bp_motor_stop(-1); bp_motor_brake(4);
    assert(host_event_count == events_before);
    assert(bp_enc_delta(-1) == 0 && bp_enc_raw_total(4) == 0);
    bp_enc_raw_reset_all();
    for (unsigned m = 0; m < 4u; ++m) assert(bp_enc_raw_total((int)m) == 0);
    bp_laser_set(0);
    assert_laser_off();
}
int main(void)
{
    test_init_and_early_on();
    test_switch_order_and_isolation();
    test_emergency_without_hal_or_handle();
    test_pwm_start_failure();
    test_existing_motor_and_encoder_adapter();
    puts("TB6612 laser adapter: 5 groups passed (real board_pins.c; init/off/order/IRQ/fatal/isolation).");
    return 0;
}
