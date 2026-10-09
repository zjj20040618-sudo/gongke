/* Real arm.c with a deterministic virtual DWT, not a physical waveform test. */
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include "main.h"
#include "tim.h"

typedef struct { uint32_t DEMCR; } HostCoreDebug;
typedef struct { uint32_t CTRL, CYCCNT; } HostDwt;
static HostCoreDebug host_debug;
static HostDwt host_dwt;
static unsigned dwt_reads;
uint32_t SystemCoreClock = 168000000u;

/* Each access advances one simulated microsecond, so busy_us cannot hang.
 * One start read plus 20 elapsed reads checks the existing busy_us(20) call. */
static HostDwt *host_dwt_access(void)
{
    assert(++dwt_reads < 1000u);
    host_dwt.CYCCNT += SystemCoreClock / 1000000u;
    return &host_dwt;
}
#define CoreDebug (&host_debug)
#define CoreDebug_DEMCR_TRCENA_Msk (1u << 24u)
#define DWT host_dwt_access()
#define DWT_CTRL_CYCCNTENA_Msk 1u

GPIO_TypeDef host_gpio_a;
GPIO_TypeDef host_gpio_c;
static TIM_TypeDef host_tim12;
TIM_HandleTypeDef htim12 = { &host_tim12, { 9999u } };
static TIM_TypeDef host_tim6;
TIM_HandleTypeDef htim6 = { &host_tim6, { 9999u } };
UART_HandleTypeDef huart2, huart3, huart4;

/* Private TIM7/RCC/NVIC fixture: do not expand the other host tests' stub. */
typedef struct {
    volatile uint32_t CR1, CR2, DIER, SR, EGR, CNT, PSC, ARR;
} HostStepTimer;
typedef struct { uint32_t APB1CLKDivider; } RCC_ClkInitTypeDef;
static HostStepTimer host_tim7;
static uint32_t host_pclk1 = 42000000u;
static uint32_t host_apb1_divider = 4u;
static uint32_t host_primask;
static unsigned host_tim7_clock_on, host_tim7_nvic_on, host_tim7_pending;
static unsigned host_priority, host_subpriority, host_clock_queries;
static unsigned host_step_irqs, host_laser_off, host_hal_irqs;

static HostStepTimer *host_tim7_access(void)
{
    /* A fault/stop before clock initialization must not touch TIM7. */
    assert(host_tim7_clock_on != 0u);
    return &host_tim7;
}
#define TIM7 host_tim7_access()
#define TIM7_IRQn 55
#define TIM_CR1_URS (1u << 2u)
#define TIM_DIER_UIE 1u
#define TIM_SR_UIF 1u
#define RCC_HCLK_DIV1 1u
#define __HAL_RCC_TIM7_CLK_ENABLE() (host_tim7_clock_on = 1u)
#define __HAL_RCC_TIM7_IS_CLK_ENABLED() (host_tim7_clock_on != 0u)

uint32_t __get_PRIMASK(void) { return host_primask; }
void __disable_irq(void) { host_primask = 1u; }
void __enable_irq(void) { host_primask = 0u; }
void __set_PRIMASK(uint32_t mask) { host_primask = mask; }
void NVIC_DisableIRQ(int irq)
{
    assert(irq == TIM7_IRQn);
    host_tim7_nvic_on = 0u;
}
void NVIC_EnableIRQ(int irq)
{
    assert(irq == TIM7_IRQn);
    host_tim7_nvic_on = 1u;
}
void NVIC_ClearPendingIRQ(int irq)
{
    assert(irq == TIM7_IRQn);
    host_tim7_pending = 0u;
}
void HAL_NVIC_SetPriority(int irq, uint32_t priority, uint32_t subpriority)
{
    assert(irq == TIM7_IRQn && host_primask == 1u);
    assert((host_tim7.CR1 & TIM_CR1_CEN) == 0u);
    host_priority = priority;
    host_subpriority = subpriority;
}
uint32_t HAL_RCC_GetPCLK1Freq(void)
{
    ++host_clock_queries;
    return host_pclk1;
}
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef *clocks, uint32_t *latency)
{
    ++host_clock_queries;
    clocks->APB1CLKDivider = host_apb1_divider;
    *latency = 0u;
}
void HAL_UART_IRQHandler(UART_HandleTypeDef *uart)
{
    (void)uart;
    ++host_hal_irqs;
}
void HAL_TIM_IRQHandler(TIM_HandleTypeDef *timer)
{
    (void)timer;
    ++host_hal_irqs;
}
void bp_laser_emergency_off(void) { ++host_laser_off; }
void test_stepper_timer_irq(void)
{
    assert((host_tim7.SR & TIM_SR_UIF) == 0u);
    ++host_step_irqs;
}
typedef struct {
    GPIO_TypeDef *port;
    uint32_t pin, cycles;
    GPIO_PinState state;
} PinWrite;
static PinWrite writes[8];
static unsigned write_count;

void HAL_GPIO_WritePin(GPIO_TypeDef *port, uint32_t pin, GPIO_PinState state)
{
    assert(write_count < sizeof(writes) / sizeof(writes[0]));
    writes[write_count++] = (PinWrite){ port, pin, host_dwt.CYCCNT, state };
}
void host_compare(TIM_HandleTypeDef *timer, uint32_t channel, uint32_t value)
{
    assert(timer == &htim12 && channel == TIM_CHANNEL_2);
    timer->Instance->CCR2 = value;
}
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *timer, uint32_t channel)
{
    assert(timer == &htim12 && channel == TIM_CHANNEL_2);
    return HAL_OK;
}
void Error_Handler(void) { assert(!"unexpected Error_Handler"); }

/* Match the real main.h/Cube labels; the runner checks that source contract. */
#define ARM_AXIS0_DIR_Pin GPIO_PIN_10
#define ARM_AXIS0_DIR_GPIO_Port GPIOA
#define ARM_AXIS0_STEP_Pin GPIO_PIN_9
#define ARM_AXIS0_STEP_GPIO_Port GPIOA
#define ARM_AXIS1_DIR_Pin GPIO_PIN_12
#define ARM_AXIS1_DIR_GPIO_Port GPIOA
#define ARM_AXIS1_STEP_Pin GPIO_PIN_11
#define ARM_AXIS1_STEP_GPIO_Port GPIOA
#include "../App/arm.c"
#include "../Src/stm32f4xx_it.c"

static void check_write(unsigned index, uint32_t pin, GPIO_PinState state)
{
    assert(writes[index].port == GPIOA);
    assert(writes[index].pin == pin && writes[index].state == state);
}

static void check_axis(int axis, uint32_t dir_pin, uint32_t step_pin)
{
    write_count = 0u;
    arm_stepper_dir(axis, 0);
    arm_stepper_dir(axis, 1);
    assert(write_count == 2u);
    check_write(0u, dir_pin, GPIO_PIN_RESET);
    check_write(1u, dir_pin, GPIO_PIN_SET);

    /* Also exercise the unsigned cycle-counter wrap used by production. */
    for (unsigned wrap = 0u; wrap < 2u; ++wrap) {
        write_count = dwt_reads = 0u;
        host_dwt.CYCCNT = wrap ? UINT32_MAX - 100u : 0u;
        arm_stepper_step(axis);
        assert(write_count == 2u && dwt_reads == 21u);
        check_write(0u, step_pin, GPIO_PIN_RESET);
        check_write(1u, step_pin, GPIO_PIN_SET);
        assert((uint32_t)(writes[1].cycles - writes[0].cycles)
               == 21u * (SystemCoreClock / 1000000u));
    }
}

static void check_servo_range(void)
{
    /* Real arm.c, including the 2 us timer count conversion and its saved
     * command. These are software limits, not measured claw travel limits. */
    assert(ARM_SERVO_MIN_US == 500u && ARM_SERVO_MAX_US == 2500u);
    assert(ARM_SERVO_START_US == 1150u);
    assert(arm_claw_command_us() == ARM_SERVO_START_US && host_tim12.CCR2 == ARM_SERVO_START_US / 2u);
    assert(host_tim12.CCR2 == 575u);
    arm_claw_set_us(500u);
    assert(arm_claw_command_us() == 500u && host_tim12.CCR2 == 250u);
    arm_claw_set_us(2500u);
    assert(arm_claw_command_us() == 2500u && host_tim12.CCR2 == 1250u);
    arm_claw_set_us(0u);
    assert(arm_claw_command_us() == 500u && host_tim12.CCR2 == 250u);
    arm_claw_set_us(499u);
    assert(arm_claw_command_us() == 500u && host_tim12.CCR2 == 250u);
    arm_claw_set_us(2501u);
    assert(arm_claw_command_us() == 2500u && host_tim12.CCR2 == 1250u);
    arm_claw_set_us(UINT16_MAX);
    assert(arm_claw_command_us() == 2500u && host_tim12.CCR2 == 1250u);
    arm_claw_set_us(1501u);
    assert(arm_claw_command_us() == 1501u && host_tim12.CCR2 == 750u);
    arm_claw_open();
    assert(arm_claw_command_us() == 1000u && host_tim12.CCR2 == 500u);
    arm_claw_close();
    assert(ARM_SERVO_GRIP_US == 1700u);
    assert(arm_claw_command_us() == ARM_SERVO_GRIP_US && host_tim12.CCR2 == 850u);
    arm_claw_set_us(1400u);
}

static void check_clock_stopped(void)
{
    assert(host_tim7_nvic_on == 0u && host_tim7_pending == 0u);
    if (host_tim7_clock_on) {
        assert((host_tim7.CR1 & TIM_CR1_CEN) == 0u);
        assert((host_tim7.DIER & TIM_DIER_UIE) == 0u);
        assert(host_tim7.SR == 0u && host_tim7.CNT == 0u);
    }
}

static void check_clock_rate(uint16_t rate, uint32_t timer_hz)
{
    uint64_t achieved_divisor, requested_ticks, difference;
    host_tim7.CNT = 123u;
    host_tim7.SR = TIM_SR_UIF;
    host_tim7_pending = 1u;
    assert(arm_stepper_clock_start(rate) == 1);
    assert(host_tim7.PSC <= 65535u && host_tim7.ARR <= 65535u);
    assert(host_tim7.CNT == 0u && host_tim7.SR == 0u);
    assert(host_tim7_pending == 0u && host_tim7_nvic_on == 1u);
    assert(host_priority == 6u && host_subpriority == 0u);
    assert((host_tim7.CR1 & (TIM_CR1_CEN | TIM_CR1_URS))
           == (TIM_CR1_CEN | TIM_CR1_URS));
    assert(host_tim7.DIER == TIM_DIER_UIE);
    assert(host_tim7.EGR == TIM_EGR_UG);
    achieved_divisor = ((uint64_t)host_tim7.PSC + 1u)
                       * ((uint64_t)host_tim7.ARR + 1u);
    requested_ticks = achieved_divisor * rate;
    difference = requested_ticks > timer_hz
                 ? requested_ticks - timer_hz : timer_hz - requested_ticks;
    assert(difference * 1000u < timer_hz); /* period error below 0.1%. */
}

static void check_clock(void)
{
    unsigned queries;
    write_count = dwt_reads = 0u;
    host_tim6.CNT = 0x12345678u;
    for (uint32_t rate = 1u; rate <= 20000u; ++rate) {
        check_clock_rate((uint16_t)rate, 84000000u);
    }
    check_clock_rate(500u, 84000000u);
    assert(host_tim7.PSC == 2u && host_tim7.ARR == 55999u);
    assert(host_primask == 0u);
    queries = host_clock_queries;
    host_tim7_pending = 1u;
    host_tim7.SR = TIM_SR_UIF;
    arm_stepper_clock_stop();
    check_clock_stopped();
    assert(host_clock_queries == queries); /* stop never queries HAL clocks. */

    /* APB1 undivided means timer=PCLK, not 2*PCLK. */
    host_pclk1 = 16000000u;
    host_apb1_divider = RCC_HCLK_DIV1;
    for (uint32_t rate = 1u; rate <= 20000u; ++rate) {
        check_clock_rate((uint16_t)rate, 16000000u);
    }
    check_clock_rate(1000u, 16000000u);
    assert(host_tim7.PSC == 0u && host_tim7.ARR == 15999u);
    host_pclk1 = 42000000u;
    host_apb1_divider = 4u;

    assert(arm_stepper_clock_start(0u) == 0);
    check_clock_stopped();
    check_clock_rate(500u, 84000000u);
    assert(arm_stepper_clock_start(20001u) == 0);
    check_clock_stopped();
    check_clock_rate(500u, 84000000u);
    assert(arm_stepper_clock_start(UINT16_MAX) == 0);
    check_clock_stopped();
    host_pclk1 = 0u;
    assert(arm_stepper_clock_start(500u) == 0);
    check_clock_stopped();
    host_pclk1 = 42000000u;
    host_primask = 1u;
    check_clock_rate(500u, 84000000u);
    assert(host_primask == 1u);
    arm_stepper_clock_stop();
    check_clock_stopped();
    assert(host_primask == 1u);
    host_primask = 0u;

    assert(host_tim6.CNT == 0x12345678u && host_hal_irqs == 0u);
    assert(write_count == 0u && dwt_reads == 0u); /* clock config emits no STEP. */
}

static void check_clock_rephase(void)
{
    uint32_t prescaler, reload, saved_cr1, saved_dier;
    unsigned queries = host_clock_queries;
    arm_stepper_clock_stop();
    /* Stopped rephase is inert, including pending/CNT/flags. */
    host_tim7.CNT = 123u;
    host_tim7.SR = TIM_SR_UIF;
    host_tim7.EGR = 0u;
    host_tim7_pending = 1u;
    arm_stepper_clock_rephase();
    assert(host_tim7.CNT == 123u && host_tim7.SR == TIM_SR_UIF);
    assert(host_tim7_pending == 1u && host_tim7.EGR == 0u);
    assert(host_clock_queries == queries && host_primask == 0u);

    check_clock_rate(20000u, 84000000u);
    prescaler = host_tim7.PSC;
    reload = host_tim7.ARR;
    saved_cr1 = host_tim7.CR1;
    saved_dier = host_tim7.DIER;
    queries = host_clock_queries;
    for (uint32_t mask = 0u; mask < 2u; ++mask) {
        host_primask = mask;
        host_tim7.CNT = 123u;
        host_tim7.SR = TIM_SR_UIF;
        host_tim7.EGR = 0u;
        host_tim7_pending = 1u;
        arm_stepper_clock_rephase();
        assert(host_tim7.CNT == 0u && host_tim7.SR == 0u);
        assert(host_tim7_pending == 0u && host_tim7.EGR == TIM_EGR_UG);
        assert(host_tim7.PSC == prescaler && host_tim7.ARR == reload);
        assert(host_tim7.CR1 == saved_cr1 && host_tim7.DIER == saved_dier);
        assert(host_clock_queries == queries && host_primask == mask);
    }
    host_primask = 0u;

    /* CEN alone is insufficient: a disabled update source stays untouched. */
    host_tim7.DIER = 0u;
    host_tim7.CNT = 456u;
    host_tim7.SR = TIM_SR_UIF;
    host_tim7_pending = 1u;
    arm_stepper_clock_rephase();
    assert(host_tim7.CNT == 456u && host_tim7.SR == TIM_SR_UIF);
    assert(host_tim7_pending == 1u);
    arm_stepper_clock_stop();
    check_clock_stopped();
    assert(write_count == 0u && dwt_reads == 0u);
}

static void check_timer_irq_and_fault(void)
{
    unsigned queries;
    check_clock_rate(1000u, 84000000u);
    host_step_irqs = 0u;
    TIM7_IRQHandler();
    assert(host_step_irqs == 0u); /* no UIF */
    host_tim7.SR = TIM_SR_UIF;
    host_tim7.DIER = 0u;
    TIM7_IRQHandler();
    assert(host_step_irqs == 0u && host_tim7.SR == TIM_SR_UIF);
    host_tim7.DIER = TIM_DIER_UIE;
    TIM7_IRQHandler();
    assert(host_step_irqs == 1u);
    TIM7_IRQHandler();
    assert(host_step_irqs == 1u); /* serviced flag cannot produce a second step */

    host_tim7_pending = 1u;
    host_tim7.SR = TIM_SR_UIF;
    queries = host_clock_queries;
    fault_outputs_off();
    check_clock_stopped();
    assert(host_clock_queries == queries && host_hal_irqs == 0u);
    assert(host_laser_off == 1u);
    assert(host_gpio_a.BSRR == ARM_AXIS1_STEP_Pin);
    assert(host_gpio_c.BSRR == (uint32_t)GPIO_PIN_8 << 16u);
    assert(write_count == 0u && dwt_reads == 0u); /* fault uses direct writes. */
    host_step_irqs = 0u;
    TIM7_IRQHandler();
    assert(host_step_irqs == 0u);
}

int main(void)
{
    arm_stepper_clock_rephase();
    assert(host_tim7_clock_on == 0u && host_primask == 0u);
    host_tim7_pending = host_tim7_nvic_on = 1u;
    arm_stepper_clock_stop();
    check_clock_stopped();
    assert(host_tim7_clock_on == 0u && host_primask == 0u);
    arm_init();
    assert((host_debug.DEMCR & CoreDebug_DEMCR_TRCENA_Msk) != 0u);
    assert((host_dwt.CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u);
    assert(write_count == 2u);
    check_write(0u, GPIO_PIN_10, GPIO_PIN_RESET);
    check_write(1u, GPIO_PIN_12, GPIO_PIN_RESET);
    check_servo_range();
    check_axis(0, GPIO_PIN_10, GPIO_PIN_9);
    check_axis(1, GPIO_PIN_12, GPIO_PIN_11);
    write_count = dwt_reads = 0u;
    arm_stepper_dir(-1, 0);
    arm_stepper_dir(ARM_STEPPER_NUM, 1);
    arm_stepper_step(-1);
    arm_stepper_step(ARM_STEPPER_NUM);
    assert(write_count == 0u && dwt_reads == 0u);
    check_clock();
    check_clock_rephase();
    check_timer_irq_and_fault();
    puts("arm stepper: axis0 STEP PA9/DIR PA10; axis1 STEP PA11/DIR PA12; low/high pulse and virtual 20us passed");
    puts("arm TIM7: 1..20000pps fit; APB1 x1/x2; restart/stop/IRQ/fault; TIM6 untouched passed");
    puts("arm TIM7 rephase: active reset; inactive no-op; PSC/ARR/PRIMASK retained passed");
    puts("arm servo: 500..2500us actual CCR2 conversion/clamp, boot1150/CCR575 independent of open1000/grip1700 passed");
    return 0;
}
