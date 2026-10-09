#include "arm.h"
#include "main.h"
#include "tim.h"

/* 爪子舵机：TIM12 CH2(PB15) 50Hz，ARR=9999 → 每计数=2us，脉宽 us → 计数 us/2 */
#define CLAW_OPEN_US   1000u   /* TODO 开爪脉宽尚待装配实测；不是初始1150。 */
#define CLAW_CLOSE_US  ARM_SERVO_GRIP_US /* Legacy cc/formal1700; current31/43/42 grip is privately1900. */
#define CLAW_TIM_CH     TIM_CHANNEL_2
static uint16_t s_claw_command_us; /* 仅记录命令脉宽；舵机无位置反馈 */

/* 爪舵机底层:脉宽 us 折算成 TIM12 CH2 比较值写入(50Hz,每计数=2us,故 us/2) */
static void claw_set_cmp_us(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(&htim12, CLAW_TIM_CH, (uint32_t)us / 2u);
}

/* 机械臂初始化:爪到用户确认的初始脉宽、步进 DIR 设方向0(开漏拉低)(robot_init 调一次) */
void arm_init(void)
{
    arm_stepper_clock_stop();
    /* Cortex-M4 DWT 周期计数器用于稳定的微秒级 STEP 脉宽。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 首个有效PWM周期即为初始1150us，避免先以CubeMX的CCR=0起波。 */
    arm_claw_set_us(ARM_SERVO_START_US);
    if (HAL_TIM_PWM_Start(&htim12, CLAW_TIM_CH) != HAL_OK) {
        Error_Handler();
    }
    for (int a = 0; a < ARM_STEPPER_NUM; a++)
        arm_stepper_dir(a, 0);                              /* DIR 拉低；STEP 保持初始高阻 */
}

/* 设爪舵机脉宽 us(共享软件限幅；不是装爪后已标定的机械安全范围) */
void arm_claw_set_us(uint16_t us)
{
    if (us < ARM_SERVO_MIN_US) us = ARM_SERVO_MIN_US;
    if (us > ARM_SERVO_MAX_US) us = ARM_SERVO_MAX_US;
    claw_set_cmp_us(us);
    s_claw_command_us = us;
}

uint16_t arm_claw_command_us(void) { return s_claw_command_us; }

/* 爪开/合到台调极限脉宽(合=咬目标,堵转压紧 1~2s 内可接受,见 arch 决定) */
void arm_claw_open(void)  { arm_claw_set_us(CLAW_OPEN_US); }
void arm_claw_close(void) { arm_claw_set_us(CLAW_CLOSE_US); }

/* 步进脚（开漏、低有效；初始高=Hi-Z；驱动器输入电气规格需独立核对）：
 * 2026-10-06 用户决定飞线，恢复原映射：axis0 STEP=PA9 DIR=PA10；axis1 STEP=PA11 DIR=PA12。
 * 24~27及正式机械动作统一使用CubeMX对应标签；脉冲方式保持不变。 */
static GPIO_TypeDef *const s_step_port[ARM_STEPPER_NUM] = { ARM_AXIS0_STEP_GPIO_Port, ARM_AXIS1_STEP_GPIO_Port };
static const uint32_t       s_step_pin[ARM_STEPPER_NUM]  = { ARM_AXIS0_STEP_Pin, ARM_AXIS1_STEP_Pin };
static GPIO_TypeDef *const s_dir_port[ARM_STEPPER_NUM]  = { ARM_AXIS0_DIR_GPIO_Port, ARM_AXIS1_DIR_GPIO_Port };
static const uint32_t       s_dir_pin[ARM_STEPPER_NUM]   = { ARM_AXIS0_DIR_Pin, ARM_AXIS1_DIR_Pin };

/* DWT 周期计数微秒延时；系统时钟 168MHz 时不依赖编译优化级别。 */
static void busy_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = (SystemCoreClock / 1000000u) * us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { ; }
}

/* 设某轴步进方向位(开漏:低=驱动、高=Hi-Z;dir 0/1 与机械伸缩的对应台上校) */
void arm_stepper_dir(int axis, int dir)
{
    if (axis < 0 || axis >= ARM_STEPPER_NUM) return;
    /* 低=方向 0，高(Hi-Z)=方向 1；语义以光耦接法为准 */
    HAL_GPIO_WritePin(s_dir_port[axis], s_dir_pin[axis],
                      dir == 0 ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* 某轴发 1 个步进脉冲(STEP 拉低 ~20µs 再回高阻;调用者决定频率，不在这里循环发步) */
void arm_stepper_step(int axis)
{
    if (axis < 0 || axis >= ARM_STEPPER_NUM) return;
    HAL_GPIO_WritePin(s_step_port[axis], s_step_pin[axis], GPIO_PIN_RESET); /* 拉低 ~20us */
    busy_us(20);
    HAL_GPIO_WritePin(s_step_port[axis], s_step_pin[axis], GPIO_PIN_SET);   /* 回高阻 */
}

/* TIM7 is private to the bench pulse clock; TIM6 remains the HAL timebase.
 * No HAL clock queries, delays, UART or RTOS calls on the stop/fault path. */
void arm_stepper_clock_stop(void)
{
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    NVIC_DisableIRQ(TIM7_IRQn);
    if (__HAL_RCC_TIM7_IS_CLK_ENABLED()) {
        TIM7->DIER &= ~TIM_DIER_UIE;
        TIM7->CR1 &= ~TIM_CR1_CEN;
        TIM7->SR = 0u;
        TIM7->CNT = 0u;
    }
    NVIC_ClearPendingIRQ(TIM7_IRQn);
    __set_PRIMASK(irq_mask);
}

/* IRQ dispatch can be delayed by UART/critical work. Start the next interval
 * near the actual STEP, never catch up old periods with adjacent pulses. This
 * favors a full idle interval over exact wall-clock pps under IRQ latency. */
void arm_stepper_clock_rephase(void)
{
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    if (__HAL_RCC_TIM7_IS_CLK_ENABLED() &&
        (TIM7->CR1 & TIM_CR1_CEN) != 0u &&
        (TIM7->DIER & TIM_DIER_UIE) != 0u) {
        TIM7->CNT = 0u;
        TIM7->EGR = TIM_EGR_UG;
        TIM7->SR = 0u;
        NVIC_ClearPendingIRQ(TIM7_IRQn);
    }
    __set_PRIMASK(irq_mask);
}

int arm_stepper_clock_start(uint16_t pps)
{
    RCC_ClkInitTypeDef clocks;
    uint32_t flash_latency;
    uint32_t timer_hz;
    uint32_t divider;
    uint32_t period_ticks;
    uint32_t irq_mask;
    uint64_t period_span;
    uint64_t denominator;

    if (pps == 0u || pps > 20000u) {
        arm_stepper_clock_stop();
        return 0;
    }
    HAL_RCC_GetClockConfig(&clocks, &flash_latency);
    timer_hz = HAL_RCC_GetPCLK1Freq();
    if (clocks.APB1CLKDivider != RCC_HCLK_DIV1) {
        if (timer_hz > UINT32_MAX / 2u) {
            arm_stepper_clock_stop();
            return 0;
        }
        timer_hz *= 2u;
    }
    if (timer_hz == 0u) {
        arm_stepper_clock_stop();
        return 0;
    }

    /* Use the smallest prescaler that fits the 16-bit ARR, then round the
     * period to the nearest counter tick. 64-bit intermediates avoid rate
     * product overflow; PSC and ARR both represent divisor minus one. */
    period_span = (uint64_t)pps * 65536u;
    divider = (uint32_t)(((uint64_t)timer_hz + period_span - 1u)
                         / period_span);
    if (divider == 0u || divider > 65536u) {
        arm_stepper_clock_stop();
        return 0;
    }
    denominator = (uint64_t)pps * divider;
    period_ticks = (uint32_t)(((uint64_t)timer_hz + denominator / 2u)
                             / denominator);
    if (period_ticks == 0u || period_ticks > 65536u) {
        arm_stepper_clock_stop();
        return 0;
    }

    irq_mask = __get_PRIMASK();
    __disable_irq();
    NVIC_DisableIRQ(TIM7_IRQn);
    __HAL_RCC_TIM7_CLK_ENABLE();
    TIM7->DIER = 0u;
    TIM7->CR1 = TIM_CR1_URS; /* UG loads PSC without becoming a STEP event. */
    TIM7->CR2 = 0u;
    TIM7->PSC = divider - 1u;
    TIM7->ARR = period_ticks - 1u;
    TIM7->CNT = 0u;
    TIM7->EGR = TIM_EGR_UG;
    TIM7->SR = 0u;
    TIM7->CNT = 0u;
    NVIC_ClearPendingIRQ(TIM7_IRQn);
    HAL_NVIC_SetPriority(TIM7_IRQn, 6u, 0u);
    TIM7->DIER = TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM7_IRQn);
    TIM7->CR1 |= TIM_CR1_CEN;
    __set_PRIMASK(irq_mask);
    return 1;
}
