/*
 * 初学者导读：机械臂有两类执行器，控制方法不同。
 * 爪舵机：持续输出 PWM，改变高电平持续时间（脉宽 us）来发位置指令。
 * 步进电机：DIR 指定方向，STEP 每发一个脉冲请求走一步；行程还取决于机构和细分。
 * axis0 是齿条轴，axis1 是升降轴；dir=0/1 的机械方向仍按实测确认。
 * 寄存器可以看作硬件专用的“变量”；DWT->CYCCNT 就是 CPU 周期计数器。
 * 这里只发命令，没有舵机位置或步进原点反馈，不能从脉冲数断言实物已经到位。
 */

#include "arm.h"
#include "main.h"
#include "tim.h"

/* 爪子舵机：TIM12 CH2(PB15) 50Hz，ARR=9999 → 每计数=2us，脉宽 us → 计数 us/2 */
#define SERVO_MIN_US 500u
#define SERVO_MAX_US 2500u
#define CLAW_OPEN_US   1000u   /* TODO 台上实测两极限脉宽 */
#define CLAW_CLOSE_US  1800u
#define CLAW_TIM_CH     TIM_CHANNEL_2
static uint16_t s_claw_command_us; /* 仅记录命令脉宽；舵机无位置反馈 */

/* 爪舵机底层:脉宽 us 折算成 TIM12 CH2 比较值写入(50Hz,每计数=2us,故 us/2) */
static void claw_set_cmp_us(uint16_t us)
{
    __HAL_TIM_SET_COMPARE(&htim12, CLAW_TIM_CH, (uint32_t)us / 2u);
}

/* 机械臂初始化:爪回中立脉宽、步进 DIR 先回高阻不驱动(robot_init 调一次) */
void arm_init(void)
{
    /* Cortex-M4 DWT 周期计数器用于稳定的微秒级 STEP 脉宽。 */
    /* |=是按位或后赋值：只打开指定bit，保留寄存器其它bit；->表示访问指针指向的结构。 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    /* 首个有效PWM周期就处于中立脉宽，避免先以CubeMX的CCR=0起波。 */
    arm_claw_set_us((CLAW_OPEN_US + CLAW_CLOSE_US) / 2u);
    if (HAL_TIM_PWM_Start(&htim12, CLAW_TIM_CH) != HAL_OK) {
        Error_Handler();
    }
    for (int a = 0; a < ARM_STEPPER_NUM; a++)
        arm_stepper_dir(a, 0);                              /* DIR 开漏先回高阻(Hi-Z) */
}

/* 设爪舵机脉宽 us(自动限幅到 SERVO_MIN/MAX;OPEN/CLOSE 两个极限在上面) */
/**
 * @brief 限制脉宽并写入舵机PWM比较值。
 * @param us 脉宽微秒，限制到500..2500；实际机械安全范围须实测。
 * @retval 无。
 * @note us是时间，不是角度；比较值us/2来自本工程定时器每计数2微秒。
 */
void arm_claw_set_us(uint16_t us)
{
    if (us < SERVO_MIN_US) us = SERVO_MIN_US;
    if (us > SERVO_MAX_US) us = SERVO_MAX_US;
    claw_set_cmp_us(us);
    s_claw_command_us = us;
}

uint16_t arm_claw_command_us(void) { return s_claw_command_us; }

/* 爪开/合：发预置脉宽；实际夹力、机械极限与堵转安全须台测，不由命令成功证明。 */
void arm_claw_open(void)  { arm_claw_set_us(CLAW_OPEN_US); }
void arm_claw_close(void) { arm_claw_set_us(CLAW_CLOSE_US); }

/* 步进脚（共阳极光耦，低有效；开漏初始高=Hi-Z 不驱动）：
 * 2026-09-12 改（按队友布线版）：axis0 = 铰链① STEP=PA9  DIR=PA10 ；axis1 = 铰链② STEP=PA11 DIR=PA12 */
static GPIO_TypeDef *const s_step_port[ARM_STEPPER_NUM] = { GPIOA, GPIOA };
static const uint32_t       s_step_pin[ARM_STEPPER_NUM]  = { GPIO_PIN_9, GPIO_PIN_11 };
static GPIO_TypeDef *const s_dir_port[ARM_STEPPER_NUM]  = { GPIOA, GPIOA };
static const uint32_t       s_dir_pin[ARM_STEPPER_NUM]   = { GPIO_PIN_10, GPIO_PIN_12 };

/* DWT 周期计数微秒延时；系统时钟 168MHz 时不依赖编译优化级别。 */
/**
 * @brief 用CPU周期计数器忙等指定微秒数。
 * @param us 延时微秒；此处用于短STEP脉冲。
 * @retval 无。
 * @note while等待期间不主动让出任务；不能拿这个短脉冲办法替代长时间的osDelay。
 */
static void busy_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    /* CPU每秒SystemCoreClock个周期，先算每微秒多少周期，再乘延时微秒。 */
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

/* 某轴发 1 个步进脉冲(STEP 拉低 ~20µs 再回高阻;一 tick 一步,便于台上被 g2 随时打断) */
/**
 * @brief 在指定轴输出一次约20微秒低电平STEP脉冲。
 * @param axis 轴号0或1；非法编号直接返回。
 * @retval 无。
 * @note 一次调用仅请求一步；步间隔、总步数和中止检查由上层负责。
 */
void arm_stepper_step(int axis)
{
    if (axis < 0 || axis >= ARM_STEPPER_NUM) return;
    HAL_GPIO_WritePin(s_step_port[axis], s_step_pin[axis], GPIO_PIN_RESET); /* 拉低 ~20us */
    busy_us(20);
    HAL_GPIO_WritePin(s_step_port[axis], s_step_pin[axis], GPIO_PIN_SET);   /* 回高阻 */
}
