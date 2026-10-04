/*
 * 初学者导读：电机、编码器、激光与蓝牙发送的硬件适配层。
 * HAL 是 STM32 的库函数；例如 HAL_GPIO_WritePin 把某个引脚输出置高或置低。
 * htim1 等“句柄”是保存外设信息的结构体；&htim1 取地址，传给库函数操作该外设。
 * s_pw[m] 保存定时器结构体的指针，s_pw_ch[m] 保存对应通道，配合表示一路 PWM。
 * GPIOB 是 B 组端口，GPIO_PIN_13 是第 13 号引脚的位掩码，合起来才表示 PB13。
 * 这里的轮序、方向修正表按现有源码保留；注释不会代替实物接线核验。
 */

#include "board_pins.h"
#include "main.h"       /* HAL 全部已启用(hal_conf)；含 TIM/UART/GPIO 宏 */
#include "tim.h"        /* htim1/2/3/4/8/12 */
#include "usart.h"      /* huart2/3/4 */
#include <string.h>

/* ---------- 电机 4 路：PWM 全部走 TIM1 CH1..4（PE9/11/13/14） ---------- */
static TIM_HandleTypeDef *const s_pw[4] = { &htim1, &htim1, &htim1, &htim1 };
static const uint32_t s_pw_ch[4] = { TIM_CHANNEL_1, TIM_CHANNEL_2,
                                     TIM_CHANNEL_3, TIM_CHANNEL_4 };
/* 方向脚 IN1/IN2（2026-09-12 改，按队友布线版）：m0=PB13/PB14 m1=PB12/PB11 m2=PE12/PE10 m3=PE15/PB10 */
static GPIO_TypeDef *const s_in1p[4] = { GPIOB, GPIOB, GPIOE, GPIOE };
static const uint32_t    s_in1[4] = { GPIO_PIN_13, GPIO_PIN_12, GPIO_PIN_12, GPIO_PIN_15 };
static GPIO_TypeDef *const s_in2p[4] = { GPIOB, GPIOB, GPIOE, GPIOB };
static const uint32_t    s_in2[4] = { GPIO_PIN_14, GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_10 };

/* 当前电机与编码器索引：m0/TIM2 m1/TIM3 m2/TIM4 m3/TIM8。
 * m1/TIM3、m3/TIM8 有单轮驱动+回传证据；m0 本次仅收到实体轮向口述，
 * 尚缺模式7原始回包以独立复核 m0/TIM2 配对，故烧录后先架空短测。
 * 2026-09-25 实体轮位复核：m0左后、m1右后、m2右前、m3左前。 */
static TIM_HandleTypeDef *const s_enc[4] = { &htim2, &htim3, &htim4, &htim8 };
static const int s_enc_bits[4] = { 32, 16, 16, 16 };
static int32_t s_enc_last[4];
static volatile int32_t s_enc_raw_total[4];

#define PIN_STBY   (GPIOC)  /* 电机总使能，高=使能 */
#define PIN_STBY_N GPIO_PIN_8
#define PIN_LASER  (GPIOA)  /* 激光触发，高=点亮（2026-09-12 由 PC9 改 PA15）*/
#define PIN_LASER_N GPIO_PIN_15

/* 软件正方向统一为各实体轮向车头前滚动。
 * 2026-09-25 模式23: 四路旧正命令/计数均为正，但前排前滚、后排后滚；
 * 单轮模式7/8复核 m0左后、m1右后，旧正命令均使后轮后滚。
 * 因此只翻后两路电机和反馈极性，使闭环仍为负反馈；前两路保持不变。
 * 这改变了架空单轮正向定义，烧录后必须先复核四轮正命令均前滚、计数均为正。 */
static const int8_t s_motor_inv[MOTOR_NUM] = { 1, -1, -1, 1 };
static const int8_t s_encoder_inv[MOTOR_NUM] = { 1, -1, -1, 1 };

/* 底层一次性初始化:起四路编码器+四路 PWM、STBY 使能、激光关(robot_init 最先调) */
void bp_init(void)
{
    for (int m = 0; m < MOTOR_NUM; m++) {
        HAL_TIM_Encoder_Start(s_enc[m], TIM_CHANNEL_ALL);
        __HAL_TIM_SET_COUNTER(s_enc[m], 0);
        s_enc_last[m] = 0;
        s_enc_raw_total[m] = 0;
        HAL_TIM_PWM_Start(s_pw[m], s_pw_ch[m]);
    }
    HAL_GPIO_WritePin(PIN_STBY, PIN_STBY_N, GPIO_PIN_SET);   /* STBY 拉高使能 */
    HAL_GPIO_WritePin(PIN_LASER, PIN_LASER_N, GPIO_PIN_RESET);
}

/* 写某轮 IN1/IN2 电平对:in2 恒 = !in1,二者一起定正转/反转(TB6612 两态) */
static void set_in(int m, int in1_high)
{
    /* in2 = !in1：正转/反转二选一 */
    HAL_GPIO_WritePin(s_in1p[m], s_in1[m], in1_high ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(s_in2p[m], s_in2[m], in1_high ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* 设某轮转动:PWM 占空比 duty(限幅)+ 方向(dir 符号 → IN1);duty=0 只置方向不转 */
/**
 * @brief 给指定轮写PWM比较值，并按方向表设置IN1/IN2。
 * @param m 轮号0..3。
 * @param dir 非负取软件正向，负数取反向；最后应用现有方向修正表。
 * @param duty PWM比较计数；取绝对值并限制到0..MOTOR_PWM_PERIOD。
 * @retval 无。
 * @note duty不是rpm；例如比较值约为周期计数一半时，对应约50%占空比。
 */
void bp_motor_set(int m, int dir, int duty)
{
    if (m < 0 || m >= MOTOR_NUM) return;
    if (duty < 0) duty = -duty;
    if (duty > (int)MOTOR_PWM_PERIOD) duty = (int)MOTOR_PWM_PERIOD;
    __HAL_TIM_SET_COMPARE(s_pw[m], s_pw_ch[m], (uint32_t)duty);
    int fwd = (dir >= 0);
    if (s_motor_inv[m] < 0) fwd = !fwd;   /* 按实测轮向翻电机 */
    set_in(m, fwd);                  /* 方向由 IN1/IN2 决定，PWM 只管速度 */
}

/* 停某轮:PWM 置 0(IN 保持,自由滑行;要抱死走 brake) */
void bp_motor_stop(int m)
{
    if (m < 0 || m >= MOTOR_NUM) return;
    __HAL_TIM_SET_COMPARE(s_pw[m], s_pw_ch[m], 0);
}

/* 刹某轮:IN1=IN2 同高(TB6612 抱死档)+ PWM 置 0 */
void bp_motor_brake(int m)
{
    if (m < 0 || m >= MOTOR_NUM) return;
    HAL_GPIO_WritePin(s_in1p[m], s_in1[m], GPIO_PIN_SET);
    HAL_GPIO_WritePin(s_in2p[m], s_in2[m], GPIO_PIN_SET);
    __HAL_TIM_SET_COMPARE(s_pw[m], s_pw_ch[m], 0);
}

/* 读某轮本周期编码器脉冲增量(自上次读;自动处理 32/16 位计数回绕) */
/**
 * @brief 读取距上次调用增加的计数，并更新本轮读取基线。
 * @param m 轮号0..3。
 * @retval 带方向修正的编码器增量；非法轮号返回0。
 * @note 调用一次就消费本段增量，不能为看日志额外调用，否则会影响1ms控制层的读取。
 */
int32_t bp_enc_delta(int m)
{
    if (m < 0 || m >= MOTOR_NUM) return 0;
    uint32_t cur = __HAL_TIM_GET_COUNTER(s_enc[m]);
    int32_t  d;
    /* 计数器到最大值会回到0；按硬件位宽做差再转有符号数，可恢复跨界的小增量。 */
    if (s_enc_bits[m] == 32) d = (int32_t)(cur - (uint32_t)s_enc_last[m]);
    else                     d = (int16_t)((uint16_t)cur - (uint16_t)s_enc_last[m]);
    s_enc_last[m] = (int32_t)cur;
    s_enc_raw_total[m] += d;
    return (s_encoder_inv[m] < 0) ? -d : d;
}

/* 原始硬件计数不经过方向表，供上板核对编码器极性。 */
int32_t bp_enc_raw_total(int m)
{
    if (m < 0 || m >= MOTOR_NUM) return 0;
    return s_enc_raw_total[m];
}

void bp_enc_raw_reset_all(void)
{
    for (int m = 0; m < MOTOR_NUM; m++) s_enc_raw_total[m] = 0;
}

/* 激光开/关(on=PA15 拉高点亮;反恐开枪口) */
void bp_laser_set(int on)
{
    HAL_GPIO_WritePin(PIN_LASER, PIN_LASER_N,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 调试口 = 蓝牙所在 USART3(PD8/PD9, 9600)。2026-09-12:蓝牙由 PA9/PA10(USART1) 挪到 PD8/PD9(USART3)。 */
#define DEBUG_UART (&huart3)

/* 蓝牙 TX 不依赖 RTOS 任务等待：调用者只入队，USART3 TXE/TC 中断逐字节发。
 * RX 仍由 robot.c 的 HAL_UART_Receive_IT 处理；HAL 的 TX/RX 状态互相独立。
 * 留足一个字节区分满/空，队列满时整条消息丢弃并记数，绝不截断半行。 */
#define BT_TX_N 2048u
#define BT_TX_CHUNK 32u
static uint8_t s_bt_tx[BT_TX_N];
static volatile uint16_t s_bt_tx_wr, s_bt_tx_rd, s_bt_tx_active;
static volatile uint32_t s_bt_tx_drop;

/* 调用时已关中断，或位于 USART3 中断回调内。 */
static void bt_tx_start(void)
{
    uint16_t available, chunk;
    if (s_bt_tx_active || s_bt_tx_wr == s_bt_tx_rd) return;
    available = (s_bt_tx_wr > s_bt_tx_rd) ?
                (uint16_t)(s_bt_tx_wr - s_bt_tx_rd) :
                (uint16_t)(BT_TX_N - s_bt_tx_rd);
    chunk = (available > BT_TX_CHUNK) ? BT_TX_CHUNK : available;
    if (HAL_UART_Transmit_IT(DEBUG_UART, &s_bt_tx[s_bt_tx_rd], chunk) == HAL_OK)
        s_bt_tx_active = chunk;
}

/**
 * @brief 把一整条字符串复制进蓝牙发送队列，由中断分批送出。
 * @param s 以结束符结尾的字符串；空指针或空串直接返回。
 * @retval 无。
 * @note 调用返回不表示线缆已经发送完；队列空间不足会丢整条并增加诊断计数。
 */
void bp_debug_send(const char *s)
{
    size_t len;
    uint16_t used, free_bytes;
    uint32_t primask;
    if (!s) return;
    len = strlen(s);
    if (len == 0u) return;

    /* 先保存原中断开关，再短暂关中断保护队列；结束时只恢复原来允许的状态。 */
    primask = __get_PRIMASK();
    __disable_irq();
    /* 队列长度2048是2的幂，&2047能把下标限制在0..2047，实现尾部回到开头。 */
    used = (uint16_t)((s_bt_tx_wr - s_bt_tx_rd) & (BT_TX_N - 1u));
    free_bytes = (uint16_t)(BT_TX_N - 1u - used);
    if (len > free_bytes) {
        s_bt_tx_drop++;
    } else {
        for (size_t i = 0u; i < len; ++i) {
            s_bt_tx[s_bt_tx_wr] = (uint8_t)s[i];
            s_bt_tx_wr = (uint16_t)((s_bt_tx_wr + 1u) & (BT_TX_N - 1u));
        }
        bt_tx_start();
    }
    if (!primask) __enable_irq();
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != DEBUG_UART || !s_bt_tx_active) return;
    s_bt_tx_rd = (uint16_t)((s_bt_tx_rd + s_bt_tx_active) & (BT_TX_N - 1u));
    s_bt_tx_active = 0u;
    bt_tx_start();
}

uint32_t bp_debug_tx_dropped(void)
{
    return s_bt_tx_drop;
}
