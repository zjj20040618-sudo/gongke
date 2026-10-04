/*
 * 初学者导读：IMU 测量车身姿态，串口每次只送来一个字节，要自己拼成完整帧。
 * yaw 是车头左右朝向，pitch 是车头抬起/低下，roll 是车身左右倾斜，角度单位为度。
 * 状态机就是“记住目前收到第几部分”，下一字节按这个状态解释。
 * uint8_t/uint16_t/uint32_t 分别是无符号 8/16/32 位整数；int16_t 可以表示负数。
 * 原始协议把角度放大 100 倍存成整数，除以 100.0f 才还原为小数角度。
 * 普通 yaw 在一圈处回绕，连续 heading 会累加增量，适合转身和道路进度计算。
 */

#include "imu.h"
#include "main.h"

#define IMU_LINK_TIMEOUT_MS  200u   /* 超过此时长没数据视为链路断 */
#define IMU_DATA_MAX         18u    /* 数据域上限 = 模式0 全数据帧(尾 6 字节才是姿态) */
#define IMU_DEV_ID           0x60u  /* 模块默认 DevID */

/* 解析状态机:找帧头 → 收帧身 → 验 CS(帧身按序 DevID|CMD|LEN|DATA[LEN]) */
enum { IMU_ST_IDLE = 0, IMU_ST_H55, IMU_ST_DEV, IMU_ST_CMD, IMU_ST_LEN, IMU_ST_DATA, IMU_ST_CS };

static float    s_yaw;              /* 航向 0..360(0/360 回绕,上层判向自己处理跨绕) */
static float    s_pitch;            /* 俯仰 ±180(静态 ~0;校安装 / 开枪前水平判据用) */
static float    s_roll;             /* 横滚 ±180 */
static float    s_yaw_last;         /* 上一帧 yaw(算跨绕增量用) */
static float    s_heading_cont;     /* 连续航向(度,不回绕,供里程/保向积分) */
static float    s_leg_heading_zero; /* 分段航向零点；绝不清全局连续航向 */
static uint8_t  s_yaw_have_last;
static uint8_t  s_valid;            /* 解析出过有效姿态(供 imu_ok 判"数据可用") */
static uint32_t s_valid_ms;         /* 最近一次有效姿态时刻 */
static uint32_t s_last_ms;          /* 最近任意字节时刻(链路活性) */

/* 状态机工作区 */
static uint8_t  s_st;
static uint8_t  s_cmd, s_len;
static uint8_t  s_buf[IMU_DATA_MAX];
static uint8_t  s_ix;
static uint8_t  s_sum;

/* 小端读 16 位(姿态原始值 ×100,见 imu.h 头注) */
static uint16_t le_u16(const uint8_t *b)
{
    return (uint16_t)((uint16_t)b[0] | ((uint16_t)b[1] << 8));
}
static int16_t le_i16(const uint8_t *b)
{
    return (int16_t)le_u16(b);
}

/* 整帧校验通过后分派:CMD 0x01 = 姿态上报;按 LEN 区分两档(排位不同别写死) */
static void imu_frame(uint8_t cmd, uint8_t len, const uint8_t *d)
{
    uint8_t y, p, r;   /* 三个姿态角在数据域里的首字节偏移 */
    if (cmd != 0x01u) return;
    if (len == 6u)      { y = 0u;  p = 2u;  r = 4u;  }   /* 模式1 仅姿态:Yaw 在前 */
    else if (len == 18u){ p = 12u; r = 14u; y = 16u; }   /* 模式0 全数据:姿态在尾 */
    else return;                                          /* 非姿态帧(0xF0 ACK 等)忽略 */

    /* d+y移动到yaw字段首字节；数组/指针加法按元素走，这里uint8_t每个元素1字节。 */
    s_yaw   = le_u16(d + y) / 100.0f;    /* uint16 0~360,0/360 回绕 */
    s_pitch = le_i16(d + p) / 100.0f;    /* int16 ±180 */
    s_roll  = le_i16(d + r) / 100.0f;

    /* 连续航向:每帧按跨绕±180 取增量累加,不回绕(0→359 只算 -1°) */
    if (!s_yaw_have_last) { s_yaw_last = s_yaw; s_heading_cont = s_yaw; s_yaw_have_last = 1u; }
    else {
        /* 例如359°到0°，原差为-359°；加360后是+1°，避免误以为倒转一整圈。 */
        float ddeg = s_yaw - s_yaw_last;
        while (ddeg >  180.0f) ddeg -= 360.0f;
        while (ddeg < -180.0f) ddeg += 360.0f;
        s_heading_cont += ddeg;
        s_yaw_last = s_yaw;
    }

    s_valid = 1u;
    s_valid_ms = HAL_GetTick();
}

/* IMU 状态清零:姿态归零、数据标记清、解析态回找帧头(robot_init 调一次) */
void imu_init(void)
{
    s_yaw = 0.0f; s_pitch = 0.0f; s_roll = 0.0f;
    s_yaw_last = 0.0f; s_heading_cont = 0.0f; s_leg_heading_zero = 0.0f; s_yaw_have_last = 0u;
    s_valid = 0u; s_valid_ms = 0u; s_last_ms = 0u;
    s_st = IMU_ST_IDLE;
    /* TX 切"仅姿态"未接:解析已兼容模式0/1 两档上报,不依赖先切模式。
     * 等 TX(PC10) 接通后再发 AA 55 60 0B 01 01 6D + 0A 01 01 6C 降带宽(见 memory/imu-601-protocol.md)。 */
}

/* UART4 逐字节喂入(ISR 回调里被调):字节式状态机拼帧,收满验 CS 后分派 */
/**
 * @brief 接收一个字节，根据当前状态找帧头、收数据和验校验和。
 * @param ch 刚收到的8位字节，不是整条字符串。
 * @retval 无。
 * @note s_st在多次调用之间保存进度；校验通过才更新姿态。
 */
void imu_feed(uint8_t ch)
{
    s_last_ms = HAL_GetTick();       /* 任意字节都算链路活着 */

    switch (s_st) {
    case IMU_ST_IDLE:                /* 找帧头 AA */
        if (ch == 0xAAu) s_st = IMU_ST_H55;
        break;
    case IMU_ST_H55:                 /* 等 55;非 55 → 回找(该字节若是新 AA 直接续等) */
        if (ch == 0x55u) { s_st = IMU_ST_DEV; s_sum = 0u; }
        else s_st = (ch == 0xAAu) ? IMU_ST_H55 : IMU_ST_IDLE;
        break;
    case IMU_ST_DEV:                 /* DevID(不对就丢帧重找) */
        s_sum = (uint8_t)(s_sum + ch);
        s_st = (ch == IMU_DEV_ID) ? IMU_ST_CMD : IMU_ST_IDLE;
        break;
    case IMU_ST_CMD:
        s_cmd = ch;
        s_sum = (uint8_t)(s_sum + ch);
        s_st = IMU_ST_LEN;
        break;
    case IMU_ST_LEN:                 /* LEN:0=无数据直接等 CS;超上限=不支持的帧丢 */
        s_len = ch;
        s_sum = (uint8_t)(s_sum + ch);
        if (s_len > IMU_DATA_MAX)      s_st = IMU_ST_IDLE;
        else if (s_len == 0u)          s_st = IMU_ST_CS;
        else { s_ix = 0u;              s_st = IMU_ST_DATA; }
        break;
    case IMU_ST_DATA:                /* DATA[LEN] */
        /* 后置++先用当前下标存字节，再把s_ix加1；与++s_ix的先加后用不同。 */
        s_buf[s_ix++] = ch;
        s_sum = (uint8_t)(s_sum + ch);
        if (s_ix >= s_len) s_st = IMU_ST_CS;
        break;
    case IMU_ST_CS:                  /* CS=(DevID+CMD+LEN+DATA 各字节和)&0xFF */
        if (ch == s_sum) imu_frame(s_cmd, s_len, s_buf);
        s_st = IMU_ST_IDLE;
        break;
    default:
        s_st = IMU_ST_IDLE;
        break;
    }
}

/* ImuTask 周期调:链路静默超时 → 清"数据可用"并复位解析态(防掉线重连被当续体吃) */
void imu_tick_parse(void)
{
    if ((uint32_t)(HAL_GetTick() - s_last_ms) > IMU_LINK_TIMEOUT_MS) {
        s_valid = 0u;
        s_st = IMU_ST_IDLE;
    }
}

/* 读当前航向角(度 0..360;IMU 没接/没解析出前恒 0,上层要等 imu_ok 才信) */
float imu_yaw_deg(void)   { return s_yaw; }
float imu_heading_deg(void) { return s_heading_cont; }   /* 连续航向(度,跨绕已解) */
/**
 * @brief 把当前有效连续航向记为本段软件零点。
 * @retval 1=已记录，0=没有足够新的有效姿态。
 * @note 相当于记下起始角度供后续相减，不给传感器发清零指令，也不改变全局heading。
 */
uint8_t imu_zero_leg_heading(void)
{
    uint32_t pm = __get_PRIMASK();
    uint8_t ok;
    __disable_irq();
    /* &&是逻辑与：既解析成功过，又没过有效期，才允许把当前角度记作本段零点。 */
    ok = s_valid && (uint32_t)(HAL_GetTick() - s_valid_ms) < IMU_LINK_TIMEOUT_MS;
    if (ok) s_leg_heading_zero = s_heading_cont;
    __set_PRIMASK(pm);
    return ok;
}
float imu_leg_heading_deg(void) { return s_heading_cont - s_leg_heading_zero; }
float imu_pitch_deg(void) { return s_pitch; }
float imu_roll_deg(void)  { return s_roll; }

/* 数据可用?最近解析出过有效姿态且在超时窗内回 1(链路活 + 帧校验过才算) */
uint8_t imu_ok(void)
{
    return s_valid
        && (uint32_t)(HAL_GetTick() - s_valid_ms) < IMU_LINK_TIMEOUT_MS;
}

uint32_t imu_last_valid_age_ms(void)
{
    if (!s_yaw_have_last) return UINT32_MAX;
    return (uint32_t)(HAL_GetTick() - s_valid_ms);
}
