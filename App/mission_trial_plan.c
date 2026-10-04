/*
 * 初学者导读：这里不发电机命令，只保存路线表并计算模式32的道路进度。
 * 路线表每个结构体按“模式、距离、转角”填写；方向由模式或角度正负决定。
 * road 是指向 MissionTrialRoad 结构体的指针；road->字段 等价于 (*road).字段。
 * 每次只积分“本次累计里程减上次累计里程”的增量，不能把累计值反复相加。
 * cosf/sinf 把车身方向上的位移投影到固定道路方向；输入角度必须是弧度。
 */

#include "mission_trial_plan.h"
#include <math.h>

#define TRIAL_DEG_TO_RAD 0.01745329251994329577f

const MissionTrialRouteLeg mission_trial_route_plan[MISSION_TRIAL_ROUTE_LEGS] = {
    { 17u,  500u,   0 },
    { 16u,  600u,   0 },
    { 30u,    0u, -95 },
    { 15u,  750u,   0 },
    { 17u,  730u,   0 },
    { 15u,  830u,   0 },
    { 20u,    0u,  85 },
    { 15u, 2450u,   0 },
    { 20u,    0u,  85 },
    { 15u, 2125u,   0 }
};

/**
 * @brief 从当前里程样本建立一条新道路账本，进度从0开始。
 * @param road 要写入的账本地址。
 * @param total_mm 新道路的总长mm。
 * @param road_heading_deg 道路固定朝向，度。
 * @param fore_mm 当前累计前后里程mm。
 * @param lat_mm 当前累计横向里程mm。
 * @retval 无。
 */
void mission_trial_road_init(MissionTrialRoad *road, float total_mm,
                            float road_heading_deg,
                            float fore_mm, float lat_mm)
{
    if (!road) return;
    road->road_heading_deg = road_heading_deg;
    road->total_mm = total_mm;
    road->progress_mm = 0.0f;
    road->last_fore_mm = fore_mm;
    road->last_lat_mm = lat_mm;
    road->initialized = 1u;
}

/**
 * @brief 把两次里程样本的差值投影到道路轴并累计。
 * @param road 已初始化的账本地址。
 * @param continuous_heading_deg 当前全局连续航向，度；不是本段清零航向。
 * @param fore_mm 本次累计前后里程mm。
 * @param lat_mm 本次累计横向里程mm。
 * @retval 无。
 * @note 转到180°后，车身向前移动会减少原道路进度；带符号投影保留这条信息。
 */
void mission_trial_road_update(MissionTrialRoad *road,
                              float continuous_heading_deg,
                              float fore_mm, float lat_mm)
{
    float delta_fore, delta_lat, theta;
    if (!road || !road->initialized) return;

    /* 先取这次新增的位移。累计读数100到105，本拍只增加5，不是增加105。 */
    delta_fore = fore_mm - road->last_fore_mm;
    delta_lat = lat_mm - road->last_lat_mm;
    /* Reduce continuous headings before trig; do not change the road anchor. */
    theta = fmodf(continuous_heading_deg - road->road_heading_deg, 360.0f)
            * TRIAL_DEG_TO_RAD;
    /* theta=0时只加前后位移；theta=180°时cos=-1，向车头走会使原路进度减少。 */
    road->progress_mm += delta_fore * cosf(theta) - delta_lat * sinf(theta);
    road->last_fore_mm = fore_mm;
    road->last_lat_mm = lat_mm;
}

/**
 * @brief 调用者明确清零里程后，仅更新下一次相减的采样基线。
 * @param road 已初始化账本地址。
 * @param fore_mm 清零后的前后里程。
 * @param lat_mm 清零后的横向里程。
 * @retval 无。
 * @note 不清已有道路进度；正常对位运动不能用此接口藏掉。
 */
void mission_trial_road_rebase_odometry(MissionTrialRoad *road,
                                      float fore_mm, float lat_mm)
{
    if (!road || !road->initialized) return;
    road->last_fore_mm = fore_mm;
    road->last_lat_mm = lat_mm;
}

float mission_trial_road_remaining(const MissionTrialRoad *road)
{
    if (!road || !road->initialized) return 0.0f;
    return road->total_mm - road->progress_mm;
}
