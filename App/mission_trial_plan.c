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

void mission_trial_road_update(MissionTrialRoad *road,
                              float continuous_heading_deg,
                              float fore_mm, float lat_mm)
{
    float delta_fore, delta_lat, theta;
    if (!road || !road->initialized) return;

    delta_fore = fore_mm - road->last_fore_mm;
    delta_lat = lat_mm - road->last_lat_mm;
    /* Reduce continuous headings before trig; do not change the road anchor. */
    theta = fmodf(continuous_heading_deg - road->road_heading_deg, 360.0f)
            * TRIAL_DEG_TO_RAD;
    road->progress_mm += delta_fore * cosf(theta) - delta_lat * sinf(theta);
    road->last_fore_mm = fore_mm;
    road->last_lat_mm = lat_mm;
}

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
