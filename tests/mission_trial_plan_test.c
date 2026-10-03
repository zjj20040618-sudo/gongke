/* Pure host test: no hardware, vision detection, arm operation, or motion. */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "mission_trial_plan.h"

#define CHECK(expr) do { \
    if (!(expr)) { \
        fprintf(stderr, "mission trial plan line %d: %s\n", __LINE__, #expr); \
        return 1; \
    } \
} while (0)

static int close_mm(float actual, float expected)
{
    return fabsf(actual - expected) < 0.002f;
}

static int test_recipe(void)
{
    const uint8_t modes[10] = {17, 16, 30, 15, 17, 15, 20, 15, 20, 15};
    const uint16_t distances[10] = {500, 600, 0, 750, 730, 830, 0, 2450, 0, 2125};
    const int16_t turns[10] = {0, 0, -95, 0, 0, 0, 85, 0, 85, 0};
    CHECK(MISSION_TRIAL_MODE == 32u);
    CHECK(MISSION_TRIAL_ROUTE_LEGS == 10u);
    CHECK(MISSION_TRIAL_ROUTE_SPEED_MMS == 100.0f);
    CHECK(MISSION_TRIAL_TASK_CORRIDOR_MM == 2450.0f);
    CHECK(MISSION_TRIAL_RESCUE_CORRIDOR_MM == 2125.0f);
    CHECK(MISSION_TRIAL_BALL_HOLD_MS == 10000u);
    CHECK(MISSION_TRIAL_BUCKET_HOLD_MS == 10000u);
    CHECK(MISSION_TRIAL_HOSTAGE_HOLD_MS == 10000u);
    for (unsigned i = 0; i < 10u; ++i) {
        CHECK(mission_trial_route_plan[i].mode == modes[i]);
        CHECK(mission_trial_route_plan[i].distance_mm == distances[i]);
        CHECK(mission_trial_route_plan[i].turn_deg == turns[i]);
    }
    return 0;
}

static int test_forward_and_stationary(void)
{
    MissionTrialRoad road;
    mission_trial_road_init(&road, 2450.0f, 10.0f, 3000.0f, -200.0f);
    CHECK(close_mm(road.progress_mm, 0.0f));
    CHECK(close_mm(mission_trial_road_remaining(&road), 2450.0f));
    mission_trial_road_update(&road, 10.0f, 3500.0f, -200.0f);
    CHECK(close_mm(road.progress_mm, 500.0f));
    /* Each 10-second hold would produce identical stationary samples. */
    for (unsigned i = 0; i < 1500u; ++i)
        mission_trial_road_update(&road, 10.0f, 3500.0f, -200.0f);
    CHECK(close_mm(road.progress_mm, 500.0f));
    mission_trial_road_update(&road, 10.0f, 4000.0f, -200.0f);
    CHECK(close_mm(mission_trial_road_remaining(&road), 1450.0f));
    /* Forward arrival followed by overshoot retains signed evidence. */
    mission_trial_road_update(&road, 10.0f, 5450.0f, -200.0f);
    CHECK(close_mm(mission_trial_road_remaining(&road), 0.0f));
    mission_trial_road_update(&road, 10.0f, 5462.0f, -200.0f);
    CHECK(close_mm(mission_trial_road_remaining(&road), -12.0f));
    return 0;
}

static int test_two_180_and_bucket_offset(void)
{
    MissionTrialRoad road;
    mission_trial_road_init(&road, 2450.0f, 0.0f, 0.0f, 0.0f);
    mission_trial_road_update(&road, 0.0f, 400.0f, 0.0f);  /* ball scan */
    mission_trial_road_update(&road, 0.0f, 425.0f, 0.0f);  /* ball align */
    mission_trial_road_update(&road, 180.0f, 425.0f, 0.0f);/* first +180 */
    mission_trial_road_update(&road, 180.0f, 525.0f, 0.0f);/* reverse-facing scan */
    mission_trial_road_update(&road, 180.0f, 545.0f, 0.0f);/* bucket align */
    CHECK(close_mm(road.progress_mm, 305.0f));
    CHECK(close_mm(mission_trial_road_remaining(&road), 2145.0f));
    mission_trial_road_update(&road, 360.0f, 545.0f, 0.0f);/* second +180 */
    mission_trial_road_update(&road, 360.0f, 1245.0f, 0.0f);/* anti single pass */
    mission_trial_road_update(&road, 360.0f, 1225.0f, 0.0f);/* signed alignment */
    CHECK(close_mm(road.progress_mm, 985.0f));
    CHECK(close_mm(mission_trial_road_remaining(&road), 1465.0f));
    /* Continue only remaining distance, not another complete 2450. */
    mission_trial_road_update(&road, 360.0f, 2690.0f, 0.0f);
    CHECK(close_mm(road.progress_mm, 2450.0f));
    CHECK(close_mm(mission_trial_road_remaining(&road), 0.0f));
    return 0;
}

static int test_projection(void)
{
    MissionTrialRoad road;
    mission_trial_road_init(&road, 1000.0f, 0.0f, 0.0f, 0.0f);
    mission_trial_road_update(&road, 90.0f, 0.0f, 100.0f);
    CHECK(close_mm(road.progress_mm, -100.0f)); /* right at +90 is road-back */
    mission_trial_road_update(&road, -90.0f, 0.0f, 200.0f);
    CHECK(close_mm(road.progress_mm, 0.0f));
    mission_trial_road_update(&road, 180.0f, 20.0f, 200.0f);
    CHECK(close_mm(road.progress_mm, -20.0f));
    mission_trial_road_update(&road, 540.0f, 50.0f, 200.0f);
    CHECK(close_mm(road.progress_mm, -50.0f));
    mission_trial_road_update(&road, 720.0f, 60.0f, 200.0f);
    CHECK(close_mm(road.progress_mm, -40.0f));
    /* Both axes participate at a non-cardinal heading. */
    mission_trial_road_update(&road, 30.0f, 160.0f, 240.0f);
    CHECK(close_mm(road.progress_mm, -40.0f + 100.0f * cosf(0.5235987756f) - 20.0f));
    return 0;
}

static int test_explicit_reset_and_new_corridor(void)
{
    MissionTrialRoad road;
    mission_trial_road_init(&road, 2450.0f, 100.0f, 900.0f, 100.0f);
    mission_trial_road_update(&road, 100.0f, 1400.0f, 100.0f);
    CHECK(close_mm(road.progress_mm, 500.0f));
    /* Caller has first accounted for braking, then explicitly reset totals. */
    mission_trial_road_update(&road, 100.0f, 1407.0f, 100.0f);
    mission_trial_road_rebase_odometry(&road, 0.0f, 0.0f);
    CHECK(close_mm(road.progress_mm, 507.0f));
    CHECK(close_mm(road.road_heading_deg, 100.0f));
    CHECK(close_mm(road.total_mm, 2450.0f));
    mission_trial_road_update(&road, 100.0f, 200.0f, 0.0f);
    CHECK(close_mm(road.progress_mm, 707.0f));
    CHECK(close_mm(mission_trial_road_remaining(&road), 1743.0f));
    /* +85 turn into rescue: new anchor, not leftover task progress. */
    mission_trial_road_init(&road, 2125.0f, 185.0f, 200.0f, 0.0f);
    CHECK(close_mm(road.progress_mm, 0.0f));
    mission_trial_road_update(&road, 185.0f, 900.0f, 0.0f);  /* single pass */
    mission_trial_road_update(&road, 185.0f, 925.0f, 0.0f);  /* hostage align */
    for (unsigned i = 0; i < 500u; ++i)
        mission_trial_road_update(&road, 185.0f, 925.0f, 0.0f);
    CHECK(close_mm(mission_trial_road_remaining(&road), 1400.0f));
    mission_trial_road_update(&road, 185.0f, 2325.0f, 0.0f);
    CHECK(close_mm(mission_trial_road_remaining(&road), 0.0f));
    return 0;
}

static int test_no_automatic_reset_guess(void)
{
    MissionTrialRoad road;
    MissionTrialRoad untouched;
    memset(&untouched, 0, sizeof untouched);
    mission_trial_road_update(&untouched, 0.0f, 500.0f, 0.0f);
    CHECK(untouched.initialized == 0u && untouched.progress_mm == 0.0f);
    mission_trial_road_init(&road, 2450.0f, 0.0f, 1000.0f, 0.0f);
    mission_trial_road_update(&road, 0.0f, 1100.0f, 0.0f);
    /* Deliberately violate the reset contract. No automatic reset detection:
     * -1100 is integrated as an actual reverse movement, not discarded. */
    mission_trial_road_update(&road, 0.0f, 0.0f, 0.0f);
    CHECK(close_mm(road.progress_mm, -1000.0f));
    CHECK(close_mm(mission_trial_road_remaining(&road), 3450.0f));
    return 0;
}

int main(void)
{
    CHECK(test_recipe() == 0);
    CHECK(test_forward_and_stationary() == 0);
    CHECK(test_two_180_and_bucket_offset() == 0);
    CHECK(test_projection() == 0);
    CHECK(test_explicit_reset_and_new_corridor() == 0);
    CHECK(test_no_automatic_reset_guess() == 0);
    puts("mission trial plan: recipe/three holds/0-180-360 projection/remaining/reset contract/new rescue corridor passed (host only)");
    return 0;
}
