#ifndef APP_BUCKET36_PLAN_H
#define APP_BUCKET36_PLAN_H

#include "route_test_plan.h"

/* Independent mode36 trial without QR or bucket alignment.
 * 2026-10-06: stable reverse crossing is BACK650/v300, FWD80/v20 against
 * the board, BACK190/v100. No lateral d35 offset.
 * Body reversed: old LEFT730/FWD780/RIGHT90 becomes RIGHT730/BACK780/LEFT92.
 * Preserve the bench entry FWD200 then STOP, not a full task corridor.
 * Bucket alignment constants below remain used only by route31/34 while
 * their separate bucket/manual-d tail remains unchanged by this update. */
#define BUCKET_ROUTE_MODE       36
#define BUCKET_ROUTE_STAGES     10u
#define CROSS_ONLY_MODE         37
#define CROSS37_STAGES           3u /* no approach, offset, turns, QR or bucket stage */
#define BUCKET36_ALIGN_STAGE     8u
#define BUCKET36_BACK_STAGE      9u
#define BUCKET36_CX            500
#define BUCKET36_TOL_PX          5
#define BUCKET36_ALIGN_V_MMS    50.0f
#define BUCKET36_GOOD_FRAMES     5u
#define BUCKET36_FRESH_MS      300u
#define BUCKET36_REPORT_MS     500u

static const RouteTestLeg s_bucket36_plan[BUCKET_ROUTE_STAGES] = {
    { 17u, 530u, 100.0f, "START_LEFT", 1u },
    { 16u, 650u, 100.0f, "BACK_TO_3RD", 1u },
    { 20u,   0u, 100.0f, "RIGHT90_BEFORE_CROSS", 1u },
    { 16u, ROUTE_CROSS_BACK_MM, ROUTE_CROSS_BACK_V_MMS, "BACK_FULL_OBSTACLE_ROAD", 0u },
    { 15u, ROUTE_POST_CROSS_ALIGN_MM, ROUTE_POST_CROSS_ALIGN_V_MMS, "POST_CROSS_FORWARD_ALIGN", 0u },
    { 16u, ROUTE_POST_CROSS_CLEAR_MM, ROUTE_POST_CROSS_CLEAR_V_MMS, "POST_CROSS_BACK_CLEAR", 1u },
    { 18u, 730u, 100.0f, "EXIT_RIGHT", 1u },
    { 16u, 780u, 100.0f, "BACK_TO_TASK_CORNER", 1u },
    { 30u,   0u, 100.0f, "LEFT92_TO_EOD", 1u },
    { 15u, 200u, 100.0f, "EOD_ENTRY_200_STOP", 1u }
};

/* Mode37 keeps only the same three crossing/contact/clearance legs as36.
 * 2026-10-06 user road test: omit LEFT35; add30 to board50 ->650/80/190.
 * Place the car in the post-RIGHT90 orientation before g; no turn is added. */
static const RouteTestLeg s_cross37_plan[CROSS37_STAGES] = {
    { 16u, ROUTE_CROSS_BACK_MM, ROUTE_CROSS_BACK_V_MMS, "BACK_FULL_OBSTACLE_ROAD", 0u },
    { 15u, ROUTE_POST_CROSS_ALIGN_MM, ROUTE_POST_CROSS_ALIGN_V_MMS, "POST_CROSS_FORWARD_ALIGN", 0u },
    { 16u, ROUTE_POST_CROSS_CLEAR_MM, ROUTE_POST_CROSS_CLEAR_V_MMS, "POST_CROSS_BACK_CLEAR", 1u }
};

#endif
