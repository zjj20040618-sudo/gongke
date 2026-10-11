#ifndef APP_ROUTE44_PLAN_H
#define APP_ROUTE44_PLAN_H
#include "route_test_plan.h"

/* Independent physical plan: never index31/43 arrays with this index.
 * Roles keep their existing business/arm ownership. QR opens after node1.
 * Ordinary speed50, cross/contact/fine exempt; zeroFF retains yaw feedback. */
#define ROUTE44_MODE 44
#define ROUTE44_STAGES 22u
#define ROUTE44_ROAD_ROLE 16u
#define ROUTE44_QR_AFTER_INDEX 1u
#define ROUTE44_BALL_ENTRY_INDEX 14u
#define ROUTE44_CROSS_INDEX 4u
/* Unmeasured trial: reverse-crossing only, left-front/rear target RPM +10%.
 * Whole-side differential can turn the chassis; it is not lateral-only FF. */
#define ROUTE44_CROSS_LEFT_SCALE 1.10f
#define ROUTE44_V_MMS 50.0f
#define ROUTE44_BALL_OFFSET_MM 30u
typedef struct { RouteTestLeg leg; uint8_t role; } Route44Node;
static const Route44Node s_route44_plan[ROUTE44_STAGES] = {
    {{15u, 600u, ROUTE44_V_MMS, "START_FORWARD600", 1u}, 0u},
    {{20u,   0u, ROUTE44_V_MMS, "RIGHT90_THEN_QR", 1u}, ROUTE44_ROAD_ROLE},
    {{15u, 600u, ROUTE44_V_MMS, "QR_THEN_FORWARD600", 1u}, 1u},
    {{20u,   0u, ROUTE44_V_MMS, "RIGHT90_BEFORE_CROSS", 1u}, 2u},
    {{16u, ROUTE31_CROSS_BACK_MM, ROUTE_CROSS_BACK_V_MMS, "BACK_CROSS620", 0u}, 3u},
    {{15u,   0u, ROUTE31_BOARD_CONTACT_V_MMS, "FORWARD_TILT_CONTACT", 0u}, 4u},
    {{16u, ROUTE_POST_CROSS_CLEAR_MM, ROUTE44_V_MMS, "BACK_CLEAR190", 1u}, 5u},
    {{20u,   0u, ROUTE44_V_MMS, "RIGHT90_AFTER_BACK_CLEAR190", 1u}, ROUTE44_ROAD_ROLE},
    {{15u, 780u, ROUTE44_V_MMS, "FORWARD780", 1u}, 6u},
    {{20u,   0u, ROUTE44_V_MMS, "RIGHT90_BEFORE_BACK800", 1u}, ROUTE44_ROAD_ROLE},
    {{16u, 800u, ROUTE44_V_MMS, "BACK800", 1u}, ROUTE44_ROAD_ROLE},
    {{20u,   0u, ROUTE44_V_MMS, "RIGHT90_BEFORE_BACK780", 1u}, ROUTE44_ROAD_ROLE},
    {{16u, 780u, ROUTE44_V_MMS, "BACK780", 1u}, 7u},
    {{20u,   0u, ROUTE44_V_MMS, "RIGHT90_TO_BALL", 1u}, ROUTE44_ROAD_ROLE},
    {{15u, 200u, ROUTE44_V_MMS, "FORWARD200_BALL_RX_RACK_ON_FIRST_TARGET", 1u}, ROUTE44_ROAD_ROLE},
    {{41u,   0u, ROUTE44_V_MMS, "BALL_GRAB_TURN_BUCKET", 1u}, ROUTE31_PAIR_STAGE},
    {{22u,   0u, ROUTE44_V_MMS, "BUCKET_RELEASE_RETRACT_RETURN180", 1u}, ROUTE31_RETURN180_STAGE},
    {{35u,   0u, ROUTE44_V_MMS, "SELECTED_TARGET_X_FIRE2S", 1u}, ROUTE31_TARGET_STAGE},
    {{15u,   0u, ROUTE44_V_MMS, "TARGET_TO_CORNER", 1u}, ROUTE31_TARGET_CORNER_STAGE},
    {{20u,   0u, ROUTE44_V_MMS, "RIGHT_TO_HOSTAGE_AS31", 1u}, ROUTE31_HOSTAGE_TURN_STAGE},
    {{40u,   0u, ROUTE44_V_MMS, "HOSTAGE_GRAB_RANK_AS31", 1u}, ROUTE31_HOSTAGE_STAGE},
    {{15u,   0u, ROUTE44_V_MMS, "HOSTAGE_RANK_TO_END", 1u}, ROUTE31_HOSTAGE_EXIT_STAGE}
};
static const RouteTestLeg s_route44_left_leg = {
    17u, ROUTE44_BALL_OFFSET_MM, ROUTE44_V_MMS, "BALL_ALIGNED_BODY_LEFT30", 1u
};
static const RouteTestLeg s_route44_right_leg = {
    18u, ROUTE44_BALL_OFFSET_MM, ROUTE44_V_MMS, "BALL_GRABBED_BODY_RIGHT30_BEFORE180", 1u
};
#endif
