#include "route45_plan.h"
#include <math.h>
#include <string.h>
#include <limits.h>

#define R45_DEG_RAD 0.01745329251994329577f

void route45_config_defaults(Route45Config *out)
{
    if (out) {
        memset(out, 0, sizeof *out);
        out->bucket_policy = R45_BUCKET_SEARCH_BACKWARD; /* User confirmed; distances remain unmeasured. */
    }
}

const char *route45_config_missing(const Route45Config *config)
{
    static const char *const names[3] = {
        "BALL1_BACK_MM_UNMEASURED", "BALL2_BACK_MM_UNMEASURED", "BALL3_BACK_MM_UNMEASURED"
    };
    unsigned i;
    if (!config) return "CONFIG_MISSING";
    for (i = 0u; i < 3u; ++i) {
        if (!config->ball_to_corner_mm[i]) return names[i];
        if (config->ball_to_corner_mm[i] > ROUTE45_INPUT_MAX_MM)
            return "BACK_MM_OUT_OF_INPUT_RANGE";
    }
    if (config->bucket_policy == R45_BUCKET_POLICY_UNSET) return "BUCKET_SEARCH_POLICY_UNSET";
    if (config->bucket_policy < R45_BUCKET_WAIT_FIRST_COORDINATE ||
        config->bucket_policy > R45_BUCKET_SEARCH_BACKWARD) return "BUCKET_SEARCH_POLICY_INVALID";
    return 0;
}

void route45_plan_init(Route45Plan *plan)
{
    if (!plan) return;
    memset(plan, 0, sizeof *plan);
    plan->reason = "PREPARATION_ONLY_NOT_FIRMWARE_REGISTERED";
}

int route45_plan_active(const Route45Plan *plan)
{
    return plan && plan->state >= R45_PREFIX && plan->state <= R45_TARGET_AND_RESCUE;
}

void route45_plan_stop(Route45Plan *plan, const char *reason)
{
    if (!plan) return;
    memset(&plan->intent, 0, sizeof plan->intent);
    plan->state = R45_STOPPED;
    plan->tracking = 0u;
    plan->reason = reason ? reason : "STOPPED";
}

static int r45_issue(Route45Plan *plan, Route45State state, Route45IntentKind kind)
{
    if (plan->next_intent_id == UINT32_MAX) {
        route45_plan_stop(plan, "INTENT_ID_EXHAUSTED");
        return 0;
    }
    memset(&plan->intent, 0, sizeof plan->intent);
    plan->intent.id = ++plan->next_intent_id;
    plan->intent.kind = kind;
    plan->intent.ball_rank = plan->ball_rank;
    plan->intent.bucket_policy = plan->config.bucket_policy;
    plan->state = state;
    plan->reason = route45_state_name(state);
    return 1;
}

int route45_plan_begin(Route45Plan *plan, const Route45Config *config)
{
    const char *missing;
    Route45Config snapshot;
    uint32_t serial;
    if (!plan || route45_plan_active(plan)) return 0;
    missing = route45_config_missing(config);
    if (missing) {
        memset(&plan->intent, 0, sizeof plan->intent);
        plan->state = R45_WAIT_CONFIG;
        plan->tracking = 0u;
        plan->reason = missing;
        return 0;
    }
    snapshot = *config; /* Also supports &plan->config without clearing input. */
    serial = plan->next_intent_id;
    memset(plan, 0, sizeof *plan);
    plan->next_intent_id = serial;
    plan->config = snapshot;
    return r45_issue(plan, R45_PREFIX, R45_INTENT_PREFIX_WITH_QR_GATE);
}

const char *route45_state_name(Route45State state)
{
    static const char *const names[] = {
        "OFF", "WAIT_CONFIG", "PREFIX_QR_GATE", "ALIGN_BALL", "GRAB_BALL",
        "WAIT_BALL_RANK", "BACK_TO_COMMON_CORNER", "FIRST180_AT_CORNER",
        "NEW_BUCKET_REQUEST_ALIGN", "PLACE_AND_RETRACT", "RETURN_SAME_CORNER",
        "SECOND180_AT_CORNER", "TARGET_RESCUE_TAIL", "DONE", "STOPPED"
    };
    return (unsigned)state < sizeof names / sizeof names[0] ? names[state] : "INVALID_STATE";
}

int route45_plan_bind_request(Route45Plan *plan, uint32_t intent_id,
                             uint16_t request, uint8_t task, uint8_t digit)
{
    uint16_t *binding;
    if (!plan || !request || !intent_id || plan->intent.id != intent_id) return 0;
    if (plan->state == R45_ALIGN_BALL && task == PROTO_TASK_BALL && digit == plan->qr[0])
        binding = &plan->ball_request;
    else if (plan->state == R45_ALIGN_BUCKET && task == PROTO_TASK_BUCKET && digit == 0u &&
             request != plan->ball_request)
        binding = &plan->bucket_request;
    else return 0;
    if (*binding && *binding != request) return 0;
    *binding = request;
    return 1;
}

static int r45_back(Route45Plan *plan)
{
    uint8_t rank = plan->ball_rank;
    if (rank < 1u || rank > 3u || !plan->config.ball_to_corner_mm[rank - 1u]) {
        route45_plan_stop(plan, "BALL_RANK_OR_DISTANCE_MISSING");
        return 0;
    }
    if (!r45_issue(plan, R45_BACK_TO_CORNER, R45_INTENT_BACK_TO_CORNER)) return 0;
    plan->intent.forward_mm = -(float)plan->config.ball_to_corner_mm[rank - 1u];
    return 1;
}

int route45_plan_accept_ball_rank(Route45Plan *plan,
                                 const ProtoTargetRank *rank,
                                 const ProtoWireDiag *link)
{
    if (!plan || !rank || !link || !plan->ball_request ||
        (plan->state != R45_ALIGN_BALL && plan->state != R45_GRAB_BALL &&
         plan->state != R45_WAIT_BALL_RANK) ||
        !link->receiving || !link->controlled || !link->ack || link->failed ||
        link->task != PROTO_TASK_BALL || link->selection != plan->qr[0] ||
        link->request != plan->ball_request || rank->request != plan->ball_request ||
        rank->task != PROTO_TASK_BALL || rank->digit != plan->qr[0] ||
        rank->rank < 1u || rank->rank > 3u) return 0;
    if (plan->ball_rank && plan->ball_rank != rank->rank) return 0;
    plan->ball_order = *rank;
    plan->ball_rank = rank->rank;
    if (plan->state == R45_WAIT_BALL_RANK) return r45_back(plan);
    return 1;
}

static int r45_sample_valid(const Route45OdoSample *sample)
{
    return sample && sample->valid && isfinite(sample->fore_mm) &&
        isfinite(sample->right_mm) && isfinite(sample->heading_deg);
}

static void r45_anchor(Route45Plan *plan, const Route45OdoSample *sample)
{
    /* Anchor BEFORE the first180, retaining translational turn drift. */
    float heading = fmodf(sample->heading_deg, 360.0f);
    mission_trial_road_init(&plan->corner_fore, 0.0f, heading,
                           sample->fore_mm, sample->right_mm);
    mission_trial_road_init(&plan->corner_right, 0.0f, heading + 90.0f,
                           sample->fore_mm, sample->right_mm);
    plan->odo_epoch = sample->encoder_epoch;
    plan->tracking = 1u;
}

int route45_plan_track_odometry(Route45Plan *plan, const Route45OdoSample *sample)
{
    if (!plan || !plan->tracking || !route45_plan_active(plan)) return 0;
    if (!r45_sample_valid(sample)) {
        route45_plan_stop(plan, "ODO_OR_CONTINUOUS_HEADING_INVALID");
        return 0;
    }
    if (sample->encoder_epoch != plan->odo_epoch) {
        route45_plan_stop(plan, "ODO_EPOCH_CHANGE_WITHOUT_REBASE");
        return 0;
    }
    /* Reuse only the stateless math API, never mission32's global s_road. */
    mission_trial_road_update(&plan->corner_fore, sample->heading_deg,
                             sample->fore_mm, sample->right_mm);
    mission_trial_road_update(&plan->corner_right, sample->heading_deg,
                             sample->fore_mm, sample->right_mm);
    if (!isfinite(plan->corner_fore.progress_mm) || !isfinite(plan->corner_right.progress_mm)) {
        route45_plan_stop(plan, "ODO_ACCUMULATION_INVALID");
        return 0;
    }
    return 1;
}

int route45_plan_rebase_odometry(Route45Plan *plan,
                                const Route45OdoSample *before_reset,
                                const Route45OdoSample *after_reset)
{
    if (!plan || !plan->tracking || !r45_sample_valid(before_reset) ||
        !r45_sample_valid(after_reset) || before_reset->encoder_epoch != plan->odo_epoch ||
        after_reset->encoder_epoch == before_reset->encoder_epoch) {
        if (plan && plan->tracking) route45_plan_stop(plan, "INVALID_EXPLICIT_ODO_REBASE");
        return 0;
    }
    if (!route45_plan_track_odometry(plan, before_reset)) return 0;
    mission_trial_road_rebase_odometry(&plan->corner_fore, after_reset->fore_mm, after_reset->right_mm);
    mission_trial_road_rebase_odometry(&plan->corner_right, after_reset->fore_mm, after_reset->right_mm);
    plan->odo_epoch = after_reset->encoder_epoch;
    return 1;
}

static int r45_alignment_ok(const Route45Completion *event, uint16_t binding)
{
    return binding && event->request == binding &&
        (event->evidence == R45_EVIDENCE_FRESH_STOPPED_ALIGNMENT ||
         event->evidence == R45_EVIDENCE_EXPLICIT_SEEN_THEN_LOST);
}

static int r45_return(Route45Plan *plan, const Route45OdoSample *sample)
{
    float theta, fore, right;
    if (!route45_plan_track_odometry(plan, sample)) return 0;
    theta = fmodf(sample->heading_deg - plan->corner_fore.road_heading_deg, 360.0f) * R45_DEG_RAD;
    fore = plan->corner_fore.progress_mm;
    right = plan->corner_right.progress_mm;
    /* Reverse NET position in the current body frame, not abs(total wheel
     * distance), not the ball's rank-selected back distance. Preserve both axes. */
    plan->return_fore_mm = -fore * cosf(theta) - right * sinf(theta);
    plan->return_right_mm = fore * sinf(theta) - right * cosf(theta);
    if (fabsf(plan->return_fore_mm) > ROUTE45_INPUT_MAX_MM ||
        fabsf(plan->return_right_mm) > ROUTE45_INPUT_MAX_MM) {
        route45_plan_stop(plan, "RETURN_VECTOR_OUT_OF_INPUT_RANGE");
        return 0;
    }
    if (!r45_issue(plan, R45_RETURN_TO_CORNER, R45_INTENT_RETURN_TO_CORNER)) return 0;
    plan->intent.forward_mm = plan->return_fore_mm;
    plan->intent.right_mm = plan->return_right_mm;
    return 1;
}

int route45_plan_complete(Route45Plan *plan, const Route45Completion *event)
{
    unsigned i;
    if (!plan || !event || !route45_plan_active(plan) || !event->intent_id ||
        event->intent_id != plan->intent.id || plan->intent.kind == R45_INTENT_NONE) return 0;
    if (!event->succeeded || !event->stopped) {
        route45_plan_stop(plan, "EXECUTOR_FAILED_OR_NOT_STOPPED");
        return 0;
    }
    switch (plan->state) {
    case R45_PREFIX:
        for (i = 0u; i < 3u; ++i) {
            if (event->qr[i] < 1 || event->qr[i] > 3) {
                route45_plan_stop(plan, "CURRENT_RUN_LEGAL_QR_REQUIRED");
                return 0;
            }
            plan->qr[i] = event->qr[i];
        }
        if (!r45_issue(plan, R45_ALIGN_BALL, R45_INTENT_REQUEST_AND_ALIGN_BALL)) return 0;
        plan->intent.task = PROTO_TASK_BALL;
        plan->intent.digit = (uint8_t)plan->qr[0];
        return 1;
    case R45_ALIGN_BALL:
        if (!r45_alignment_ok(event, plan->ball_request)) return 0;
        return r45_issue(plan, R45_GRAB_BALL, R45_INTENT_COMPLETE_GRAB_RECIPE);
    case R45_GRAB_BALL:
        if (plan->ball_rank) return r45_back(plan);
        memset(&plan->intent, 0, sizeof plan->intent);
        plan->state = R45_WAIT_BALL_RANK;
        plan->reason = "GRAB_DONE_KEEP_BALL_RX_OPEN_WAIT_LEGAL54";
        return 1;
    case R45_BACK_TO_CORNER:
        if (!r45_sample_valid(&event->odo)) {
            route45_plan_stop(plan, "CORNER_ANCHOR_SAMPLE_REQUIRED");
            return 0;
        }
        r45_anchor(plan, &event->odo);
        if (!r45_issue(plan, R45_TURN_TO_BUCKET, R45_INTENT_FIRST180)) return 0;
        plan->intent.turn_deg = 180.0f;
        return 1;
    case R45_TURN_TO_BUCKET:
        if (!route45_plan_track_odometry(plan, &event->odo)) return 0;
        if (!r45_issue(plan, R45_ALIGN_BUCKET, R45_INTENT_NEW_BUCKET_REQUEST_AND_ALIGN)) return 0;
        plan->intent.task = PROTO_TASK_BUCKET;
        plan->intent.digit = 0u;
        return 1;
    case R45_ALIGN_BUCKET:
        if (!r45_alignment_ok(event, plan->bucket_request)) return 0;
        if (!route45_plan_track_odometry(plan, &event->odo)) return 0;
        return r45_issue(plan, R45_PLACE_AND_RETRACT, R45_INTENT_RELEASE_AND_RETRACT);
    case R45_PLACE_AND_RETRACT:
        /* Executor completion must cover release + lift + rack retraction,
         * not just servo-open. It must supply the final coast/still sample. */
        return r45_return(plan, &event->odo);
    case R45_RETURN_TO_CORNER:
        if (!route45_plan_track_odometry(plan, &event->odo)) return 0;
        /* succeeded is the executor's reached/stopped result, not a timeout
         * promoted to success. The remaining vector remains inspectable. */
        plan->tracking = 0u;
        if (!r45_issue(plan, R45_TURN_TO_TARGET, R45_INTENT_SECOND180)) return 0;
        plan->intent.turn_deg = 180.0f;
        return 1;
    case R45_TURN_TO_TARGET:
        return r45_issue(plan, R45_TARGET_AND_RESCUE, R45_INTENT_TARGET_AND_RESCUE_TAIL);
    case R45_TARGET_AND_RESCUE:
        memset(&plan->intent, 0, sizeof plan->intent);
        plan->state = R45_DONE;
        plan->reason = "PLAN_DONE_EXECUTOR_RESULT_NOT_PHYSICAL_ACCEPTANCE";
        return 1;
    default:
        return 0;
    }
}
