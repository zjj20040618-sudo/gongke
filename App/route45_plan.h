#ifndef APP_ROUTE45_PLAN_H
#define APP_ROUTE45_PLAN_H

#include <stdint.h>
#include "proto.h"
#include "mission_trial_plan.h"

/* PREPARATION ONLY. Reserved candidate45 is NOT registered in test.c or the
 * Keil project. This module produces intents, never drives hardware. Current
 *31/35/43/44, their HEX, QR windows, motion tuning and vision remain untouched.
 * A later, separately verified nonblocking executor must implement these
 * intents, including the existing31 prefix and target/hostage tail.
 *
 * Ball position means the validated task1/54 first-seen rank, NOT QR colour.
 * All three ball-to-corner distances start UNMEASURED(0). Bucket search
 * policy is independent (user confirmed BACK if unseen): rank1/3's old
 * bucket directions cannot be inherited after relocating every ball to
 * one common corner.
 *
 * Sequence: prefix/QR -> ball alignment -> complete existing grab/return
 * recipe -> rank-selected BACK -> anchor BEFORE first180 -> first180 ->
 * NEW bucket4/0 request/alignment -> release AND retract -> return to the
 * SAME anchor -> second180 -> existing target/hostage tail. Both new turns
 * request180, independently of31's current182 return trial.
 *
 * The caller must bind the actual ACKed request and keep BALL RX open until
 * its legal rank is copied. Bucket request intent is emitted only after the
 * first turn has succeeded and stopped. Alignment/loss decisions still need
 * the real vision client; rank54 is never coordinate/arrival evidence.
 */
#define ROUTE45_CANDIDATE_MODE 45u
#define ROUTE45_INPUT_MAX_MM 20000u /* Existing bench input bound, not a measured safe travel. */

typedef enum {
    R45_BUCKET_POLICY_UNSET = 0,
    R45_BUCKET_WAIT_FIRST_COORDINATE,
    R45_BUCKET_SEARCH_FORWARD,
    R45_BUCKET_SEARCH_BACKWARD
} Route45BucketPolicy;

typedef struct {
    uint32_t ball_to_corner_mm[3]; /*1/2/3. Positive BACK distances from the
                                    restored grab stop; never infer from colour. */
    Route45BucketPolicy bucket_policy;
} Route45Config;

typedef enum {
    R45_OFF = 0, R45_WAIT_CONFIG, R45_PREFIX, R45_ALIGN_BALL, R45_GRAB_BALL,
    R45_WAIT_BALL_RANK, R45_BACK_TO_CORNER, R45_TURN_TO_BUCKET,
    R45_ALIGN_BUCKET, R45_PLACE_AND_RETRACT, R45_RETURN_TO_CORNER,
    R45_TURN_TO_TARGET, R45_TARGET_AND_RESCUE, R45_DONE, R45_STOPPED
} Route45State;

typedef enum {
    R45_INTENT_NONE = 0,
    R45_INTENT_PREFIX_WITH_QR_GATE,
    R45_INTENT_REQUEST_AND_ALIGN_BALL,
    R45_INTENT_COMPLETE_GRAB_RECIPE,
    R45_INTENT_BACK_TO_CORNER,
    R45_INTENT_FIRST180,
    R45_INTENT_NEW_BUCKET_REQUEST_AND_ALIGN,
    R45_INTENT_RELEASE_AND_RETRACT,
    R45_INTENT_RETURN_TO_CORNER,
    R45_INTENT_SECOND180,
    R45_INTENT_TARGET_AND_RESCUE_TAIL
} Route45IntentKind;

typedef enum {
    R45_EVIDENCE_NONE = 0,
    R45_EVIDENCE_FRESH_STOPPED_ALIGNMENT,
    R45_EVIDENCE_EXPLICIT_SEEN_THEN_LOST
} Route45AlignmentEvidence;

/* Input must be one coherent encoder+IMU snapshot. Continuous clockwise
 * heading, forward-positive fore and RIGHT-positive lateral odometry; NOT
 * imu_leg_heading_deg() which can be zeroed between nodes. Track frequently
 * through turns, search, microadjustment, coast and arm action. A before/after
 * scalar difference alone cannot recover a path spanning180 degrees.
 * Epoch changes are accepted ONLY through explicit rebase with a final
 * pre-reset sample. Otherwise STOPPED/ODO_EPOCH_CHANGE, not a guessed reset.
 */
typedef struct {
    float fore_mm, right_mm, heading_deg;
    uint32_t encoder_epoch;
    uint8_t valid;
} Route45OdoSample;

typedef struct {
    Route45IntentKind kind;
    uint32_t id; /* Completion/binding must echo this ID; no stale worker completion. */
    float forward_mm, right_mm; /* Signed body-frame return vector, not total path length. */
    float turn_deg;
    uint8_t task, digit, ball_rank;
    Route45BucketPolicy bucket_policy;
} Route45Intent;

typedef struct {
    uint32_t intent_id;
    uint8_t succeeded, stopped;
    int32_t qr[3]; /* PREFIX completion: legal QR tuple from this run only. */
    uint16_t request; /* BALL/BUCKET alignment: bound, ACKed current request. */
    Route45AlignmentEvidence evidence;
    Route45OdoSample odo; /* BACK anchor; turn/release/return final stopped snapshots. */
} Route45Completion;

typedef struct {
    Route45State state;
    Route45Config config; /* Frozen by begin; no global/shared tuning writes. */
    Route45Intent intent;
    uint32_t next_intent_id; /* Kept across begin/stop; init only at owner startup. */
    int32_t qr[3];
    uint16_t ball_request, bucket_request;
    uint8_t ball_rank, tracking;
    ProtoTargetRank ball_order; /* Copied before BALL RX closes. */
    MissionTrialRoad corner_fore, corner_right; /* Private pure-math ledgers, NOT mode32 state. */
    uint32_t odo_epoch;
    float return_fore_mm, return_right_mm;
    const char *reason;
} Route45Plan;

void route45_config_defaults(Route45Config *out);
const char *route45_config_missing(const Route45Config *config);
void route45_plan_init(Route45Plan *plan);
/* Returns0 / WAIT_CONFIG / no intent until ALL distances and policy exist.
 * Returns1 for PLANNING only; does not authorize motors or indicate firmware
 * integration. No running plan may be overwritten by another begin. */
int route45_plan_begin(Route45Plan *plan, const Route45Config *config);
int route45_plan_active(const Route45Plan *plan);
const char *route45_state_name(Route45State state);
void route45_plan_stop(Route45Plan *plan, const char *reason);

/* Bind actual ACKed63 request after handling the emitted intent. Only one
 * binding per intent. No replacement request may silently retain old rank. */
int route45_plan_bind_request(Route45Plan *plan, uint32_t intent_id,
                             uint16_t request, uint8_t task, uint8_t digit);
/* Read proto_target_rank_get + proto_wire_diag_get atomically in the eventual
 * executor. Parser CRC/sequence/session validation must already have passed.
 * No raw UART parser or protocol acceptance is added by this pure planner. */
int route45_plan_accept_ball_rank(Route45Plan *plan,
                                 const ProtoTargetRank *rank,
                                 const ProtoWireDiag *link);
int route45_plan_complete(Route45Plan *plan, const Route45Completion *event);
int route45_plan_track_odometry(Route45Plan *plan, const Route45OdoSample *sample);
int route45_plan_rebase_odometry(Route45Plan *plan,
                                const Route45OdoSample *before_reset,
                                const Route45OdoSample *after_reset);

/* Return is wheel-odometry-based, not proof of physical two-dimensional
 * return, safe turning clearance, skid-free motion or visual reacquisition.
 * Preserve the lateral residual instead of silently using only fore counts;
 * executor treatment of that vector still requires road/clearance validation.
 * g/a/0 must cancel its real motor/STEP/laser owners as well as this plan.
 */
#endif
