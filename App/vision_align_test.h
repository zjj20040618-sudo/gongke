#ifndef APP_VISION_ALIGN_TEST_H
#define APP_VISION_ALIGN_TEST_H

#include <stdint.h>
#include "proto.h"
#include "turn_profile.h"

/* Bench candidates reused by31; never write mission/32 workpoints. Image
 * coordinates and signs still require physical validation after installation. */
#define VAT_X_PX               190
#define VAT_ROUTE_BALL_X_PX     135 /*125..145 inclusive. */
#define VAT_ROUTE_BUCKET_X_PX   105 /*31:95..115 inclusive; strict fine gate abs(error)<15. */
#define VAT_ROUTE43_BUCKET_X_PX 125 /* Deferred43 keeps its previous bucket workpoint. */
#define VAT_ROUTE_HOSTAGE_X_PX  215 /*205..225 inclusive. */
#define VAT_BALL_Y_PX          390
#define VAT_BUCKET_Y_PX        420
#define VAT_HOSTAGE_Y_PX       220
#define VAT_TOL_PX             10
/* Temporary X-only trial for the shared31/38..41 engine. Set1 to restore Y
 * microsteps and joint XY arrival; cy is still received, recorded and checked
 * against the actual image bounds. No effect on35/32/formal alignment. */
#ifndef VAT_Y_ALIGN_ENABLE
#define VAT_Y_ALIGN_ENABLE      0
#endif
/* Fine X and standalone search retain v20; lateral Y uses v30.
 * Y is dormant while VAT_Y_ALIGN_ENABLE=0. Restored Y stops, fixes the original
 * heading and revalidates NEW joint X/Y frames.
 * This local correction floor does not change route/90/180 turn tuning. */
#define VAT_SPEED_MMS          20.0f
#define VAT_X_SPEED_MMS        VAT_SPEED_MMS
#define VAT_Y_SPEED_MMS        30.0f
#define VAT_ROUTE_SEARCH_SPEED_MMS 100.0f /* Route API fallback;31/43 owner supplies its RAM cruise. */
#define VAT_ROUTE_SEARCH_ACC_MMS2 700.0f /*31/43 coarse approach only; no instant cruise step. */
#define VAT_ROUTE_FINE_ERROR_PX  15 /*31 ball/bucket/hostage: abs(cx-goal)<15 latches fine v20. */
#define VAT_ROUTE43_FINE_ERROR_PX 30 /* Deferred43 keeps its previous gate. */
#define VAT_ROUTE_BUCKET_FINE_ERROR_PX VAT_ROUTE_FINE_ERROR_PX /* Existing bucket API name. */
#define VAT_ROUTE_BUCKET_MISSING_MS 2000u /*31 paired bucket only, AFTER turn+newACK. */
#define VAT_ROUTE_HOSTAGE_LOST_MS 2000u /* Deferred43: seen selected01, then no NEW valid selected01. */
#define VAT_ROUTE_BUCKET_LOST_MS 2000u /* Deferred43: AFTER fine entry, no NEW legal bucket coordinates. */
#define VAT_ROUTE31_LOST_MS     300u /*31 only: strictly greater than300 since last NEW selected01. */
#define VAT_ROUTE_BUCKET_SEARCH_SPEED_MMS (-VAT_X_SPEED_MMS) /* Legacy fine-step constant; coarse bucket uses owner RAM cruise. */
#define VAT_YAW_MIN_W          TURN_HOLD_MIN_W_RADS
/*31 BALL/HOSTAGE: no stopped in-place yaw correction or heading-arrival gate.
 * Still brake, wait for stationary wheels and use NEW matching image frames;
 * moving heading hold and bucket/43/standalone yaw rules stay unchanged.
 * Set1 only for the historical correction candidate's host tests. */
#ifndef VAT_ROUTE31_BALL_HOSTAGE_STOP_YAW_ENABLE
#define VAT_ROUTE31_BALL_HOSTAGE_STOP_YAW_ENABLE 0
#endif
/* Loaded mode31 BALL stopped-heading trial only. Preserve the original task
 * heading and new-image recheck; these are angular commands, not PWM/torque.
 * Bucket, hostage,43 and independent38..41 retain their previous yaw profile. */
#define VAT_ROUTE31_BALL_YAW_MIN_W 0.30f
#define VAT_ROUTE31_BALL_YAW_MAX_W 0.60f
/* One encoder-projected small step, then a stopped/new-image decision.
 * Wheel odometry is not a guarantee of physical millimetres (slip/calibration).
 * The time cap also bounds a step when an encoder reports no movement. */
#define VAT_STEP_MM             3.0f
#define VAT_STEP_MAX_MS          250u
#define VAT_FRESH_MS           300u
#define VAT_GOOD_FRAMES        5u
#define VAT_POLL_MS            20u
#define VAT_PLACEHOLDER_MS     5000u

typedef enum {
    VAT_OFF = 0, VAT_BRAKE, VAT_YAW_FIX, VAT_RECHECK, VAT_ALIGN, VAT_STEP_MOVE,
    VAT_HOLD_BALL, VAT_TURN_REQUEST, VAT_TURN_ACTIVE, VAT_HOLD_BUCKET,
    VAT_DONE, VAT_STOPPED,
    VAT_WAIT_BALL_ACTION, VAT_WAIT_BUCKET_ACTION, VAT_ROUTE_SEARCH,
    VAT_ROUTE_BUCKET_SEARCH,
    VAT_WAIT_HOSTAGE_ACTION, VAT_WAIT_HOSTAGE_RANK,
    VAT_WAIT_BALL_RANK, /* Route31/43 only: mechanical ball action done, await legitimate position54. */
    VAT_FINE_CONTINUOUS /*31 X-only: continuous v20; reverse/arrival brake before NEW-frame recheck. */
} VisionAlignTestState;

typedef enum {
    VAT_ROUTE_ACTION_NONE = 0, VAT_ROUTE_ACTION_BALL, VAT_ROUTE_ACTION_BUCKET,
    VAT_ROUTE_ACTION_HOSTAGE
} VisionAlignRouteAction;

typedef struct {
    VisionAlignTestState state;
    uint8_t mode, task, digit, good, axis, latest;
    int cls, label, cx, cy;
    int x_goal; /*31:ball135/bucket105/hostage215;43 bucket125; standalone190. */
    uint8_t alignment_confirmed; /*31 hostage only: latched fresh/still enabled-axis alignment, even while rank0 waits. */
    uint8_t target_rank; /* Current task's validated54 rank, never QR color/shape. */
    uint8_t ball_rank; /* Route-only: preserve ball position across new bucket requests/180. */
    uint8_t ball_seen, ball_fallback; /*31: first NEW selected01 / explicit seen-lost300 action source. */
    uint32_t ball_age_ms; /* Last NEW selected01; never refreshed by54/empty/replay/old request. */
    uint8_t hostage_seen; /* Sticky current-request NEW selected01 evidence; not alignment. */
    uint8_t hostage_fallback; /* HOSTAGE source:31 lost>300ms;43 lost2s. Never pixel alignment. */
    uint32_t hostage_age_ms; /* Since last NEW valid selected01; empty/54/replay cannot refresh. */
    uint8_t bucket_seen; /* Sticky NEW post-turn selected01 evidence, distinct from the owner fine gate. */
    uint8_t bucket_fallback; /*31 lost>300ms /43 fine-seen-lost2s release; never pixel alignment. */
    uint32_t bucket_age_ms; /* Since last NEW valid bucket01, independent of empty/replay frames. */
    uint16_t rank_sequence;
    uint16_t sequence, img_w, img_h, request;
    /* Receive evidence only: may be shown during braking, but never
     * used as arrival proof. cx/cy/latest above remain the control snapshot. */
    int rx_cx, rx_cy;
    uint16_t rx_sequence, rx_img_w, rx_img_h;
    uint8_t rx_fresh;
    uint32_t age_ms;
    float vx, vy, w;
    float yaw_target, yaw_error; /* Continuous original heading / wrapped error. */
    uint8_t yaw_dirty, yaw_ever; /* Pending correction / Y or route-search heading must be restored. */
    uint32_t step, step_ms;
    float step_mm; /* Signed encoder progress along the latched step direction. */
    uint8_t step_capped;
    const char *reason; /* Static ASCII string, never an ISR-formatted buffer. */
} VisionAlignTestStatus;

void vision_align_test_init(void);
/* Wrapper opens a NEW QR request for38/40/41 and starts only after g plus its
 * legal QR. This entry verifies qr exactly matches proto_qr_get for those modes.
 * Only39 starts from g directly as bucket task4/digit0; qr may be NULL/ignored.
 * 38=ball,39=bucket(noQR),40=hostage,41=ball/hold5/+180/bucket/hold5.
 * Valid IMU required. X translation uses w=0, X20 in bounded3mm steps;
 * optionalY30 microsteps are compiled only when VAT_Y_ALIGN_ENABLE=1.
 * do not reverse/retarget on each incoming image while a step is moving.
 * Every step brakes/settles and needs new imagery. After enabled lateral motion,
 * brake, restore this task's original heading, discard old images and recheck
 * enabled-axis arrival. No initial correction or heading/encoder zeroing, arm or laser. */
int vision_align_test_start(uint8_t mode, const int32_t qr[3]);
/* Modes31/43 own a previously validated, locked QR tuple after closing
 * its R1 receive window. Use only41(ball/action/externally-run180/bucket/action)
 * or40(hostage): validate all three selections, but do not reopen/read QR.
 * Same IMU/abort/running gates and motion engine as the standalone entry.
 * Each task request chooses its route-only X goal, including both bucket
 * requests around the paired41 turn. Standalone38..41 retain X190.
 * Ball/hostage alone use continuous X search after matching ACK, ramped
 * from0 at700mm/s2 to the owner's cruise (default100), while preserving
 * the initial continuous heading. The owner must set its route
 * search gain before the first poll (default snapshots the global gain).
 * Fresh selected cx>goal searches forward, cx<goal searches backward; no
 * selected frame defaults forward. Each reversal restarts the acceleration ramp.
 * The first fresh abs(cx-goal)<15 (31;43 retains30) brakes and permanently latches fine v20.
 * The +/-10px arrival band is unchanged. Stop/yaw recheck needs NEW imagery before
 * fine X20/optionalY30 steps.31 with Y disabled instead holds continuous X20
 * and the original task heading, braking/settling before reversing or proving
 * arrival. A stale-but-current image may maintain this command for300ms,
 * never count as another arrival frame. Empty/invalid images cannot restart it.
 * Paired bucket alone, AFTER completed180 and its
 * new matchingACK, waits2s without a selected bucket; ball rank1 searches
 * backward, rank3 forward even with far coordinates; rank2 waits for real
 * coordinates and chooses by cx-goal. A300ms-old image stops movement;
 * a2s absence allows rank1/3 search again, restarting the acceleration ramp.
 * Cruise holds the post-turn heading without lateral feedforward. Only
 * abs(cx-goal)<15 (31;43 retains30) brakes/rechecks NEW images and permanently latches fine v20.
 * No coarse-search restart after the fine gate, even if cx rises again or frames are lost.
 * For31 (including a Y-enabled test build), any ball/bucket/hostage task first
 * seen as a NEW legal current-request selected01 may expose its real action
 * when that NEW-coordinate age is strictly>300ms and four wheels are stopped
 *250ms. This does not require fine entry and never claims pixel alignment.
 * New coordinates before owner take cancel pending loss; never-seen/54/empty/
 * replay/unACKed/old-request frames cannot start or refresh the timer.
 * Deferred43 retains the following legacy loss policy. Routehostage exposes HOSTAGE action after stopped fresh-frame alignment, or
 * after seeing its selected01 at least once then losing NEW valid coordinates
 * for2s and confirming stopped wheels250ms. The latter is an explicit blind
 * grab trial, not alignment proof. A new selected01 before the owner takes
 * the action (including its brake/pending phase) cancels loss and restarts
 * the2s timer. No first target means no blind grab.
 * Routebucket exposes BUCKET only after the fine gate and NEW selected01 has been absent2s
 * and four wheels are stopped250ms. This is an explicit blind release trial,
 * not alignment. New01 before the owner takes it cancels the pending release;
 * never-seen bucket cannot release, and no dropout can reopen coarse search.
 * RX remains open during the owner arm action and afterward for legitimate54;
 * owner success plus locked rank1..3 completes, otherwise WAIT_HOSTAGE_RANK
 * stays braked. Route BALL likewise keeps RX open for its validated54; if
 * mechanical action completes without rank it waits braked at WAIT_BALL_RANK.
 * Its rank survives the bucket requests. Standalone38..41 need no ball54.
 * Caller must route new frames, service poll/turn notification and cancellation;
 * this is not a motion/QR entry for32 or a replacement for standalone freshness. */
int vision_align_test_start_route(uint8_t mode, const int32_t qr[3]);
/* Owner31 selects the15px gate; deferred43 retains30. Captured before RX opens. */
int vision_align_test_start_route_scoped(uint8_t mode, const int32_t qr[3], uint8_t owner_mode);
/* Success only for a stopped, initial route ball/hostage BRAKE before search;
 * accept finite0..5, including0. Does not change fine XY/yaw-fix/standalone gain
 * or existing start signatures. Call immediately after start_route succeeds. */
int vision_align_test_route_search_kp_set(float kp);
/* Same initial route ball/hostage BRAKE-only window; finite1..600 mm/s. The
 * route owner supplies its RAM straight speed. Paired bucket inherits it
 * across both bucket requests/turn; standalone and fine-step speeds unchanged. */
int vision_align_test_route_search_speed_set(float speed_mms);
/* Initial route ball/hostage BRAKE-only; caller31 selects400,43/standalone
 * keep the default700. Reset on each start; preserve across bucket requests.
 * Does not weaken yaw tolerance, stopped-wheel or fresh-image requirements. */
int vision_align_test_route_yaw_settle_ms_set(uint32_t ms);
/* Route initial ball/hostage BRAKE only, before continuous search starts.
 * Finite0..0.1; default0 on each start. Positive adds body-right vy=ratio*vx
 * ONLY to forward coarse approach; backward search has no uncalibrated side
 * compensation. Fine XY/yaw correction and standalones stay
 * unchanged; this is trial feedforward, not measured lateral closed-loop. */
int vision_align_test_route_search_ff_set(float ratio);
/* Route-started41 ball/bucket, or route-started40 hostage only. Aligned
 * BALL brakes/keeps RX for54; aligned BUCKET brakes/closes RX. Seen-lost BUCKET keeps RX until the owner
 * takes its pending release, then ignores coordinate recovery during action.
 * HOSTAGE keeps RX open for54 through the owner action. Take returns a pending
 * action exactly once; all31 seen-lost actions, HOSTAGE and seen-lost BUCKET recheck the matching ACK,
 * pending recovery and four-wheel stopped250ms. WAIT remains until owner success.
 * Failure stops with LIFT_ERROR; ball success plus rank requests bucket then +180,
 * bucket success completes; hostage success completes only with real rank1..3,
 * otherwise waits braked with RXopen. Standalone38..41 never expose these actions;
 * standalone41 retains its two5s placeholders. No stepper/servo/timer calls. */
int vision_align_test_take_route_action(void);
/* Optional owner31/mode40 path after taking HOSTAGE, once an external offset and
 * endpoint yaw correction finish. Cancel the taken action and discard all
 * pre-stop imagery, then brake/recheck NEW coordinates in the SAME task/request.
 * Preserve legal54, original heading and actual last NEW01 loss timestamp;
 * the authorized seen-then-lost2s fallback is not synthetic alignment.
 * Owner must not poll VAT during the external move/correction. Returns0 outside
 * this window; run/IMU/ACK faults in the valid window stop the task.
 * Current31 disables the external offset and does not call this API. */
int vision_align_test_route_hostage_offset_resume(void);
void vision_align_test_notify_route_action_result(int success);
void vision_align_test_poll(void); /* Owner task, nonblocking, internally20ms. */
void vision_align_test_feed_frame(const ProtoFrame *frame); /* Parser callback. */
void vision_align_test_cancel(void); /* Immediate brake/localRX close/laser off. */
int vision_align_test_active(void);
void vision_align_test_status(VisionAlignTestStatus *out);
/* Only41: take once, then wrapper starts the EXISTING mode22 +180 controller.
 * Poll is inert while VAT_TURN_ACTIVE so it cannot brake/overwrite mode22.
 * Notify only after controller DONE/STOP and any route-owned post-turn offset;
 * VAT remains inert throughout that offset. DONE records the new bucket heading
 * once, not on every XY recheck, and requires new bucket ACK/images. */
int vision_align_test_take_turn_request(void);
void vision_align_test_notify_turn_result(int success);

#endif
