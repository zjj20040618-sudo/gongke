/* Preparation-only45 planner regression. All QR, ACK/rank snapshots, completed
 * jobs and odometry are SYNTHETIC fixtures. No real proto/UART, VAT, actuator,
 * chassis, firmware45 registration or physical safe-corner acceptance. */
#include <math.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>
#include "route45_plan.h"

#define CHECK(expr) do { if(!(expr)) { \
    fprintf(stderr,"route45 plan line %d: %s\n",__LINE__,#expr); return 1; \
} } while(0)

static int near_mm(float actual,float expected)
{ return isfinite(actual) && fabsf(actual-expected)<0.003f; }

/* Deliberately artificial and unequal: NOT measured/calibrated route values. */
static Route45Config synthetic_config(void)
{
    Route45Config config={{11u,23u,47u},R45_BUCKET_SEARCH_BACKWARD};
    return config;
}

static Route45OdoSample sample(float fore,float right,float heading,uint32_t epoch)
{
    Route45OdoSample out={fore,right,heading,epoch,1u}; return out;
}

static Route45Completion completion(const Route45Plan *plan)
{
    Route45Completion event;
    memset(&event,0,sizeof event);
    event.intent_id=plan->intent.id; event.succeeded=event.stopped=1u;
    return event;
}

static void rank_snapshot(const Route45Plan *plan,unsigned position,
                          ProtoTargetRank *rank,ProtoWireDiag *link)
{
    static const uint8_t selected_models[3]={4u,3u,5u};
    memset(rank,0,sizeof *rank); memset(link,0,sizeof *link);
    rank->request=plan->ball_request; rank->task=PROTO_TASK_BALL;
    rank->digit=(uint8_t)plan->qr[0]; rank->sequence=9u;
    rank->target_model=selected_models[rank->digit-1u];
    rank->rank=(uint8_t)position; rank->seen_count=3u;
    rank->slots[0]=3u; rank->slots[1]=4u; rank->slots[2]=5u;
    for(unsigned k=0u;k<3u;++k) if(rank->slots[k]==rank->target_model) {
        rank->slots[k]=rank->slots[position-1u];
        rank->slots[position-1u]=rank->target_model; break;
    }
    link->request=rank->request; link->task=PROTO_TASK_BALL;
    link->selection=rank->digit; link->mode=2u;
    link->receiving=link->controlled=link->ack=1u;
}

static int start_ball(Route45Plan *plan,unsigned qr_digit,uint16_t request,int initialize)
{
    Route45Config config=synthetic_config();
    Route45Completion event;
    if(initialize) route45_plan_init(plan);
    CHECK(route45_plan_begin(plan,&config));
    CHECK(plan->state==R45_PREFIX && plan->intent.kind==R45_INTENT_PREFIX_WITH_QR_GATE);
    event=completion(plan); event.qr[0]=(int32_t)qr_digit;
    event.qr[1]=2; event.qr[2]=3;
    CHECK(route45_plan_complete(plan,&event));
    CHECK(plan->state==R45_ALIGN_BALL && plan->intent.task==PROTO_TASK_BALL &&
          plan->intent.digit==qr_digit && !plan->ball_rank && !plan->tracking);
    CHECK(route45_plan_bind_request(plan,plan->intent.id,request,PROTO_TASK_BALL,(uint8_t)qr_digit));
    return 0;
}

static int grab_with_rank(Route45Plan *plan,unsigned qr_digit,unsigned position)
{
    ProtoTargetRank rank; ProtoWireDiag link;
    CHECK(start_ball(plan,qr_digit,71u,1)==0);
    rank_snapshot(plan,position,&rank,&link);
    CHECK(route45_plan_accept_ball_rank(plan,&rank,&link));
    CHECK(plan->state==R45_ALIGN_BALL); /*54 is never alignment evidence. */
    Route45Completion event=completion(plan);
    event.request=plan->ball_request; event.evidence=R45_EVIDENCE_FRESH_STOPPED_ALIGNMENT;
    CHECK(route45_plan_complete(plan,&event) && plan->state==R45_GRAB_BALL &&
          plan->intent.kind==R45_INTENT_COMPLETE_GRAB_RECIPE);
    event=completion(plan);
    CHECK(route45_plan_complete(plan,&event) && plan->state==R45_BACK_TO_CORNER &&
          plan->intent.kind==R45_INTENT_BACK_TO_CORNER &&
          plan->intent.ball_rank==position && plan->intent.forward_mm==
              -(float)plan->config.ball_to_corner_mm[position-1u] && !plan->intent.right_mm);
    return 0;
}

static int anchor_before_turn(Route45Plan *plan,Route45OdoSample anchor)
{
    CHECK(grab_with_rank(plan,1u,3u)==0);
    Route45Completion event=completion(plan); event.odo=anchor;
    CHECK(route45_plan_complete(plan,&event) && plan->state==R45_TURN_TO_BUCKET &&
          plan->intent.kind==R45_INTENT_FIRST180 && plan->intent.turn_deg==180.0f &&
          plan->tracking && near_mm(plan->corner_fore.progress_mm,0.0f) &&
          near_mm(plan->corner_right.progress_mm,0.0f));
    return 0;
}

static int bucket_after_turn(Route45Plan *plan,Route45OdoSample stopped_turn)
{
    Route45Completion event=completion(plan); event.odo=stopped_turn;
    CHECK(route45_plan_complete(plan,&event));
    CHECK(plan->state==R45_ALIGN_BUCKET &&
          plan->intent.kind==R45_INTENT_NEW_BUCKET_REQUEST_AND_ALIGN &&
          plan->intent.task==PROTO_TASK_BUCKET && !plan->intent.digit &&
          plan->intent.bucket_policy==R45_BUCKET_SEARCH_BACKWARD &&
          !plan->bucket_request && plan->ball_rank==3u);
    CHECK(!route45_plan_bind_request(plan,plan->intent.id,plan->ball_request,
                                    PROTO_TASK_BUCKET,0u));
    CHECK(!plan->bucket_request && plan->intent.kind==R45_INTENT_NEW_BUCKET_REQUEST_AND_ALIGN);
    CHECK(route45_plan_bind_request(plan,plan->intent.id,72u,PROTO_TASK_BUCKET,0u));
    return 0;
}

static int check_unmeasured_configuration_and_freeze(void)
{
    Route45Plan plan; Route45Config config;
    route45_plan_init(&plan); route45_config_defaults(&config);
    CHECK(plan.state==R45_OFF && !route45_plan_active(&plan));
    CHECK(config.bucket_policy==R45_BUCKET_SEARCH_BACKWARD);
    for(unsigned k=0u;k<3u;++k) {
        CHECK(!config.ball_to_corner_mm[k]);
        CHECK(!route45_plan_begin(&plan,&config) && plan.state==R45_WAIT_CONFIG &&
              plan.intent.kind==R45_INTENT_NONE && !plan.intent.id && !plan.tracking);
        config.ball_to_corner_mm[k]=11u+k;
    }
    config.bucket_policy=R45_BUCKET_POLICY_UNSET;
    CHECK(!route45_plan_begin(&plan,&config) && !plan.intent.id);
    config.bucket_policy=(Route45BucketPolicy)99;
    CHECK(!route45_plan_begin(&plan,&config) && !plan.intent.id);
    config.bucket_policy=R45_BUCKET_SEARCH_BACKWARD;
    config.ball_to_corner_mm[1]=ROUTE45_INPUT_MAX_MM+1u;
    CHECK(!route45_plan_begin(&plan,&config) && !plan.intent.id);
    config=synthetic_config(); CHECK(route45_plan_begin(&plan,&config));
    uint32_t id=plan.intent.id;
    config.ball_to_corner_mm[0]=777u; config.bucket_policy=R45_BUCKET_SEARCH_FORWARD;
    CHECK(plan.config.ball_to_corner_mm[0]==11u &&
          plan.config.bucket_policy==R45_BUCKET_SEARCH_BACKWARD);
    CHECK(!route45_plan_begin(&plan,&config) && plan.intent.id==id &&
          plan.config.ball_to_corner_mm[0]==11u);
    route45_plan_stop(&plan,"CONFIG_FIXTURE_STOP");
    CHECK(route45_plan_begin(&plan,&plan.config) && plan.intent.id>id &&
          plan.config.ball_to_corner_mm[0]==11u); /* alias-safe new-run snapshot */
    CHECK(route45_config_missing(NULL)!=NULL && !route45_plan_begin(NULL,&config));
    puts("45 zero/unset/out-of-range configuration emits no intent; synthetic frozen config and running-begin/alias-safe restart contracts passed");
    return 0;
}

static int check_rank_is_position_not_qr_and_late_rank(void)
{
    Route45Plan plan;
    for(unsigned digit=1u;digit<=3u;++digit) for(unsigned position=1u;position<=3u;++position)
        CHECK(grab_with_rank(&plan,digit,position)==0);
    CHECK(start_ball(&plan,1u,71u,1)==0);
    Route45Completion event=completion(&plan);
    event.request=71u; event.evidence=R45_EVIDENCE_NONE;
    CHECK(!route45_plan_complete(&plan,&event) && plan.state==R45_ALIGN_BALL);
    event.evidence=R45_EVIDENCE_EXPLICIT_SEEN_THEN_LOST;
    CHECK(route45_plan_complete(&plan,&event) && plan.state==R45_GRAB_BALL);
    event=completion(&plan); uint32_t completed_grab=event.intent_id;
    CHECK(route45_plan_complete(&plan,&event) && plan.state==R45_WAIT_BALL_RANK &&
          !plan.intent.id && plan.intent.kind==R45_INTENT_NONE && !plan.tracking);
    CHECK(!route45_plan_complete(&plan,&event) && plan.state==R45_WAIT_BALL_RANK);
    ProtoTargetRank rank; ProtoWireDiag link; rank_snapshot(&plan,2u,&rank,&link);
    rank.rank=0u;
    CHECK(!route45_plan_accept_ball_rank(&plan,&rank,&link) && plan.state==R45_WAIT_BALL_RANK);
    rank.rank=2u;
    CHECK(route45_plan_accept_ball_rank(&plan,&rank,&link) && plan.state==R45_BACK_TO_CORNER &&
          plan.intent.id>completed_grab && plan.intent.forward_mm==-23.0f &&
          plan.intent.ball_rank==2u);
    puts("45 nine QRcolor/rank combinations use rank distances only;54 cannot align/grab, and complete grab waits for late nonzero rank without guessing passed");
    return 0;
}

static int check_rank_binding_and_alignment_request(void)
{
    Route45Plan plan; ProtoTargetRank rank,bad_rank; ProtoWireDiag link,bad_link;
    CHECK(start_ball(&plan,1u,71u,1)==0);
    uint32_t id=plan.intent.id;
    CHECK(route45_plan_bind_request(&plan,id,71u,PROTO_TASK_BALL,1u));
    CHECK(!route45_plan_bind_request(&plan,id,73u,PROTO_TASK_BALL,1u));
    CHECK(!route45_plan_bind_request(&plan,id-1u,71u,PROTO_TASK_BALL,1u));
    CHECK(!route45_plan_bind_request(&plan,id,0u,PROTO_TASK_BALL,1u));
    CHECK(!route45_plan_bind_request(&plan,id,71u,PROTO_TASK_BUCKET,0u));
    rank_snapshot(&plan,3u,&rank,&link);
    for(unsigned k=0u;k<5u;++k) {
        bad_rank=rank;
        if(k==0u) bad_rank.request=70u;
        else if(k==1u) bad_rank.task=PROTO_TASK_TARGET;
        else if(k==2u) bad_rank.digit=2u;
        else if(k==3u) bad_rank.rank=0u;
        else bad_rank.rank=4u;
        CHECK(!route45_plan_accept_ball_rank(&plan,&bad_rank,&link) && !plan.ball_rank);
    }
    for(unsigned k=0u;k<7u;++k) {
        bad_link=link;
        if(k==0u) bad_link.request=70u;
        else if(k==1u) bad_link.task=PROTO_TASK_TARGET;
        else if(k==2u) bad_link.selection=2u;
        else if(k==3u) bad_link.receiving=0u;
        else if(k==4u) bad_link.controlled=0u;
        else if(k==5u) bad_link.ack=0u;
        else bad_link.failed=1u;
        CHECK(!route45_plan_accept_ball_rank(&plan,&rank,&bad_link) && !plan.ball_rank);
    }
    CHECK(route45_plan_accept_ball_rank(&plan,&rank,&link));
    bad_rank=rank; bad_rank.rank=2u;
    CHECK(!route45_plan_accept_ball_rank(&plan,&bad_rank,&link) && plan.ball_rank==3u);
    Route45Completion event=completion(&plan);
    event.request=70u; event.evidence=R45_EVIDENCE_FRESH_STOPPED_ALIGNMENT;
    CHECK(!route45_plan_complete(&plan,&event) && plan.intent.id==id);
    event.request=71u;
    CHECK(route45_plan_complete(&plan,&event) && plan.state==R45_GRAB_BALL);
    CHECK(!route45_plan_accept_ball_rank(NULL,&rank,&link));
    CHECK(!route45_plan_accept_ball_rank(&plan,NULL,&link));
    CHECK(!route45_plan_accept_ball_rank(&plan,&rank,NULL));
    puts("45 synthetic current request/task/digit/controlled ACK/receiving/failed gates, immutable rank and bound alignment evidence passed (not UART/CRC tests)");
    return 0;
}

static int check_turn_before_bucket_and_two_dimensional_return(void)
{
    Route45Plan plan;
    CHECK(anchor_before_turn(&plan,sample(100.0f,-80.0f,0.0f,3u))==0);
    uint32_t turn_id=plan.intent.id;
    CHECK(!route45_plan_bind_request(&plan,turn_id,72u,PROTO_TASK_BUCKET,0u));
    Route45Completion stale=completion(&plan); stale.intent_id=turn_id-1u;
    stale.odo=sample(110.0f,-77.0f,180.0f,3u);
    CHECK(!route45_plan_complete(&plan,&stale) && plan.intent.kind==R45_INTENT_FIRST180);
    Route45OdoSample turn_mid=sample(110.0f,-80.0f,90.0f,3u);
    CHECK(route45_plan_track_odometry(&plan,&turn_mid));
    CHECK(near_mm(plan.corner_fore.progress_mm,0.0f) && near_mm(plan.corner_right.progress_mm,10.0f));
    CHECK(bucket_after_turn(&plan,sample(110.0f,-77.0f,180.0f,3u))==0);
    CHECK(near_mm(plan.corner_right.progress_mm,7.0f)); /* turn drift not dropped */
    Route45OdoSample back_search=sample(10.0f,-77.0f,180.0f,3u);
    Route45OdoSample fine_forward=sample(30.0f,-72.0f,180.0f,3u);
    CHECK(route45_plan_track_odometry(&plan,&back_search));
    CHECK(route45_plan_track_odometry(&plan,&fine_forward));
    Route45Completion event=completion(&plan); event.request=72u;
    event.evidence=R45_EVIDENCE_FRESH_STOPPED_ALIGNMENT; event.odo=fine_forward;
    CHECK(route45_plan_complete(&plan,&event) && plan.state==R45_PLACE_AND_RETRACT &&
          plan.intent.kind==R45_INTENT_RELEASE_AND_RETRACT && plan.tracking);
    CHECK(!plan.return_fore_mm && !plan.return_right_mm); /* release intent is not full completion */
    Route45OdoSample final_arm=sample(32.0f,-71.0f,180.0f,3u);
    CHECK(route45_plan_track_odometry(&plan,&final_arm) && plan.state==R45_PLACE_AND_RETRACT);
    event=completion(&plan); event.odo=final_arm;
    CHECK(route45_plan_complete(&plan,&event) && plan.state==R45_RETURN_TO_CORNER &&
          plan.intent.kind==R45_INTENT_RETURN_TO_CORNER &&
          near_mm(plan.intent.forward_mm,78.0f) && near_mm(plan.intent.right_mm,1.0f));
    /* A BACK bucket search needs a positive/FWD return, not an unconditional
     * BACK or abs(total travel). The lateral term also cancels turn drift. */
    CHECK(near_mm(plan.corner_fore.progress_mm,78.0f) && near_mm(plan.corner_right.progress_mm,1.0f));
    uint32_t return_id=plan.intent.id;
    Route45OdoSample partial_return=sample(72.0f,-70.5f,180.0f,3u);
    CHECK(route45_plan_track_odometry(&plan,&partial_return) && plan.intent.id==return_id &&
          near_mm(plan.intent.forward_mm,78.0f) && near_mm(plan.intent.right_mm,1.0f));
    event=completion(&plan); event.odo=sample(110.0f,-70.0f,180.0f,3u);
    CHECK(route45_plan_complete(&plan,&event) && plan.state==R45_TURN_TO_TARGET &&
          plan.intent.kind==R45_INTENT_SECOND180 && plan.intent.turn_deg==180.0f && !plan.tracking &&
          near_mm(plan.corner_fore.progress_mm,0.0f) && near_mm(plan.corner_right.progress_mm,0.0f));
    CHECK(!route45_plan_complete(&plan,&event)); /* late return result cannot start tail */
    event=completion(&plan);
    CHECK(route45_plan_complete(&plan,&event) && plan.state==R45_TARGET_AND_RESCUE &&
          plan.intent.kind==R45_INTENT_TARGET_AND_RESCUE_TAIL);
    event=completion(&plan);
    CHECK(route45_plan_complete(&plan,&event) && plan.state==R45_DONE &&
          !route45_plan_active(&plan) && !plan.intent.id);
    puts("45 first180-before-NEWbucket, preturn anchor drift, BACK-search signedFWD return, net2D correction, full release/retract freeze, second180 then tail passed");
    return 0;
}

static int check_signed_forward_search_and_continuous_heading(void)
{
    Route45Plan plan;
    CHECK(anchor_before_turn(&plan,sample(400.0f,50.0f,1230.0f,6u))==0);
    CHECK(bucket_after_turn(&plan,sample(400.0f,50.0f,1410.0f,6u))==0);
    Route45OdoSample moved=sample(440.0f,62.0f,1440.0f,6u);
    Route45Completion event=completion(&plan); event.request=72u;
    event.evidence=R45_EVIDENCE_EXPLICIT_SEEN_THEN_LOST; event.odo=moved;
    CHECK(route45_plan_complete(&plan,&event));
    CHECK(near_mm(plan.corner_fore.progress_mm,-28.641016f) &&
          near_mm(plan.corner_right.progress_mm,-30.392305f));
    event=completion(&plan); event.odo=moved;
    CHECK(route45_plan_complete(&plan,&event) && near_mm(plan.intent.forward_mm,-40.0f) &&
          near_mm(plan.intent.right_mm,-12.0f));
    puts("45 continuous/noncardinal heading projects signedfore+right; netforward approach produces negativeBACK return rather than abs distance passed");
    return 0;
}

static int check_explicit_rebase_and_invalid_odometry(void)
{
    Route45Plan plan;
    CHECK(anchor_before_turn(&plan,sample(100.0f,-80.0f,0.0f,1u))==0);
    Route45OdoSample before=sample(130.0f,-60.0f,0.0f,1u);
    Route45OdoSample after=sample(0.0f,0.0f,0.0f,2u);
    CHECK(route45_plan_rebase_odometry(&plan,&before,&after));
    CHECK(plan.odo_epoch==2u && near_mm(plan.corner_fore.progress_mm,30.0f) &&
          near_mm(plan.corner_right.progress_mm,20.0f));
    Route45OdoSample next=sample(20.0f,-5.0f,0.0f,2u);
    CHECK(route45_plan_track_odometry(&plan,&next) && near_mm(plan.corner_fore.progress_mm,50.0f) &&
          near_mm(plan.corner_right.progress_mm,15.0f));
    for(unsigned bad=0u;bad<6u;++bad) {
        CHECK(anchor_before_turn(&plan,sample(100.0f,-80.0f,0.0f,1u))==0);
        before=sample(130.0f,-60.0f,0.0f,1u); after=sample(0.0f,0.0f,0.0f,2u);
        if(bad==0u) before.encoder_epoch=9u;
        else if(bad==1u) after.encoder_epoch=1u;
        else if(bad==2u) before.valid=0u;
        else if(bad==3u) after.heading_deg=NAN;
        CHECK(!route45_plan_rebase_odometry(&plan,bad==4u?NULL:&before,bad==5u?NULL:&after) &&
              plan.state==R45_STOPPED && !plan.tracking && !plan.intent.id);
    }
    for(unsigned bad=0u;bad<5u;++bad) {
        CHECK(anchor_before_turn(&plan,sample(100.0f,-80.0f,0.0f,1u))==0);
        next=sample(101.0f,-80.0f,0.0f,1u);
        if(bad==0u) next.encoder_epoch=2u;
        else if(bad==1u) next.valid=0u;
        else if(bad==2u) next.fore_mm=NAN;
        else if(bad==3u) next.right_mm=INFINITY;
        else next.heading_deg=-INFINITY;
        CHECK(!route45_plan_track_odometry(&plan,&next) && plan.state==R45_STOPPED &&
              !plan.tracking && !plan.intent.id);
    }
    puts("45 final-pre-reset sample+explicit epoch rebase preserves net2D; silent epoch change/invalid samples or rebase stop rather than guess passed");
    return 0;
}

static int check_stop_stale_completion_new_run_and_failures(void)
{
    Route45Plan plan; ProtoTargetRank old_rank,new_rank; ProtoWireDiag old_link,new_link;
    CHECK(anchor_before_turn(&plan,sample(100.0f,-80.0f,0.0f,1u))==0);
    Route45Completion old=completion(&plan); old.odo=sample(100.0f,-80.0f,180.0f,1u);
    uint32_t old_id=old.intent_id;
    rank_snapshot(&plan,3u,&old_rank,&old_link);
    route45_plan_stop(&plan,"SYNTHETIC_MANUAL_STOP");
    CHECK(plan.state==R45_STOPPED && !plan.tracking && !plan.intent.id &&
          !route45_plan_complete(&plan,&old) &&
          !route45_plan_accept_ball_rank(&plan,&old_rank,&old_link));
    CHECK(start_ball(&plan,2u,101u,0)==0 && plan.intent.id>old_id &&
          !plan.ball_rank && !plan.bucket_request && !plan.tracking &&
          !plan.corner_fore.initialized && !plan.corner_right.initialized &&
          !plan.return_fore_mm && !plan.return_right_mm && !plan.ball_order.request);
    CHECK(!route45_plan_complete(&plan,&old));
    CHECK(!route45_plan_accept_ball_rank(&plan,&old_rank,&old_link) && !plan.ball_rank);
    rank_snapshot(&plan,1u,&new_rank,&new_link);
    CHECK(route45_plan_accept_ball_rank(&plan,&new_rank,&new_link) && plan.ball_rank==1u);
    for(unsigned fail=0u;fail<2u;++fail) {
        CHECK(anchor_before_turn(&plan,sample(100.0f,-80.0f,0.0f,1u))==0);
        Route45Completion event=completion(&plan); event.odo=sample(100.0f,-80.0f,180.0f,1u);
        if(fail) event.stopped=0u; else event.succeeded=0u;
        CHECK(!route45_plan_complete(&plan,&event) && plan.state==R45_STOPPED &&
              plan.intent.kind==R45_INTENT_NONE && !plan.bucket_request);
    }
    route45_plan_init(&plan); plan.next_intent_id=UINT32_MAX;
    Route45Config config=synthetic_config();
    CHECK(!route45_plan_begin(&plan,&config) && plan.state==R45_STOPPED && !plan.intent.id);
    CHECK(!strcmp(route45_state_name((Route45State)99),"INVALID_STATE"));
    puts("45 cancellation invalidates old completions/rank; new run clears bindings/order/ledgers/return while IDs remain monotonic; failed/not-stopped firstturn and ID exhaustion cannot request bucket passed");
    return 0;
}

int main(void)
{
    CHECK(ROUTE45_CANDIDATE_MODE==45u);
    CHECK(check_unmeasured_configuration_and_freeze()==0);
    CHECK(check_rank_is_position_not_qr_and_late_rank()==0);
    CHECK(check_rank_binding_and_alignment_request()==0);
    CHECK(check_turn_before_bucket_and_two_dimensional_return()==0);
    CHECK(check_signed_forward_search_and_continuous_heading()==0);
    CHECK(check_explicit_rebase_and_invalid_odometry()==0);
    CHECK(check_stop_stale_completion_new_run_and_failures()==0);
    puts("route45_plan_test: preparation-only synthetic intent/math regression; NOT firmware45, real proto/UART or physical safe-corner acceptance");
    return 0;
}
