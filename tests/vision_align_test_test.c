/* Standalone nonblocking VAT module + real CRC/request/QR/object parser.
 * No motor/IMU plant is simulated; heading and pixels are injected explicitly.
 * gcc -Itests/stubs -IApp this.c App/vision_align_test.c App/proto.c -lm */
#include "vision_align_test.h"
#include "imu.h"
#include "control.h"
#include "motion.h"
#include "board_pins.h"
#include "steps.h"
#include "test_config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

static uint32_t now_ms;
static float yaw, vx, vy, w;
static int imu_valid, aborted, laser;
static int32_t counts[4];
static unsigned request_count, laser_on, imu_zero, arm_calls;
static uint16_t seq;
static uint8_t last_task, last_digit, last_opcode;
static VisionAlignTestStatus st;
static const int32_t qr123[3] = {1,2,3};

#define CHECK(expr) do { if (!(expr)) { vision_align_test_status(&st); \
    fprintf(stderr,"VAT line%d:%s state%d reason%s now%lu v%.1f/%.1f/%.2f good%u\n", \
    __LINE__,#expr,st.state,st.reason,(unsigned long)now_ms,vx,vy,w,st.good); return 0; } } while(0)

uint32_t HAL_GetTick(void) { return now_ms; }
uint8_t imu_ok(void) { return (uint8_t)imu_valid; }
float imu_heading_deg(void) { return yaw; }
uint8_t imu_zero_leg_heading(void) { ++imu_zero; return 1u; }
int32_t ctrl_enc_total(int wheel) { return counts[wheel]; }
int run_aborted(void) { return aborted; }
void motion_brake(void) { vx=vy=w=0.0f; }
void motion_vel_set_precise(float x, float y, float turn) { vx=x;vy=y;w=turn; }
void bp_laser_set(int on) { laser=on; if(on)laser_on++; }

static uint16_t crc16(const uint8_t *p,unsigned n)
{
    uint16_t crc=0xffffu;
    for(unsigned i=0;i<n;i++) {
        crc ^= (uint16_t)p[i]<<8;
        for(unsigned bit=0;bit<8u;bit++) crc=crc&0x8000u?(uint16_t)((crc<<1)^0x1021u):(uint16_t)(crc<<1);
    }
    return crc;
}
static void le16(uint8_t *p,unsigned n) {p[0]=(uint8_t)n;p[1]=(uint8_t)(n>>8);}
static void feed(const uint8_t *p,unsigned n)
{
    uint16_t crc=crc16(p,n);
    proto_feed_byte(0xaau);proto_feed_byte(0x55u);
    for(unsigned i=0;i<n;i++)proto_feed_byte(p[i]);
    proto_feed_byte((uint8_t)crc);proto_feed_byte((uint8_t)(crc>>8));
}
static uint16_t request(void) {ProtoWireDiag d;proto_wire_diag_get(&d);return d.request;}
static void tx(const uint8_t *p,uint16_t n)
{
    request_count++;last_opcode=p[2];
    if(n==9u&&p[2]==0x63u){last_task=p[5];last_digit=p[6];}
}
static void ack(uint16_t req,uint8_t mode,uint8_t result)
{
    uint8_t p[5]={0x61u,0,0,mode,result};le16(p+1,req);feed(p,sizeof p);
}
static void qr_feed(uint16_t req)
{
    uint8_t p[12]={0x62u,0,0,7u,0,0x53u,1u,0,1u,'1','2','3'};
    le16(p+1,req);feed(p,sizeof p);
}
static void object(uint16_t req,int model,int x,int y,unsigned width,unsigned height,uint16_t sequence)
{
    uint8_t p[30]={0x62u,0,0,0,0,1u,0,0,0};
    unsigned n=model<0?14u:25u;
    le16(p+1,req);le16(p+3,n);le16(p+6,sequence);p[8]=(uint8_t)(model>=0);
    le16(p+9,width);le16(p+11,height);
    if(model>=0){p[19]=(uint8_t)model;le16(p+20,900);le16(p+22,(unsigned)x);le16(p+24,(unsigned)y);le16(p+26,10);le16(p+28,10);}
    feed(p,n+5u);
}
static void tick(unsigned ms)
{
    now_ms+=ms;proto_service();vision_align_test_poll();vision_align_test_status(&st);
}
static void tick20(unsigned n) {for(unsigned i=0;i<n;i++)tick(20u);}
static int stopped(void) {return vx==0.0f&&vy==0.0f&&w==0.0f&&!laser&&!laser_on&&!imu_zero&&!arm_calls;}
static int reset(unsigned mode)
{
    now_ms=0;vx=vy=w=0.0f;yaw=37.0f;imu_valid=1;aborted=laser=0;
    memset(counts,0,sizeof counts);request_count=laser_on=imu_zero=arm_calls=0;seq=1;
    last_opcode=last_task=last_digit=0;
    proto_init();proto_set_binary_mode(1);proto_set_binary_tx(tx);proto_set_on_frame(vision_align_test_feed_frame);
    vision_align_test_init();proto_send_scene(SCENE_QR);proto_service();
    CHECK(stopped()&&request_count==1u);
    ack(request(),1u,0u);qr_feed(request());
    CHECK(vision_align_test_start((uint8_t)mode,qr123,1.0f));proto_service();
    CHECK(stopped()&&request_count==2u&&last_opcode==0x63u);
    return 1;
}
static int enter_align(void)
{
    ack(request(),2u,0u);
    tick20(50u);
    CHECK(st.state==VAT_ALIGN||st.state==VAT_RECHECK);
    return 1;
}
static int fresh_target(int model,int x,int y)
{
    object(request(),model,x,y,640u,480u,seq++);tick(20u);return 1;
}
static int stable_new_frame_gate(void)
{
    tick20(50u);
    CHECK(st.state==VAT_RECHECK&&stopped());return 1;
}
static int finish_current_target(int model,int x,int y)
{
    for(unsigned i=0;i<18u&&vision_align_test_active();i++) {
        fresh_target(model,x,y);tick(20u);
        if(st.state==VAT_HOLD_BALL||st.state==VAT_HOLD_BUCKET)break;
    }
    CHECK(st.state==VAT_DONE||st.state==VAT_HOLD_BALL||st.state==VAT_HOLD_BUCKET);
    CHECK(stopped());return 1;
}
static int check_qr_gate_selection_and_initial_search(void)
{
    CHECK(reset(38u));CHECK(last_task==1u&&last_digit==1u);
    tick20(50u);CHECK(stopped()&&st.state==VAT_ALIGN); /* no ACK never moves */
    ack((uint16_t)(request()-1u),2u,0u);tick(20u);CHECK(stopped());
    ack(request(),2u,0u);tick(20u);CHECK(vx==16.0f&&vy==0.0f&&w==0.0f);
    vision_align_test_cancel();CHECK(stopped());
    object(request(),4,230,420,640u,480u,1u);tick20(100u);CHECK(stopped()&&st.state==VAT_STOPPED);
    CHECK(reset(39u));CHECK(last_task==4u&&!last_digit);CHECK(enter_align());CHECK(stopped());
    object(request(),-1,0,0,640u,480u,seq++);tick20(20u);CHECK(stopped());
    CHECK(reset(40u));CHECK(last_task==3u&&last_digit==3u);CHECK(enter_align());CHECK(vx==16.0f);
    vision_align_test_cancel();
    proto_send_scene(SCENE_QR);CHECK(!vision_align_test_start(38u,qr123,1.0f));CHECK(stopped());
    ack(request(),1u,0u);qr_feed(request());
    int32_t wrong[3]={2,2,3};CHECK(!vision_align_test_start(38u,wrong,1.0f));
    CHECK(!vision_align_test_start(38u,qr123,NAN));CHECK(stopped());
    return 1;
}
static int check_xy_axis_recheck_and_yaw(void)
{
    CHECK(reset(38u));CHECK(enter_align());CHECK(vx==16.0f);
    fresh_target(4,230,420);CHECK(vx==16.0f&&vy==0.0f);
    fresh_target(4,150,420);CHECK(st.state==VAT_BRAKE&&stopped());
    yaw=40.0f;tick20(13u);CHECK(st.state==VAT_YAW_FIX&&vx==0.0f&&vy==0.0f&&w<0.0f);
    yaw=37.0f;CHECK(stable_new_frame_gate());
    fresh_target(4,150,420);CHECK(vx==-16.0f&&vy==0.0f);
    fresh_target(4,190,380);CHECK(st.state==VAT_BRAKE&&stopped());
    CHECK(stable_new_frame_gate());
    fresh_target(4,190,380);CHECK(vx==0.0f&&vy==-16.0f);
    fresh_target(4,230,420);CHECK(st.state==VAT_BRAKE&&stopped());
    CHECK(stable_new_frame_gate());fresh_target(4,230,420);CHECK(vx==16.0f&&vy==0.0f);
    fresh_target(4,190,420);CHECK(st.state==VAT_BRAKE&&stopped());
    CHECK(stable_new_frame_gate());
    CHECK(finish_current_target(4,200,430));CHECK(st.state==VAT_DONE&&!vision_align_test_active());
    CHECK(reset(39u));CHECK(enter_align());fresh_target(9,190,440);CHECK(vx==0.0f&&vy==16.0f);
    fresh_target(9,190,400);CHECK(st.state==VAT_BRAKE);CHECK(stable_new_frame_gate());
    CHECK(finish_current_target(9,180,390));CHECK(st.state==VAT_DONE);
    CHECK(reset(40u));CHECK(enter_align());fresh_target(0,190,220);CHECK(st.state==VAT_BRAKE);
    CHECK(stable_new_frame_gate());CHECK(finish_current_target(0,190,220));CHECK(st.state==VAT_DONE);
    return 1;
}
static int check_slow_speed_and_tolerance_boundaries(void)
{
    /* Explicit user contract: do not let expected and actual values change
     * together unnoticed. Both endpoints are in-band; +/-11 must still move. */
    CHECK(VAT_SPEED_MMS == 16.0f && VAT_SPEED_MMS <= 50.0f / 3.0f && VAT_TOL_PX == 10);
    const int x_out[] = {179, 201};
    const int y_out[] = {389, 411};
    for (unsigned i=0u; i<2u; ++i) {
        CHECK(reset(39u)); CHECK(enter_align());
        fresh_target(9,x_out[i],400);
        CHECK(vx == (i ? 16.0f : -16.0f) && vy == 0.0f && st.good == 0u);
        CHECK(reset(39u)); CHECK(enter_align());
        fresh_target(9,190,y_out[i]);
        CHECK(vx == 0.0f && vy == (i ? 16.0f : -16.0f) && st.good == 0u);
        CHECK(reset(39u)); CHECK(enter_align());
        CHECK(finish_current_target(9,i ? 200 : 180,i ? 410 : 390));
        CHECK(st.state == VAT_DONE && stopped());
    }
    return 1;
}
static int check_initial_yaw_rejects_cached_image(void)
{
    CHECK(reset(39u));ack(request(),2u,0u);
    tick(260u);CHECK(st.state==VAT_YAW_FIX&&stopped());
    tick(T_DIST_ALIGN_STABLE_MS-20u);CHECK(st.state==VAT_YAW_FIX&&stopped());
    object(request(),9,190,400,640u,480u,seq++); /* in-band, but captured during correction */
    tick(20u);CHECK(st.state==VAT_ALIGN&&st.good==0u&&!st.latest&&stopped());
    tick(20u);CHECK(st.state==VAT_ALIGN&&st.good==0u&&!st.latest&&stopped());
    for(unsigned i=1u;i<VAT_GOOD_FRAMES;i++) {
        fresh_target(9,190,400);CHECK(st.state==VAT_ALIGN&&st.good==i&&st.latest&&stopped());
    }
    fresh_target(9,190,400);CHECK(st.state==VAT_DONE&&st.good==VAT_GOOD_FRAMES&&stopped());
    /* The cache reset must not turn ball's permitted initial search into an
     * endless RECHECK wait when no post-correction first frame exists yet. */
    CHECK(reset(38u));ack(request(),2u,0u);tick(260u);tick(T_DIST_ALIGN_STABLE_MS-20u);
    object(request(),4,190,420,640u,480u,seq++);tick(20u);tick(20u);
    CHECK(st.state==VAT_ALIGN&&st.good==0u&&!st.latest&&vx==VAT_SPEED_MMS&&vy==0.0f);
    return 1;
}
static int check_empty_stale_duplicate_and_dimensions(void)
{
    CHECK(reset(38u));CHECK(enter_align());fresh_target(4,230,420);CHECK(vx>0.0f&&st.latest);
    object(request(),-1,0,0,640u,480u,seq++);tick(20u);CHECK(st.state==VAT_BRAKE&&!st.latest&&stopped());
    CHECK(stable_new_frame_gate());tick20(50u);CHECK(stopped()&&st.state==VAT_RECHECK);
    fresh_target(4,230,420);CHECK(vx>0.0f&&st.latest);
    now_ms+=301u;vision_align_test_status(&st);CHECK(!st.latest&&st.age_ms>VAT_FRESH_MS);
    tick(0u);CHECK(st.state==VAT_BRAKE&&!st.latest&&stopped());
    CHECK(stable_new_frame_gate());
    fresh_target(4,190,420);
    for(unsigned i=0;i<8u;i++){object(request(),4,190,420,640u,480u,(uint16_t)(seq-1u));tick(20u);}
    CHECK(st.state==VAT_ALIGN&&st.good==1u&&stopped());
    object(request(),5,190,420,640u,480u,seq++);tick(20u);CHECK(st.good==0u&&stopped());
    CHECK(reset(39u));CHECK(enter_align());
    object(request(),9,190,310,640u,320u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&strcmp(st.reason,"IMAGE_GEOMETRY")==0&&stopped());
    CHECK(reset(39u));CHECK(enter_align());fresh_target(9,190,400);
    object(request(),9,190,400,639u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(39u));CHECK(enter_align());seq=65533u;
    CHECK(finish_current_target(9,190,400));CHECK(st.state==VAT_DONE&&seq<20u);
    CHECK(reset(39u));CHECK(enter_align());seq=10u;fresh_target(9,190,400);
    object(request(),9,190,400,640u,480u,9u);tick(20u);CHECK(st.good==0u&&!st.latest&&stopped());
    tick20(10u);CHECK(st.good==0u&&!st.latest&&st.state==VAT_ALIGN&&stopped());
    for(unsigned i=0;i<8u;i++){seq++;fresh_target(9,190,400);}
    CHECK(st.state==VAT_ALIGN&&st.good<=1u&&stopped()); /* sequence gaps never count5 */
    return 1;
}
static int check_final_still_and_cancellation(void)
{
    CHECK(reset(39u));CHECK(enter_align());
    for(unsigned i=0;i<8u;i++){fresh_target(9,190,400);counts[2]++;}
    CHECK(st.state==VAT_ALIGN&&st.good<=1u&&stopped());
    CHECK(finish_current_target(9,190,400));CHECK(st.state==VAT_DONE);
    const VisionAlignTestState phases[]={VAT_BRAKE,VAT_YAW_FIX,VAT_RECHECK,VAT_ALIGN};
    for(unsigned n=0;n<4u;n++) {
        CHECK(reset(38u));
        if(phases[n]!=VAT_BRAKE){CHECK(enter_align());fresh_target(4,190,420);}
        if(phases[n]==VAT_YAW_FIX){yaw+=3.0f;tick20(13u);CHECK(st.state==VAT_YAW_FIX);}
        if(phases[n]==VAT_RECHECK){CHECK(stable_new_frame_gate());}
        if(phases[n]==VAT_ALIGN){CHECK(stable_new_frame_gate());fresh_target(4,190,420);}
        vision_align_test_cancel();tick20(80u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    CHECK(reset(38u));CHECK(enter_align());imu_valid=0;tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    const float bad_yaw[]={NAN,INFINITY,-INFINITY};
    for(unsigned i=0u;i<sizeof bad_yaw/sizeof bad_yaw[0];i++) {
        CHECK(reset(38u));CHECK(enter_align());yaw=bad_yaw[i];tick(1u);
        CHECK(st.state==VAT_STOPPED&&strcmp(st.reason,"IMU")==0&&stopped());
    }
    CHECK(reset(38u));CHECK(enter_align());aborted=1;tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(39u));ack(request(),2u,1u);tick(20u);CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(39u));ack(request(),2u,0u);yaw+=3.0f;tick20(650u);
    CHECK(st.state==VAT_STOPPED&&strcmp(st.reason,"YAW_TIMEOUT")==0&&stopped());
    return 1;
}
static int check_ball_turn_bucket_sequence(void)
{
    CHECK(reset(41u));CHECK(enter_align());fresh_target(4,190,420);
    CHECK(st.state==VAT_BRAKE);CHECK(stable_new_frame_gate());CHECK(finish_current_target(4,190,420));
    CHECK(st.state==VAT_HOLD_BALL&&stopped());unsigned before=request_count;
    tick(4900u);CHECK(st.state==VAT_HOLD_BALL&&request_count==before);
    tick(100u);proto_service();CHECK(st.state==VAT_TURN_REQUEST&&stopped()&&last_task==4u&&!last_digit&&request_count==before+1u);
    uint16_t before_turn_request=request();
    CHECK(vision_align_test_take_turn_request());CHECK(!vision_align_test_take_turn_request());
    CHECK(st.state==VAT_TURN_REQUEST); /* snapshot refreshed explicitly below */
    vision_align_test_status(&st);CHECK(st.state==VAT_TURN_ACTIVE);
    vx=0.0f;vy=0.0f;w=1.2f;tick(20u);CHECK(w==1.2f); /* no competing brake */
    yaw=216.0f;vision_align_test_notify_turn_result(1);proto_service();vision_align_test_status(&st);
    CHECK(request()!=before_turn_request&&request_count==before+2u&&st.heading_target_deg==217.0f&&stopped());
    ack(request(),2u,0u);object(before_turn_request,9,190,400,640u,480u,seq++);tick20(13u);
    CHECK(st.state==VAT_YAW_FIX&&w>0.0f);yaw=217.0f;
    CHECK(stable_new_frame_gate());tick20(30u);CHECK(stopped()&&st.state==VAT_RECHECK);
    CHECK(finish_current_target(9,190,400));CHECK(st.state==VAT_HOLD_BUCKET&&stopped());
    tick(5000u);CHECK(st.state==VAT_DONE&&strcmp(st.reason,"ALIGNED_BALL_BUCKET")==0&&stopped());
    CHECK(reset(41u));CHECK(enter_align());fresh_target(4,190,420);CHECK(stable_new_frame_gate());
    CHECK(finish_current_target(4,190,420));tick(5000u);CHECK(vision_align_test_take_turn_request());
    yaw=190.0f;vision_align_test_notify_turn_result(1);vision_align_test_status(&st);
    CHECK(st.state==VAT_STOPPED&&strcmp(st.reason,"TURN180_RESIDUAL")==0&&stopped());
    CHECK(reset(41u));CHECK(enter_align());fresh_target(4,190,420);CHECK(stable_new_frame_gate());
    CHECK(finish_current_target(4,190,420));tick(5000u);CHECK(vision_align_test_take_turn_request());
    w=1.0f;vision_align_test_cancel();tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    vision_align_test_notify_turn_result(1);tick20(30u);CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(41u));CHECK(enter_align());fresh_target(4,190,420);CHECK(stable_new_frame_gate());
    CHECK(finish_current_target(4,190,420));tick(5000u);CHECK(vision_align_test_take_turn_request());
    w=1.0f;ack(request(),2u,1u);tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    return 1;
}
int main(void)
{
    if(!check_qr_gate_selection_and_initial_search()||!check_xy_axis_recheck_and_yaw()
       ||!check_slow_speed_and_tolerance_boundaries()
       ||!check_initial_yaw_rejects_cached_image()
       ||!check_empty_stale_duplicate_and_dimensions()||!check_final_still_and_cancellation()
       ||!check_ball_turn_bucket_sequence())return 1;
    puts("VAT realparser: v16/tol10 inclusive+outside11, freshQR gate/task selectors, X/Y directions/coupling, initial/post-yaw newimage, sameframe5/still250, stale/empty/duplicate/geometry, cancellation/nonfiniteIMU/NACK,41 hold5/shared180 bridge/freshbucket passed; no hardware acceptance");
    return 0;
}
