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
static float yaw, vx, vy, w, fore, lateral;
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
float motion_odo_mm(void) { return fore; }
float motion_lateral_odo_mm(void) { return lateral; }
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
static void reset_environment(void)
{
    now_ms=0;vx=vy=w=fore=lateral=0.0f;yaw=37.0f;imu_valid=1;aborted=laser=0;
    memset(counts,0,sizeof counts);request_count=laser_on=imu_zero=arm_calls=0;seq=1;
    last_opcode=last_task=last_digit=0;
    proto_init();proto_set_binary_mode(1);proto_set_binary_tx(tx);proto_set_on_frame(vision_align_test_feed_frame);
    vision_align_test_init();
}
static int reset(unsigned mode)
{
    reset_environment();
    if (mode == 39u) {
        CHECK(!proto_qr_get(NULL));
        CHECK(vision_align_test_start(39u,NULL));proto_service();vision_align_test_status(&st);
        CHECK(stopped()&&request_count==1u&&last_opcode==0x63u);
        return 1;
    }
    proto_send_scene(SCENE_QR);proto_service();
    CHECK(stopped()&&request_count==1u);
    ack(request(),1u,0u);qr_feed(request());
    CHECK(vision_align_test_start((uint8_t)mode,qr123));proto_service();vision_align_test_status(&st);
    CHECK(stopped()&&request_count==2u&&last_opcode==0x63u);
    return 1;
}
static int ready(void)
{
    ack(request(),2u,0u);tick(240u);CHECK(st.state==VAT_BRAKE&&stopped());
    tick(20u);CHECK(st.state==VAT_ALIGN&&stopped());return 1;
}
static void frame(int model,int x,int y)
{object(request(),model,x,y,640u,480u,seq++);tick(20u);}
static int end_step_by_odometry(void)
{
    CHECK(st.state==VAT_STEP_MOVE&&w==0.0f&&((vx!=0.0f)!=(vy!=0.0f)));
    if(vx!=0.0f)fore+=vx>0.0f?3.0f:-3.0f;
    else lateral+=vy>0.0f?3.0f:-3.0f;
    tick(20u);CHECK(st.state==VAT_BRAKE&&stopped()&&!st.step_capped);
    CHECK(fabsf(st.step_mm-3.0f)<0.0001f);return 1;
}
static int yaw_settle(void)
{
    CHECK(st.state==VAT_YAW_FIX&&st.yaw_dirty&&st.yaw_ever);
    yaw=st.yaw_target;tick(20u);CHECK(st.state==VAT_YAW_FIX&&stopped());
    tick20(34u);CHECK(st.state==VAT_YAW_FIX&&stopped());tick(20u);
    CHECK(st.state==VAT_RECHECK&&stopped()&&!st.yaw_dirty&&!st.latest&&!st.rx_fresh&&!st.good);
    return 1;
}
static int stop_recheck(void)
{
    CHECK(st.state==VAT_BRAKE&&stopped());tick(240u);CHECK(st.state==VAT_BRAKE&&stopped());tick(20u);
    if(st.yaw_dirty){CHECK(st.state==VAT_YAW_FIX&&stopped());CHECK(yaw_settle());}
    CHECK(st.state==VAT_RECHECK&&stopped()&&!st.latest&&!st.good);return 1;
}
static int finish(int model,int x,int y)
{
    CHECK(st.state==VAT_ALIGN||st.state==VAT_RECHECK);
    for(unsigned i=1u;i<=VAT_GOOD_FRAMES;i++){
        frame(model,x,y);if(i<VAT_GOOD_FRAMES)CHECK(st.state==VAT_ALIGN&&st.good==i&&stopped());
    }
    CHECK(st.state==VAT_DONE||st.state==VAT_HOLD_BALL||st.state==VAT_HOLD_BUCKET);
    CHECK(stopped());return 1;
}
static int check_qr_ack_direct_bucket_and_initial_search(void)
{
    const unsigned modes[]={38u,39u,40u,41u};
    for(unsigned i=0u;i<4u;i++){
        CHECK(reset(modes[i]));CHECK(last_task==(modes[i]==39u?4u:modes[i]==40u?3u:1u));
        CHECK(last_digit==(modes[i]==39u?0u:modes[i]==40u?3u:1u));
        tick(1000u);CHECK(st.state==VAT_ALIGN&&stopped());
        ack((uint16_t)(request()-1u),2u,0u);tick(20u);CHECK(stopped());
        ack(request(),2u,0u);tick(20u);
        if(modes[i]==39u){CHECK(st.state==VAT_ALIGN&&stopped());tick20(100u);CHECK(stopped());}
        else {
            CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f&&w==0.0f&&st.step==1u);
            /* valid+empty before the SAME poll must stick as seen: do not
             * lose the target when its single-slot cache is overwritten. */
            object(request(),modes[i]==40u?0:4,150,modes[i]==40u?220:420,640u,480u,seq++);
            object(request(),-1,0,0,640u,480u,seq++);tick(20u);
            CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);
            CHECK(end_step_by_odometry());CHECK(stop_recheck());tick20(20u);CHECK(stopped());
        }
        vision_align_test_cancel();tick20(20u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    reset_environment();CHECK(!vision_align_test_start(38u,NULL));CHECK(!vision_align_test_start(40u,qr123));CHECK(!vision_align_test_start(41u,qr123));
    CHECK(vision_align_test_start(39u,NULL));proto_service();CHECK(last_task==4u&&!last_digit&&!proto_qr_get(NULL));
    uint16_t first=request();object(first,9,230,400,640u,480u,seq++);tick(1000u);CHECK(stopped()&&!st.latest);
    ack(first,2u,0u);frame(9,230,400);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);
    vision_align_test_cancel();CHECK(vision_align_test_start(39u,NULL));proto_service();CHECK(request()!=first);
    ack(first,2u,0u);object(first,9,230,400,640u,480u,seq++);tick(1000u);CHECK(stopped());
    ack(request(),2u,1u);tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(38u));CHECK(ready());tick(20u);CHECK(st.state==VAT_STEP_MOVE);tick(250u);
    CHECK(st.state==VAT_BRAKE&&stopped()&&st.step_capped);tick(260u);CHECK(st.state==VAT_ALIGN&&stopped());
    tick(20u);CHECK(st.state==VAT_STEP_MOVE&&st.step==2u&&vx==20.0f); /* bounded never-seen search */
    return 1;
}
static int check_fixed_step_direction_and_fresh_recheck(void)
{
    CHECK(VAT_X_SPEED_MMS==20.0f&&VAT_Y_SPEED_MMS==30.0f&&VAT_STEP_MM==3.0f&&VAT_STEP_MAX_MS==250u&&VAT_TOL_PX==10);
    CHECK(reset(39u));CHECK(ready());frame(9,201,400);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&st.step==1u);
    frame(9,179,400);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f&&w==0.0f&&st.step==1u&&!st.good);
    frame(-1,0,0);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);
    frame(9,190,400);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);
    CHECK(end_step_by_odometry());CHECK(stop_recheck());tick20(5u);CHECK(st.state==VAT_RECHECK&&stopped());
    frame(9,179,400);CHECK(st.state==VAT_STEP_MOVE&&vx==-20.0f&&vy==0.0f&&st.step==2u);
    CHECK(end_step_by_odometry());CHECK(stop_recheck());
    frame(9,190,389);CHECK(st.state==VAT_STEP_MOVE&&vy==-30.0f&&vx==0.0f&&w==0.0f&&st.yaw_dirty);
    yaw=37.4f;frame(9,190,411);CHECK(st.state==VAT_STEP_MOVE&&vy==-30.0f&&w==0.0f);
    CHECK(end_step_by_odometry());tick(260u);CHECK(st.state==VAT_YAW_FIX&&stopped());tick(20u);
    CHECK(fabsf(w+0.18f)<0.00001f&&vx==0.0f&&vy==0.0f);
    frame(9,190,400);CHECK(st.rx_fresh&&!st.latest&&!st.good&&st.state==VAT_YAW_FIX);
    CHECK(yaw_settle());tick20(5u);CHECK(st.state==VAT_RECHECK&&stopped());
    frame(9,190,411);CHECK(st.state==VAT_STEP_MOVE&&vy==30.0f&&st.step==4u);
    CHECK(end_step_by_odometry());CHECK(stop_recheck());
    frame(9,201,400);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f); /* Y changed X */
    CHECK(end_step_by_odometry());CHECK(stop_recheck());CHECK(finish(9,190,400));CHECK(st.state==VAT_DONE);
    return 1;
}
static int check_step_distance_time_and_age_exits(void)
{
    const int xs[]={201,179,190,190},ys[]={400,400,389,411};
    for(unsigned i=0u;i<4u;i++){
        CHECK(reset(39u));CHECK(ready());frame(9,xs[i],ys[i]);CHECK(st.state==VAT_STEP_MOVE);
        float sign=i==1u||i==2u?-1.0f:1.0f;
        if(i<2u)lateral+=100.0f;else fore+=100.0f;tick(20u);CHECK(st.state==VAT_STEP_MOVE);
        if(i<2u)fore-=sign*1.0f;else lateral-=sign*1.0f;tick(20u);CHECK(st.state==VAT_STEP_MOVE);
        if(i<2u)fore+=sign*3.99f;else lateral+=sign*3.99f;tick(20u);CHECK(st.state==VAT_STEP_MOVE); /* net2.99 */
        if(i<2u)fore+=sign*0.02f;else lateral+=sign*0.02f;tick(20u);
        CHECK(st.state==VAT_BRAKE&&stopped()&&!st.step_capped&&st.step_mm>=3.0f);
    }
    CHECK(reset(39u));CHECK(ready());frame(9,201,400);tick(240u);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);
    tick(20u);CHECK(st.state==VAT_BRAKE&&stopped()&&st.step_capped&&st.step_mm==0.0f);
    CHECK(stop_recheck());tick20(30u);CHECK(st.state==VAT_RECHECK&&stopped());
    /* The bounded step uses the latched image's real receive age, not a new
     *250ms clock to keep driving on a coordinate that was already280ms old. */
    CHECK(reset(39u));CHECK(ready());object(request(),9,201,400,640u,480u,seq++);now_ms+=280u;
    vision_align_test_poll();vision_align_test_status(&st);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);
    tick(40u);CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good);
    CHECK(reset(39u));CHECK(ready());frame(9,201,400);
    object(request(),9,201,200,320u,320u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&strcmp(st.reason,"IMAGE_GEOMETRY")==0&&stopped());
    return 1;
}
static int check_pixels_sequences_geometry_and_still(void)
{
    const int xs[]={180,200},ys[]={390,410};
    for(unsigned i=0u;i<2u;i++){CHECK(reset(39u));CHECK(ready());CHECK(finish(9,xs[i],ys[i]));}
    CHECK(reset(38u));CHECK(ready());frame(4,280,333);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);
    frame(4,190,420);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);CHECK(end_step_by_odometry());CHECK(stop_recheck());
    frame(4,190,333);CHECK(st.state==VAT_STEP_MOVE&&vy==-30.0f);CHECK(end_step_by_odometry());CHECK(stop_recheck());
    CHECK(finish(4,190,420));CHECK(st.state==VAT_DONE);
    CHECK(reset(39u));ack(request(),2u,0u);tick(240u);object(request(),9,190,400,640u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_ALIGN&&!st.latest&&!st.good&&stopped());
    for(unsigned wheel=0u;wheel<4u;wheel++){
        CHECK(reset(39u));ack(request(),2u,0u);tick(240u);counts[wheel]++;tick(20u);
        tick(240u);CHECK(st.state==VAT_BRAKE&&stopped());tick(20u);CHECK(st.state==VAT_ALIGN&&stopped());
    }
    CHECK(reset(39u));CHECK(ready());frame(9,190,400);
    for(unsigned i=0u;i<5u;i++){object(request(),9,190,400,640u,480u,(uint16_t)(seq-1u));tick(20u);}
    CHECK(st.good==1u&&st.state==VAT_ALIGN&&stopped());
    object(request(),9,190,400,640u,480u,(uint16_t)(seq-2u));tick(20u);CHECK(!st.good&&!st.latest&&stopped());
    frame(4,190,400);CHECK(!st.good&&stopped());frame(-1,0,0);CHECK(!st.good&&!st.latest&&stopped());
    CHECK(reset(39u));CHECK(ready());seq=65533u;CHECK(finish(9,190,400));CHECK(seq<20u);
    CHECK(reset(39u));CHECK(ready());object(request(),9,190,310,640u,320u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&strcmp(st.reason,"IMAGE_GEOMETRY")==0&&stopped());
    CHECK(reset(39u));CHECK(ready());frame(9,190,400);object(request(),9,190,400,639u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(39u));CHECK(ready());frame(9,190,400);counts[0]++;frame(9,190,400);
    for(unsigned i=1u;i<VAT_GOOD_FRAMES;i++)frame(9,190,400);
    CHECK(st.good==5u&&st.state==VAT_ALIGN&&stopped());tick(160u);frame(9,190,400);CHECK(st.state==VAT_DONE&&stopped());
    return 1;
}
static int check_post_yaw_stability_final_gate_timeout(void)
{
    const float errors[]={-0.43f,-0.31f,0.31f,0.43f};
    for(unsigned i=0u;i<4u;i++){
        CHECK(reset(39u));CHECK(ready());frame(9,190,389);yaw=37.0f+errors[i];CHECK(end_step_by_odometry());tick(260u);tick(20u);
        CHECK(st.state==VAT_YAW_FIX&&fabsf(w-(errors[i]<0.0f?0.18f:-0.18f))<0.00001f);
        vision_align_test_cancel();tick20(40u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    CHECK(reset(39u));CHECK(ready());frame(9,190,411);uint16_t last_seq=(uint16_t)(seq-1u);
    CHECK(end_step_by_odometry());tick(260u);CHECK(st.state==VAT_YAW_FIX);
    yaw=37.25f;tick(20u);CHECK(stopped());tick20(30u);CHECK(st.state==VAT_YAW_FIX);
    yaw=36.95f;tick(20u);tick20(34u);CHECK(st.state==VAT_YAW_FIX&&stopped());
    counts[3]++;tick(20u);tick20(12u);CHECK(st.state==VAT_YAW_FIX&&stopped());tick20(40u);
    CHECK(st.state==VAT_RECHECK&&stopped()&&!st.latest&&!st.good);
    object(request(),9,190,400,640u,480u,last_seq);tick(20u);CHECK(st.state==VAT_RECHECK&&!st.latest&&!st.good);
    object(request(),9,190,400,640u,480u,(uint16_t)(last_seq-1u));tick(20u);CHECK(st.state==VAT_RECHECK&&!st.latest&&!st.good);
    frame(9,201,400);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);CHECK(end_step_by_odometry());CHECK(stop_recheck());
    frame(9,190,400);counts[0]++;frame(9,190,400);for(unsigned i=1u;i<VAT_GOOD_FRAMES;i++)frame(9,190,400);
    CHECK(st.state==VAT_ALIGN&&st.good==5u&&stopped());yaw=37.4f;frame(9,190,400);
    CHECK(st.state==VAT_BRAKE&&st.yaw_dirty&&!st.good&&stopped());CHECK(stop_recheck());CHECK(finish(9,190,400));
    CHECK(reset(39u));CHECK(ready());frame(9,190,389);yaw=40.0f;CHECK(end_step_by_odometry());tick(260u);tick(20u);
    CHECK(fabsf(w+0.30f)<0.00001f);tick20(600u);CHECK(st.state==VAT_STOPPED&&strcmp(st.reason,"YAW_TIMEOUT")==0&&stopped());
    return 1;
}
static int check_imu_faults_and_no_initial_correction(void)
{
    const float faults[]={NAN,INFINITY,-INFINITY,37.0f};
    for(unsigned mode=38u;mode<=41u;mode++)for(unsigned i=0u;i<4u;i++){
        reset_environment();if(mode!=39u){proto_send_scene(SCENE_QR);proto_service();ack(request(),1u,0u);qr_feed(request());}
        unsigned before=request_count;yaw=faults[i];imu_valid=i<3u;
        CHECK(!vision_align_test_start((uint8_t)mode,mode==39u?NULL:qr123));proto_service();CHECK(request_count==before&&stopped());
        CHECK(reset(mode));CHECK(ready());yaw=faults[i];imu_valid=i<3u;tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
        CHECK(reset(mode));CHECK(ready());frame(mode==39u?9:mode==40u?0:4,201,mode==39u?400:mode==40u?220:420);
        CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);yaw=faults[i];imu_valid=i<3u;tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    CHECK(reset(39u));yaw=90.0f;CHECK(ready());frame(9,201,400);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&w==0.0f&&!st.yaw_ever);
    CHECK(end_step_by_odometry());CHECK(stop_recheck());CHECK(finish(9,190,400));CHECK(!st.yaw_ever&&st.state==VAT_DONE);
    CHECK(reset(39u));CHECK(ready());fore=NAN;frame(9,201,400);CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(39u));CHECK(ready());frame(9,190,389);lateral=INFINITY;tick(1u);tick(20u);CHECK(st.state==VAT_STOPPED&&stopped());
    return 1;
}
static int check_cancellation_and_abort_nack(void)
{
    for(unsigned phase=0u;phase<5u;phase++){
        CHECK(reset(39u));if(phase>0u)CHECK(ready());
        if(phase==2u){frame(9,201,400);CHECK(st.state==VAT_STEP_MOVE);}
        if(phase==3u){frame(9,201,400);CHECK(end_step_by_odometry());CHECK(stop_recheck());}
        if(phase==4u){frame(9,190,389);CHECK(end_step_by_odometry());tick(260u);CHECK(st.state==VAT_YAW_FIX);}
        vision_align_test_cancel();tick20(100u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    CHECK(reset(39u));CHECK(ready());frame(9,201,400);aborted=1;tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(39u));CHECK(ready());frame(9,201,400);ack(request(),2u,1u);tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    return 1;
}
static int check_ball_turn_bucket_sequence(void)
{
    CHECK(reset(41u));CHECK(ready());CHECK(finish(4,190,420));CHECK(st.state==VAT_HOLD_BALL);
    unsigned before=request_count;tick(4980u);CHECK(st.state==VAT_HOLD_BALL);tick(20u);proto_service();
    CHECK(st.state==VAT_TURN_REQUEST&&request_count==before+1u&&last_task==4u&&!last_digit&&stopped());
    uint16_t preturn=request();CHECK(vision_align_test_take_turn_request());CHECK(!vision_align_test_take_turn_request());
    w=1.2f;tick(20u);CHECK(st.state==VAT_TURN_ACTIVE&&w==1.2f);
    yaw=217.0f;vision_align_test_notify_turn_result(1);proto_service();vision_align_test_status(&st);
    CHECK(request()!=preturn&&st.state==VAT_BRAKE&&st.yaw_target==217.0f&&!st.yaw_ever&&stopped());
    ack(request(),2u,0u);object(preturn,9,190,400,640u,480u,seq++);tick(20u);
    object(request(),9,190,400,640u,480u,seq++);tick(20u);CHECK(st.rx_fresh&&!st.latest&&!st.good);
    tick(260u);CHECK(st.state==VAT_RECHECK&&stopped()&&!st.latest&&!st.good);
    frame(9,190,389);CHECK(st.state==VAT_STEP_MOVE&&vy==-30.0f);yaw=217.4f;CHECK(end_step_by_odometry());CHECK(stop_recheck());
    CHECK(st.yaw_target==217.0f);CHECK(finish(9,190,400));CHECK(st.state==VAT_HOLD_BUCKET);
    tick(5000u);CHECK(st.state==VAT_DONE&&strcmp(st.reason,"ALIGNED_BALL_BUCKET")==0&&stopped());
    for(unsigned fault=0u;fault<3u;fault++){
        CHECK(reset(41u));CHECK(ready());CHECK(finish(4,190,420));tick(5000u);CHECK(vision_align_test_take_turn_request());
        if(fault==0u)vision_align_test_notify_turn_result(0);
        else if(fault==1u){imu_valid=0;vision_align_test_notify_turn_result(1);}
        else {w=1.0f;vision_align_test_cancel();vision_align_test_notify_turn_result(1);}
        tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    return 1;
}
int main(void)
{
    if(!check_qr_ack_direct_bucket_and_initial_search()||!check_fixed_step_direction_and_fresh_recheck()
    ||!check_step_distance_time_and_age_exits()||!check_pixels_sequences_geometry_and_still()
    ||!check_post_yaw_stability_final_gate_timeout()||!check_imu_faults_and_no_initial_correction()
    ||!check_cancellation_and_abort_nack()||!check_ball_turn_bucket_sequence())return 1;
    puts("VAT realparser: bounded3mm/250ms X20/Y30 steps, no midstep reversal, signedaxis odo+age exits, stop250/newimage, postY-yaw700/minW/finalgate/seqhistory,±10/fivefresh/still,QR/ACK/direct39/stickyseen/cancel/IMU/NACK/ODO,41shared180/newheading passed; no physical acceptance");return 0;
}
