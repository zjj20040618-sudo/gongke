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
static float yaw, leg_yaw, vx, vy, w, fore, lateral;
static float heading_hold_target;
static float heading_hold_kp;
static unsigned heading_hold_calls;
static int imu_valid, aborted, laser;
static int32_t counts[4];
static unsigned request_count, laser_on, imu_zero, arm_calls;
static uint16_t seq;
static uint8_t last_task, last_digit, last_opcode;
static VisionAlignTestStatus st;
static int route_environment;
static int route_auto_hostage_rank;
static int route_auto_ball_rank;
static const int32_t qr123[3] = {1,2,3};

#define CHECK(expr) do { if (!(expr)) { vision_align_test_status(&st); \
    fprintf(stderr,"VAT line%d:%s state%d reason%s now%lu v%.1f/%.1f/%.2f good%u\n", \
    __LINE__,#expr,st.state,st.reason,(unsigned long)now_ms,vx,vy,w,st.good); return 0; } } while(0)

uint32_t HAL_GetTick(void) { return now_ms; }
uint8_t imu_ok(void) { return (uint8_t)imu_valid; }
float imu_heading_deg(void) { return yaw; }
float imu_leg_heading_deg(void) { return leg_yaw; }
/* A deterministic navigation mock: validates the VAT continuous-heading to
 * current-leg bridge, not the physical IMU/mecanum controller response. */
float step_heading_hold_w_kp(float heading,float kp)
{
    heading_hold_target=heading;heading_hold_kp=kp;heading_hold_calls++;
    return (heading-leg_yaw)*kp*0.0174533f;
}
float step_heading_hold_w(float heading) { return step_heading_hold_w_kp(heading,0.3f); }
float step_heading_kp_deg(void) { return 0.3f; }
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
static void rank_frame(uint16_t req,uint16_t sequence,uint8_t model,uint8_t rank,
                       uint8_t seen,uint8_t slot0,uint8_t slot1,uint8_t slot2)
{
    uint8_t p[14]={0x62u,0,0,9u,0,0x54u,0,0,model,rank,seen,slot0,slot1,slot2};
    le16(p+1,req);le16(p+6,sequence);feed(p,sizeof p);
}
static void ack(uint16_t req,uint8_t mode,uint8_t result)
{
    uint8_t p[5]={0x61u,0,0,mode,result};le16(p+1,req);feed(p,sizeof p);
    if(route_environment&&route_auto_hostage_rank&&last_task==PROTO_TASK_HOSTAGE
       &&req==request()&&mode==2u&&!result)
        rank_frame(req,seq,0u,1u,1u,0u,0xffu,0xffu);
    if(route_environment&&route_auto_ball_rank&&last_task==PROTO_TASK_BALL
       &&req==request()&&mode==2u&&!result)
        rank_frame(req,seq,4u,(uint8_t)route_auto_ball_rank,3u,
            route_auto_ball_rank==1?4u:3u,
            route_auto_ball_rank==2?4u:route_auto_ball_rank==1?3u:5u,
            route_auto_ball_rank==3?4u:5u);
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
    leg_yaw=heading_hold_target=heading_hold_kp=0.0f;heading_hold_calls=0u;
    route_environment=0;route_auto_hostage_rank=1;route_auto_ball_rank=1;
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
static int ready(void);
static int finish(int model,int x,int y);
static int hostage_owner_complete(VisionAlignTestState expected)
{
    CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&stopped());
    vision_align_test_notify_route_action_result(1);vision_align_test_status(&st);
    CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION); /* Notification without take is inert. */
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_HOSTAGE);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    vision_align_test_notify_route_action_result(1);tick(20u);
    CHECK(st.state==expected&&stopped());
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    return 1;
}
static int route_ramp100(float ratio)
{
    CHECK(st.state==VAT_ROUTE_SEARCH&&vx==0.0f&&vy==0.0f);
    tick(20u);CHECK(fabsf(vx-14.0f)<0.00001f&&fabsf(vy-14.0f*ratio)<0.00001f);
    tick(20u);CHECK(fabsf(vx-28.0f)<0.00001f&&fabsf(vy-28.0f*ratio)<0.00001f);
    tick(200u);CHECK(vx==100.0f&&vy==100.0f*ratio&&st.state==VAT_ROUTE_SEARCH);
    return 1;
}
static int reset_route(unsigned mode)
{
    reset_environment();
    proto_send_scene(SCENE_QR);proto_service();ack(request(),1u,0u);qr_feed(request());
    int32_t locked[3]={0,0,0};CHECK(proto_qr_get(locked));
    proto_receive_end();CHECK(!proto_qr_get(NULL));
    CHECK(!vision_align_test_start((uint8_t)mode,locked));
    CHECK(vision_align_test_start_route((uint8_t)mode,locked));proto_service();vision_align_test_status(&st);
    route_environment=1;
    CHECK(stopped()&&request_count==2u&&last_opcode==0x63u);
    return 1;
}
static int check_route_locked_qr_entry(void)
{
    const unsigned modes[]={40u,41u};
    const int32_t bad_values[]={-1,0,4};
    const float faults[]={NAN,INFINITY,-INFINITY,37.0f};
    reset_environment();
    for(unsigned mode=0u;mode<=42u;mode++)if(mode!=40u&&mode!=41u){
        CHECK(!vision_align_test_start_route((uint8_t)mode,qr123));
    }
    CHECK(!vision_align_test_start_route(40u,NULL));CHECK(!vision_align_test_start_route(41u,NULL));
    proto_service();CHECK(request_count==0u&&stopped());
    for(unsigned m=0u;m<2u;m++)for(unsigned digit=0u;digit<3u;digit++)for(unsigned b=0u;b<3u;b++){
        reset_environment();int32_t invalid[3]={1,2,3};invalid[digit]=bad_values[b];
        CHECK(!vision_align_test_start_route((uint8_t)modes[m],invalid));proto_service();
        CHECK(request_count==0u&&stopped());
    }
    for(unsigned m=0u;m<2u;m++)for(unsigned f=0u;f<4u;f++){
        reset_environment();yaw=faults[f];imu_valid=f<3u;
        CHECK(!vision_align_test_start_route((uint8_t)modes[m],qr123));proto_service();
        CHECK(request_count==0u&&stopped());
    }
    reset_environment();aborted=1;
    CHECK(!vision_align_test_start_route(40u,qr123));CHECK(!vision_align_test_start_route(41u,qr123));
    proto_service();CHECK(request_count==0u&&stopped());
    for(unsigned m=0u;m<2u;m++){
        CHECK(reset_route(modes[m]));CHECK(last_task==(modes[m]==40u?3u:1u));
        CHECK(last_digit==(modes[m]==40u?3u:1u));unsigned before=request_count;
        CHECK(!vision_align_test_start_route((uint8_t)modes[m],qr123));
        CHECK(!vision_align_test_start(39u,NULL));proto_service();CHECK(request_count==before&&st.state==VAT_BRAKE);
        CHECK(ready());CHECK(finish(modes[m]==40u?0:4,modes[m]==40u?215:135,modes[m]==40u?220:VAT_BALL_Y_PX));
        CHECK(st.state==(modes[m]==40u?VAT_WAIT_HOSTAGE_ACTION:VAT_WAIT_BALL_ACTION));
        if(modes[m]==40u)CHECK(hostage_owner_complete(VAT_DONE));
        vision_align_test_cancel();CHECK(stopped());
    }
    /* The new route entry cannot weaken the independent parser-current gate. */
    reset_environment();proto_send_scene(SCENE_QR);proto_service();ack(request(),1u,0u);qr_feed(request());
    const int32_t different[3]={3,3,1};unsigned before=request_count;
    CHECK(!vision_align_test_start(38u,different));CHECK(!vision_align_test_start(40u,different));
    CHECK(!vision_align_test_start(41u,different));proto_service();CHECK(request_count==before&&stopped());
    CHECK(vision_align_test_start_route(40u,different));proto_service();CHECK(last_task==3u&&last_digit==1u);
    vision_align_test_cancel();CHECK(stopped());
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
    if(route_environment){
        /* A first route image now brakes even when it arrives beforeSEARCH.
         * Its stopped NEW images alone may pass the existing five-frame gate. */
        for(unsigned i=0u;i<120u;i++){
            frame(model,x,y);CHECK(stopped());
            if(st.state==VAT_DONE||st.state==VAT_WAIT_BALL_ACTION||st.state==VAT_WAIT_BUCKET_ACTION||st.state==VAT_WAIT_HOSTAGE_ACTION)break;
        }
    }else for(unsigned i=1u;i<=VAT_GOOD_FRAMES;i++){
        frame(model,x,y);if(i<VAT_GOOD_FRAMES)CHECK(st.state==VAT_ALIGN&&st.good==i&&stopped());
    }
    CHECK(st.state==VAT_DONE||st.state==VAT_HOLD_BALL||st.state==VAT_HOLD_BUCKET||
          st.state==VAT_WAIT_BALL_ACTION||st.state==VAT_WAIT_BUCKET_ACTION||st.state==VAT_WAIT_HOSTAGE_ACTION);
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
            object(request(),modes[i]==40u?0:4,150,modes[i]==40u?220:VAT_BALL_Y_PX,640u,480u,seq++);
            object(request(),-1,0,0,640u,480u,seq++);tick(20u);
            CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);
            CHECK(end_step_by_odometry());CHECK(stop_recheck());tick20(20u);CHECK(stopped());
        }
        vision_align_test_cancel();tick20(20u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    reset_environment();CHECK(!vision_align_test_start(38u,NULL));CHECK(!vision_align_test_start(40u,qr123));CHECK(!vision_align_test_start(41u,qr123));
    CHECK(vision_align_test_start(39u,NULL));proto_service();CHECK(last_task==4u&&!last_digit&&!proto_qr_get(NULL));
    uint16_t first=request();object(first,9,230,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(1000u);CHECK(stopped()&&!st.latest);
    ack(first,2u,0u);frame(9,230,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);
    vision_align_test_cancel();CHECK(vision_align_test_start(39u,NULL));proto_service();CHECK(request()!=first);
    ack(first,2u,0u);object(first,9,230,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(1000u);CHECK(stopped());
    ack(request(),2u,1u);tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(38u));CHECK(ready());tick(20u);CHECK(st.state==VAT_STEP_MOVE);tick(250u);
    CHECK(st.state==VAT_BRAKE&&stopped()&&st.step_capped);tick(260u);CHECK(st.state==VAT_ALIGN&&stopped());
    tick(20u);CHECK(st.state==VAT_STEP_MOVE&&st.step==2u&&vx==20.0f); /* bounded never-seen search */
    return 1;
}
static int check_fixed_step_direction_and_fresh_recheck(void)
{
    CHECK(VAT_X_SPEED_MMS==20.0f&&VAT_Y_SPEED_MMS==30.0f&&VAT_STEP_MM==3.0f&&VAT_STEP_MAX_MS==250u&&VAT_TOL_PX==10);
    CHECK(VAT_X_PX==190&&VAT_BALL_Y_PX==390&&VAT_BUCKET_Y_PX==420&&VAT_HOSTAGE_Y_PX==220);
    CHECK(reset(39u));CHECK(ready());frame(9,201,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&st.step==1u);
    frame(9,179,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f&&w==0.0f&&st.step==1u&&!st.good);
    frame(-1,0,0);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);
    frame(9,190,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);
    CHECK(end_step_by_odometry());CHECK(stop_recheck());tick20(5u);CHECK(st.state==VAT_RECHECK&&stopped());
    frame(9,179,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==-20.0f&&vy==0.0f&&st.step==2u);
    CHECK(end_step_by_odometry());CHECK(stop_recheck());
    frame(9,190,(VAT_BUCKET_Y_PX-11));CHECK(st.state==VAT_STEP_MOVE&&vy==-30.0f&&vx==0.0f&&w==0.0f&&st.yaw_dirty);
    yaw=37.4f;frame(9,190,(VAT_BUCKET_Y_PX+11));CHECK(st.state==VAT_STEP_MOVE&&vy==-30.0f&&w==0.0f);
    CHECK(end_step_by_odometry());tick(260u);CHECK(st.state==VAT_YAW_FIX&&stopped());tick(20u);
    CHECK(fabsf(w+0.18f)<0.00001f&&vx==0.0f&&vy==0.0f);
    frame(9,190,VAT_BUCKET_Y_PX);CHECK(st.rx_fresh&&!st.latest&&!st.good&&st.state==VAT_YAW_FIX);
    CHECK(yaw_settle());tick20(5u);CHECK(st.state==VAT_RECHECK&&stopped());
    frame(9,190,(VAT_BUCKET_Y_PX+11));CHECK(st.state==VAT_STEP_MOVE&&vy==30.0f&&st.step==4u);
    CHECK(end_step_by_odometry());CHECK(stop_recheck());
    frame(9,201,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f); /* Y changed X */
    CHECK(end_step_by_odometry());CHECK(stop_recheck());CHECK(finish(9,190,VAT_BUCKET_Y_PX));CHECK(st.state==VAT_DONE);
    return 1;
}
static int check_step_distance_time_and_age_exits(void)
{
    const int xs[]={201,179,190,190},ys[]={VAT_BUCKET_Y_PX,VAT_BUCKET_Y_PX,VAT_BUCKET_Y_PX-11,VAT_BUCKET_Y_PX+11};
    for(unsigned i=0u;i<4u;i++){
        CHECK(reset(39u));CHECK(ready());frame(9,xs[i],ys[i]);CHECK(st.state==VAT_STEP_MOVE);
        float sign=i==1u||i==2u?-1.0f:1.0f;
        if(i<2u)lateral+=100.0f;else fore+=100.0f;tick(20u);CHECK(st.state==VAT_STEP_MOVE);
        if(i<2u)fore-=sign*1.0f;else lateral-=sign*1.0f;tick(20u);CHECK(st.state==VAT_STEP_MOVE);
        if(i<2u)fore+=sign*3.99f;else lateral+=sign*3.99f;tick(20u);CHECK(st.state==VAT_STEP_MOVE); /* net2.99 */
        if(i<2u)fore+=sign*0.02f;else lateral+=sign*0.02f;tick(20u);
        CHECK(st.state==VAT_BRAKE&&stopped()&&!st.step_capped&&st.step_mm>=3.0f);
    }
    CHECK(reset(39u));CHECK(ready());frame(9,201,VAT_BUCKET_Y_PX);tick(240u);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);
    tick(20u);CHECK(st.state==VAT_BRAKE&&stopped()&&st.step_capped&&st.step_mm==0.0f);
    CHECK(stop_recheck());tick20(30u);CHECK(st.state==VAT_RECHECK&&stopped());
    /* The bounded step uses the latched image's real receive age, not a new
     *250ms clock to keep driving on a coordinate that was already280ms old. */
    CHECK(reset(39u));CHECK(ready());object(request(),9,201,VAT_BUCKET_Y_PX,640u,480u,seq++);now_ms+=280u;
    vision_align_test_poll();vision_align_test_status(&st);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);
    tick(40u);CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good);
    CHECK(reset(39u));CHECK(ready());frame(9,201,VAT_BUCKET_Y_PX);
    object(request(),9,201,200,320u,320u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&strcmp(st.reason,"IMAGE_GEOMETRY")==0&&stopped());
    return 1;
}
static int check_pixels_sequences_geometry_and_still(void)
{
    const int xs[]={180,200},ys[]={VAT_BUCKET_Y_PX-10,VAT_BUCKET_Y_PX+10};
    for(unsigned i=0u;i<2u;i++){CHECK(reset(39u));CHECK(ready());CHECK(finish(9,xs[i],ys[i]));}
    CHECK(reset(38u));CHECK(ready());frame(4,280,333);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);
    frame(4,190,VAT_BALL_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);CHECK(end_step_by_odometry());CHECK(stop_recheck());
    frame(4,190,333);CHECK(st.state==VAT_STEP_MOVE&&vy==-30.0f);CHECK(end_step_by_odometry());CHECK(stop_recheck());
    CHECK(finish(4,190,VAT_BALL_Y_PX));CHECK(st.state==VAT_DONE);
    CHECK(reset(39u));ack(request(),2u,0u);tick(240u);object(request(),9,190,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_ALIGN&&!st.latest&&!st.good&&stopped());
    for(unsigned wheel=0u;wheel<4u;wheel++){
        CHECK(reset(39u));ack(request(),2u,0u);tick(240u);counts[wheel]++;tick(20u);
        tick(240u);CHECK(st.state==VAT_BRAKE&&stopped());tick(20u);CHECK(st.state==VAT_ALIGN&&stopped());
    }
    CHECK(reset(39u));CHECK(ready());frame(9,190,VAT_BUCKET_Y_PX);
    for(unsigned i=0u;i<5u;i++){object(request(),9,190,VAT_BUCKET_Y_PX,640u,480u,(uint16_t)(seq-1u));tick(20u);}
    CHECK(st.good==1u&&st.state==VAT_ALIGN&&stopped());
    object(request(),9,190,VAT_BUCKET_Y_PX,640u,480u,(uint16_t)(seq-2u));tick(20u);CHECK(!st.good&&!st.latest&&stopped());
    frame(4,190,VAT_BUCKET_Y_PX);CHECK(!st.good&&stopped());frame(-1,0,0);CHECK(!st.good&&!st.latest&&stopped());
    CHECK(reset(39u));CHECK(ready());seq=65533u;CHECK(finish(9,190,VAT_BUCKET_Y_PX));CHECK(seq<20u);
    CHECK(reset(39u));CHECK(ready());object(request(),9,190,310,640u,320u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&strcmp(st.reason,"IMAGE_GEOMETRY")==0&&stopped());
    CHECK(reset(39u));CHECK(ready());frame(9,190,VAT_BUCKET_Y_PX);object(request(),9,190,VAT_BUCKET_Y_PX,639u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(39u));CHECK(ready());frame(9,190,VAT_BUCKET_Y_PX);counts[0]++;frame(9,190,VAT_BUCKET_Y_PX);
    for(unsigned i=1u;i<VAT_GOOD_FRAMES;i++)frame(9,190,VAT_BUCKET_Y_PX);
    CHECK(st.good==5u&&st.state==VAT_ALIGN&&stopped());tick(160u);frame(9,190,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_DONE&&stopped());
    return 1;
}
static int check_post_yaw_stability_final_gate_timeout(void)
{
    const float errors[]={-0.43f,-0.31f,0.31f,0.43f};
    for(unsigned i=0u;i<4u;i++){
        CHECK(reset(39u));CHECK(ready());frame(9,190,(VAT_BUCKET_Y_PX-11));yaw=37.0f+errors[i];CHECK(end_step_by_odometry());tick(260u);tick(20u);
        CHECK(st.state==VAT_YAW_FIX&&fabsf(w-(errors[i]<0.0f?0.18f:-0.18f))<0.00001f);
        vision_align_test_cancel();tick20(40u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    CHECK(reset(39u));CHECK(ready());frame(9,190,(VAT_BUCKET_Y_PX+11));uint16_t last_seq=(uint16_t)(seq-1u);
    CHECK(end_step_by_odometry());tick(260u);CHECK(st.state==VAT_YAW_FIX);
    yaw=37.25f;tick(20u);CHECK(stopped());tick20(30u);CHECK(st.state==VAT_YAW_FIX);
    yaw=36.95f;tick(20u);tick20(34u);CHECK(st.state==VAT_YAW_FIX&&stopped());
    counts[3]++;tick(20u);tick20(12u);CHECK(st.state==VAT_YAW_FIX&&stopped());tick20(40u);
    CHECK(st.state==VAT_RECHECK&&stopped()&&!st.latest&&!st.good);
    object(request(),9,190,VAT_BUCKET_Y_PX,640u,480u,last_seq);tick(20u);CHECK(st.state==VAT_RECHECK&&!st.latest&&!st.good);
    object(request(),9,190,VAT_BUCKET_Y_PX,640u,480u,(uint16_t)(last_seq-1u));tick(20u);CHECK(st.state==VAT_RECHECK&&!st.latest&&!st.good);
    frame(9,201,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);CHECK(end_step_by_odometry());CHECK(stop_recheck());
    frame(9,190,VAT_BUCKET_Y_PX);counts[0]++;frame(9,190,VAT_BUCKET_Y_PX);for(unsigned i=1u;i<VAT_GOOD_FRAMES;i++)frame(9,190,VAT_BUCKET_Y_PX);
    CHECK(st.state==VAT_ALIGN&&st.good==5u&&stopped());yaw=37.4f;frame(9,190,VAT_BUCKET_Y_PX);
    CHECK(st.state==VAT_BRAKE&&st.yaw_dirty&&!st.good&&stopped());CHECK(stop_recheck());CHECK(finish(9,190,VAT_BUCKET_Y_PX));
    CHECK(reset(39u));CHECK(ready());frame(9,190,(VAT_BUCKET_Y_PX-11));yaw=40.0f;CHECK(end_step_by_odometry());tick(260u);tick(20u);
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
        CHECK(reset(mode));CHECK(ready());frame(mode==39u?9:mode==40u?0:4,201,mode==39u?VAT_BUCKET_Y_PX:mode==40u?220:VAT_BALL_Y_PX);
        CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f);yaw=faults[i];imu_valid=i<3u;tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    CHECK(reset(39u));yaw=90.0f;CHECK(ready());frame(9,201,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&w==0.0f&&!st.yaw_ever);
    CHECK(end_step_by_odometry());CHECK(stop_recheck());CHECK(finish(9,190,VAT_BUCKET_Y_PX));CHECK(!st.yaw_ever&&st.state==VAT_DONE);
    CHECK(reset(39u));CHECK(ready());fore=NAN;frame(9,201,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(39u));CHECK(ready());frame(9,190,(VAT_BUCKET_Y_PX-11));lateral=INFINITY;tick(1u);tick(20u);CHECK(st.state==VAT_STOPPED&&stopped());
    return 1;
}
static int check_cancellation_and_abort_nack(void)
{
    for(unsigned phase=0u;phase<5u;phase++){
        CHECK(reset(39u));if(phase>0u)CHECK(ready());
        if(phase==2u){frame(9,201,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE);}
        if(phase==3u){frame(9,201,VAT_BUCKET_Y_PX);CHECK(end_step_by_odometry());CHECK(stop_recheck());}
        if(phase==4u){frame(9,190,(VAT_BUCKET_Y_PX-11));CHECK(end_step_by_odometry());tick(260u);CHECK(st.state==VAT_YAW_FIX);}
        vision_align_test_cancel();tick20(100u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    CHECK(reset(39u));CHECK(ready());frame(9,201,VAT_BUCKET_Y_PX);aborted=1;tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    CHECK(reset(39u));CHECK(ready());frame(9,201,VAT_BUCKET_Y_PX);ack(request(),2u,1u);tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    return 1;
}
static int check_ball_turn_bucket_sequence(int route)
{
    int bucket_x=route?125:190;
    CHECK(route?reset_route(41u):reset(41u));CHECK(ready());CHECK(finish(4,route?135:190,VAT_BALL_Y_PX));
    unsigned before=request_count;
    if(route){
        CHECK(st.state==VAT_WAIT_BALL_ACTION&&!vision_align_test_take_turn_request());
        vision_align_test_notify_route_action_result(1);tick(5000u);
        CHECK(st.state==VAT_WAIT_BALL_ACTION&&request_count==before&&stopped());
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        tick(5000u);CHECK(st.state==VAT_WAIT_BALL_ACTION&&request_count==before&&stopped());
        vision_align_test_notify_route_action_result(1);tick(20u);
    }else{
        CHECK(st.state==VAT_HOLD_BALL&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        vision_align_test_notify_route_action_result(1);
        tick(4980u);CHECK(st.state==VAT_HOLD_BALL&&request_count==before);tick(20u);
    }
    proto_service();
    CHECK(st.state==VAT_TURN_REQUEST&&request_count==before+1u&&last_task==4u&&!last_digit&&stopped());
    uint16_t preturn=request();CHECK(vision_align_test_take_turn_request());CHECK(!vision_align_test_take_turn_request());
    w=1.2f;tick(20u);CHECK(st.state==VAT_TURN_ACTIVE&&w==1.2f);
    yaw=217.0f;vision_align_test_notify_turn_result(1);proto_service();vision_align_test_status(&st);
    CHECK(request()!=preturn&&st.state==VAT_BRAKE&&st.yaw_target==217.0f&&!st.yaw_ever&&stopped());
    ack(request(),2u,0u);object(preturn,9,bucket_x,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);
    object(request(),9,bucket_x,VAT_BUCKET_Y_PX,640u,480u,seq++);tick(20u);CHECK(st.rx_fresh&&!st.latest&&!st.good);
    tick(260u);CHECK(st.state==VAT_RECHECK&&stopped()&&!st.latest&&!st.good);
    tick(1000u);CHECK(st.state==VAT_RECHECK&&stopped()&&!st.good); /* Fine gate cannot reopen coarse search. */
    frame(9,bucket_x,(VAT_BUCKET_Y_PX-11));CHECK(st.state==VAT_STEP_MOVE&&vy==-30.0f);yaw=217.4f;CHECK(end_step_by_odometry());CHECK(stop_recheck());
    CHECK(st.yaw_target==217.0f);CHECK(finish(9,bucket_x,VAT_BUCKET_Y_PX));
    if(route){
        CHECK(st.state==VAT_WAIT_BUCKET_ACTION);before=request_count;
        tick(5000u);CHECK(st.state==VAT_WAIT_BUCKET_ACTION&&request_count==before&&stopped());
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BUCKET);
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        vision_align_test_notify_route_action_result(1);tick(20u);
    }else{
        CHECK(st.state==VAT_HOLD_BUCKET&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        tick(4980u);CHECK(st.state==VAT_HOLD_BUCKET);tick(20u);
    }
    CHECK(st.state==VAT_DONE&&strcmp(st.reason,"ALIGNED_BALL_BUCKET")==0&&stopped());
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    for(unsigned fault=0u;fault<3u;fault++){
        CHECK(route?reset_route(41u):reset(41u));CHECK(ready());CHECK(finish(4,route?135:190,VAT_BALL_Y_PX));
        if(route){CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);vision_align_test_notify_route_action_result(1);}
        else tick(5000u);
        CHECK(vision_align_test_take_turn_request());
        if(fault==0u)vision_align_test_notify_turn_result(0);
        else if(fault==1u){imu_valid=0;vision_align_test_notify_turn_result(1);}
        else {w=1.0f;vision_align_test_cancel();vision_align_test_notify_turn_result(1);}
        tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    return 1;
}
static int check_route_action_failure_and_cancel(void)
{
    for(unsigned fault=0u;fault<5u;fault++){
        CHECK(reset_route(41u));CHECK(ready());CHECK(finish(4,135,VAT_BALL_Y_PX));
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
        unsigned before=request_count;
        if(fault==0u)vision_align_test_notify_route_action_result(0);
        else if(fault==1u){aborted=1;vision_align_test_notify_route_action_result(1);}
        else if(fault==2u){imu_valid=0;vision_align_test_notify_route_action_result(1);}
        else if(fault==3u){yaw=NAN;vision_align_test_notify_route_action_result(1);}
        else {vision_align_test_cancel();vision_align_test_notify_route_action_result(1);}
        tick(20u);CHECK(st.state==VAT_STOPPED&&stopped()&&request_count==before);
        tick(10000u);vision_align_test_notify_route_action_result(1);tick(20u);
        CHECK(st.state==VAT_STOPPED&&stopped()&&request_count==before);
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE&&!vision_align_test_take_turn_request());
    }
    return 1;
}
static int check_route_search100_first_image_brake_and_fine(void)
{
    const unsigned modes[]={41u,40u};
    const int models[]={4,0},xs[]={135,215},ys[]={VAT_BALL_Y_PX,VAT_HOSTAGE_Y_PX};
    for(unsigned kind=0u;kind<2u;kind++)for(unsigned sticky=0u;sticky<2u;sticky++){
        CHECK(reset_route(modes[kind]));
        object(request(),models[kind],xs[kind],ys[kind],640u,480u,seq++);
        tick(1000u);CHECK(st.state==VAT_ALIGN&&stopped()&&!st.good&&!heading_hold_calls);
        ack((uint16_t)(request()-1u),2u,0u);tick(20u);CHECK(stopped());
        ack(request(),2u,0u);tick(20u);
        CHECK(route_ramp100(0.0f));CHECK(w==0.0f&&st.step==0u);
        tick(1000u);CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&st.step==0u&&!arm_calls);
        yaw=37.5f;leg_yaw=5.0f;tick(20u);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&w<0.0f&&fabsf(heading_hold_target-4.5f)<0.0001f);
        yaw=-322.5f;tick(20u);CHECK(w<0.0f&&fabsf(heading_hold_target-4.5f)<0.0001f);
        object((uint16_t)(request()-1u),models[kind],xs[kind],ys[kind],640u,480u,seq++);tick(20u);
        frame(models[kind]==4?0:4,190,ys[kind]);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&!st.good);
        yaw=37.0f;tick(20u);
        uint16_t first_sequence=seq;
        if(sticky){object(request(),models[kind],xs[kind],ys[kind],640u,480u,seq++);object(request(),-1,0,0,640u,480u,seq++);tick(20u);}
        else frame(models[kind],xs[kind],ys[kind]);
        CHECK(st.state==VAT_BRAKE&&stopped()&&!st.latest&&!st.good&&st.step==0u);
        tick(240u);CHECK(st.state==VAT_BRAKE&&stopped());tick(20u);
        CHECK(st.state==VAT_RECHECK&&stopped()&&!st.latest&&!st.good);
        tick(1000u);CHECK(st.state==VAT_RECHECK&&stopped()&&!st.good); /* Search image cannot prove arrival. */
        object(request(),models[kind],xs[kind],ys[kind],640u,480u,first_sequence);tick(20u);
        CHECK(st.state==VAT_RECHECK&&stopped()&&!st.good&&!st.latest); /* Same sequence in a newer packet is still old. */
        frame(models[kind],xs[kind]+11,ys[kind]);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f&&w==0.0f);
        CHECK(end_step_by_odometry());CHECK(stop_recheck());tick(1000u);
        CHECK(st.state==VAT_RECHECK&&stopped()); /* Once seen, missing frames never reopen100. */
        frame(models[kind],xs[kind],ys[kind]-11);CHECK(st.state==VAT_STEP_MOVE&&vy==-30.0f&&vx==0.0f);
        CHECK(end_step_by_odometry());CHECK(stop_recheck());
        CHECK(finish(models[kind],xs[kind],ys[kind]));
        CHECK(st.state==(kind?VAT_WAIT_HOSTAGE_ACTION:VAT_WAIT_BALL_ACTION)&&stopped());
        if(kind)CHECK(hostage_owner_complete(VAT_DONE));
        vision_align_test_cancel();
    }
    for(unsigned fault=0u;fault<6u;fault++){
        CHECK(reset_route(41u));CHECK(ready());tick(20u);CHECK(route_ramp100(0.0f));
        if(fault==0u)vision_align_test_cancel();
        else if(fault==1u)aborted=1;
        else if(fault==2u)imu_valid=0;
        else if(fault==3u)ack(request(),2u,1u);
        else if(fault==4u)yaw=NAN;
        else leg_yaw=NAN;
        tick(20u);CHECK(st.state==VAT_STOPPED&&stopped());tick(10000u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    return 1;
}
static int check_route_search_gain_and_late_yaw_recheck(void)
{
    const float invalid[]={-1.0f,6.0f,NAN,INFINITY,-INFINITY};
    CHECK(reset_route(41u));
    for(unsigned i=0u;i<sizeof invalid/sizeof invalid[0];i++)CHECK(!vision_align_test_route_search_kp_set(invalid[i]));
    CHECK(vision_align_test_route_search_kp_set(0.0f));CHECK(vision_align_test_route_search_kp_set(5.0f));
    CHECK(vision_align_test_route_search_kp_set(3.0f));
    CHECK(ready());tick(20u);CHECK(route_ramp100(0.0f));
    CHECK(!vision_align_test_route_search_kp_set(1.0f));yaw=37.5f;leg_yaw=5.0f;tick(20u);
    CHECK(heading_hold_kp==3.0f&&fabsf(w-(-0.5f*3.0f*0.0174533f))<0.00001f&&step_heading_kp_deg()==0.3f);
    yaw=37.0f;frame(4,135,VAT_BALL_Y_PX);CHECK(st.state==VAT_BRAKE&&stopped()&&st.yaw_ever);
    yaw=37.5f;tick(260u);CHECK(st.state==VAT_YAW_FIX&&st.yaw_dirty&&stopped());
    tick(20u);CHECK(w<0.0f&&vx==0.0f&&vy==0.0f);CHECK(yaw_settle());
    tick(1000u);CHECK(st.state==VAT_RECHECK&&stopped()&&!st.good);CHECK(finish(4,135,VAT_BALL_Y_PX));
    CHECK(st.state==VAT_WAIT_BALL_ACTION);vision_align_test_cancel();

    CHECK(reset_route(40u));CHECK(vision_align_test_route_search_kp_set(0.0f));
    CHECK(ready());tick(20u);CHECK(route_ramp100(0.0f));yaw=37.5f;tick(20u);
    CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&w==0.0f&&heading_hold_kp==0.0f);
    vision_align_test_cancel();
    for(unsigned mode=38u;mode<=41u;mode++){
        CHECK(reset(mode));CHECK(!vision_align_test_route_search_kp_set(3.0f));vision_align_test_cancel();
    }
    /* A target IRQ may arrive between start_route and the owner's setter.
     * It has not driven SEARCH yet, so configuring the route gain stays legal. */
    CHECK(reset_route(41u));ack(request(),2u,0u);
    object(request(),4,135,VAT_BALL_Y_PX,640u,480u,seq++);
    CHECK(vision_align_test_route_search_kp_set(3.0f));tick(260u);
    CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good&&!st.yaw_ever&&heading_hold_calls==0u);
    tick(20u); /* The first owner poll only consumes the IRQ's frame and brakes. */
    CHECK(st.state==VAT_RECHECK&&stopped()&&!st.good&&!st.yaw_ever&&heading_hold_calls==0u);
    CHECK(finish(4,135,VAT_BALL_Y_PX));CHECK(st.state==VAT_WAIT_BALL_ACTION);vision_align_test_cancel();
    return 1;
}
static int check_route_search_threshold_right_ff_and_isolation(void)
{
    const unsigned modes[]={41u,40u};
    const int models[]={4,0},xs[]={135,215},ys[]={VAT_BALL_Y_PX,VAT_HOSTAGE_Y_PX};
    const float invalid[]={-0.01f,0.1001f,NAN,INFINITY,-INFINITY};
    CHECK(VAT_ROUTE_FINE_ERROR_PX==30&&VAT_ROUTE_BUCKET_FINE_ERROR_PX==30);
    for(unsigned kind=0u;kind<2u;kind++)for(unsigned sticky=0u;sticky<2u;sticky++){
        CHECK(reset_route(modes[kind]));
        for(unsigned i=0u;i<sizeof invalid/sizeof invalid[0];i++)
            CHECK(!vision_align_test_route_search_ff_set(invalid[i]));
        CHECK(vision_align_test_route_search_ff_set(0.0f));
        CHECK(vision_align_test_route_search_ff_set(0.1f));
        CHECK(vision_align_test_route_search_ff_set(0.05f));
        /* Pre-ACK pixels cannot latch fine or start any approach. */
        object(request(),models[kind],xs[kind]+29,ys[kind],640u,480u,seq++);
        tick(1000u);CHECK(stopped()&&!st.good);
        ack(request(),2u,0u);tick(20u);
        CHECK(route_ramp100(0.05f));CHECK(!st.step);
        CHECK(!vision_align_test_route_search_ff_set(0.0f));
        frame(models[kind],xs[kind]+30,ys[kind]);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&vy==5.0f);
        frame(models[kind],400,ys[kind]);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&vy==5.0f&&!st.good);
        uint16_t far_sequence=(uint16_t)(seq-1u);
        object(request(),models[kind],xs[kind]+29,ys[kind],640u,480u,far_sequence);tick(20u);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&vy==5.0f); /* duplicate */
        object(request(),models[kind],xs[kind]+29,ys[kind],640u,480u,(uint16_t)(far_sequence-1u));tick(20u);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&vy==5.0f); /* backwards */
        object((uint16_t)(request()-1u),models[kind],xs[kind]+29,ys[kind],640u,480u,seq++);tick(20u);
        frame(models[kind]==4?0:4,xs[kind]+29,ys[kind]);
        object(request(),models[kind],xs[kind]+29,480,640u,480u,seq++);tick(20u);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&vy==5.0f); /* old request/class/illegal cy */
        frame(-1,0,0);tick(1000u);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&vy==5.0f); /* No valid crossing yet. */
        object(request(),models[kind],xs[kind]+29,ys[kind],640u,480u,seq++);
        if(sticky)object(request(),-1,0,0,640u,480u,seq++);
        tick(20u);CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good&&!st.latest);
        CHECK(stop_recheck());tick(1000u);CHECK(st.state==VAT_RECHECK&&stopped());
        frame(models[kind],400,ys[kind]);
        CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f&&w==0.0f);
        CHECK(end_step_by_odometry());CHECK(stop_recheck());
        frame(-1,0,0);tick(1000u);CHECK(st.state==VAT_RECHECK&&stopped());
        frame(models[kind],xs[kind]+29,ys[kind]);
        CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f);
        CHECK(!vision_align_test_route_search_ff_set(0.05f));
        CHECK(end_step_by_odometry());CHECK(stop_recheck());
        CHECK(finish(models[kind],xs[kind],ys[kind]));
        CHECK(st.state==(kind?VAT_WAIT_HOSTAGE_ACTION:VAT_WAIT_BALL_ACTION));
        if(kind)CHECK(hostage_owner_complete(VAT_DONE));
        vision_align_test_cancel();
    }
    /* Every new route starts with FF0; before-coarse IRQ evidence still allows
     * its owner to configure, but cannot restart100 after the relative gate. */
    CHECK(reset_route(40u));CHECK(ready());tick(20u);
    CHECK(route_ramp100(0.0f));vision_align_test_cancel();
    CHECK(reset_route(41u));ack(request(),2u,0u);
    object(request(),4,164,VAT_BALL_Y_PX,640u,480u,seq++);
    CHECK(vision_align_test_route_search_ff_set(0.05f));tick(260u);
    CHECK(st.state==VAT_BRAKE&&stopped()&&!st.yaw_ever&&!heading_hold_calls);
    tick(20u);CHECK(st.state==VAT_RECHECK&&stopped());
    frame(4,400,VAT_BALL_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f);
    vision_align_test_cancel();
    /* Standalone38..41 retain immediate fine20, no threshold/FF/100 mode. */
    for(unsigned mode=38u;mode<=41u;mode++){
        CHECK(reset(mode));CHECK(!vision_align_test_route_search_ff_set(0.05f));CHECK(ready());
        int model=mode==39u?9:mode==40u?0:4;
        int y=mode==39u?VAT_BUCKET_Y_PX:mode==40u?VAT_HOSTAGE_Y_PX:VAT_BALL_Y_PX;
        frame(model,400,y);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f);
        vision_align_test_cancel();
    }
    /* Paired bucket inherits the owner's cruise but never its lateral FF. */
    CHECK(reset_route(41u));CHECK(vision_align_test_route_search_ff_set(0.05f));
    CHECK(vision_align_test_route_search_speed_set(200.0f));
    CHECK(ready());CHECK(finish(4,135,VAT_BALL_Y_PX));
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
    vision_align_test_notify_route_action_result(1);CHECK(vision_align_test_take_turn_request());
    yaw=217.0f;vision_align_test_notify_turn_result(1);proto_service();
    CHECK(!vision_align_test_route_search_ff_set(0.05f));ack(request(),2u,0u);tick(260u);
    CHECK(st.state==VAT_RECHECK&&stopped());tick(1000u);CHECK(stopped());
    frame(9,400,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==0.0f&&vy==0.0f);
    tick(20u);CHECK(fabsf(vx+14.0f)<0.00001f&&vy==0.0f);
    tick(260u);CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&fabsf(vx+196.0f)<0.001f&&vy==0.0f);
    frame(9,400,VAT_BUCKET_Y_PX);CHECK(vx==-200.0f&&vy==0.0f);
    frame(9,154,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_BRAKE&&stopped());CHECK(stop_recheck());
    frame(9,400,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f);
    vision_align_test_cancel();
    puts("route coarse: ball/hostage goal+30 stayscoarse,goal+29 latchesfine; stale/empty/oldrequest/class do not latch; FF and standalone isolation; rank1bucket far400 backward/owner200/noFF then abs(error)<30 fine20 passed");
    return 1;
}

static int check_route_signed_coarse_strict_boundaries(void)
{
    const unsigned modes[]={41u,40u};
    const int models[]={4,0},goals[]={135,215},ys[]={VAT_BALL_Y_PX,VAT_HOSTAGE_Y_PX};
    for(unsigned kind=0u;kind<2u;kind++)for(unsigned side=0u;side<2u;side++){
        int direction=side?1:-1;
        CHECK(reset_route(modes[kind]));CHECK(vision_align_test_route_search_ff_set(0.05f));
        CHECK(ready());tick(20u);CHECK(route_ramp100(0.05f));
        /* Both exact30 boundaries are coarse; only a NEW accepted selected
         * image can reverse cruise, and its ramp restarts at zero. */
        frame(models[kind],goals[kind]-30,ys[kind]);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==0.0f&&vy==0.0f&&!st.good);
        tick(20u);CHECK(fabsf(vx+14.0f)<0.00001f&&vy==0.0f);
        tick(20u);CHECK(fabsf(vx+28.0f)<0.00001f&&vy==0.0f);
        tick(120u);CHECK(st.state==VAT_ROUTE_SEARCH&&vx==-100.0f&&vy==0.0f);
        uint16_t last=(uint16_t)(seq-1u);
        object(request(),models[kind],goals[kind]+29,ys[kind],640u,480u,last);tick(20u);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==-100.0f&&vy==0.0f); /* duplicate */
        object(request(),models[kind],goals[kind]+29,ys[kind],640u,480u,(uint16_t)(last-1u));tick(20u);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==-100.0f&&vy==0.0f); /* backwards */
        frame(models[kind],goals[kind]+30,ys[kind]);
        CHECK(st.state==VAT_ROUTE_SEARCH&&vx==0.0f&&vy==0.0f);
        tick(20u);CHECK(fabsf(vx-14.0f)<0.00001f&&fabsf(vy-0.7f)<0.00001f);
        tick(20u);CHECK(fabsf(vx-28.0f)<0.00001f&&fabsf(vy-1.4f)<0.00001f);
        tick(120u);CHECK(st.state==VAT_ROUTE_SEARCH&&vx==100.0f&&vy==5.0f);
        /* +/-29 latches fine but is outside unchanged +/-10 arrival. An
         * empty packet after the crossing cannot erase the fine latch. */
        object(request(),models[kind],goals[kind]+29*direction,ys[kind],640u,480u,seq++);
        object(request(),-1,0,0,640u,480u,seq++);tick(20u);
        CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good);CHECK(stop_recheck());
        frame(models[kind],goals[kind]+29*direction,ys[kind]);
        CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f*(float)direction&&vy==0.0f&&!st.good);
        CHECK(end_step_by_odometry());CHECK(stop_recheck());
        CHECK(finish(models[kind],goals[kind],ys[kind]));
        CHECK(st.state==(kind?VAT_WAIT_HOSTAGE_ACTION:VAT_WAIT_BALL_ACTION)&&stopped());
        if(kind)CHECK(hostage_owner_complete(VAT_DONE));
        vision_align_test_cancel();
    }
    puts("route coarse signed: +/-30 staycoarse, +/-29 brake/stickyfine but not arrival; accepted sequence alone reverses ramp0/14/28, positive-onlyFF, negativeFF0, fresh/still final5frames passed");
    return 1;
}
static int route_bucket_goal_ready(void)
{
    CHECK(reset_route(41u));CHECK(st.x_goal==135);CHECK(ready());
    CHECK(finish(4,135,VAT_BALL_Y_PX));CHECK(st.state==VAT_WAIT_BALL_ACTION&&st.x_goal==135);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
    vision_align_test_notify_route_action_result(1);vision_align_test_status(&st);
    CHECK(st.state==VAT_TURN_REQUEST&&st.task==PROTO_TASK_BUCKET&&st.x_goal==125);
    uint16_t before_turn=request();CHECK(vision_align_test_take_turn_request());
    yaw=217.0f;vision_align_test_notify_turn_result(1);proto_service();vision_align_test_status(&st);
    CHECK(st.state==VAT_BRAKE&&request()!=before_turn&&st.x_goal==125);
    ack(request(),2u,0u);tick(260u);CHECK(st.state==VAT_RECHECK&&stopped());
    return 1;
}
static int check_route_bucket_strict_boundaries_and_rank_direction(void)
{
    for(unsigned rank=1u;rank<=3u;rank++)for(unsigned side=0u;side<2u;side++){
        int pixel_direction=side?1:-1;
        int cruise_direction=rank==1u?-1:rank==3u?1:pixel_direction;
        CHECK(reset_route(41u));route_auto_ball_rank=(int)rank;
        CHECK(vision_align_test_route_search_ff_set(0.05f));
        CHECK(ready());CHECK(finish(4,135,VAT_BALL_Y_PX));
        CHECK(st.ball_rank==rank&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
        vision_align_test_notify_route_action_result(1);CHECK(vision_align_test_take_turn_request());
        yaw=217.0f;vision_align_test_notify_turn_result(1);proto_service();
        ack(request(),2u,0u);tick(260u);CHECK(st.state==VAT_RECHECK&&stopped());
        frame(9,125+30*pixel_direction,VAT_BUCKET_Y_PX);
        CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==0.0f&&vy==0.0f&&!st.good);
        tick(20u);CHECK(fabsf(vx-14.0f*(float)cruise_direction)<0.00001f&&vy==0.0f);
        tick(20u);CHECK(fabsf(vx-28.0f*(float)cruise_direction)<0.00001f&&vy==0.0f);
        tick(120u);CHECK(vx==100.0f*(float)cruise_direction&&vy==0.0f&&st.state==VAT_ROUTE_BUCKET_SEARCH);
        object(request(),9,125+29*pixel_direction,VAT_BUCKET_Y_PX,640u,480u,seq++);
        object(request(),-1,0,0,640u,480u,seq++);tick(20u);
        CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good);CHECK(stop_recheck());
        frame(9,125+29*pixel_direction,VAT_BUCKET_Y_PX);
        CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f*(float)pixel_direction&&vy==0.0f&&!st.good);
        CHECK(end_step_by_odometry());CHECK(stop_recheck());
        CHECK(finish(9,125,VAT_BUCKET_Y_PX));CHECK(st.state==VAT_WAIT_BUCKET_ACTION&&stopped());
        vision_align_test_cancel();
    }
    puts("bucket coarse: +/-30 remaincoarse, +/-29 stickyfine; rank1back/rank3forward/rank2coords, all noFF; fine usespixel sign and retains +/-10/fivefresh arrival passed");
    return 1;
}
static int check_route_task_x_goals_and_standalone_isolation(void)
{
    CHECK(VAT_X_PX==190&&VAT_ROUTE_BALL_X_PX==135&&VAT_ROUTE_BUCKET_X_PX==125&&VAT_ROUTE_HOSTAGE_X_PX==215);
    CHECK(VAT_TOL_PX==10&&VAT_ROUTE_BUCKET_FINE_ERROR_PX==30);
    const int goals[]={215,125},boundaries[][2]={{205,225},{115,135}},outside[][2]={{204,226},{114,136}};
    for(unsigned bucket=0u;bucket<2u;bucket++)for(unsigned edge=0u;edge<2u;edge++){
        if(bucket){CHECK(route_bucket_goal_ready());frame(9,125,VAT_BUCKET_Y_PX);
            CHECK(st.state==VAT_BRAKE);CHECK(stop_recheck());}
        else {CHECK(reset_route(40u));CHECK(st.x_goal==215);CHECK(!st.alignment_confirmed);CHECK(ready());}
        CHECK(finish(bucket?9:0,boundaries[bucket][edge],bucket?VAT_BUCKET_Y_PX:VAT_HOSTAGE_Y_PX));
        CHECK(st.x_goal==goals[bucket]&&st.cx==boundaries[bucket][edge]&&!st.step&&stopped());
        CHECK(st.alignment_confirmed==(bucket?0u:1u));
        CHECK(st.state==(bucket?VAT_WAIT_BUCKET_ACTION:VAT_WAIT_HOSTAGE_ACTION));
        if(!bucket)CHECK(hostage_owner_complete(VAT_DONE));
        vision_align_test_cancel();
    }
    for(unsigned bucket=0u;bucket<2u;bucket++)for(unsigned edge=0u;edge<2u;edge++){
        if(bucket){CHECK(route_bucket_goal_ready());frame(9,125,VAT_BUCKET_Y_PX);
            CHECK(st.state==VAT_BRAKE);CHECK(stop_recheck());}
        else {
            CHECK(reset_route(40u));CHECK(ready());frame(0,215,VAT_HOSTAGE_Y_PX);
            CHECK(st.state==VAT_BRAKE);CHECK(stop_recheck());
        }
        frame(bucket?9:0,outside[bucket][edge],bucket?VAT_BUCKET_Y_PX:VAT_HOSTAGE_Y_PX);
        CHECK(st.x_goal==goals[bucket]&&st.state==VAT_STEP_MOVE&&vx==(edge?20.0f:-20.0f)&&vy==0.0f&&!st.good);
        CHECK(!st.alignment_confirmed);
        vision_align_test_cancel();
    }
    /* Route31/43 use ball135. Both inclusive edges and one pixel
     * outside each edge use the private goal; old190 must never prove arrival. */
    const int ball_edges[]={125,145},ball_outside[]={124,146};
    for(unsigned edge=0u;edge<2u;edge++){
        CHECK(reset_route(41u));CHECK(st.x_goal==135);CHECK(ready());
        CHECK(finish(4,ball_edges[edge],VAT_BALL_Y_PX));
        CHECK(st.x_goal==135&&st.cx==ball_edges[edge]&&st.state==VAT_WAIT_BALL_ACTION&&stopped());
        vision_align_test_cancel();
        CHECK(reset_route(41u));CHECK(ready());frame(4,135,VAT_BALL_Y_PX);
        CHECK(st.state==VAT_BRAKE);CHECK(stop_recheck());
        frame(4,ball_outside[edge],VAT_BALL_Y_PX);
        CHECK(st.x_goal==135&&st.state==VAT_STEP_MOVE&&vx==(edge?20.0f:-20.0f)&&vy==0.0f&&!st.good);
        vision_align_test_cancel();
    }
    CHECK(reset_route(41u));CHECK(ready());frame(4,164,VAT_BALL_Y_PX);
    CHECK(st.state==VAT_BRAKE);CHECK(stop_recheck());
    for(unsigned i=0u;i<VAT_GOOD_FRAMES;i++){
        frame(4,190,VAT_BALL_Y_PX);
        CHECK(st.x_goal==135&&st.state==VAT_STEP_MOVE&&vx==20.0f&&!st.good);
    }
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    CHECK(end_step_by_odometry());CHECK(stop_recheck());
    for(unsigned i=1u;i<=VAT_GOOD_FRAMES;i++){
        frame(4,135,VAT_BALL_Y_PX);CHECK(stopped()&&st.good==i);
        CHECK(st.state==(i<VAT_GOOD_FRAMES?VAT_ALIGN:VAT_WAIT_BALL_ACTION));
    }
    CHECK(st.x_goal==135&&st.cx==135);
    vision_align_test_cancel();
    /* All independent entries still keep the original190 +/-10 window. */
    for(unsigned mode=38u;mode<=41u;mode++){
        CHECK(reset(mode));CHECK(st.x_goal==190);CHECK(ready());
        CHECK(finish(mode==39u?9:mode==40u?0:4,200,
              mode==39u?VAT_BUCKET_Y_PX:mode==40u?VAT_HOSTAGE_Y_PX:VAT_BALL_Y_PX));
        CHECK(st.x_goal==190&&st.cx==200&&stopped());vision_align_test_cancel();
    }
    /* Hostage215 +/-10 requires226px; bucket125 +/-10 fits136px but NOT135.
     * Real geometry/final arrival use the current goal, never old bucket180. */
    CHECK(reset_route(40u));CHECK(ready());
    object(request(),0,215,VAT_HOSTAGE_Y_PX,226u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_BRAKE);CHECK(stop_recheck());
    for(unsigned i=0u;i<VAT_GOOD_FRAMES;i++){
        object(request(),0,215,VAT_HOSTAGE_Y_PX,226u,480u,seq++);tick(20u);CHECK(stopped());
    }
    CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&st.x_goal==215&&st.img_w==226u&&st.alignment_confirmed);
    CHECK(hostage_owner_complete(VAT_DONE));
    CHECK(reset_route(40u));CHECK(ready());
    object(request(),0,215,VAT_HOSTAGE_Y_PX,225u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&st.x_goal==215&&strcmp(st.reason,"IMAGE_GEOMETRY")==0&&stopped()&&!st.alignment_confirmed);
    CHECK(route_bucket_goal_ready());
    object(request(),9,125,VAT_BUCKET_Y_PX,136u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_BRAKE);CHECK(stop_recheck());
    for(unsigned i=0u;i<VAT_GOOD_FRAMES;i++){
        object(request(),9,125,VAT_BUCKET_Y_PX,136u,480u,seq++);tick(20u);CHECK(stopped());
    }
    CHECK(st.state==VAT_WAIT_BUCKET_ACTION&&st.x_goal==125&&st.img_w==136u);vision_align_test_cancel();
    CHECK(route_bucket_goal_ready());
    object(request(),9,125,VAT_BUCKET_Y_PX,135u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&st.x_goal==125&&strcmp(st.reason,"IMAGE_GEOMETRY")==0&&stopped()&&!st.bucket_seen);
    CHECK(reset(39u));CHECK(ready());
    object(request(),9,190,VAT_BUCKET_Y_PX,191u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&st.x_goal==190&&strcmp(st.reason,"IMAGE_GEOMETRY")==0&&stopped());
    puts("route taskX: ball135/125..145 inclusive,124/146 microstep,old190 cannot align,new135 needs5 fresh; bucket125/115..135 inclusive,114/136 microstep,width136 accepts/135 rejects;hostage215/205..225 inclusive,204/226 microstep,width226 accepts/225 rejects; both pairedbucketrequests updategoal; standalone38..41 remain190,geometry/finalarrival use currentgoal passed");
    return 1;
}
static int route_bucket_unacked_after_turn(void)
{
    CHECK(reset_route(41u));CHECK(vision_align_test_route_search_kp_set(3.0f));
    CHECK(vision_align_test_route_search_speed_set(200.0f));
    CHECK(ready());CHECK(finish(4,135,VAT_BALL_Y_PX));
    CHECK(st.ball_rank==1u&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
    vision_align_test_notify_route_action_result(1);proto_service();
    CHECK(stopped());ack(request(),2u,0u);tick(5000u);
    CHECK(st.state==VAT_TURN_REQUEST&&stopped()); /* Preturn request cannot search. */
    CHECK(vision_align_test_take_turn_request());w=1.2f;tick(5000u);
    CHECK(st.state==VAT_TURN_ACTIVE&&w==1.2f); /* Never overwrite external turn. */
    yaw=217.0f;vision_align_test_notify_turn_result(1);proto_service();vision_align_test_status(&st);
    CHECK(st.state==VAT_BRAKE&&st.task==PROTO_TASK_BUCKET&&st.x_goal==125&&stopped());
    return 1;
}
static int check_route_bucket_wait_search_sticky_and_loss(void)
{
    CHECK(VAT_ROUTE_BUCKET_SEARCH==VAT_ROUTE_SEARCH+1&&VAT_ROUTE_BUCKET_MISSING_MS==2000u);
    for(unsigned sticky=0u;sticky<2u;sticky++){
        CHECK(route_bucket_unacked_after_turn());uint16_t current=request();
        tick(5000u);CHECK(st.state==VAT_RECHECK&&stopped());
        ack((uint16_t)(current-1u),2u,0u);
        object((uint16_t)(current-1u),9,125,VAT_BUCKET_Y_PX,640u,480u,seq++);
        tick(5000u);CHECK(st.state==VAT_RECHECK&&stopped()&&!st.good);
        ack(current,2u,0u);tick(20u);
        tick(1999u);CHECK(st.state==VAT_RECHECK&&stopped());
        tick(1u);CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==0.0f&&vy==0.0f&&w==0.0f&&!st.step);
        tick(20u);CHECK(fabsf(vx+14.0f)<0.00001f&&vy==0.0f);
        tick(20u);CHECK(fabsf(vx+28.0f)<0.00001f&&vy==0.0f);
        tick(260u);CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==-200.0f&&vy==0.0f&&!st.step);
        yaw=217.5f;leg_yaw=5.0f;tick(20u);
        CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==-200.0f&&vy==0.0f&&w<0.0f);
        CHECK(fabsf(heading_hold_target-4.5f)<0.0001f&&heading_hold_kp==3.0f);
        object(current,0,180,VAT_HOSTAGE_Y_PX,640u,480u,seq++);tick(20u);
        CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==-200.0f); /* Wrong selected class. */
        object(current,9,154,VAT_BUCKET_Y_PX,640u,480u,seq++);
        if(sticky)object(current,-1,0,0,640u,480u,seq++);
        tick(1u);CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good&&st.yaw_dirty&&st.yaw_ever);
        CHECK(stop_recheck());CHECK(st.yaw_target==217.0f);
        tick(1000u);CHECK(st.state==VAT_RECHECK&&stopped()); /* Fine latch: never coarse again. */
        frame(9,136,VAT_BUCKET_Y_PX);CHECK(st.state==VAT_STEP_MOVE&&vx==20.0f&&vy==0.0f);
        CHECK(end_step_by_odometry());CHECK(stop_recheck());
        CHECK(finish(9,125,VAT_BUCKET_Y_PX));CHECK(st.state==VAT_WAIT_BUCKET_ACTION&&stopped()&&!st.bucket_fallback);
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BUCKET);
        vision_align_test_notify_route_action_result(1);tick(20u);CHECK(st.state==VAT_DONE&&stopped());
    }
    /* A target BEFORE coarse-search delay still arms seen-then-lost release,
     * not coarse restart or a fabricated alignment. */
    CHECK(route_bucket_unacked_after_turn());ack(request(),2u,0u);tick(260u);
    object(request(),9,125,VAT_BUCKET_Y_PX,640u,480u,seq++);
    object(request(),-1,0,0,640u,480u,seq++);tick(1u);
    CHECK(st.state==VAT_BRAKE&&stopped()&&!st.yaw_ever);CHECK(stop_recheck());
    tick(2000u);CHECK(st.state==VAT_BRAKE&&stopped()&&!st.good);
    tick(250u);CHECK(st.state==VAT_WAIT_BUCKET_ACTION&&st.bucket_fallback&&!st.alignment_confirmed&&!st.good&&stopped());
    vision_align_test_cancel();tick(5000u);CHECK(st.state==VAT_STOPPED&&stopped());
    /* Standalone39/41 bucket windows retain NO blind backward motion. */
    CHECK(reset(39u));CHECK(ready());tick(10000u);CHECK(st.state==VAT_ALIGN&&stopped());
    vision_align_test_cancel();CHECK(reset(41u));CHECK(ready());CHECK(finish(4,190,VAT_BALL_Y_PX));
    tick(5000u);CHECK(vision_align_test_take_turn_request());yaw=217.0f;
    vision_align_test_notify_turn_result(1);proto_service();ack(request(),2u,0u);tick(260u);
    tick(10000u);CHECK(st.state==VAT_RECHECK&&stopped());vision_align_test_cancel();
    puts("bucket31: completed180+newACK starts2s; rank1back owner200 ramp/heading/noFF; abs(error)<30 stickyfine+empty brakes/yaw/newframes; fine seenlost2s release/still250 never fakealigned or coarse restart; standalone39/41 isolated passed");
    return 1;
}
static int check_route_bucket_faults_and_cancel(void)
{
    for(unsigned fault=0u;fault<7u;fault++){
        CHECK(route_bucket_unacked_after_turn());ack(request(),2u,0u);tick(260u);tick(2000u);
        CHECK(st.state==VAT_ROUTE_BUCKET_SEARCH&&vx==0.0f);tick(300u);CHECK(vx==-200.0f);
        if(fault==0u)vision_align_test_cancel();
        else if(fault==1u)aborted=1;
        else if(fault==2u)imu_valid=0;
        else if(fault==3u)yaw=NAN;
        else if(fault==4u)leg_yaw=NAN;
        else if(fault==5u)ack(request(),2u,1u);
        else object(request(),9,100,VAT_BUCKET_Y_PX,135u,480u,seq++);
        tick(1u);CHECK(st.state==VAT_STOPPED&&stopped());
        vision_align_test_notify_turn_result(1);tick(5000u);CHECK(st.state==VAT_STOPPED&&stopped());
    }
    puts("bucket31: cancel/abort/IMU/nonfiniteleg/NACK/geometry stop, late turn notification cannot revive passed");
    return 1;
}
static int check_route_rank_wait_capture_and_independent(void)
{
    for(unsigned position=1u;position<=3u;position++){
        CHECK(reset_route(40u));route_auto_hostage_rank=0;CHECK(ready());
        frame(0,215,VAT_HOSTAGE_Y_PX);CHECK(st.state==VAT_BRAKE);CHECK(stop_recheck());
        for(unsigned i=0u;i<VAT_GOOD_FRAMES;i++)frame(0,215,VAT_HOSTAGE_Y_PX);
        ProtoWireDiag d;proto_wire_diag_get(&d);
        CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&st.good==5u&&st.alignment_confirmed&&!st.target_rank&&d.receiving&&stopped());
        CHECK(hostage_owner_complete(VAT_WAIT_HOSTAGE_RANK));
        proto_wire_diag_get(&d);
        CHECK(d.receiving&&st.target_rank==0u&&!strcmp(st.reason,"WAIT_HOSTAGE_RANK"));
        uint8_t slots[3]={0xffu,0xffu,0xffu};
        for(unsigned i=0u;i<position;i++)slots[i]=(uint8_t)(i+1u);
        slots[position-1u]=0u;
        uint16_t rank_sequence=seq;
        rank_frame(request(),rank_sequence,0u,(uint8_t)position,(uint8_t)position,slots[0],slots[1],slots[2]);
        tick(20u);proto_wire_diag_get(&d); /* Actual54 alone releases rank gate AFTER owner success. */
        CHECK(st.state==VAT_DONE&&st.target_rank==position&&st.rank_sequence==rank_sequence&&!d.receiving&&stopped());
        ProtoTargetRank closed;CHECK(!proto_target_rank_get(&closed));
        tick(10000u);CHECK(st.state==VAT_DONE&&st.target_rank==position&&stopped());
    }
    CHECK(reset_route(40u));route_auto_hostage_rank=0;CHECK(ready());
    frame(0,215,VAT_HOSTAGE_Y_PX);CHECK(stop_recheck());
    rank_frame(request(),seq,0u,0u,0u,0xffu,0xffu,0xffu);
    for(unsigned i=0u;i<VAT_GOOD_FRAMES;i++)frame(0,215,VAT_HOSTAGE_Y_PX);
    CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&!st.target_rank&&st.alignment_confirmed);
    CHECK(hostage_owner_complete(VAT_WAIT_HOSTAGE_RANK));
    CHECK(st.target_rank==0u&&!strcmp(st.reason,"WAIT_HOSTAGE_RANK"));
    vision_align_test_cancel();rank_frame(request(),seq,0u,1u,1u,0u,0xffu,0xffu);
    tick(1000u);CHECK(st.state==VAT_STOPPED&&stopped());
    /* Route BALL captures position54 independently of the coordinate stream. */
    CHECK(reset_route(41u));route_auto_ball_rank=0;CHECK(ready());
    uint16_t ball_sequence=seq;rank_frame(request(),ball_sequence,4u,2u,2u,3u,4u,0xffu);
    CHECK(finish(4,135,VAT_BALL_Y_PX));CHECK(st.ball_rank==2u&&st.target_rank==2u&&st.rank_sequence==ball_sequence);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
    vision_align_test_notify_route_action_result(1);vision_align_test_status(&st);
    CHECK(st.task==PROTO_TASK_BUCKET&&!st.target_rank&&!st.rank_sequence&&st.ball_rank==2u&&stopped());vision_align_test_cancel();
    CHECK(reset_route(41u));route_auto_ball_rank=0;CHECK(ready());CHECK(finish(4,135,VAT_BALL_Y_PX));CHECK(!st.target_rank&&!st.ball_rank);
    vision_align_test_cancel();
    CHECK(reset(40u));CHECK(ready());CHECK(finish(0,190,VAT_HOSTAGE_Y_PX));CHECK(st.state==VAT_DONE&&!st.target_rank);
    puts("rank54: routehostage pendingaction/WAIT_HOSTAGE_RANK; routeball captures54 and preserves ball_rank into bucket; standalone40 unchanged passed");
    return 1;
}
static int check_ball_rank_wait_late_domain_and_faults(void)
{
    for(unsigned position=1u;position<=3u;position++) {
        CHECK(reset_route(41u));route_auto_ball_rank=0;CHECK(ready());
        uint16_t ball_request=request();
        rank_frame(ball_request,65534u,4u,0u,0u,255u,255u,255u);
        CHECK(finish(4,135,VAT_BALL_Y_PX));CHECK(!st.ball_rank&&st.state==VAT_WAIT_BALL_ACTION);
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
        vision_align_test_notify_route_action_result(1);tick(20u);
        ProtoWireDiag d;proto_wire_diag_get(&d);
        CHECK(st.state==VAT_WAIT_BALL_RANK&&d.receiving&&stopped());
        tick(10000u);CHECK(st.state==VAT_WAIT_BALL_RANK&&stopped()&&!vision_align_test_take_turn_request());
        rank_frame((uint16_t)(ball_request-1u),65535u,4u,1u,1u,4u,255u,255u);
        rank_frame(ball_request,65535u,0u,1u,1u,0u,255u,255u);tick(20u);
        CHECK(st.state==VAT_WAIT_BALL_RANK&&!st.ball_rank&&stopped());
        uint8_t slots[3]={position==1u?4u:3u,position==2u?4u:position==1u?3u:5u,position==3u?4u:5u};
        rank_frame(ball_request,65535u,4u,(uint8_t)position,3u,slots[0],slots[1],slots[2]);tick(20u);proto_service();
        CHECK(st.state==VAT_TURN_REQUEST&&st.ball_rank==position&&st.task==PROTO_TASK_BUCKET&&request()!=ball_request&&stopped());
        uint16_t preturn=request();CHECK(vision_align_test_take_turn_request());yaw=217.0f;
        vision_align_test_notify_turn_result(1);proto_service();vision_align_test_status(&st);
        CHECK(request()!=preturn&&st.ball_rank==position&&st.state==VAT_BRAKE&&stopped());
        rank_frame(ball_request,0u,4u,1u,1u,4u,255u,255u);tick(20u);
        CHECK(st.ball_rank==position&&stopped());vision_align_test_cancel();
    }
    for(unsigned fault=0u;fault<5u;fault++) {
        CHECK(reset_route(41u));route_auto_ball_rank=0;CHECK(ready());CHECK(finish(4,135,VAT_BALL_Y_PX));
        CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_BALL);
        vision_align_test_notify_route_action_result(1);tick(20u);CHECK(st.state==VAT_WAIT_BALL_RANK);
        if(fault==0u)vision_align_test_cancel();
        else if(fault==1u)aborted=1;
        else if(fault==2u)imu_valid=0;
        else if(fault==3u)yaw=NAN;
        else ack(request(),2u,1u);
        rank_frame(request(),seq++,4u,1u,1u,4u,255u,255u);tick(20u);
        CHECK(st.state==VAT_STOPPED&&stopped()&&!vision_align_test_take_turn_request());
    }
    for(unsigned mode=38u;mode<=41u;mode++) {
        if(mode==39u||mode==40u)continue;
        CHECK(reset(mode));CHECK(ready());rank_frame(request(),seq,4u,1u,1u,4u,255u,255u);
        CHECK(finish(4,190,VAT_BALL_Y_PX));CHECK(st.ball_rank==0u&&st.target_rank==0u);
        vision_align_test_cancel();
    }
    puts("ball54 route-only: unknown/late/mixed request/domain stop; real1..3 preserve overtwo bucketrequests/turn; wrap accepted parser; rankwait cancel/abort/IMU/NACK; standalone38/41 isolated passed");
    return 1;
}

static int check_hostage_owner_waits_and_faults(void)
{
    for(unsigned phase=0u;phase<3u;phase++)for(unsigned fault=0u;fault<5u;fault++){
        CHECK(reset_route(40u));route_auto_hostage_rank=0;CHECK(ready());
        CHECK(finish(0,215,VAT_HOSTAGE_Y_PX));
        CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&st.alignment_confirmed&&!st.target_rank&&stopped());
        if(phase){
            CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_HOSTAGE);
            CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
        }
        if(phase==2u){vision_align_test_notify_route_action_result(1);tick(20u);CHECK(st.state==VAT_WAIT_HOSTAGE_RANK);}
        if(fault==0u)vision_align_test_cancel();
        else if(fault==1u)aborted=1;
        else if(fault==2u)imu_valid=0;
        else if(fault==3u)yaw=NAN;
        else ack(request(),2u,1u);
        tick(20u);CHECK(st.state==VAT_STOPPED&&stopped());
        vision_align_test_notify_route_action_result(1);tick(10000u);
        CHECK(st.state==VAT_STOPPED&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE&&stopped());
    }
    CHECK(reset_route(40u));route_auto_hostage_rank=0;CHECK(ready());CHECK(finish(0,215,VAT_HOSTAGE_Y_PX));
    counts[0]++;CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    tick(249u);CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE&&stopped());
    tick(1u);CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_HOSTAGE);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    vision_align_test_notify_route_action_result(0);tick(20u);
    CHECK(st.state==VAT_STOPPED&&stopped());
    /* A rank arriving while the owner still holds the mechanical action
     * must not shortcut its result or turn a second take into a new grab. */
    CHECK(reset_route(40u));route_auto_hostage_rank=0;CHECK(ready());CHECK(finish(0,215,VAT_HOSTAGE_Y_PX));
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_HOSTAGE);
    rank_frame(request(),seq++,0u,1u,1u,0u,255u,255u);
    frame(0,215,VAT_HOSTAGE_Y_PX);
    CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE&&stopped());
    vision_align_test_notify_route_action_result(1);tick(20u);
    CHECK(st.state==VAT_DONE&&st.target_rank==1u&&st.alignment_confirmed&&stopped());
    puts("VAT hostage owner:pending/taken/rankwait cancel/abort/IMU/NAN/NACK stop;take checksencoder249/250 once;failedresult stops;earlyrank+repeated01 cannot bypass owner result or repeatgrab passed");
    return 1;
}

static int check_hostage_loss_wrap_recovery_and_geometry(void)
{
    CHECK(VAT_ROUTE_HOSTAGE_LOST_MS==2000u&&VAT_WAIT_HOSTAGE_ACTION==16&&VAT_WAIT_HOSTAGE_RANK==17&&VAT_ROUTE_ACTION_HOSTAGE==3);
    CHECK(reset_route(40u));route_auto_hostage_rank=0;CHECK(ready());tick(20u);CHECK(route_ramp100(0.0f));
    now_ms=UINT32_MAX-1000u;
    object(request(),0,500,VAT_HOSTAGE_Y_PX,640u,480u,65534u);tick(20u);
    object(request(),0,500,VAT_HOSTAGE_Y_PX,640u,480u,65535u);tick(20u);
    object(request(),0,500,VAT_HOSTAGE_Y_PX,640u,480u,0u);tick(20u);
    CHECK(st.hostage_seen&&st.hostage_age_ms==20u&&!st.alignment_confirmed);
    object(request(),0,500,VAT_HOSTAGE_Y_PX,640u,480u,65535u); /* Backwards after wrap. */
    rank_frame(request(),1u,0u,0u,0u,255u,255u,255u);
    object(request(),-1,0,0,640u,480u,2u);
    tick(1979u);CHECK(st.hostage_age_ms==1999u&&st.state==VAT_ROUTE_SEARCH);
    tick(1u);CHECK(st.hostage_age_ms==2000u&&st.state==VAT_BRAKE&&stopped());
    tick(249u);CHECK(st.state==VAT_BRAKE&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    tick(1u);CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&st.hostage_fallback&&!st.alignment_confirmed&&!st.good);
    /* A new selected01 at the poll->take seam cancels an uncommitted
     * fallback even though the public pending action was already exposed. */
    object(request(),0,500,VAT_HOSTAGE_Y_PX,640u,480u,3u);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);vision_align_test_status(&st);
    CHECK(st.state==VAT_BRAKE&&!st.hostage_fallback&&st.hostage_age_ms==0u);
    tick(1999u);CHECK(st.state==VAT_RECHECK&&stopped());
    tick(1u);CHECK(st.state==VAT_BRAKE&&st.hostage_age_ms==2000u);
    tick(250u);CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&st.hostage_fallback);
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_HOSTAGE);
    object(request(),0,215,VAT_HOSTAGE_Y_PX,640u,480u,4u);tick(20u);
    CHECK(st.state==VAT_WAIT_HOSTAGE_ACTION&&st.hostage_fallback&&vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    vision_align_test_notify_route_action_result(1);tick(20u);
    CHECK(st.state==VAT_WAIT_HOSTAGE_RANK&&st.hostage_fallback&&!st.alignment_confirmed&&!st.good&&stopped());
    rank_frame(request(),5u,0u,1u,1u,0u,255u,255u);tick(20u);
    CHECK(st.state==VAT_DONE&&st.target_rank==1u&&st.hostage_fallback&&!st.alignment_confirmed&&!st.good&&stopped());
    /* Preserve the old illegal-cy stimulus: the parser rejects it before
     * delivery, so it cannot arm the lost-target timer. A protocol-valid
     * frame too narrow for the private goal must stop inside VAT instead. */
    CHECK(reset_route(40u));CHECK(ready());tick(20u);CHECK(route_ramp100(0.0f));
    object(request(),0,350,480,640u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_ROUTE_SEARCH&&!st.alignment_confirmed&&!st.hostage_seen&&vx==100.0f);
    object(request(),0,180,VAT_HOSTAGE_Y_PX,191u,480u,seq++);tick(20u);
    CHECK(st.state==VAT_STOPPED&&!st.alignment_confirmed&&!st.hostage_seen&&stopped());
    CHECK(vision_align_test_take_route_action()==VAT_ROUTE_ACTION_NONE);
    puts("VAT hostage loss:16/17 states/action3;sequence65534/65535/0 and tickwrap;empty/54/replay norefresh,1999/2000+249/250;pre-take recovery cancels,but committed coordinates do not;rankwait/DONE retainfallback andnever fakealignment;parserinvalidcy ignored/privategoalgeometry stops before timer passed");
    return 1;
}
int main(void)
{
    if(!check_qr_ack_direct_bucket_and_initial_search()||!check_fixed_step_direction_and_fresh_recheck()
    ||!check_step_distance_time_and_age_exits()||!check_pixels_sequences_geometry_and_still()
    ||!check_post_yaw_stability_final_gate_timeout()||!check_imu_faults_and_no_initial_correction()
    ||!check_cancellation_and_abort_nack()||!check_route_locked_qr_entry()
    ||!check_ball_turn_bucket_sequence(0)||!check_ball_turn_bucket_sequence(1)
    ||!check_route_action_failure_and_cancel()
    ||!check_route_search100_first_image_brake_and_fine()
    ||!check_route_search_gain_and_late_yaw_recheck()
    ||!check_route_search_threshold_right_ff_and_isolation()
    ||!check_route_signed_coarse_strict_boundaries()
    ||!check_route_bucket_strict_boundaries_and_rank_direction()
    ||!check_route_task_x_goals_and_standalone_isolation()
    ||!check_route_bucket_wait_search_sticky_and_loss()||!check_route_bucket_faults_and_cancel()
    ||!check_route_rank_wait_capture_and_independent()
    ||!check_ball_rank_wait_late_domain_and_faults()
    ||!check_hostage_owner_waits_and_faults()||!check_hostage_loss_wrap_recovery_and_geometry())return 1;
    puts("VAT realparser: bounded3mm/250ms X20/Y30 steps, no midstep reversal, signedaxis odo+age exits, stop250/newimage, postY-yaw700/minW/finalgate/seqhistory,±10/fivefresh/still,strict standalone QR/ACK/direct39/stickyseen/cancel/IMU/NACK/ODO,route40/41 lockedQR after RX close,41shared180/newheading,route-onlysearch100/headingbridge/firstimagebrake/newXY/no-reblind/searchfaults passed; no physical acceptance");return 0;
}
