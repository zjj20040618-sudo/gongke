/* Real Bluetooth38..41 dispatcher, VAT engine and CRC/request parser.
 * Pixels and IMU values are explicitly injected, not physical acceptance. */
#define ROUTE31_QR_FIXTURE_MAIN xy_prior_route_fixture_main
#include "route31_qr_gate_test.c"

static unsigned xy_seq;

static void xy_advance(unsigned milliseconds)
{
    for (unsigned elapsed = 0u; elapsed < milliseconds; elapsed += 20u) {
        host_tick += 20u; wire_poll();
    }
}

static void xy_qr(uint16_t request, const char *digits)
{
    unsigned count = (unsigned)strlen(digits), size = count ? 7u : 4u;
    uint8_t p[12] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8),
                    (uint8_t)size, 0u, 0x53u, 1u, 0u, (uint8_t)(count != 0u)};
    if (count == 3u) memcpy(p + 9u, digits, 3u);
    wire_feed(p, size + 5u, 0);
}

static void xy_obj_poll(uint16_t request, int model, int x, int y, int poll)
{
    unsigned inner_size = model < 0 ? 14u : 25u;
    uint8_t p[30] = {0x62u, (uint8_t)request, (uint8_t)(request >> 8),
                    (uint8_t)inner_size, 0u, 0x01u};
    route_wire_le16(p + 6u, xy_seq++);
    p[8] = (uint8_t)(model >= 0);
    route_wire_le16(p + 9u, 640u); route_wire_le16(p + 11u, 480u);
    if (model >= 0) {
        p[19] = (uint8_t)model; route_wire_le16(p + 20u, 900u);
        route_wire_le16(p + 22u, (unsigned)x); route_wire_le16(p + 24u, (unsigned)y);
        route_wire_le16(p + 26u, 24u); route_wire_le16(p + 28u, 24u);
    }
    wire_feed_raw(p, inner_size + 5u, 0);
    if(poll) wire_poll();
}
static void xy_obj(uint16_t request,int model,int x,int y)
{xy_obj_poll(request,model,x,y,1);}

static int xy_boot(unsigned mode)
{
    char command[8];
    CHECK(wire_boot() == 0);
    host_wire_diag_hook = real_proto_wire_diag_get;
    xy_seq = 1u; snprintf(command, sizeof command, "%u", mode);
    run_cmd(command); wire_sync();
    CHECK(s_xy_owner == mode && s_xy_state == XT_READY && stopped());
    if (mode == 39u) {
        CHECK(wire_commands == 1u && wire_request == 1u && !s_receiving && !s_due);
        CHECK(!proto_qr_get(NULL) && !host_target_calls);
    } else CHECK(wire_request == 2u && wire_opcode == 0x60u && wire_mode == 1u);
    return 0;
}

static int xy_start(unsigned mode)
{
    CHECK(xy_boot(mode) == 0);
    run_cmd("g"); wire_sync();
    if (mode != 39u) {
        CHECK(s_xy_state == XT_QR_WAIT && stopped());
        wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123");
    }
    CHECK(s_xy_state == XT_ACTIVE && wire_opcode == 0x63u && stopped());
    CHECK(wire_task == (mode == 39u ? 4u : mode == 40u ? 3u : 1u));
    CHECK(wire_digit == (mode == 39u ? 0u : mode == 40u ? 3u : 1u));
    wire_ack(wire_request, 2u, 0u); xy_advance(260u);
    CHECK(s_xy_snapshot.state==VAT_ALIGN && stopped());
    return 0;
}

static int xy_finish(int model, int y)
{
    CHECK(s_xy_snapshot.state==VAT_ALIGN || s_xy_snapshot.state==VAT_RECHECK);
    for (unsigned i = 0u; i < 120u && s_xy_state == XT_ACTIVE; ++i) {
        host_tick += 20u;
        xy_obj(wire_request, model, 190, y);
        vision_align_test_status(&s_xy_snapshot);
        if (s_xy_snapshot.state == VAT_HOLD_BALL || s_xy_snapshot.state == VAT_HOLD_BUCKET) break;
    }
    vision_align_test_status(&s_xy_snapshot);
    CHECK(s_xy_snapshot.state == VAT_HOLD_BALL || s_xy_snapshot.state == VAT_HOLD_BUCKET || s_xy_state == XT_DONE);
    CHECK(stopped()); return 0;
}
static int xy_yaw_settle(void)
{
    CHECK(s_xy_snapshot.state==VAT_YAW_FIX && s_xy_snapshot.yaw_dirty && s_xy_snapshot.yaw_ever);
    host_absolute_yaw=s_xy_snapshot.yaw_target;xy_advance(20u);
    CHECK(s_xy_snapshot.state==VAT_YAW_FIX && stopped());
    xy_advance(680u);CHECK(s_xy_snapshot.state==VAT_YAW_FIX && stopped());
    xy_advance(20u);
    CHECK(s_xy_snapshot.state==VAT_RECHECK && stopped() && !s_xy_snapshot.yaw_dirty);
    CHECK(!s_xy_snapshot.latest && !s_xy_snapshot.rx_fresh && !s_xy_snapshot.good);
    return 0;
}

static int check_fresh_qr_and_selected_tasks(void)
{
    for (unsigned mode = 38u; mode <= 41u; ++mode) {
        if (mode == 39u) continue; /* direct black bucket tested separately */
        CHECK(xy_boot(mode) == 0);
        wire_ack(1u, 1u, 0u); xy_qr(1u, "123"); run_cmd("g");
        CHECK(s_xy_state == XT_QR_WAIT && stopped() && !host_target_calls);
        wire_ack(2u, 1u, 0u); xy_qr(2u, ""); xy_advance(2000u);
        CHECK(s_xy_state == XT_QR_WAIT && stopped() && !host_target_calls);
        xy_qr(2u, "103"); CHECK(s_xy_state == XT_QR_WAIT && stopped());
        xy_qr(2u, "123");
        CHECK(s_xy_state == XT_ACTIVE && stopped() && host_target_calls == 1);
        CHECK(wire_task == (mode == 39u ? 4u : mode == 40u ? 3u : 1u));
        CHECK(wire_digit == (mode == 39u ? 0u : mode == 40u ? 3u : 1u));
        run_cmd("g"); CHECK(s_xy_state == XT_STOPPED && stopped() && !s_receiving && !s_due);
        uint16_t previous = wire_request;
        run_cmd("g"); wire_sync();
        CHECK(s_xy_state == XT_QR_WAIT && wire_request > previous && wire_opcode == 0x60u && stopped());
        xy_qr(2u, "123"); xy_advance(1000u);
        CHECK(s_xy_state == XT_QR_WAIT && stopped() && host_target_calls == 1);
        run_cmd("0"); CHECK(stopped());
    }
    CHECK(xy_boot(38u) == 0);
    wire_ack(wire_request, 1u, 0u); xy_qr(wire_request, "123");
    CHECK(s_xy_state == XT_READY && stopped() && !host_target_calls);
    run_cmd("g"); wire_poll();
    CHECK(s_xy_state == XT_ACTIVE && wire_task == 1u && stopped());
    puts("XY Bluetooth: fresh QR on38/40/41 selection/rerun, empty/illegal/old QR blocked; firstg preserves selectionQR; request1/3 routing passed");
    return 0;
}
static int check_bucket_direct_no_qr_and_restarts(void)
{
    static const char *const stop_keys[] = {"g","a","0"};
    for (unsigned k=0u; k<3u; ++k) {
        CHECK(xy_boot(39u) == 0);
        wire_ack(1u,1u,0u); xy_qr(1u,"123"); xy_advance(2000u);
        CHECK(s_xy_state == XT_READY && stopped() && wire_commands == 1u && !notice_calls);
        run_cmd("g"); wire_sync();
        CHECK(s_xy_state == XT_ACTIVE && wire_opcode == 0x63u && wire_task == 4u && !wire_digit);
        CHECK(wire_request == 2u && host_target_calls == 1 && !proto_qr_get(NULL));
        CHECK(!s_xy_qr[0] && !s_xy_qr[1] && !s_xy_qr[2]);
        uint16_t first=wire_request;
        xy_obj(first,9,230,400); xy_advance(1100u);
        CHECK(stopped() && !s_xy_snapshot.latest); /* coordinate alone is not an ACK */
        wire_ack(1u,2u,0u); CHECK(stopped());
        wire_ack(first,2u,0u); xy_advance(1000u);
        CHECK(s_xy_state == XT_ACTIVE && stopped() && !s_xy_snapshot.latest);
        host_tick += 20u; xy_obj(first,4,230,400); CHECK(stopped()); /* wrong class is not a bucket */
        host_tick += 20u; xy_obj(first,9,230,400);
        CHECK(last_x == 20.0f && last_y == 0.0f && s_xy_state == XT_ACTIVE);
        run_cmd(stop_keys[k]);
        CHECK(s_xy_state == XT_STOPPED && stopped() && !s_receiving && !s_due);
        unsigned commands_after_stop=wire_commands;
        wire_ack(first,2u,0u); xy_obj(first,9,230,400); xy_advance(1000u);
        CHECK(stopped() && s_xy_state == XT_STOPPED && wire_commands == commands_after_stop);
        run_cmd("g"); wire_sync();
        CHECK(s_xy_state == XT_ACTIVE && wire_request > first && wire_opcode == 0x63u);
        CHECK(wire_task == 4u && !wire_digit && !proto_qr_get(NULL));
        wire_ack(first,2u,0u); xy_obj(first,9,230,400); xy_advance(1100u);
        CHECK(stopped() && !s_xy_snapshot.latest && s_xy_state == XT_ACTIVE);
        wire_ack(wire_request,2u,0u); CHECK(xy_finish(9,400) == 0);
        CHECK(s_xy_state == XT_DONE && stopped());
        uint16_t completed_request=wire_request;
        run_cmd("g"); wire_sync();
        CHECK(s_xy_state == XT_ACTIVE && wire_request > completed_request && wire_opcode == 0x63u);
        CHECK(wire_task == 4u && !wire_digit && stopped());
        run_cmd("0"); CHECK(stopped());
    }
    puts("XY39 directbucket: no selectionQR, g task4/0 without QR, newACK/frame required, wrongclass/oldQR/oldrequest refused, g/a/0 stop and stopped/DONE newrequest reruns passed");
    return 0;
}

static int xy_end_step(void)
{
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_w==0.0f);
    CHECK((last_x!=0.0f)!=(last_y!=0.0f));
    if(last_x!=0.0f)host_fore+=last_x>0.0f?3.0f:-3.0f;
    else host_lateral+=last_y>0.0f?3.0f:-3.0f;
    xy_advance(20u);
    CHECK(s_xy_snapshot.state==VAT_BRAKE && stopped() && !s_xy_snapshot.step_capped);
    CHECK(fabsf(s_xy_snapshot.step_mm-3.0f)<0.0001f);return 0;
}
static int xy_stop_recheck(void)
{
    CHECK(s_xy_snapshot.state==VAT_BRAKE && stopped());
    xy_advance(240u);CHECK(s_xy_snapshot.state==VAT_BRAKE && stopped());xy_advance(20u);
    if(s_xy_snapshot.yaw_dirty){CHECK(s_xy_snapshot.state==VAT_YAW_FIX);CHECK(xy_yaw_settle()==0);}
    CHECK(s_xy_snapshot.state==VAT_RECHECK && stopped() && !s_xy_snapshot.latest && !s_xy_snapshot.good);return 0;
}
static int check_fixed_steps_and_coupled_xy(void)
{
    CHECK(VAT_X_SPEED_MMS==20.0f && VAT_Y_SPEED_MMS==30.0f && VAT_STEP_MM==3.0f && VAT_STEP_MAX_MS==250u);
    CHECK(xy_start(39u)==0);
    host_tick+=20u;xy_obj(wire_request,9,201,400);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==20.0f && last_w==0.0f && s_xy_snapshot.step==1u);
    host_tick+=20u;xy_obj(wire_request,9,179,400);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==20.0f && !s_xy_snapshot.good);
    host_tick+=20u;xy_obj(wire_request,-1,0,0);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==20.0f && last_w==0.0f && !s_xy_snapshot.good);
    CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);xy_advance(200u);CHECK(stopped());
    host_tick+=20u;xy_obj(wire_request,9,179,400);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==-20.0f && s_xy_snapshot.step==2u);
    CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);
    host_tick+=20u;xy_obj(wire_request,9,190,389);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_y==-30.0f && last_x==0.0f && last_w==0.0f);
    CHECK(s_xy_snapshot.yaw_dirty && s_xy_snapshot.yaw_ever);
    host_absolute_yaw=0.4f;
    host_tick+=20u;xy_obj(wire_request,9,190,411);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_y==-30.0f && last_w==0.0f);
    CHECK(xy_end_step()==0);xy_advance(260u);CHECK(s_xy_snapshot.state==VAT_YAW_FIX && stopped());
    xy_advance(20u);CHECK(fabsf(last_w+0.18f)<0.00001f && s_xy_snapshot.yaw_target==0.0f);
    host_tick+=20u;xy_obj(wire_request,9,190,400);
    CHECK(s_xy_snapshot.rx_fresh && !s_xy_snapshot.latest && !s_xy_snapshot.good);
    CHECK(xy_yaw_settle()==0);xy_advance(100u);CHECK(stopped());
    host_tick+=20u;xy_obj(wire_request,9,190,411);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_y==30.0f && last_w==0.0f && s_xy_snapshot.step==4u);
    CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);
    host_tick+=20u;xy_obj(wire_request,9,201,400);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==20.0f && last_y==0.0f); /* Y changed X */
    CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);
    CHECK(xy_finish(9,400)==0 && s_xy_state==XT_DONE && stopped() && !zero_calls && !laser_state);
    run_cmd("31");CHECK(s_xy_state==XT_OFF && s_seq_state==SQ_READY);
    run_cmd("0");run_cmd("35");CHECK(s_msel==35 && s_target35_phase==TA_READY && stopped());
    puts("XY Bluetooth: no midstep reversal/drop chatter,3mm stop/fresh recheck,X20/Y30 coupling,postY originalyaw/nozero,31/35 isolation passed");return 0;
}
static int check_step_cap_and_sticky_seen_search(void)
{
    CHECK(xy_start(39u)==0);host_tick+=20u;xy_obj(wire_request,9,201,400);
    xy_advance(240u);CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==20.0f);
    xy_advance(20u);CHECK(s_xy_snapshot.state==VAT_BRAKE && stopped() && s_xy_snapshot.step_capped && s_xy_snapshot.step_ms==260u);
    CHECK(xy_stop_recheck()==0);xy_advance(1000u);CHECK(stopped() && s_xy_snapshot.state==VAT_RECHECK);
    CHECK(xy_start(38u)==0);xy_advance(20u);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==20.0f);
    host_tick+=20u;xy_obj_poll(wire_request,4,280,333,0);xy_obj_poll(wire_request,-1,0,0,0);wire_poll();
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==20.0f && !s_xy_snapshot.good);
    CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);xy_advance(1000u);
    CHECK(s_xy_snapshot.state==VAT_RECHECK && stopped()); /* valid+empty samepoll cannot resume blind */
    host_tick+=20u;xy_obj(wire_request,4,280,333);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==20.0f && !s_xy_snapshot.good);
    CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);
    host_tick+=20u;xy_obj(wire_request,4,190,333);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_y==-30.0f);
    CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);
    CHECK(xy_finish(4,420)==0 && s_xy_state==XT_DONE && stopped());
    puts("XY Bluetooth: noodo260ms poll-quantized cap,stop/no-newframe wait,stickyvalid+empty forbidsblind,ball280333 not arrival passed");return 0;
}
static int check_locks_and_actual_phase_stops(void)
{
    CHECK(xy_start(39u)==0);host_tick+=20u;xy_obj(wire_request,9,190,389);
    static const char *const writes[]={"31","34","35","37","38","v100","d200","ykp1","fff0","co","cc","nr5","su1500"};
    for(unsigned i=0u;i<sizeof writes/sizeof writes[0];++i){
        run_cmd(writes[i]);CHECK(s_xy_owner==39u && s_xy_state==XT_ACTIVE && !servo_calls && !pulse_calls);
    }
    static const char *const keys[]={"g","a","0"};
    for(unsigned phase=0u;phase<5u;phase++)for(unsigned k=0u;k<3u;k++){
        CHECK(xy_start(39u)==0);
        if(phase>0u){host_tick+=20u;xy_obj(wire_request,9,phase==4u?190:201,phase==4u?389:400);CHECK(s_xy_snapshot.state==VAT_STEP_MOVE);}
        if(phase==2u)CHECK(xy_end_step()==0);
        if(phase==3u){CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);}
        if(phase==4u){CHECK(xy_end_step()==0);xy_advance(260u);CHECK(s_xy_snapshot.state==VAT_YAW_FIX);}
        run_cmd(keys[k]);xy_advance(6000u);
        CHECK(s_xy_state==XT_STOPPED && stopped() && !s_receiving && !s_due && !zero_calls && !prepare_calls);
    }
    puts("XY Bluetooth: locks and15 g/a/0 stops at actualALIGN/STEP/BRAKE/RECHECK/YAW states passed");return 0;
}
static int check_imu_odo_abort_nack_and_x_no_initial_yaw(void)
{
    const float faults[]={NAN,INFINITY,-INFINITY,0.0f};
    for(unsigned i=0u;i<4u;i++){
        CHECK(xy_boot(39u)==0);host_absolute_yaw=faults[i];host_imu_valid=i<3u;
        run_cmd("g");wire_sync();CHECK(s_xy_state==XT_STOPPED && stopped() && !host_target_calls);
        CHECK(xy_start(39u)==0);host_tick+=20u;xy_obj(wire_request,9,201,400);
        CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_x==20.0f);
        host_absolute_yaw=faults[i];host_imu_valid=i<3u;wire_poll();
        CHECK(s_xy_state==XT_STOPPED && stopped() && !s_receiving && !zero_calls);
    }
    CHECK(xy_start(39u)==0);host_absolute_yaw=20.0f;host_tick+=20u;xy_obj(wire_request,9,201,400);
    CHECK(last_x==20.0f && last_w==0.0f && !s_xy_snapshot.yaw_ever);
    CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);CHECK(xy_finish(9,400)==0 && s_xy_state==XT_DONE);
    CHECK(xy_start(39u)==0);host_tick+=20u;xy_obj(wire_request,9,201,400);host_fore=NAN;xy_advance(20u);
    CHECK(s_xy_state==XT_STOPPED && stopped() && !s_receiving);
    CHECK(xy_start(39u)==0);host_tick+=20u;xy_obj(wire_request,9,201,400);host_abort=1;wire_poll();
    CHECK(s_xy_state==XT_STOPPED && stopped());
    CHECK(xy_start(39u)==0);host_tick+=20u;xy_obj(wire_request,9,201,400);wire_ack(wire_request,2u,1u);
    CHECK(s_xy_state==XT_STOPPED && stopped());
    puts("XY Bluetooth: immediateIMU/ODO/abort/NACK stop bypasses boundedstep,finiteX noinitialyaw passed");return 0;
}
static int check_ball_turn_bucket_and_stops(void)
{
    CHECK(xy_start(41u)==0);uint32_t test_id=s_active_test;
    CHECK(xy_finish(4,420)==0);CHECK(s_xy_snapshot.state==VAT_HOLD_BALL);
    xy_advance(4980u);CHECK(s_xy_state==XT_ACTIVE && stopped());xy_advance(20u);
    CHECK(s_xy_state==XT_TURN_STILL && wire_task==4u && stopped());uint16_t before_turn=wire_request;
    wire_ack(before_turn,2u,0u);xy_obj(before_turn,9,190,400);
    xy_advance(1100u);CHECK(s_xy_state==XT_TURN_RUN && s_msel==22 && !prepare_calls);
    xy_advance(20u);CHECK(last_x==0.0f && last_y==0.0f && last_w>0.0f);
    host_yaw=host_absolute_yaw=180.0f;wire_poll();CHECK(s_round==R_BRAKE);xy_advance(T_TURN_SETTLE_MS);
    CHECK(s_xy_state==XT_ACTIVE && s_msel==41 && wire_request>before_turn && stopped());
    CHECK(s_xy_snapshot.yaw_target==180.0f && !s_xy_snapshot.yaw_dirty && !s_xy_snapshot.yaw_ever);
    CHECK(s_active_test==test_id && s_test_seq==test_id);
    xy_obj(before_turn,9,190,400);xy_advance(1000u);CHECK(s_xy_snapshot.state==VAT_RECHECK && stopped());
    wire_ack(wire_request,2u,0u);host_tick+=20u;xy_obj(wire_request,9,190,389);
    CHECK(s_xy_snapshot.state==VAT_STEP_MOVE && last_y==-30.0f && s_xy_snapshot.yaw_target==180.0f);
    host_absolute_yaw=180.4f;CHECK(xy_end_step()==0);CHECK(xy_stop_recheck()==0);
    CHECK(s_xy_snapshot.yaw_target==180.0f && zero_calls==1);
    CHECK(xy_finish(9,400)==0);CHECK(s_xy_snapshot.state==VAT_HOLD_BUCKET);
    xy_advance(5000u);CHECK(s_xy_state==XT_DONE && s_msel==41 && stopped() && !s_due && !s_receiving);
    CHECK(!prepare_calls && zero_calls==1 && !laser_state && !pulse_calls && !servo_calls && s_seq_state==SQ_OFF);
    static const char *const keys[]={"g","a","0"};
    static const unsigned phases[]={XT_QR_WAIT,XT_ACTIVE,XT_TURN_STILL,XT_TURN_WAIT,XT_TURN_RUN};
    for(unsigned p=0u;p<sizeof phases/sizeof phases[0];++p)for(unsigned k=0u;k<3u;k++){
        CHECK(xy_start(41u)==0);s_xy_state=(uint8_t)phases[p];if(phases[p]==XT_TURN_RUN)s_msel=22;
        run_cmd(keys[k]);xy_advance(6000u);CHECK(s_xy_state==XT_STOPPED && s_msel==41 && stopped() && !s_receiving && !s_due);
    }
    puts("XY Bluetooth:41 hold5/shared22+180/newrequest+heading/bucketYshortsteps-postyaw/hold5STOP,testid+owner/cancel retained passed");return 0;
}
static int check_turn_done_failure_is_not_success(void)
{
    CHECK(xy_start(41u)==0);CHECK(xy_finish(4,420)==0);xy_advance(5000u);
    CHECK(s_xy_state==XT_TURN_STILL && stopped());wire_ack(wire_request,2u,0u);xy_advance(1100u);
    CHECK(s_xy_state==XT_TURN_RUN && s_msel==22);xy_advance(20u);CHECK(last_w>0.0f);
    host_messages[0]='\0';host_imu_valid=0;xy_trial_turn_done(1);
    CHECK(s_xy_state==XT_STOPPED && s_msel==41 && stopped() && !s_receiving && !s_due);
    CHECK(s_xy_snapshot.state==VAT_STOPPED && strcmp(s_xy_snapshot.reason,"TURN180_FAILED")==0);
    CHECK(strstr(host_messages,"OK XY_TURN180_DONE")==NULL);xy_advance(5000u);CHECK(stopped());
    puts("XY Bluetooth:mode22DONE then badIMU bucketcapture rejects falseOK passed");return 0;
}
int main(void)
{
    CHECK(check_fresh_qr_and_selected_tasks()==0);
    CHECK(check_bucket_direct_no_qr_and_restarts()==0);
    CHECK(check_fixed_steps_and_coupled_xy()==0);
    CHECK(check_step_cap_and_sticky_seen_search()==0);
    CHECK(check_locks_and_actual_phase_stops()==0);
    CHECK(check_imu_odo_abort_nack_and_x_no_initial_yaw()==0);
    CHECK(check_ball_turn_bucket_and_stops()==0);
    CHECK(check_turn_done_failure_is_not_success()==0);
    puts("vision_align_bluetooth_test:boundedstep host checks passed; no physical camera/laser/arm/route acceptance");return 0;
}
