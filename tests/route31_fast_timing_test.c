/* Real31 router/VAT/CRC parser timing boundaries, with host-only time/IMU.
 * Synthetic task/road entries isolate executors; this is not a full route or
 * physical stop/laser/arm acceptance. Retained35/43 timings are comparisons. */
#define VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN fast31_prior_xy_main
#include "vision_align_bluetooth_test.c"
#undef VISION_ALIGN_BLUETOOTH_FIXTURE_MAIN

static VisionAlignTestStatus fast_status;
static unsigned fast_fault;

static int fast_stopped(void)
{ return !last_x && !last_y && !last_w && !laser_state && !host_timer_active; }

static void fast_snapshot(void) { vision_align_test_status(&fast_status); }

static int fast_prepare(unsigned owner, unsigned stage)
{
    char command[8]; int32_t qr[3];
    CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
    snprintf(command,sizeof command,"%u",owner); run_cmd(command); wire_sync();
    wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
    CHECK(proto_qr_get(qr) && qr[0] == 1 && qr[1] == 2 && qr[2] == 3);
    run_cmd("g"); CHECK(s_seq_state == SQ_STILL && fast_stopped());
    memcpy(s_seq_qr,qr,sizeof qr); step_vision_receive_end();
    if (owner == 31u && stage == 4u) {
        /* Contact timing starts AFTER a legitimate cross reference/yaw phase. */
        CHECK(fixture_prepare_route31_board() == 0);
    } else {
        s_seq_stage = (uint8_t)stage; route_seq_prepare();
    }
    wire_sync();
    CHECK(s_seq_mode == owner && fast_stopped());
    return 0;
}

static int fast_start_road(unsigned owner, unsigned stage)
{
    CHECK(fast_prepare(owner,stage) == 0 && s_seq_state == SQ_STILL);
    CHECK(sequence_start_stage() == 0 && s_seq_state == SQ_RUN);
    return 0;
}

static int check_fast_private_cross_distance(void)
{
    CHECK(fast_prepare(31u,3u) == 0);
    CHECK(route_seq_leg()->distance_mm == 620u && s_d == 620.0f);
    run_cmd("0");
    CHECK(fast_prepare(43u,3u) == 0);
    CHECK(route_seq_leg()->distance_mm == 650u && s_d == 650.0f);
    run_cmd("0");
    CHECK(fast_prepare(31u,7u) == 0);
    CHECK(route_seq_leg()->mode == 16u && route_seq_leg()->distance_mm == 805u && s_d == 805.0f);
    run_cmd("0");
    CHECK(fast_prepare(43u,7u) == 0);
    CHECK(route_seq_leg()->mode == 16u && route_seq_leg()->distance_mm == 760u && s_d == 760.0f);
    run_cmd("0");
    puts("31 private cross620/BACK805 vs43 retained650/BACK760 prepare distance isolation passed");
    return 0;
}

static int check_fast_road_holds(void)
{
    CHECK(ROUTE31_STABLE_MS == 400u && ROUTE31_BOARD_ZERO_WAIT_MS == 300u &&
          T_DIST_ALIGN_STABLE_MS == 700u && T_TURN_SETTLE_MS == 700u &&
          ROUTE31_BOARD_CONTACT_ZERO_WAIT_MS == 1000u);
    const unsigned owners[] = {31u,43u};
    for (unsigned k = 0u; k < 2u; ++k) {
        unsigned hold = k ? 700u : 400u;
        CHECK(fast_start_road(owners[k],1u) == 0 && dist_align_stable_ms() == hold);
        host_fore = s_dist_odo0 + s_dist_target; wire_poll();
        CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN);
        host_tick += T_DIST_STILL_MS; wire_poll();
        CHECK(s_round == R_ALIGN); host_yaw = s_dist_heading0; wire_poll();
        CHECK(s_dist_align_hold && fast_stopped());
        uint32_t origin = s_dist_align_stable_t0;
        host_tick = origin + hold - 1u; wire_poll();
        CHECK(s_round == R_ALIGN && s_seq_state == SQ_RUN);
        ++host_tick; wire_poll();
        CHECK(s_round != R_ALIGN && s_seq_state != SQ_RUN && fast_stopped());

        const unsigned turn_stages[] = {2u, ROUTE31_HOSTAGE_TURN_STAGE};
        for (unsigned leg = 0u; leg < 2u; ++leg) {
            CHECK(fast_start_road(owners[k],turn_stages[leg]) == 0 && turn_settle_ms() == hold);
            CHECK(turn_target_deg() == (!k && leg ? 93.0f : 90.0f));
            host_yaw = turn_target_deg(); wire_poll();
            CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN);
            origin = s_turn_settle_t0;
            host_tick = origin + hold - 1u; wire_poll();
            CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && fast_stopped());
            host_messages[0] = '\0';
            host_tick++; wire_poll();
            CHECK(s_round != R_BRAKE && s_seq_state != SQ_RUN && fast_stopped());
            CHECK(strstr(host_messages, !k && leg ? "cmd_deg=93" : "cmd_deg=90") != NULL);
        }
    }
    /* Continuous stability, not merely time since first arrival. */
    CHECK(fast_start_road(31u,2u) == 0);
    float goal = turn_target_deg(); host_yaw = goal + 0.29f; wire_poll();
    CHECK(s_round == R_BRAKE); host_tick += 200u; host_yaw = goal; wire_poll();
    CHECK(s_turn_settle_t0 == host_tick && s_round == R_BRAKE);
    host_tick += 399u; wire_poll(); CHECK(s_round == R_BRAKE);
    ++host_tick; wire_poll(); CHECK(s_round != R_BRAKE);
    run_cmd("0");
    CHECK(fast_start_road(31u,1u) == 0);
    host_fore = s_dist_odo0 + s_dist_target; wire_poll();
    host_tick += T_DIST_STILL_MS; wire_poll(); host_yaw = s_dist_heading0; wire_poll();
    host_tick += 200u; ++host_counts[0]; wire_poll();
    CHECK(s_dist_align_stable_t0 == host_tick && s_round == R_ALIGN);
    host_tick += 399u; wire_poll(); CHECK(s_round == R_ALIGN);
    ++host_tick; wire_poll(); CHECK(s_round != R_ALIGN);
    reset_fixture(); s_seq_mode = 31u; s_seq_state = SQ_OFF;
    CHECK(dist_align_stable_ms() == 700u && turn_settle_ms() == 700u);
    puts("31 fast holds:distance/preCrossRIGHT90/hostageRIGHT93 turn399 wait400 finish;43 bothRIGHT90 699 wait700 finish;turn drift/wheel change restart400,retained inactive31 stays700 passed");
    return 0;
}

static int check_fast_contact_zero_wait(void)
{
    const unsigned owners[] = {31u,43u};
    for (unsigned k = 0u; k < 2u; ++k) {
        unsigned wait = k ? 1000u : 300u;
        CHECK(fast_prepare(owners[k],4u) == 0 && route_contact_zero_wait_ms() == wait);
        CHECK(route_contact_tilt_deg() == (owners[k] == 31u ? 1.0f : 1.5f));
        host_tick += T_DIST_STILL_MS - 1u; wire_poll();
        CHECK(s_seq_state == SQ_STILL && fast_stopped());
        ++host_tick; wire_poll();
        CHECK(s_seq_state == SQ_WAIT && !s_board_contact.zeroed && fast_stopped());
        uint32_t origin = s_seq_wait_t0;
        host_pitch = 0.25f; host_roll = -0.75f;
        host_tick = origin + wait - 1u; wire_poll();
        CHECK(s_seq_state == SQ_WAIT && !s_board_contact.zeroed && fast_stopped());
        ++host_tick; wire_poll();
        CHECK(s_seq_state == SQ_RUN && s_round == R_RUN && s_board_contact.zeroed &&
              s_board_contact.active && s_board_contact.pitch0 == 0.25f &&
              s_board_contact.roll0 == -0.75f && s_dist_target == 0.0f &&
              s_v == 40.0f && last_x == 40.0f && last_y == 0.0f && last_w == 0.0f);
        run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
    }
    CHECK(fast_prepare(31u,4u) == 0);
    host_tick += T_DIST_STILL_MS; wire_poll();
    host_tick += 299u; ++host_counts[0]; wire_poll();
    CHECK(s_seq_state == SQ_STILL && !s_board_contact.zeroed && fast_stopped());
    host_tick += 249u; wire_poll(); CHECK(s_seq_state == SQ_STILL);
    ++host_tick; wire_poll(); CHECK(s_seq_state == SQ_WAIT && fast_stopped());
    host_tick += 299u; wire_poll(); CHECK(s_seq_state == SQ_WAIT && !s_board_contact.zeroed);
    ++host_tick; wire_poll(); CHECK(s_seq_state == SQ_RUN && s_board_contact.zeroed);
    run_cmd("0");
    puts("31 board:still250 then299 nozero/nooutput,300 baseline/v40;43 retains999/1000;wheel change invalidates and repeats both waits passed");
    return 0;
}

static int check_fast_turn_strength_preserves_holds(void)
{
    const unsigned owners[]={31u,43u};
    for (unsigned owner=0u;owner<2u;++owner) {
        for (unsigned side=0u;side<2u;++side) {
            CHECK(fast_start_road(owners[owner],ROUTE31_RETURN180_STAGE)==0);
            unsigned hold=owner?700u:400u;
            unsigned zeros=(unsigned)zero_calls, commands=wire_commands;
            float direction=side? -1.0f:1.0f;
            float goal=owner?180.0f:185.0f;
            CHECK(turn_target_deg()==goal);
            host_yaw=goal+(side?0.5f:-0.5f);
            host_tick+=20u;wire_poll();
            CHECK(s_round==R_RUN && !last_x && !last_y &&
                  fabsf(last_w-direction*(owner?0.18f:0.30f))<0.000001f &&
                  wire_commands==commands && (unsigned)zero_calls==zeros);
            host_yaw=goal;host_tick+=20u;wire_poll();
            CHECK(s_round==R_BRAKE && fast_stopped() && turn_settle_ms()==hold);
            host_tick+=hold-1u;wire_poll();
            CHECK(s_round==R_BRAKE && s_seq_stage==ROUTE31_RETURN180_STAGE &&
                  wire_commands==commands && (unsigned)zero_calls==zeros && fast_stopped());
            ++host_tick;wire_poll();
            if (owner) {
                CHECK(s_seq_stage==ROUTE31_RETURN180_STAGE && s_seq_return_offset &&
                      s_seq_state==SQ_STEP_WAIT && wire_commands==commands);
            } else {
                CHECK(s_seq_stage==ROUTE31_TARGET_STAGE && !s_seq_return_offset &&
                      !s_seq_pair_offset && s_seq_state==SQ_TASK && s_target35_route &&
                      s_target35_phase==TA_TASK_WAIT && wire_commands==commands+1u);
            }
            CHECK((unsigned)zero_calls==zeros && !laser_state && !last_x && !last_y && !last_w);
            run_cmd("0");CHECK(fast_stopped());
        }
    }
    puts("31 actual second185 goal+/-0.5 .30 vs43 second180 .18; holds400/700 preserved;31 directly requests task2 with no lateral job;43 keeps gatedLEFT20; no early zero/request passed");
    return 0;
}

static int fast_vat_yaw(unsigned owner, unsigned stage)
{
    /* Independent equal-origin runs test the exact deadline without a1ms
     * second call being skipped by VAT's real20ms processing cadence. */
    for (unsigned boundary = 0u; boundary < 2u; ++boundary) {
    CHECK(fast_prepare(owner,stage) == 0 && s_seq_state == SQ_TASK);
    /* The synthetic task entry skips the separately tested rack deployment.
     * Supply that completed prerequisite, not an invented physical result. */
    if (owner == 31u && stage == ROUTE31_PAIR_STAGE) s_route31_rack_deployed = 1u;
    CHECK(s_route_yaw_settle_ms == (owner == 31u ? 400u : 700u));
    wire_ack(wire_request,2u,0u);
    host_tick += 260u; wire_poll(); host_tick += 20u; wire_poll(); fast_snapshot();
    CHECK(fast_status.state == VAT_ROUTE_SEARCH);
    host_absolute_yaw += 1.0f; host_yaw = 1.0f;
    host_tick += 20u; wire_poll();
    xy_obj(wire_request,stage == ROUTE31_PAIR_STAGE ? 4 : 0,
           stage == ROUTE31_PAIR_STAGE ? 135 : 215,
           stage == ROUTE31_PAIR_STAGE ? VAT_BALL_Y_PX : VAT_HOSTAGE_Y_PX);
    host_tick += T_DIST_STILL_MS; wire_poll(); fast_snapshot();
    if (owner == 31u && !VAT_ROUTE31_BALL_HOSTAGE_STOP_YAW_ENABLE) {
        CHECK(fast_status.state == VAT_RECHECK && !fast_status.yaw_dirty &&
              !fast_status.good && fast_stopped());
        /* The moving image is discarded. Residual yaw remains1 degree,
         * but five NEW stopped images, not a hidden pure-yaw command, finish. */
        CHECK(fabsf(fast_status.yaw_error + 1.0f) < 0.000001f);
        for (unsigned fresh = 1u; fresh <= VAT_GOOD_FRAMES; ++fresh) {
            host_tick += 20u;
            xy_obj(wire_request,stage == ROUTE31_PAIR_STAGE ? 4 : 0,
                   stage == ROUTE31_PAIR_STAGE ? 135 : 215,
                   stage == ROUTE31_PAIR_STAGE ? VAT_BALL_Y_PX : VAT_HOSTAGE_Y_PX);
            fast_snapshot();
            CHECK(!fast_status.yaw_dirty && !last_x && !last_y && !last_w && !laser_state);
            if (fresh < VAT_GOOD_FRAMES)
                CHECK(fast_status.state == VAT_ALIGN && fast_status.good == fresh && !host_timer_active);
        }
        CHECK(fast_status.state == (stage == ROUTE31_PAIR_STAGE ?
              VAT_WAIT_BALL_ACTION : VAT_WAIT_HOSTAGE_ACTION) && host_timer_active);
        run_cmd("0"); CHECK(fast_stopped());
        continue;
    }
    CHECK(fast_status.state == VAT_YAW_FIX && fast_stopped());
    host_absolute_yaw -= 1.0f; host_yaw = 0.0f;
    host_tick += 20u; wire_poll(); fast_snapshot();
    CHECK(fast_status.state == VAT_YAW_FIX && s_yaw_stable && fast_stopped());
    unsigned hold = owner == 31u ? 400u : 700u;
    uint32_t origin = s_yaw_stable_from;
    /* Keep NEW selected observations alive: this case isolates yaw settling,
     * not the mode31 >300ms seen-then-lost continuation branch. */
    for (unsigned refresh = 100u; refresh < hold; refresh += 100u) {
        host_tick = origin + refresh;
        xy_obj(wire_request,stage == ROUTE31_PAIR_STAGE ? 4 : 0,
               stage == ROUTE31_PAIR_STAGE ? 135 : 215,
               stage == ROUTE31_PAIR_STAGE ? VAT_BALL_Y_PX : VAT_HOSTAGE_Y_PX);
    }
    host_tick = origin + hold - (boundary ? 0u : 1u); wire_poll(); fast_snapshot();
    CHECK(fast_status.state == (boundary ? VAT_RECHECK : VAT_YAW_FIX) &&
          !fast_status.good && !host_timer_active);
    CHECK(!vision_align_test_route_yaw_settle_ms_set(700u)); /* Initial-only API. */
    run_cmd("0"); CHECK(fast_stopped());
    }
    return 0;
}

static int check_fast_vat_scope(void)
{
    CHECK(fast_vat_yaw(31u,ROUTE31_PAIR_STAGE) == 0);
    CHECK(fast_vat_yaw(31u,ROUTE31_HOSTAGE_STAGE) == 0);
    CHECK(fast_vat_yaw(43u,ROUTE31_HOSTAGE_STAGE) == 0);
    CHECK(fast_prepare(31u,ROUTE31_PAIR_STAGE) == 0 && s_route_yaw_settle_ms == 400u);
    CHECK(!vision_align_test_route_yaw_settle_ms_set(0u) &&
          !vision_align_test_route_yaw_settle_ms_set(399u) &&
          !vision_align_test_route_yaw_settle_ms_set(401u) &&
          !vision_align_test_route_yaw_settle_ms_set(1000u));
    /* Exercise the real bucket request core; no arm/turn completion is invented. */
    CHECK(vat_request(PROTO_TASK_BUCKET,0u,host_tick) && s_route_yaw_settle_ms == 400u);
    CHECK(vat_stopped_yaw_enabled()); /* Disabled31 ball/hostage must not cover bucket. */
    CHECK(!vision_align_test_route_yaw_settle_ms_set(700u));
    run_cmd("0");
    CHECK(xy_boot(40u) == 0); run_cmd("g"); wire_sync();
    wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
    CHECK(s_route_yaw_settle_ms == 700u && !vision_align_test_route_yaw_settle_ms_set(400u));
    run_cmd("0");
    puts(VAT_ROUTE31_BALL_HOSTAGE_STOP_YAW_ENABLE
         ? "historical enabled31 VAT:ball/hostage yaw399/400;43 699/700;real bucket/new standalone profile isolation passed (not default behavior)"
         : "default31 VAT:ball/hostage residual1deg still250 then fiveNEW without stopped rotation;43 yaw699/700 retained;real bucket request preserves400,new standalone resets700 passed");
    return 0;
}

static int fast_target_seek(unsigned owner)
{
    if (owner == 35u) {
        CHECK(wire_boot() == 0); host_wire_diag_hook = real_proto_wire_diag_get; xy_seq = 1u;
        run_cmd("35"); run_cmd("g"); wire_sync();
        wire_ack(wire_request,1u,0u); xy_qr(wire_request,"123");
    } else CHECK(fast_prepare(owner,ROUTE31_TARGET_STAGE) == 0);
    wire_sync(); CHECK(wire_task == 2u && wire_digit == 2u);
    wire_ack(wire_request,2u,0u);
    for (unsigned n = 0u; n < 100u && s_target35_phase != TA_SEEK; ++n) {
        host_tick += 20u; wire_poll();
    }
    CHECK(s_target35_phase == TA_SEEK && !laser_state &&
          s_target35_route == (owner != 35u));
    CHECK(target35_point() == (owner == 35u ? 255 : owner == 31u ? 250 : 240) &&
          target35_low() == (owner == 35u ? 250 : owner == 31u ? 247 : 237) &&
          target35_high() == (owner == 35u ? 260 : owner == 31u ? 253 : 243));
    return 0;
}

static int fast_target_four(void)
{
    CHECK(fast_target_seek(31u) == 0);
    for (unsigned n = 1u; n <= 4u; ++n) {
        host_tick += 20u; xy_obj(wire_request,8,250,160);
        CHECK(s_target35_good == n && s_target35_phase == TA_SEEK && fast_stopped());
    }
    return 0;
}

static int check_fast_target_and_legacy(void)
{
    CHECK(fast_target_four() == 0); host_tick += 20u; uint32_t fifth = host_tick;
    xy_obj(wire_request,8,250,160);
    CHECK(s_target35_phase == TA_FIRE && laser_state && host_laser_on_calls == 1u &&
          s_target35_phase_t0 == fifth && !s_receiving && !last_x && !last_y && !last_w);
    host_tick += 1999u; wire_poll(); CHECK(s_target35_phase == TA_FIRE && laser_state);
    ++host_tick; wire_poll(); CHECK(!laser_state && s_target35_phase != TA_FIRE);
    if (ROUTE31_TARGET_STOP_YAW_ENABLE) {
        CHECK(s_seq_state == SQ_TASK_YAW && s_seq_stage == ROUTE31_TARGET_STAGE &&
              route_seq_active() && fast_stopped());
        wire_poll(); CHECK(s_route31_task_yaw_stable && fast_stopped());
        uint32_t post_yaw_origin = s_route31_task_yaw_stable_t0;
        host_tick = post_yaw_origin + 399u; wire_poll();
        CHECK(s_seq_state == SQ_TASK_YAW && fast_stopped());
        ++host_tick; wire_poll();
    }
    CHECK(s_seq_state == SQ_STILL && s_seq_stage == ROUTE31_TARGET_CORNER_STAGE && fast_stopped());
    CHECK(route_seq_leg()->distance_mm == 445u && s_d == 445.0f);
    const unsigned owners[] = {35u,43u};
    for (unsigned k = 0u; k < 2u; ++k) {
        CHECK(fast_target_seek(owners[k]) == 0);
        int aim = owners[k] == 35u ? 255 : 240;
        for (unsigned n = 0u; n < 5u; ++n) { host_tick += 20u; xy_obj(wire_request,8,aim,160); }
        CHECK(s_target35_phase == TA_SETTLE && fast_stopped() && !host_laser_on_calls);
        uint32_t origin = s_target35_phase_t0;
        for (unsigned n = 0u; n < 9u; ++n) { host_tick += 100u; xy_obj(wire_request,8,aim,160); }
        host_tick = origin + 999u; xy_obj(wire_request,8,aim,160);
        CHECK(s_target35_phase == TA_SETTLE && !laser_state);
        ++host_tick; xy_obj(wire_request,8,aim,160);
        CHECK(s_target35_phase == TA_FIRE && laser_state && host_laser_on_calls == 1u);
        run_cmd("0"); CHECK(!laser_state && !s_receiving);
    }
    puts(ROUTE31_TARGET_STOP_YAW_ENABLE
         ? "historical enabled31 target:four fresh nofire,fifth samepoll laser/brake/RXclose,1999 on2000 off,original-yaw399 wait400 then green445;35/43 stillwait999/1000 passed (not default behavior)"
         : "default31 target:four fresh nofire,fifth samepoll laser/brake/RXclose,1999 on2000 off directly prepares green445 without SQ_TASK_YAW;35/43 stillwait999/1000 passed");
    return 0;
}

static int check_fast_target_stale_and_stop(void)
{
    CHECK(fast_target_four() == 0);
    unsigned following = xy_seq; xy_seq -= 1u; xy_obj(wire_request,8,250,160); xy_seq = following;
    CHECK(!laser_state && s_target35_good <= 4u);
    host_tick += T_TARGET35_FRESH_MS + 1u; wire_poll();
    CHECK(!laser_state && s_target35_good == 0u);
    for (unsigned n = 1u; n <= 5u; ++n) {
        host_tick += 20u; xy_obj(wire_request,8,250,160);
        CHECK(laser_state == (n == 5u));
    }
    run_cmd("0");
    const char *const keys[] = {"g","a","0"};
    for (unsigned k = 0u; k < 3u; ++k) {
        CHECK(fast_target_four() == 0); run_cmd(keys[k]);
        host_tick += 20u; xy_obj(wire_request,8,250,160);
        CHECK(s_seq_state == SQ_STOPPED && fast_stopped() && !s_receiving && !host_laser_on_calls);
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && !laser_state);
    }
    puts("31 immediatefire:duplicate/cache/expired aim cannot supply fifth;expiry needs five new frames;g/a/0 beforefifth and lateframes never laser/resume passed");
    return 0;
}

static void fast_atomic_interrupt(int wheel)
{
    if (wheel != 0) return;
    host_counts_hook = NULL;
    if (fast_fault == 0u) xy_obj_poll(wire_request,-1,0,0,0);
    else if (fast_fault == 1u) xy_obj_poll(wire_request,7,250,160,0);
    else if (fast_fault == 2u) xy_obj_poll(wire_request,8,246,160,0);
    else if (fast_fault == 3u) xy_obj_poll(wire_request,8,250,160,0);
    else if (fast_fault == 4u) host_imu_valid = 0;
    else if (fast_fault == 5u) host_abort = 1;
    else if (fast_fault == 6u) {
        uint8_t nack[5] = {0x61u,(uint8_t)wire_request,(uint8_t)(wire_request>>8),2u,1u};
        wire_feed_raw(nack,sizeof nack,0); wire_sync();
    } else host_tick += T_TARGET35_FRESH_MS + 1u;
    /* Empty/wrong-class packets have no selected-frame callback; reflect
     * their real parser counter/status before the final owner snapshot. */
    wire_sync();
}

static int check_fast_target_atomic_recovery(void)
{
    for (fast_fault = 0u; fast_fault < 8u; ++fast_fault) {
        CHECK(fast_target_four() == 0); host_counts_hook = fast_atomic_interrupt;
        host_tick += 20u; xy_obj(wire_request,8,250,160);
        if (host_counts_hook || laser_state || host_laser_on_calls || last_x || last_y || last_w)
            fprintf(stderr,"fast atomic case=%u hook=%u phase=%u good=%u laser=%d calls=%u v=%g,%g,%g\n",
                    fast_fault,host_counts_hook != NULL,s_target35_phase,s_target35_good,
                    laser_state,host_laser_on_calls,last_x,last_y,last_w);
        CHECK(!host_counts_hook && !laser_state && !host_laser_on_calls && !last_x && !last_y && !last_w);
        wire_poll();
        if (fast_fault == 3u) {
            CHECK(laser_state && s_target35_phase == TA_FIRE && host_laser_on_calls == 1u);
        } else if (fast_fault >= 4u && fast_fault <= 6u) {
            CHECK(s_seq_state == SQ_STOPPED && !s_receiving && fast_stopped());
        } else {
            CHECK(!laser_state && s_target35_phase == TA_SEEK && !s_target35_good);
            for (unsigned n = 1u; n <= 5u; ++n) {
                host_tick += 20u; xy_obj(wire_request,8,250,160);
                CHECK(laser_state == (n == 5u));
            }
        }
        run_cmd("0"); CHECK(!laser_state && !last_x && !last_y && !last_w);
    }
    puts("31 samepoll finalatomic:empty/wrong/outbound/newinband/IMU/abort/NACK/stale injected after initialsnapshot cannot fireoldaim;latestfresh recovery or stop wins passed");
    return 0;
}

#ifndef ROUTE31_FAST_TIMING_FIXTURE_MAIN
#define ROUTE31_FAST_TIMING_FIXTURE_MAIN main
#endif
int ROUTE31_FAST_TIMING_FIXTURE_MAIN(void)
{
    CHECK(check_fast_private_cross_distance() == 0);
    CHECK(check_fast_road_holds() == 0);
    CHECK(check_fast_contact_zero_wait() == 0);
    CHECK(check_fast_turn_strength_preserves_holds() == 0);
    CHECK(check_fast_vat_scope() == 0);
    CHECK(check_fast_target_and_legacy() == 0);
    CHECK(check_fast_target_stale_and_stop() == 0);
    CHECK(check_fast_target_atomic_recovery() == 0);
    puts("route31_fast_timing_test:host software only;no physical stop/route/laser acceptance");
    return 0;
}
