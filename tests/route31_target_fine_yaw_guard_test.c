/* Real dispatcher/CRC parser; injected motion/IMU/pixels are host evidence only. */
#define ROUTE31_FAST_TIMING_FIXTURE_MAIN retained_target_fine_fast_main
#include "route31_fast_timing_test.c"
#undef ROUTE31_FAST_TIMING_FIXTURE_MAIN

static void fine_target_tick(uint32_t ms) { host_tick += ms; wire_poll(); }

static int fine_target_start(unsigned owner)
{
    CHECK(fast_target_seek(owner) == 0);
    fine_target_tick(20u);
    xy_obj(wire_request,8,target35_point()+20,100);
    CHECK(s_target35_phase == TA_SEEK && !laser_state &&
          last_x == target35_fine_speed());
    CHECK(owner == 35u || s_target31_fine);
    return 0;
}

static int check_fine_repair_new_image_and_retry(void)
{
    for (int sign=-1; sign<=1; sign+=2) for (unsigned timeout=0u; timeout<2u; ++timeout) {
        CHECK(fine_target_start(31u) == 0);
        const uint16_t request=wire_request;
        const unsigned commands=wire_commands, zeros=(unsigned)zero_calls;
        const float goal=s_target35_heading;
        host_yaw=goal+(float)sign*1.1f;
        fine_target_tick(20u);
        CHECK(s_target31_mid_phase == 1u && s_target31_fine && fast_stopped());
        fine_target_tick(T_DIST_STILL_MS);
        CHECK(s_target31_mid_phase == 2u && last_w*(float)sign < 0.0f && !last_x && !last_y);
        /* In-band turning-time coordinates cannot authorize firing. */
        fine_target_tick(20u); xy_obj(request,8,target35_point(),100);
        CHECK(!laser_state && !s_target35_good);
        if (timeout) {
            host_tick=s_target31_mid_t0+ROUTE31_POST_YAW_MAX_MS; wire_poll();
            CHECK(s_target31_mid_phase == 4u && fast_stopped());
            fine_target_tick(T_DIST_STILL_MS);
        } else {
            host_yaw=goal+(float)sign*.2f; fine_target_tick(20u);
            CHECK(s_target31_mid_phase == 3u && fast_stopped());
            fine_target_tick(ROUTE31_STABLE_MS);
        }
        CHECK(!s_target31_mid_phase && s_target31_mid_need_new && s_target31_fine &&
              s_target35_phase == TA_SEEK && fast_stopped() && !s_target35_good &&
              s_target35_heading == goal && wire_request == request &&
              wire_commands == commands && (unsigned)zero_calls == zeros);
        unsigned barrier=xy_seq-1u;
        fine_target_tick(1000u);
        CHECK(!s_target31_mid_phase && s_target31_mid_need_new && fast_stopped());
        /* A replay and a NEW empty object packet do not unlock motion. */
        unsigned next=xy_seq; xy_seq=barrier;
        xy_obj(request,8,target35_point(),100); xy_seq=next;
        CHECK(s_target31_mid_need_new && fast_stopped() && !laser_state);
        fine_target_tick(20u); xy_obj(request,-1,0,0);
        CHECK(s_target31_mid_need_new && fast_stopped() && !laser_state);
        /* A far NEW image must remain fine30; never coarse50 after repair. */
        fine_target_tick(20u); xy_obj(request,8,400,100);
        CHECK(!s_target31_mid_need_new && s_target31_fine &&
              !s_target31_mid_phase && last_x == 30.0f && !last_y &&
              s_target35_heading == goal && wire_request == request && !laser_state);
        if (timeout) {
            CHECK(!s_target31_mid_retry_wait_motion && s_target31_mid_timeout_latched);
            fine_target_tick(399u);
            CHECK(!s_target31_mid_phase && last_x == 30.0f);
            fine_target_tick(1u);
            CHECK(s_target31_mid_phase == 1u && fast_stopped());
        } else {
            for (unsigned n=1u; n<=5u; ++n) {
                fine_target_tick(20u); xy_obj(request,8,target35_point(),100);
                CHECK(n<5u ? !laser_state : laser_state);
            }
        }
        run_cmd("0"); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
    }
    return 0;
}

static int check_scope_and_cancel(void)
{
    const char *keys[]={"g","a","0"};
    for (unsigned n=0u; n<3u; ++n) {
        CHECK(fine_target_start(31u) == 0);
        host_yaw=s_target35_heading+1.1f; fine_target_tick(20u);
        CHECK(s_target31_mid_phase == 1u);
        run_cmd(keys[n]); fine_target_tick(5000u);
        CHECK(s_seq_state == SQ_STOPPED && fast_stopped() && !host_laser_on_calls);
    }
    CHECK(fine_target_start(31u) == 0);
    host_yaw=s_target35_heading+1.1f; fine_target_tick(20u);
    host_imu_valid=0; fine_target_tick(20u);
    CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
    for (unsigned owner=35u; owner<=43u; owner+=8u) {
        CHECK(fine_target_start(owner) == 0);
        host_yaw=s_target35_heading+3.0f; fine_target_tick(20u);
        CHECK(!s_target31_mid_phase && last_x == target35_fine_speed() && !laser_state);
        run_cmd("0"); CHECK(fast_stopped());
    }
    return 0;
}

int main(void)
{
    CHECK(check_fine_repair_new_image_and_retry() == 0);
    CHECK(check_scope_and_cancel() == 0);
    puts("31 TARGET fine yaw guard: signed repair, same heading/request, stopped NEW-image barrier, replay/empty isolation, fine30 latch, timeout400ms actual-motion retry, fiveNEW fire and manual/IMU priority;35/43 isolation passed (host only)");
    return 0;
}
