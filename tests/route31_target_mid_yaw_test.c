/* Real route31 target dispatcher + wire protocol. Time/pixels/IMU are host
 * inputs, not proof of chassis position, stopping distance or laser aim. */
#define ROUTE31_FAST_TIMING_FIXTURE_MAIN retained_fast_target_main
#include "route31_fast_timing_test.c"
#undef ROUTE31_FAST_TIMING_FIXTURE_MAIN

static void target_mid_elapsed(unsigned ms)
{ host_tick += ms; wire_poll(); }

static int target_mid_begin(float yaw)
{
    CHECK(fast_target_seek(31u) == 0);
    CHECK(!s_target31_fine && s_target35_heading == 0.0f);
    host_yaw = yaw; target_mid_elapsed(20u);
    CHECK(s_target31_mid_phase == 1u && fast_stopped() && !laser_state);
    target_mid_elapsed(T_DIST_STILL_MS);
    CHECK(s_target31_mid_phase == 2u && last_x == 0.0f && last_y == 0.0f &&
          last_w * yaw < 0.0f);
    return 0;
}

static int check_target_mid_resume_fresh_and_signs(void)
{
    for (unsigned side = 0u; side < 2u; ++side) {
        CHECK(target_mid_begin(side ? -3.0f : 3.0f) == 0);
        unsigned zeros = (unsigned)zero_calls, commands = wire_commands;
        uint16_t request = wire_request;
        /* Turning frames in the fire band may be received, never fire. */
        xy_seq = 10u;
        for (unsigned rotating = 0u; rotating < 5u; ++rotating)
            xy_obj(request,8,250,160);
        CHECK(s_target31_mid_phase == 2u && !laser_state && s_target35_good == 0u);
        unsigned old_seq = xy_seq - 1u;
        host_yaw = 0.1f; target_mid_elapsed(20u);
        CHECK(s_target31_mid_phase == 3u && fast_stopped());
        target_mid_elapsed(399u); CHECK(s_target31_mid_phase == 3u && fast_stopped());
        target_mid_elapsed(1u);
        CHECK(!s_target31_mid_phase && s_target31_mid_need_new && !s_target31_fine &&
              s_target35_phase == TA_SEEK && last_x == route31_straight_speed() &&
              fabsf(last_y - last_x * .045f) < .00001f && !laser_state &&
              wire_commands == commands && wire_request == request &&
              (unsigned)zero_calls == zeros && s_target35_heading == 0.0f);
        unsigned next_seq = xy_seq; xy_seq = old_seq;
        target_mid_elapsed(20u); xy_obj(request,8,250,160); xy_seq = next_seq;
        CHECK(s_target31_mid_need_new && !s_target31_fine && !s_target35_good && !laser_state);
        target_mid_elapsed(20u); xy_obj(request,8,250,160);
        CHECK(!s_target31_mid_need_new && s_target31_fine && s_target35_good == 1u && !laser_state);
        next_seq = xy_seq;
        uint32_t accepted_packet = s_target35_sample.packet_index;
        uint32_t accepted_tick = s_target35_sample.tick;
        uint16_t accepted_sequence = s_target35_sample.frame.sequence;
        /* First NEW does not release the sequence barrier: five delayed
         * turning frames10..14 must not become a new fire-confirmation run.
         * Replays preserve the one genuine frame's original freshness lease,
         * but cannot renew its timestamp, replace its pixels, or add good. */
        xy_seq = 10u;
        for (unsigned stale = 0u; stale < 5u; ++stale) {
            target_mid_elapsed(20u); xy_obj(request,8,250,160);
            CHECK(s_target35_good == 1u && !laser_state && !host_laser_on_calls &&
                  s_target35_sample.packet_index == accepted_packet &&
                  s_target35_sample.tick == accepted_tick &&
                  s_target35_sample.frame.sequence == accepted_sequence);
        }
        xy_seq = next_seq;
        /* Strict request-wide rejection also leaves stats.obj unchanged, so
         * four genuine consecutive NEW images complete the retained good1.
         * No stale packet supplies a frame or interrupts this valid run. */
        for (unsigned frame = 2u; frame <= 5u; ++frame) {
            target_mid_elapsed(20u); xy_obj(request,8,250,160);
            CHECK(!s_target31_mid_need_new && s_target31_fine && s_target35_good == frame);
            CHECK(s_target35_sample.packet_index == accepted_packet + frame - 1u &&
                  s_target35_sample.frame.sequence == accepted_sequence + frame - 1u &&
                  s_target35_sample.tick == host_tick);
            CHECK(laser_state == (frame == 5u));
        }
        run_cmd("0"); CHECK(fast_stopped());
    }
    return 0;
}

static int check_target_mid_threshold_timeout_and_scope(void)
{
    CHECK(fast_target_seek(31u) == 0);
    host_yaw = 2.5f; target_mid_elapsed(20u);
    CHECK(!s_target31_mid_phase && last_x > 0.0f);
    host_yaw = -2.5f; target_mid_elapsed(20u);
    CHECK(!s_target31_mid_phase && last_x > 0.0f);
    for (unsigned side = 0u; side < 2u; ++side)
      for (unsigned accepted = 0u; accepted < 2u; ++accepted) {
        const float sign = side ? -1.0f : 1.0f;
        CHECK(target_mid_begin(sign * 2.501f) == 0);
        unsigned zeros = (unsigned)zero_calls;
        host_yaw = sign * (accepted ? 2.5f : 2.501f);
        target_mid_elapsed(1999u);
        CHECK(s_target31_mid_phase == 2u && s_seq_state == SQ_TASK && !laser_state);
        target_mid_elapsed(1u);
        if (!accepted) {
            CHECK(s_seq_state == SQ_STOPPED && fast_stopped() && !s_target35_route &&
                  strstr(host_messages,"TARGET_MID_YAW_TIMEOUT_GT2P5") != NULL);
        } else {
            CHECK(s_target31_mid_phase == 4u && fast_stopped());
            target_mid_elapsed(249u); CHECK(s_target31_mid_phase == 4u && fast_stopped());
            ++host_counts[2]; target_mid_elapsed(1u); CHECK(s_target31_mid_phase == 4u);
            target_mid_elapsed(250u);
            CHECK(!s_target31_mid_phase && s_target31_mid_need_new && last_x > 0.0f &&
                  strstr(host_messages,"timeout_accept=1") != NULL &&
                  host_yaw == sign * 2.5f && s_target35_heading == 0.0f);
        }
        CHECK((unsigned)zero_calls == zeros && !laser_state); run_cmd("0");
    }
    for (unsigned owner = 0u; owner < 2u; ++owner) {
        CHECK(fast_target_seek(owner ? 43u : 35u) == 0);
        host_yaw = 4.0f; target_mid_elapsed(20u);
        CHECK(!s_target31_mid_phase && last_x > 0.0f && !laser_state);
        if (owner) CHECK(fabsf(last_y - .065f * last_x) < .00001f);
        else CHECK(last_y == 0.0f);
        run_cmd("0");
    }
    CHECK(fast_target_seek(31u) == 0); xy_obj(wire_request,8,280,160);
    CHECK(s_target31_fine);
    host_yaw = 4.0f; target_mid_elapsed(20u);
    CHECK(!s_target31_mid_phase && last_x == 30.0f && !laser_state);
    run_cmd("0"); return 0;
}

static int check_target_mid_stop(void)
{
    const char *keys[] = {"g","a","0"};
    for (unsigned phase = 0u; phase < 3u; ++phase)
        for (unsigned key = 0u; key < 3u; ++key) {
            CHECK(target_mid_begin(3.0f) == 0);
            if (phase == 1u) { host_yaw = 0.0f; target_mid_elapsed(20u); }
            if (phase == 2u) { host_yaw = 1.0f; target_mid_elapsed(2000u); }
            run_cmd(keys[key]); CHECK(s_seq_state == SQ_STOPPED && fast_stopped());
            target_mid_elapsed(5000u); CHECK(fast_stopped() && !s_target35_route);
        }
    return 0;
}

#ifndef ROUTE31_TARGET_MID_FIXTURE_MAIN
#define ROUTE31_TARGET_MID_FIXTURE_MAIN main
#endif
int ROUTE31_TARGET_MID_FIXTURE_MAIN(void)
{
    CHECK(check_target_mid_resume_fresh_and_signs() == 0);
    CHECK(check_target_mid_threshold_timeout_and_scope() == 0);
    CHECK(check_target_mid_stop() == 0);
    puts("31 target coarse mid-yaw: signs/strict2.5 no-trigger2.501 trigger/still250+400/original-heading/new-image+replay barrier/1999-2000ms exact residual2.5 gate/cancel/43 and35/fine isolation passed");
    return 0;
}
