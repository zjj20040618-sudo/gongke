/* The real Bluetooth dispatcher and distance state machine, with only
 * peripherals/time replaced. Host assertions do not prove physical alignment. */
#define main retained_g_fixture_main
#include "g_command_stop_test.c"
#undef main
#include <float.h>

static int yaw_stopped(void)
{
    return last_x == 0.0f && last_y == 0.0f && last_w == 0.0f;
}

static void yaw_elapsed(unsigned milliseconds)
{
    host_tick += milliseconds;
    test_poll();
}

static int yaw_start_distance(int mode, float initial_heading)
{
    char command[16];
    reset_fixture();
    snprintf(command, sizeof command, "%d", mode); run_cmd(command);
    run_cmd("v300"); run_cmd("d100"); run_cmd("ykp2");
    host_yaw = initial_heading;
    run_cmd("g"); test_poll();
    CHECK(s_round == R_RUN && s_dist_heading0 == initial_heading);
    return 0;
}

static int yaw_arrive(float angle)
{
    host_yaw = angle;
    if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
    else host_fore = s_dist_odo0 + s_dist_target;
    test_poll();
    CHECK(s_round == R_BRAKE && s_dist_reason == 1u && yaw_stopped());
    return 0;
}

/* Do not assume the implementation's phase names; require an observable
 * low-speed rotation, without translation or zeroing, within a bounded poll. */
static int yaw_wait_correction(int expected_sign)
{
    unsigned before = (unsigned)zero_calls;
    for (unsigned poll = 0u; poll < 100u && last_w == 0.0f; ++poll) {
        yaw_elapsed(20u);
        CHECK((unsigned)zero_calls == before && last_x == 0.0f && last_y == 0.0f);
    }
    CHECK(last_w * (float)expected_sign > 0.0f);
    CHECK(fabsf(last_w) <= T_DIST_ALIGN_MAX_W);
    CHECK((unsigned)zero_calls == before && s_round != R_READY);
    return 0;
}

static int yaw_wait_zero(unsigned before)
{
    for (unsigned poll = 0u; poll < 100u && (unsigned)zero_calls == before; ++poll) {
        yaw_elapsed(20u);
        CHECK(last_x == 0.0f && last_y == 0.0f);
    }
    CHECK((unsigned)zero_calls == before + 1u && host_yaw == 0.0f && yaw_stopped());
    return 0;
}

static int check_normal_done_realigns_before_zero(void)
{
    for (int mode = 15; mode <= 18; ++mode) {
        CHECK(yaw_start_distance(mode, 7.0f) == 0);
        unsigned before = (unsigned)zero_calls;
        CHECK(yaw_arrive(11.0f) == 0);
        /* Encoder coast prevents correction and prevents zeroing. */
        yaw_elapsed(T_DIST_STILL_MS - 1u);
        CHECK(yaw_stopped() && (unsigned)zero_calls == before);
        host_counts[0]++; yaw_elapsed(1u);
        CHECK(yaw_stopped() && (unsigned)zero_calls == before);
        yaw_elapsed(T_DIST_STILL_MS - 1u);
        CHECK(yaw_stopped() && (unsigned)zero_calls == before);
        CHECK(yaw_wait_correction(-1) == 0);
        CHECK(s_dist_heading0 == 7.0f);
        /* Overshoot must reverse correction, not accept the crooked heading. */
        host_yaw = 6.2f; yaw_elapsed(20u);
        CHECK(last_w > 0.0f && last_x == 0.0f && last_y == 0.0f);
        CHECK((unsigned)zero_calls == before);
        host_yaw = 7.0f; yaw_elapsed(20u);
        CHECK(yaw_stopped() && (unsigned)zero_calls == before);
        /* A wheel moving during the final hold must restart the still timer. */
        host_counts[3]++; yaw_elapsed(20u);
        CHECK((unsigned)zero_calls == before && yaw_stopped());
        yaw_elapsed(T_DIST_STILL_MS - 1u);
        CHECK((unsigned)zero_calls == before && yaw_stopped());
        CHECK(yaw_wait_zero(before) == 0);
        CHECK(s_round == R_READY && strstr(host_messages, "status=DONE") != NULL);
        CHECK(!host_laser_on_calls && !pulse_calls && !servo_calls);
    }
    puts("post-yaw: four distance directions return to captured start heading before zero, signed overshoot, encoder coast/final still and no translation passed");
    return 0;
}

static int check_wrap_and_already_aligned(void)
{
    CHECK(yaw_start_distance(17, 179.0f) == 0);
    CHECK(yaw_arrive(-178.0f) == 0);
    CHECK(yaw_wait_correction(-1) == 0); /* shortest error is -3, not +357 */
    unsigned before = (unsigned)zero_calls;
    host_yaw = 179.0f; yaw_elapsed(20u);
    CHECK(yaw_wait_zero(before) == 0);

    CHECK(yaw_start_distance(18, -179.0f) == 0);
    CHECK(yaw_arrive(178.0f) == 0);
    CHECK(yaw_wait_correction(1) == 0);
    before = (unsigned)zero_calls;
    host_yaw = -179.0f; yaw_elapsed(20u);
    CHECK(yaw_wait_zero(before) == 0);

    CHECK(yaw_start_distance(17, 0.0f) == 0);
    CHECK(yaw_arrive(0.1f) == 0);
    before = (unsigned)zero_calls;
    unsigned dispatch_before = host_precise_calls;
    CHECK(yaw_wait_zero(before) == 0);
    CHECK(host_precise_calls == dispatch_before && s_round == R_READY);
    puts("post-yaw: wrapped headings take the short signed correction; already-aligned completion does not issue a rotation passed");
    return 0;
}

static int check_stops_do_not_correct_or_resume(void)
{
    static const char *const stops[] = {"g", "a", "0"};
    for (unsigned phase = 0u; phase < 3u; ++phase)
        for (unsigned key = 0u; key < 3u; ++key) {
            CHECK(yaw_start_distance(17, 0.0f) == 0);
            CHECK(yaw_arrive(4.0f) == 0);
            if (phase >= 1u) CHECK(yaw_wait_correction(-1) == 0);
            if (phase == 2u) {
                host_yaw = 0.0f; yaw_elapsed(20u); CHECK(yaw_stopped());
            }
            unsigned zero_before = (unsigned)zero_calls;
            run_cmd(stops[key]);
            CHECK(yaw_stopped());
            unsigned before = host_precise_calls;
            for (unsigned poll = 0u; poll < 100u; ++poll) {
                host_yaw = 4.0f; yaw_elapsed(20u);
                CHECK(yaw_stopped() && host_precise_calls == before);
            }
            CHECK(s_round == R_READY && (unsigned)zero_calls == zero_before);
        }
    CHECK(yaw_start_distance(15, 0.0f) == 0);
    host_yaw = 4.0f; host_fore = 50.0f; run_cmd("g");
    CHECK(s_dist_reason == 0u && yaw_stopped());
    unsigned before = host_precise_calls;
    yaw_elapsed(T_DIST_STILL_MS);
    CHECK(s_round == R_READY && yaw_stopped() && host_precise_calls == before);
    CHECK(strstr(last_message, "status=STOP") != NULL);
    puts("post-yaw: nine g/a/0 cancellations across brake/correction/final hold; manual partial-distance stop never corrects or auto-returns passed");
    return 0;
}

static int check_invalid_imu_and_abort(void)
{
    for (unsigned cause = 0u; cause < 2u; ++cause)
        for (unsigned phase = 0u; phase < 2u; ++phase) {
            CHECK(yaw_start_distance(17, 0.0f) == 0);
            CHECK(yaw_arrive(4.0f) == 0);
            if (phase) CHECK(yaw_wait_correction(-1) == 0);
            unsigned zero_before = (unsigned)zero_calls;
            if (cause) host_abort = 1; else host_imu_valid = 0;
            yaw_elapsed(20u); CHECK(yaw_stopped());
            unsigned before = host_precise_calls;
            for (unsigned poll = 0u; poll < 100u; ++poll) {
                yaw_elapsed(20u); CHECK(yaw_stopped() && host_precise_calls == before);
            }
            CHECK(s_round == R_READY && (unsigned)zero_calls == zero_before);
        }
    puts("post-yaw: IMU loss and abort during initial brake/correction cancel without later motion passed");
    return 0;
}

static int check_toggle_limits_and_failures(void)
{
    reset_fixture();
    CHECK(s_dist_align_on == 1u);
    run_cmd("yfix0"); CHECK(s_dist_align_on == 0u);
    run_cmd("yfix1"); CHECK(s_dist_align_on == 1u);
    static const char *const invalid[] = {"yfix2", "yfix-1", "yfix", "yfix1junk", "yfixnan"};
    for (unsigned n = 0u; n < sizeof invalid / sizeof invalid[0]; ++n) {
        run_cmd(invalid[n]);
        CHECK(s_dist_align_on == 1u && strstr(last_message, "ERR") != NULL);
    }
    run_cmd("17"); run_cmd("v300"); run_cmd("d100"); run_cmd("yfix0"); run_cmd("g");
    CHECK(s_dist_align_enabled == 0u);
    CHECK(yaw_arrive(4.0f) == 0);
    unsigned before = (unsigned)zero_calls;
    unsigned dispatch_before = host_precise_calls;
    yaw_elapsed(T_DIST_STILL_MS);
    CHECK(s_round == R_READY && yaw_stopped() && (unsigned)zero_calls == before + 1u);
    CHECK(host_precise_calls == dispatch_before);

    CHECK(yaw_start_distance(17, 0.0f) == 0);
    CHECK(s_dist_align_enabled == 1u);
    run_cmd("yfix0");
    CHECK(s_dist_align_on == 1u && s_dist_align_enabled == 1u && strstr(last_message, "ERR") != NULL);
    CHECK(yaw_arrive(4.0f) == 0);
    CHECK(yaw_wait_correction(-1) == 0);
    before = (unsigned)zero_calls;
    yaw_elapsed(T_DIST_ALIGN_MAX_MS + 1u);
    CHECK(s_round == R_READY && yaw_stopped() && (unsigned)zero_calls == before);
    CHECK(strstr(host_messages, "TIMEOUT") != NULL);

    for (unsigned kind = 0u; kind < 2u; ++kind) {
        CHECK(yaw_start_distance(18, 0.0f) == 0);
        CHECK(yaw_arrive(4.0f) == 0);
        CHECK(yaw_wait_correction(-1) == 0);
        before = (unsigned)zero_calls;
        host_yaw = kind ? INFINITY : NAN;
        yaw_elapsed(20u);
        CHECK(s_round == R_READY && yaw_stopped() && (unsigned)zero_calls == before);
        CHECK(strstr(host_messages, "IMUERR") != NULL);
    }
    puts("post-yaw: default/toggle/invalid/running locks, yfix0 legacy path,12s timeout and non-finite IMU never declare correction success or zero passed");
    return 0;
}

static int check_translation_record_not_corrupted_by_rotation(void)
{
    CHECK(yaw_start_distance(17,0.0f) == 0);
    host_counts[0] = 100; host_counts[1] = -100;
    host_counts[2] = 100; host_counts[3] = -100;
    CHECK(yaw_arrive(4.0f) == 0);
    CHECK(yaw_wait_correction(-1) == 0);
    unsigned before = (unsigned)zero_calls;
    CHECK(s_dist_end_saved && s_dist_end_mm == -100.0f);
    CHECK(s_dist_end_counts[0] == 100 && s_dist_end_counts[1] == -100 &&
          s_dist_end_counts[2] == 100 && s_dist_end_counts[3] == -100);
    /* The IMU return rotates the wheels; its count changes are not translation. */
    host_counts[0] += 37; host_counts[1] -= 32;
    host_counts[2] -= 29; host_counts[3] += 31;
    host_lateral += 11.0f; host_fore += 9.0f;
    yaw_elapsed(20u);
    CHECK((unsigned)zero_calls == before && last_x == 0.0f && last_y == 0.0f);
    host_yaw = 0.0f; yaw_elapsed(20u);
    CHECK(yaw_wait_zero(before) == 0);
    CHECK(strstr(last_message,"enc_mm=-100.0") != NULL);
    CHECK(strstr(last_message,"c0=100 c1=-100 c2=100 c3=-100") != NULL);
    CHECK(strstr(last_message,"pre_yaw=4.00") != NULL && strstr(last_message,"yaw_deg=0.00") != NULL);
    puts("post-yaw: translation end distance/counts frozen before correction; REC separates pre-correction and corrected pre-zero heading passed");
    return 0;
}

static int check_alignment_owner_write_locks(void)
{
    static const char *const writes[] = {"15","36","v600","d1000","ykp5","fff0","bff0",
                                        "lff0","rff0","yfix0","acc100","dec100","co","cc"};
    CHECK(yaw_start_distance(17,0.0f) == 0);
    CHECK(yaw_arrive(4.0f) == 0);
    CHECK(yaw_wait_correction(-1) == 0);
    CHECK(s_round == R_ALIGN);
    float left_before = s_left_ff_ratio, right_before = s_right_ff_ratio;
    float forward_before = s_forward_ff_ratio, back_before = s_backward_ff_ratio;
    for (unsigned n = 0u; n < sizeof writes / sizeof writes[0]; ++n) {
        run_cmd(writes[n]);
        CHECK(strstr(last_message,"ERR") != NULL);
        CHECK(s_msel == 17 && s_v == 300.0f && s_d == 100.0f && s_round == R_ALIGN);
        CHECK(s_dist_heading_kp == 2.0f && s_dist_align_on == 1u && s_dist_align_enabled == 1u);
        CHECK(s_left_ff_ratio == left_before && s_right_ff_ratio == right_before);
        CHECK(s_forward_ff_ratio == forward_before && s_backward_ff_ratio == back_before);
        CHECK(!pulse_calls && !servo_calls && last_x == 0.0f && last_y == 0.0f && last_w < 0.0f);
    }
    static const char *const reports[] = {"param","diag","?"};
    for (unsigned n = 0u; n < sizeof reports / sizeof reports[0]; ++n) {
        run_cmd(reports[n]);
        CHECK(s_round == R_ALIGN && s_msel == 17 && last_x == 0.0f && last_y == 0.0f && last_w < 0.0f);
    }
    run_cmd("g");
    CHECK(s_round == R_READY && yaw_stopped());
    puts("post-yaw:14 mode/speed/distance/gain/FFF/servo writes rejected duringR_ALIGN; param/diag/help stay read-only, g stays immediate passed");
    return 0;
}

static int check_subtraction_overflow_and_large_finite_yaw(void)
{
    /* During translation the hold target and measurement may each be finite,
     * yet opposite extremes overflow their difference. Stop before the helper. */
    CHECK(yaw_start_distance(17,-FLT_MAX) == 0);
    unsigned before = (unsigned)zero_calls;
    host_yaw = FLT_MAX; yaw_elapsed(20u);
    CHECK(s_round == R_BRAKE && s_dist_reason == 2u && yaw_stopped());
    yaw_elapsed(T_DIST_STILL_MS);
    CHECK(s_round == R_READY && (unsigned)zero_calls == before && yaw_stopped());
    CHECK(strstr(host_messages,"IMUERR") != NULL);

    CHECK(yaw_start_distance(18,0.0f) == 0);
    CHECK(yaw_arrive(4.0f) == 0);
    CHECK(yaw_wait_correction(-1) == 0);
    before = (unsigned)zero_calls;
    s_dist_heading0 = -FLT_MAX; host_yaw = FLT_MAX;
    yaw_elapsed(20u);
    CHECK(s_round == R_READY && yaw_stopped() && (unsigned)zero_calls == before);
    CHECK(strstr(host_messages,"IMUERR") != NULL);

    CHECK(yaw_start_distance(17,0.0f) == 0);
    CHECK(yaw_arrive(4.0f) == 0);
    CHECK(yaw_wait_correction(-1) == 0);
    before = (unsigned)zero_calls;
    host_yaw = 1.0e10f; yaw_elapsed(20u);
    CHECK(s_round == R_ALIGN && isfinite(last_w) && fabsf(last_w) <= T_DIST_ALIGN_MAX_W);
    CHECK(last_x == 0.0f && last_y == 0.0f && (unsigned)zero_calls == before);
    run_cmd("g"); CHECK(s_round == R_READY && yaw_stopped() && (unsigned)zero_calls == before);
    puts("post-yaw: finite subtraction overflow inRUN/ALIGN stops without zero; huge finite wrapped error returns bounded motion and immediate g remains usable passed");
    return 0;
}

static int check_cross37_heading_disabled_is_exempt(void)
{
    for (unsigned stage = 0u; stage < 3u; ++stage) {
        reset_fixture(); run_cmd("37"); run_cmd("g");
        s_seq_stage = (uint8_t)stage; route_seq_prepare();
        CHECK(sequence_start_stage() == 0);
        CHECK(route_seq_leg()->heading_hold == (stage == 2u));
        CHECK(yaw_arrive(4.0f) == 0);
        unsigned before = (unsigned)zero_calls;
        if (stage < 2u) {
            unsigned dispatch_before = host_precise_calls;
            yaw_elapsed(T_DIST_STILL_MS);
            CHECK(s_seq_stage == stage + 1u && s_seq_state == SQ_STILL && yaw_stopped());
            CHECK((unsigned)zero_calls == before + 1u && host_precise_calls == dispatch_before);
        } else {
            CHECK(yaw_wait_correction(-1) == 0);
            CHECK(s_seq_stage == stage && s_seq_state == SQ_RUN);
            host_yaw = 0.0f; yaw_elapsed(20u);
            CHECK(yaw_wait_zero(before) == 0);
            CHECK(s_seq_state == SQ_DONE && s_seq_stage == 2u && yaw_stopped());
        }
    }
    puts("post-yaw: reverse crossing650 and forward80 board-contact legs remain exempt; normal back190 corrects before route terminal passed");
    return 0;
}

static void yaw_select_route(unsigned owner)
{
    char command[16];
    snprintf(command,sizeof command,"%u",owner); run_cmd(command);
}

static int check_every_route_normal_leg_and_crossing_exemptions(void)
{
    static const unsigned owners[] = {31u,34u,36u,37u};
    unsigned normal_checked = 0u, exempt_checked = 0u;
    for (unsigned n = 0u; n < sizeof owners / sizeof owners[0]; ++n) {
        reset_fixture(); yaw_select_route(owners[n]);
        unsigned count = route_seq_stage_count();
        for (unsigned stage = 0u; stage < count; ++stage) {
            reset_fixture(); yaw_select_route(owners[n]);
            s_seq_stage = (uint8_t)stage;
            const RouteTestLeg *leg = route_seq_leg();
            if (leg->mode < 15u || leg->mode > 18u || !leg->distance_mm) continue;
            int normal = leg->heading_hold != 0u;
            run_cmd("yfix1"); run_cmd("g");
            s_seq_stage = (uint8_t)stage; route_seq_prepare();
            CHECK(sequence_start_stage() == 0 && s_seq_mode == owners[n]);
            CHECK(s_dist_align_enabled == (unsigned)normal);
            CHECK(yaw_arrive(4.0f) == 0);
            unsigned before = (unsigned)zero_calls;
            if (normal) {
                CHECK(yaw_wait_correction(-1) == 0);
                CHECK(s_seq_state == SQ_RUN && s_seq_stage == stage);
                host_yaw = s_dist_heading0; yaw_elapsed(20u);
                CHECK(yaw_wait_zero(before) == 0);
                CHECK(s_seq_state != SQ_RUN && yaw_stopped());
                normal_checked++;
            } else {
                CHECK((leg->mode == 16u && leg->distance_mm == 650u && leg->speed_mms == 300.0f) ||
                      (leg->mode == 15u && leg->distance_mm == 80u && leg->speed_mms == 20.0f));
                unsigned dispatch_before = host_precise_calls;
                yaw_elapsed(T_DIST_STILL_MS);
                CHECK((unsigned)zero_calls == before + 1u && yaw_stopped());
                CHECK(host_precise_calls == dispatch_before && s_seq_state == SQ_STILL);
                exempt_checked++;
            }
        }
    }
    CHECK(normal_checked == 19u && exempt_checked == 8u);
    printf("post-yaw:31/34/36/37 all%u ordinary distance nodes correct before advance; all%u back650/forward80 heading-disabled crossing/contact nodes remain exempt passed\n",normal_checked,exempt_checked);
    return 0;
}

static int check_all_route_owners_cancel_and_toggle(void)
{
    static const unsigned owners[] = {31u,34u,36u,37u};
    static const char *const stops[] = {"g","a","0"};
    for (unsigned n = 0u; n < sizeof owners / sizeof owners[0]; ++n)
        for (unsigned phase = 0u; phase < 3u; ++phase)
            for (unsigned key = 0u; key < 3u; ++key) {
                reset_fixture(); yaw_select_route(owners[n]); run_cmd("g");
                unsigned stage = owners[n] == 37u ? 2u : owners[n] == 36u ? 6u : 0u;
                s_seq_stage = (uint8_t)stage; route_seq_prepare();
                CHECK(sequence_start_stage() == 0 && s_dist_align_enabled);
                CHECK(yaw_arrive(4.0f) == 0);
                if (phase >= 1u) CHECK(yaw_wait_correction(-1) == 0);
                if (phase == 2u) { host_yaw = 0.0f; yaw_elapsed(20u); CHECK(yaw_stopped()); }
                unsigned before = (unsigned)zero_calls;
                run_cmd(stops[key]); CHECK(s_seq_state == SQ_STOPPED && yaw_stopped());
                unsigned dispatch_before = host_precise_calls;
                for (unsigned poll = 0u; poll < 100u; ++poll) {
                    host_yaw = 4.0f; yaw_elapsed(20u);
                    CHECK(s_seq_state == SQ_STOPPED && yaw_stopped());
                    CHECK((unsigned)zero_calls == before && host_precise_calls == dispatch_before);
                }
                run_cmd("g");
                CHECK(s_seq_state == SQ_STOPPED && yaw_stopped());
            }
    for (unsigned n = 0u; n < sizeof owners / sizeof owners[0]; ++n) {
        reset_fixture(); yaw_select_route(owners[n]); run_cmd("yfix0"); run_cmd("g");
        unsigned stage = owners[n] == 37u ? 2u : 0u;
        s_seq_stage = (uint8_t)stage; route_seq_prepare();
        CHECK(sequence_start_stage() == 0 && !s_dist_align_enabled);
        CHECK(yaw_arrive(4.0f) == 0);
        unsigned dispatch_before = host_precise_calls;
        yaw_elapsed(T_DIST_STILL_MS);
        CHECK(yaw_stopped() && host_precise_calls == dispatch_before && s_seq_state != SQ_RUN);
    }
    puts("post-yaw:36 g/a/0 cancellations across all4 route owners and brake/correction/hold; no resume/zero; yfix0 preserves each legacy completion path passed");
    return 0;
}

int main(void)
{
    CHECK(check_normal_done_realigns_before_zero() == 0);
    CHECK(check_wrap_and_already_aligned() == 0);
    CHECK(check_stops_do_not_correct_or_resume() == 0);
    CHECK(check_invalid_imu_and_abort() == 0);
    CHECK(check_toggle_limits_and_failures() == 0);
    CHECK(check_translation_record_not_corrupted_by_rotation() == 0);
    CHECK(check_alignment_owner_write_locks() == 0);
    CHECK(check_subtraction_overflow_and_large_finite_yaw() == 0);
    CHECK(check_cross37_heading_disabled_is_exempt() == 0);
    CHECK(check_every_route_normal_leg_and_crossing_exemptions() == 0);
    CHECK(check_all_route_owners_cancel_and_toggle() == 0);
    puts("translation_post_yaw_test: all host checks passed; physical <=1deg/road projection remains unverified");
    return 0;
}
