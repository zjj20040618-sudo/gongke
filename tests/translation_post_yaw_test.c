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
    if (s_seq_mode == ROUTE_TEST_MODE && s_seq_state == SQ_RUN && s_seq_stage == 3u)
        host_absolute_yaw += angle - host_yaw; /* Coherent two IMU frames through crossing. */
    host_yaw = angle;
    if (route31_owner() && s_seq_stage == 4u) {
        CHECK(fixture_trigger_board_contact() == 0);
    } else if (dist_lateral()) host_lateral = s_dist_odo0 + s_dist_target;
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
    CHECK(fabsf(last_w) <= (dist_align_route31() && s_seq_mode == ROUTE_TEST_MODE
          ? ROUTE31_POST_YAW_MAX_W : T_DIST_ALIGN_MAX_W));
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
    static const unsigned expected_normal[] = {6u,6u,6u,1u};
    unsigned normal_checked = 0u, exempt_checked = 0u, final_checked = 0u;
    for (unsigned n = 0u; n < sizeof owners / sizeof owners[0]; ++n) {
        unsigned owner_normal = 0u, owner_exempt = 0u;
        reset_fixture(); yaw_select_route(owners[n]);
        unsigned count = route_seq_stage_count();
        for (unsigned stage = 0u; stage < count; ++stage) {
            reset_fixture(); yaw_select_route(owners[n]);
            s_seq_stage = (uint8_t)stage;
            const RouteTestLeg *leg = route_seq_leg();
            /* Dynamic31 roads must not disappear from yaw coverage: target
             * color selects12; completed hostage rank, NOT QR shape, selects15. */
            int dynamic = owners[n] == 31u &&
                          (stage == ROUTE31_TARGET_CORNER_STAGE || stage == ROUTE31_HOSTAGE_EXIT_STAGE);
            int contact = owners[n] == 31u && stage == 4u;
            if (leg->mode < 15u || leg->mode > 18u || (!leg->distance_mm && !dynamic && !contact)) continue;
            int normal = leg->heading_hold != 0u;
            run_cmd("yfix1"); run_cmd("g");
            if (owners[n] == 31u) {
                s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3; /* R1 owner snapshot, not a new camera request. */
                s_route31_hostage_rank = 2u; /* Inject the completed-task rank solely for executor coverage. */
                if (contact) {
                    /* Isolated contact-node coverage supplies the genuine
                     * pre-cross reference; never synthesize a post-cross zero. */
                    s_route31_cross_heading_valid = 1u;
                    s_route31_cross_heading = host_absolute_yaw;
                }
            }
            s_seq_stage = (uint8_t)stage; route_seq_prepare();
            leg = route_seq_leg();
            if (dynamic) {
                const unsigned expected = stage == ROUTE31_TARGET_CORNER_STAGE ? 445u : 1315u;
                CHECK(leg->distance_mm == expected && s_d == (float)expected);
            }
            CHECK(sequence_start_stage() == 0 && s_seq_mode == owners[n]);
            CHECK(s_dist_align_enabled == (unsigned)normal);
            CHECK(yaw_arrive(4.0f) == 0);
            unsigned before = (unsigned)zero_calls;
            if (normal) {
                CHECK(yaw_wait_correction(-1) == 0);
                CHECK(s_seq_state == SQ_RUN && s_seq_stage == stage);
                host_yaw = s_dist_heading0;
                yaw_elapsed(20u);
                CHECK(yaw_wait_zero(before) == 0);
                CHECK(s_seq_state != SQ_RUN && yaw_stopped());
                if (owners[n] == 31u && stage == ROUTE31_HOSTAGE_EXIT_STAGE) {
                    CHECK(ROUTE31_HOSTAGE_EXIT_RIGHT_FF_RATIO == 0.075f &&
                          ROUTE43_HOSTAGE_EXIT_RIGHT_FF_RATIO == 0.065f &&
                          s_dist_ff_ratio == -0.075f);
                    CHECK(s_seq_state == SQ_DONE && s_seq_stage == ROUTE31_HOSTAGE_EXIT_STAGE);
                    final_checked++;
                } else {
                    normal_checked++;
                    owner_normal++;
                }
            } else {
                CHECK((leg->mode == 16u && leg->distance_mm == (owners[n] == 31u ? 620u : 650u) && leg->speed_mms == 300.0f) ||
                        (leg->mode == 15u && leg->distance_mm == (owners[n] == 31u ? 0u : 80u) &&
                         leg->speed_mms == (owners[n] == 31u ? 40.0f : 20.0f)));
                unsigned dispatch_before = host_precise_calls;
                yaw_elapsed(T_DIST_STILL_MS);
                if (owners[n] == 31u && stage == 3u) {
                    CHECK(s_seq_state == SQ_CROSS_YAW && s_seq_stage == 4u &&
                          (unsigned)zero_calls == before && yaw_stopped());
                    yaw_elapsed(0u);
                    CHECK(last_w < 0.0f && last_x == 0.0f && last_y == 0.0f &&
                          (unsigned)zero_calls == before);
                    CHECK(fixture_complete_route31_cross_yaw() == 0);
                    CHECK(s_seq_state == SQ_STILL && s_route31_cross_yaw_done &&
                          (unsigned)zero_calls == before && yaw_stopped());
                } else {
                    CHECK((unsigned)zero_calls == before + (contact ? 0u : 1u) && yaw_stopped());
                    if (contact) CHECK(s_seq_stage == 5u && s_seq_contact_post == 0u);
                    CHECK(host_precise_calls == dispatch_before && s_seq_state == SQ_STILL);
                }
                exempt_checked++;
                owner_exempt++;
            }
        }
        CHECK(owner_normal == expected_normal[n] && owner_exempt == 2u);
    }
    CHECK(normal_checked == 19u && final_checked == 1u && exempt_checked == 8u);
    printf("post-yaw:31/34/36/37 all%u distance nodes plus%u final15 correct before advance/DONE; all%u crossing/contact drives keep yaw-disabled;31 cross retains originalheading and corrects before contactzero passed\n",normal_checked,final_checked,exempt_checked);
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
                if (phase == 2u) {
                    host_yaw = owners[n] == 31u ? -0.1f : 0.0f;
                    yaw_elapsed(20u); CHECK(yaw_stopped());
                    CHECK(s_dist_align_hold);
                }
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

/* Exercise the actual distance executor at a normal route node. The fixture
 * supplies a route-owned QR snapshot only; no camera or physical motion runs. */
static int yaw_start_route_align(unsigned owner, unsigned stage, float angle)
{
    reset_fixture(); yaw_select_route(owner); run_cmd("g");
    if (owner == 31u) {
        s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
    }
    s_seq_stage = (uint8_t)stage; route_seq_prepare();
    CHECK(sequence_start_stage() == 0 && s_dist_align_enabled);
    CHECK(s_seq_mode == owner && s_dist_heading0 == 0.0f);
    CHECK(yaw_arrive(angle) == 0);
    yaw_elapsed(T_DIST_STILL_MS);
    CHECK(s_round == R_ALIGN && s_seq_state == SQ_RUN && s_seq_stage == stage);
    yaw_elapsed(0u); /* First alignment poll, not a manufactured elapsed hold. */
    return 0;
}

static int yaw_check31_next_leg(unsigned zero_before)
{
    CHECK((unsigned)zero_calls == zero_before + 1u && host_yaw == 0.0f);
    CHECK(s_seq_mode == 31u && s_seq_stage == 7u && s_seq_state == SQ_STILL);
    CHECK(route_seq_leg()->mode == 16u && route_seq_leg()->distance_mm == 805u &&
          route_seq_leg()->speed_mms == 200.0f && yaw_stopped());
    CHECK(strstr(host_messages, "tol=0.40") != NULL);
    return 0;
}

static int check_route31_continuous_zero_scope(void)
{
    CHECK(ROUTE31_POST_YAW_TOL_DEG == 0.4f && ROUTE31_POST_YAW_MIN_W == 0.30f &&
          ROUTE31_POST_YAW_MAX_W == 0.60f && ROUTE43_POST_YAW_MIN_W == 0.18f &&
          ROUTE31_TURN_MIN_W == 0.30f);
    CHECK(T_DIST_ALIGN_KP == 0.15f && T_DIST_ALIGN_MAX_W == 0.30f &&
          T_DIST_ALIGN_MIN_W == 0.08f);
    CHECK(T_DIST_ALIGN_TOL_DEG == 0.3f && TURN_HOLD_TOL_DEG == 0.3f &&
          TURN90_TOL_DEG == 0.3f && T_TURN_TOL_DEG == 0.3f);
    CHECK(ROUTE31_STABLE_MS == 400u && ROUTE31_POST_YAW_MAX_MS == 2000u &&
          T_DIST_ALIGN_STABLE_MS == 700u && T_DIST_ALIGN_MAX_MS == 12000u &&
          T_DIST_ALIGN_STILL_DEG == 0.2f && ROUTE31_POST_CROSS_ALIGN_MM == 40u);
    reset_fixture(); s_seq_mode = 31u;
    static const unsigned inactive[] = { SQ_OFF, SQ_READY, SQ_STILL, SQ_WAIT,
        SQ_DONE, SQ_STOPPED, SQ_QR_WAIT, SQ_BUCKET_ALIGN, SQ_MANUAL_D_WAIT, SQ_TASK, SQ_ARM_PREP };
    for (unsigned n = 0u; n < sizeof inactive / sizeof inactive[0]; ++n) {
        s_seq_state = (uint8_t)inactive[n];
        CHECK(dist_align_tolerance_deg() == 0.3f);
    }

    static const float signs[] = { -1.0f, 1.0f };
    static const float in_band[] = { 0.0f, 0.01f, 0.29f, 0.39f };
    for (unsigned n = 0u; n < sizeof signs / sizeof signs[0]; ++n)
        for (unsigned sample = 0u; sample < sizeof in_band / sizeof in_band[0]; ++sample) {
            /* No crossing history: even an initially same-side0.39 may hold. */
            CHECK(yaw_start_route_align(31u, 6u, signs[n] * in_band[sample]) == 0);
            unsigned before = (unsigned)zero_calls;
            unsigned dispatch = host_precise_calls;
            CHECK(s_dist_align_hold && yaw_stopped() && dist_align_tolerance_deg() == 0.4f);
            yaw_elapsed(399u);
            CHECK(s_round == R_ALIGN && s_seq_stage == 6u && yaw_stopped());
            CHECK((unsigned)zero_calls == before && host_precise_calls == dispatch);
            yaw_elapsed(1u); CHECK(yaw_check31_next_leg(before) == 0);
            CHECK(host_precise_calls == dispatch && !pulse_calls && !servo_calls);
        }

    CHECK(yaw_start_route_align(31u, 6u, 0.29f) == 0);
    unsigned before = (unsigned)zero_calls;
    uint32_t held_from = s_dist_align_stable_t0;
    for (unsigned poll = 1u; poll <= 19u; ++poll) {
        host_yaw = poll & 1u ? 0.31f : 0.29f; yaw_elapsed(20u);
        CHECK(s_dist_align_hold && s_dist_align_stable_t0 == held_from && yaw_stopped());
        CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
    }
    yaw_elapsed(20u); CHECK(yaw_check31_next_leg(before) == 0);
    puts("post-yaw:31 continuous true0 accepts either-side initial0/0.01/0.29/0.39 without crossing or rotation, requires400ms hold, keeps0.29/0.31 jitter stable and other owners at0.3/700ms passed");
    return 0;
}

static int check_route31_strict_bound_continuity_min_and_cap(void)
{
    static const float signs[] = { -1.0f, 1.0f };
    static const float outside[] = { 0.4f, 0.41f, 1.0f, 1.5f, 2.0f, 4.0f };
    for (unsigned n = 0u; n < sizeof signs / sizeof signs[0]; ++n)
        for (unsigned sample = 0u; sample < sizeof outside / sizeof outside[0]; ++sample) {
            float sign = signs[n], angle = outside[sample];
            float expected = angle * 0.15f;
            if (expected < 0.30f) expected = 0.30f;
            if (expected > 0.60f) expected = 0.60f;
            CHECK(yaw_start_route_align(31u, 6u, sign * angle) == 0);
            unsigned before = (unsigned)zero_calls;
            unsigned dispatch = host_precise_calls;
            CHECK(!s_dist_align_hold && fabsf(last_w + sign * expected) < 0.000001f);
            /* Repeated20ms polls span600ms, both old250ms deadlines, with
             * no brake/gap and a new precise speed command at EVERY poll. */
            for (unsigned poll = 1u; poll <= 30u; ++poll) {
                yaw_elapsed(20u);
                CHECK(s_round == R_ALIGN && s_seq_state == SQ_RUN && s_seq_stage == 6u);
                CHECK(!s_dist_align_hold && (unsigned)zero_calls == before);
                CHECK(host_precise_calls == dispatch + poll && last_x == 0.0f && last_y == 0.0f);
                float expected_now = poll < 20u ? expected : ROUTE31_POST_YAW_MAX_W;
                CHECK(fabsf(last_w + sign * expected_now) < 0.000001f);
                CHECK(s_dist_yaw_progress.boosted == (poll >= 20u));
            }
            /* Feedback can reverse continuously after overshoot.3*.15
             * verifies true-zero P gain, not the removed0.1deg target bias. */
            host_yaw = -sign * 3.0f; yaw_elapsed(20u);
            CHECK(!s_dist_align_hold && fabsf(last_w - sign * 0.45f) < 0.000001f &&
                  !s_dist_yaw_progress.boosted);
            host_yaw = sign * 0.29f; yaw_elapsed(20u);
            CHECK(s_dist_align_hold && yaw_stopped() && (unsigned)zero_calls == before);
            yaw_elapsed(399u);
            CHECK(s_round == R_ALIGN && s_seq_stage == 6u && (unsigned)zero_calls == before);
            yaw_elapsed(1u); CHECK(yaw_check31_next_leg(before) == 0);
            CHECK(!pulse_calls && !servo_calls && !host_laser_on_calls);
        }
    puts("post-yaw:31 both+/-0.4 strict boundary and0.41/1/1.5/2/4 reject completion, continuously command past600ms with min0.30/no-progress400ms-floor0.60, reverse on measured overshoot toward true0 without pulse/gap or bias passed");
    return 0;
}

static int check_route31_continuous_hold_restarts(void)
{
    static const float signs[] = { -1.0f, 1.0f };
    for (unsigned n = 0u; n < sizeof signs / sizeof signs[0]; ++n) {
        float sign = signs[n];
        CHECK(yaw_start_route_align(31u, 6u, sign * 2.0f) == 0);
        unsigned before = (unsigned)zero_calls;
        host_yaw = sign * 0.1f; yaw_elapsed(20u);
        CHECK(s_dist_align_hold && yaw_stopped());
        uint32_t held_from = s_dist_align_stable_t0;
        host_yaw = 0.0f; yaw_elapsed(20u);
        CHECK(s_dist_align_hold && s_dist_align_stable_t0 == held_from && yaw_stopped());
        host_yaw = -sign * 0.05f; yaw_elapsed(20u); /* Either side is legal. */
        CHECK(s_dist_align_hold && s_dist_align_stable_t0 == held_from && yaw_stopped());
        yaw_elapsed(300u);
        host_yaw = sign * 0.4f; yaw_elapsed(20u);
        CHECK(!s_dist_align_hold && last_w * sign < 0.0f && (unsigned)zero_calls == before);
        host_yaw = sign * 0.05f; yaw_elapsed(20u);
        CHECK(s_dist_align_hold && s_dist_align_stable_t0 == host_tick && yaw_stopped());
        yaw_elapsed(300u);
        host_counts[1]++; yaw_elapsed(20u);
        CHECK(s_dist_align_stable_t0 == host_tick && yaw_stopped());
        yaw_elapsed(300u);
        host_yaw = sign * 0.30f; yaw_elapsed(20u); /* Inside band, moved>0.2. */
        CHECK(s_dist_align_stable_t0 == host_tick && s_dist_align_hold && yaw_stopped());
        yaw_elapsed(399u);
        CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
        yaw_elapsed(1u); CHECK(yaw_check31_next_leg(before) == 0);
    }
    puts("post-yaw:31 tiny true0/either-side changes do not demand crossing; strict0.4 excursion, encoder coast and in-band>0.2 angular change restart400ms before zero/advance passed");
    return 0;
}

static int check_route31_new_leg_reset_and_reselect(void)
{
    CHECK(yaw_start_route_align(31u, 6u, 0.29f) == 0);
    unsigned before = (unsigned)zero_calls;
    uint32_t old_hold = s_dist_align_stable_t0;
    yaw_elapsed(399u); CHECK(s_round == R_ALIGN);
    yaw_elapsed(1u); CHECK(yaw_check31_next_leg(before) == 0);
    /* The real next BACK805 must receive its own full400ms stable window. */
    CHECK(sequence_start_stage() == 0 && !s_dist_align_hold);
    CHECK(s_seq_stage == 7u && s_round == R_RUN);
    CHECK(yaw_arrive(-0.29f) == 0);
    yaw_elapsed(T_DIST_STILL_MS); yaw_elapsed(0u);
    CHECK(s_round == R_ALIGN && s_dist_align_hold && yaw_stopped());
    CHECK(s_dist_align_stable_t0 > old_hold);
    before = (unsigned)zero_calls;
    yaw_elapsed(399u);
    CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
    run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && yaw_stopped());
    unsigned dispatch = host_precise_calls;
    yaw_elapsed(800u);
    CHECK(s_seq_state == SQ_STOPPED && yaw_stopped() && host_precise_calls == dispatch);
    CHECK((unsigned)zero_calls == before);

    /* Reselect31 WITHOUT powering off: no old deadline or zero-cross latch. */
    run_cmd("31"); run_cmd("g");
    s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
    s_seq_stage = 6u; route_seq_prepare();
    CHECK(sequence_start_stage() == 0 && !s_dist_align_hold);
    CHECK(yaw_arrive(-0.29f) == 0);
    yaw_elapsed(T_DIST_STILL_MS); yaw_elapsed(0u);
    CHECK(s_dist_align_hold && yaw_stopped());
    before = (unsigned)zero_calls;
    yaw_elapsed(399u); CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
    yaw_elapsed(1u); CHECK(yaw_check31_next_leg(before) == 0);

    CHECK(yaw_start_route_align(31u, 6u, 0.0f) == 0);
    before = (unsigned)zero_calls; dispatch = host_precise_calls;
    CHECK(s_dist_align_hold && yaw_stopped());
    yaw_elapsed(399u); CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
    yaw_elapsed(1u); CHECK(yaw_check31_next_leg(before) == 0);
    CHECK(host_precise_calls == dispatch && !pulse_calls && !servo_calls);
    puts("post-yaw:real nextBACK805, stopped-owner reselect31 and fixture reset each receive fresh400ms hold; initial true0 never issues a forced rotation passed");
    return 0;
}

static int check_route31_continuous_cancel_and_faults(void)
{
    static const char *const stops[] = { "g", "a", "0" };
    for (unsigned phase = 0u; phase < 3u; ++phase)
        for (unsigned key = 0u; key < sizeof stops / sizeof stops[0]; ++key) {
            CHECK(yaw_start_route_align(31u, 6u, phase == 0u ? 4.0f : 0.41f) == 0);
            if (phase == 2u) {
                host_yaw = 0.29f; yaw_elapsed(20u);
                CHECK(s_dist_align_hold && yaw_stopped());
            } else CHECK(!s_dist_align_hold && last_w < 0.0f);
            unsigned before = (unsigned)zero_calls;
            run_cmd(stops[key]); CHECK(s_seq_state == SQ_STOPPED && yaw_stopped());
            unsigned dispatch = host_precise_calls;
            for (unsigned poll = 0u; poll < 40u; ++poll) {
                host_yaw = 0.0f; yaw_elapsed(20u);
                CHECK(s_seq_state == SQ_STOPPED && yaw_stopped() && host_precise_calls == dispatch);
                CHECK((unsigned)zero_calls == before && !pulse_calls && !servo_calls);
            }
            run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && yaw_stopped());
        }
    for (unsigned hold = 0u; hold < 2u; ++hold)
        for (unsigned cause = 0u; cause < 3u; ++cause) {
            CHECK(yaw_start_route_align(31u, 6u, 0.41f) == 0);
            if (hold) {
                host_yaw = 0.29f; yaw_elapsed(20u);
                CHECK(s_dist_align_hold && yaw_stopped());
            }
            unsigned before = (unsigned)zero_calls;
            if (cause == 0u) host_imu_valid = 0;
            if (cause == 1u) host_abort = 1;
            if (cause == 2u) host_yaw = NAN;
            host_tick = s_dist_align_t0 + ROUTE31_POST_YAW_MAX_MS;
            yaw_elapsed(0u); /* Faults win even on the accepted-timeout boundary. */
            CHECK(s_seq_state == SQ_STOPPED && yaw_stopped() && (unsigned)zero_calls == before);
            CHECK(!s_dist_align_timeout_accepted);
            unsigned dispatch = host_precise_calls;
            yaw_elapsed(800u);
            CHECK(s_seq_state == SQ_STOPPED && yaw_stopped() && host_precise_calls == dispatch);
            CHECK((unsigned)zero_calls == before && !pulse_calls && !servo_calls);
        }
    puts("post-yaw:31 nine g/a/0 stops across far/near correction and hold; six IMU/abort/NaN faults take priority at2s and never zero, advance or move later passed");
    return 0;
}

static int check_route31_two_second_timeout_acceptance(void)
{
    static const float angles[] = { -2.0f, 2.0f };
    static const char *const residuals[] = { "residual_err_deg=2.00", "residual_err_deg=-2.00" };
    for (unsigned sample = 0u; sample < 2u; ++sample) {
        CHECK(yaw_start_route_align(31u, 6u, angles[sample]) == 0);
        unsigned zeros = (unsigned)zero_calls;
        uint32_t began = s_dist_align_t0;
        CHECK(!s_dist_align_timeout_accepted && !s_dist_align_hold);
        host_tick = began + 1999u; yaw_elapsed(0u);
        CHECK(s_seq_stage == 6u && s_seq_state == SQ_RUN && s_round == R_ALIGN &&
              (unsigned)zero_calls == zeros && !s_dist_align_timeout_accepted && last_w != 0.0f);
        host_tick = began + 2000u; yaw_elapsed(0u);
        CHECK(yaw_check31_next_leg(zeros) == 0);
        CHECK(s_dist_align_timeout_accepted && s_dist_reason == 1u &&
              strstr(host_messages, "yaw_timeout_accept=1") && strstr(host_messages, residuals[sample]));
        CHECK(!pulse_calls && !servo_calls && !host_laser_on_calls);

        /* A later ordinary leg must not inherit either accepted flag or t0. */
        CHECK(sequence_start_stage() == 0 && !s_dist_align_timeout_accepted);
        CHECK(yaw_arrive(0.1f) == 0);
        yaw_elapsed(T_DIST_STILL_MS); yaw_elapsed(0u);
        CHECK(s_dist_align_t0 > began && s_dist_align_hold && !s_dist_align_timeout_accepted);
        zeros = (unsigned)zero_calls;
        yaw_elapsed(399u); CHECK(s_round == R_ALIGN && (unsigned)zero_calls == zeros);
        yaw_elapsed(1u);
        CHECK((unsigned)zero_calls == zeros + 1u && s_dist_reason == 1u &&
              !s_dist_align_timeout_accepted && strstr(host_messages, "yaw_timeout_accept=0"));
    }

    /* The2s budget includes the400ms stable window. Here the vehicle enters
     * the band at1700ms: at1999 its hold is299ms, not a proven400ms arrival. */
    CHECK(yaw_start_route_align(31u, 6u, 0.6f) == 0);
    unsigned zeros = (unsigned)zero_calls;
    uint32_t began = s_dist_align_t0;
    host_yaw = 0.1f; host_tick = began + 1700u; yaw_elapsed(0u);
    CHECK(s_dist_align_hold && s_dist_align_stable_t0 == host_tick);
    host_tick = began + 1999u; yaw_elapsed(0u);
    CHECK(s_round == R_ALIGN && (unsigned)zero_calls == zeros && !s_dist_align_timeout_accepted);
    host_tick = began + 2000u; yaw_elapsed(0u);
    CHECK(yaw_check31_next_leg(zeros) == 0 && s_dist_align_timeout_accepted);
    CHECK(strstr(host_messages, "residual_err_deg=-0.10"));

    static const char *const keys[] = { "g", "a", "0" };
    for (unsigned key = 0u; key < 3u; ++key) {
        CHECK(yaw_start_route_align(31u, 6u, 2.0f) == 0);
        zeros = (unsigned)zero_calls;
        host_tick = s_dist_align_t0 + 1999u; yaw_elapsed(0u);
        run_cmd(keys[key]); yaw_elapsed(1u);
        CHECK(s_seq_state == SQ_STOPPED && yaw_stopped() &&
              (unsigned)zero_calls == zeros && !s_dist_align_timeout_accepted);
    }
    /* Both finite values may overflow their subtraction; this fault cannot
     * be turned into success merely because2s elapsed. */
    CHECK(yaw_start_route_align(31u, 6u, 2.0f) == 0);
    zeros = (unsigned)zero_calls;
    s_dist_heading0 = -FLT_MAX; host_yaw = FLT_MAX;
    host_tick = s_dist_align_t0 + 2000u; yaw_elapsed(0u);
    CHECK(s_seq_state == SQ_STOPPED && yaw_stopped() && !s_dist_align_timeout_accepted &&
          (unsigned)zero_calls == zeros && strstr(host_messages, "IMUERR"));

    /* Isolated distance and43 use the original12s failure, not31's trial
     * continuation.43's step gate also must not advance at2s. */
    for (unsigned legacy = 0u; legacy < 2u; ++legacy) {
        if (legacy) CHECK(yaw_start_route_align(43u, 6u, 2.0f) == 0);
        else {
            CHECK(yaw_start_distance(17, 0.0f) == 0 && yaw_arrive(2.0f) == 0);
            yaw_elapsed(T_DIST_STILL_MS); yaw_elapsed(0u);
        }
        zeros = (unsigned)zero_calls; began = s_dist_align_t0;
        host_tick = began + 2000u; yaw_elapsed(0u);
        CHECK(s_round == R_ALIGN && !s_dist_align_timeout_accepted && (unsigned)zero_calls == zeros);
        host_tick = began + T_DIST_ALIGN_MAX_MS; yaw_elapsed(0u);
        CHECK(s_round == (legacy ? R_DONE : R_READY) && yaw_stopped() && (unsigned)zero_calls == zeros &&
              !s_dist_align_timeout_accepted && s_dist_reason == 3u);
        if (legacy) CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 6u);
    }
    puts("post-yaw:31 exact1999/2000ms acceptance logs signed PREZERO residual/flag, includes stable400 budget, resets per leg;deadline g/a/0 and overflow win;standalone/43 keep12s failure passed");
    return 0;
}

static int check_route31_wrapped_side_and_turn_isolation(void)
{
    static const float wrapped[] = { 359.0f, -359.0f };
    for (unsigned n = 0u; n < sizeof wrapped / sizeof wrapped[0]; ++n) {
        float direction = n == 0u ? 1.0f : -1.0f;
        CHECK(yaw_start_route_align(31u, 6u, wrapped[n]) == 0);
        CHECK(last_w * direction > 0.0f && fabsf(last_w) == 0.30f);
        unsigned before = (unsigned)zero_calls;
        host_yaw = direction * 359.75f; yaw_elapsed(20u);
        CHECK(s_dist_align_hold && yaw_stopped());
        yaw_elapsed(399u); CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
        yaw_elapsed(1u); CHECK(yaw_check31_next_leg(before) == 0);
    }
    /* Real31 pre-cross right90, hostage right93 and left90, plus standalone20/22/30, retain the0.3
     * acceptance band and may finish on the approach side. The31 distance
     * correction's0.4 band must not change these executors'0.3 band. */
    static const unsigned modes[] = { 31u, 31u, 31u, 20u, 22u, 30u };
    static const unsigned stages[] = { 2u, 8u, 13u, 0u, 0u, 0u };
    for (unsigned n = 0u; n < sizeof modes / sizeof modes[0]; ++n) {
        reset_fixture(); yaw_select_route(modes[n]); run_cmd("g");
        if (modes[n] == 31u) {
            s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
            s_seq_stage = (uint8_t)stages[n]; route_seq_prepare();
            CHECK(sequence_start_stage() == 0);
            CHECK(turn_target_deg() == (stages[n] == 8u ? -90.0f :
                                       stages[n] == ROUTE31_HOSTAGE_TURN_STAGE ? 93.0f : 90.0f));
        } else CHECK(s_round == R_RUN);
        float target = turn_target_deg();
        float direction = target > 0.0f ? 1.0f : -1.0f;
        unsigned before = (unsigned)zero_calls;
        unsigned precise_before = host_precise_calls, integer_before = host_integer_calls;
        host_yaw = target - direction * 0.35f; yaw_elapsed(20u);
        CHECK(s_round == R_RUN && last_w * direction > 0.0f);
        CHECK(host_precise_calls == precise_before + (modes[n] == 31u));
        CHECK(host_integer_calls == integer_before + (modes[n] != 31u));
        yaw_elapsed(700u);
        CHECK(s_round == R_RUN && (unsigned)zero_calls == before);
        host_yaw = target - direction * 0.25f; yaw_elapsed(20u);
        CHECK(s_round == R_BRAKE && yaw_stopped() && !strcmp(s_turn_result, "DONE"));
        yaw_elapsed(modes[n] == 31u ? 399u : 699u);
        CHECK(s_round == R_BRAKE && (unsigned)zero_calls == before);
        yaw_elapsed(1u);
        CHECK(yaw_stopped() && (unsigned)zero_calls == before);
        CHECK(strstr(host_messages, "status=DONE") != NULL);
        if (modes[n] == 31u) {
            CHECK(s_seq_stage == stages[n] + 1u);
            CHECK(s_seq_state == (stages[n] == 8u || stages[n] == 13u ? SQ_TASK : SQ_STILL));
        }
        else CHECK(s_round == R_DONE);
    }
    puts("post-yaw:31 wrapped +/-359 correction follows shortest signed error and accepts same-side wrapped0.25; real31 both right90/left90 and independent20/22/30 still reject0.35, accept same-side0.25 after400ms(31)/700ms(independent) passed");
    return 0;
}

static int check_non31_post_yaw_tolerance_preserved(void)
{
    for (int mode = 15; mode <= 18; ++mode) {
        CHECK(yaw_start_distance(mode, 0.0f) == 0 && yaw_arrive(0.31f) == 0);
        yaw_elapsed(T_DIST_STILL_MS); yaw_elapsed(0u);
        unsigned before = (unsigned)zero_calls;
        CHECK(dist_align_tolerance_deg() == 0.3f && !s_dist_align_hold && last_w == -0.08f);
        yaw_elapsed(700u);
        CHECK(s_round == R_ALIGN && !s_dist_align_hold && (unsigned)zero_calls == before);
        CHECK(last_w == -0.08f && !s_dist_yaw_progress.active);
        host_yaw = 0.3f; yaw_elapsed(0u);
        CHECK(s_dist_align_hold && yaw_stopped());
        yaw_elapsed(699u); CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
        yaw_elapsed(1u);
        CHECK(s_round == R_READY && (unsigned)zero_calls == before + 1u && yaw_stopped());
        CHECK(strstr(host_messages, "tol=0.30") != NULL);
    }
    static const unsigned owners[] = {34u,36u,37u};
    for (unsigned n = 0u; n < sizeof owners / sizeof owners[0]; ++n) {
        unsigned stage = owners[n] == 37u ? 2u : 1u;
        CHECK(yaw_start_route_align(owners[n], stage, 0.31f) == 0);
        unsigned before = (unsigned)zero_calls;
        CHECK(dist_align_tolerance_deg() == 0.3f && !s_dist_align_hold && last_w == -0.08f);
        yaw_elapsed(700u);
        CHECK(s_round == R_ALIGN && s_seq_state == SQ_RUN && s_seq_stage == stage);
        CHECK(last_w == -0.08f && !s_dist_yaw_progress.active);
        CHECK(!s_dist_align_hold && (unsigned)zero_calls == before);
        host_yaw = 0.3f; yaw_elapsed(0u);
        CHECK(s_dist_align_hold && yaw_stopped());
        yaw_elapsed(699u); CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
        yaw_elapsed(1u);
        CHECK((unsigned)zero_calls == before + 1u && yaw_stopped());
        if (owners[n] == 37u) CHECK(s_seq_state == SQ_DONE && s_seq_stage == 2u);
        else {
            CHECK(s_seq_state == SQ_STILL && s_seq_stage == 2u);
            CHECK(route_seq_leg()->mode == 20u && route_seq_leg()->distance_mm == 0u);
        }
        CHECK(strstr(host_messages, "tol=0.30") != NULL);
    }
    puts("post-yaw:standalone15..18 and34/36/37 still reject0.31, accept0.3 only after700ms and preserve original next/terminal states passed");
    return 0;
}

static int check_turn_body_timeout_is_not_endpoint_cap(void)
{
    static const unsigned owners[] = { 31u, 31u, 20u, 22u, 30u };
    static const unsigned stages[] = { 2u, ROUTE31_RETURN180_STAGE, 0u, 0u, 0u };
    CHECK(T_TURN_MAX_MS == 12000u && T_TURN180_MAX_MS == 12000u);
    for (unsigned sample = 0u; sample < 5u; ++sample) {
        reset_fixture(); yaw_select_route(owners[sample]); run_cmd("g");
        if (owners[sample] == 31u) {
            s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
            s_seq_stage = (uint8_t)stages[sample]; route_seq_prepare();
            CHECK(sequence_start_stage() == 0);
        }
        CHECK(turn_closed_loop_mode() && s_round == R_RUN && turn_max_ms() == 12000u);
        unsigned zeros = (unsigned)zero_calls;
        uint32_t began = s_meas_t0;
        host_yaw = 0.0f; host_tick = began + 2000u; yaw_elapsed(0u);
        CHECK(s_round == R_RUN && (unsigned)zero_calls == zeros && last_w != 0.0f);
        if (owners[sample] == 31u) CHECK(s_seq_state == SQ_RUN && s_seq_stage == stages[sample]);
        host_tick = began + 11999u; yaw_elapsed(0u);
        CHECK(s_round == R_RUN && (unsigned)zero_calls == zeros && last_w != 0.0f);
        host_tick = began + 12000u; yaw_elapsed(0u);
        CHECK(s_round == R_BRAKE && yaw_stopped() && !strcmp(s_turn_result, "TIMEOUT"));
        yaw_elapsed(turn_settle_ms());
        CHECK(s_round == R_DONE && (unsigned)zero_calls == zeros && yaw_stopped());
        if (owners[sample] == 31u) CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == stages[sample]);
    }
    puts("post-yaw:31 body90/body185 and standalone20/22/30 remainRUN at2s/11999;only12s body timeout fails,never endpoint-cap success passed");
    return 0;
}

static int check_route31_turn_endpoint_strength_scope(void)
{
    const unsigned owners[] = {31u,31u,31u,43u,43u,20u,22u,30u};
    const unsigned stages[] = {2u,8u,ROUTE31_RETURN180_STAGE,2u,ROUTE31_RETURN180_STAGE,0u,0u,0u};
    const float offsets[] = {-0.5f,0.5f};
    CHECK(ROUTE31_TURN_MIN_W == 0.30f && T_TURN_MIN_W == 0.18f &&
          T_TURN_MAX_W == 2.0f && T_TURN_TOL_DEG == 0.3f);
    for (unsigned n=0u;n<sizeof owners/sizeof owners[0];++n) {
        for (unsigned side=0u;side<2u;++side) {
            reset_fixture(); yaw_select_route(owners[n]); run_cmd("g");
            if (owners[n]==31u || owners[n]==43u) {
                s_seq_qr[0]=1;s_seq_qr[1]=2;s_seq_qr[2]=3;
                s_seq_stage=(uint8_t)stages[n];route_seq_prepare();
                CHECK(sequence_start_stage()==0 && turn_closed_loop_mode());
            }
            float goal=turn_target_deg();
            if (owners[n]==31u && stages[n]==2u) CHECK(goal==90.0f);
            if (owners[n]==31u && stages[n]==8u) CHECK(goal==-90.0f);
            if ((owners[n]==43u && stages[n]==2u) || owners[n]==20u) CHECK(goal==90.0f);
            if (stages[n]==ROUTE31_RETURN180_STAGE && (owners[n]==31u || owners[n]==43u))
                CHECK(goal==(owners[n]==31u?185.0f:180.0f));
            if (owners[n]==22u) CHECK(goal==180.0f);
            unsigned zeros=(unsigned)zero_calls, precise=host_precise_calls, integer=host_integer_calls;
            host_yaw=goal+offsets[side];yaw_elapsed(20u);
            float expected=offsets[side]<0.0f?1.0f:-1.0f;
            expected*=owners[n]==31u?0.30f:0.18f;
            CHECK(s_round==R_RUN && last_x==0.0f && last_y==0.0f &&
                  fabsf(last_w-expected)<0.000001f && (unsigned)zero_calls==zeros);
            CHECK(host_precise_calls==precise+(owners[n]==31u) &&
                  host_integer_calls==integer+(owners[n]!=31u));
            yaw_elapsed(400u);
            CHECK(s_round==R_RUN && fabsf(last_w-expected)<0.000001f && (unsigned)zero_calls==zeros);
            run_cmd("0");CHECK(yaw_stopped());
        }
    }
    puts("31 real right90/left90/return185 at goal+/-0.5 command precise pure-yaw+/-0.30 without zero or prematureDONE;43 and standalone20 right90,43 return180/standalone22 remain180 and retain0.18/integer;primary turn has no endpoint0.60 boost passed");
    return 0;
}

static int check_route31_both_right90_cancel_scope(void)
{
    static const unsigned stages[]={2u,13u};
    static const char *const stops[]={"g","a","0"};
    for (unsigned stage=0u;stage<2u;stage++)
        for (unsigned phase=0u;phase<2u;phase++)
            for (unsigned key=0u;key<3u;key++) {
                reset_fixture();run_cmd("31");run_cmd("g");
                s_seq_qr[0]=1;s_seq_qr[1]=2;s_seq_qr[2]=3;
                s_seq_stage=(uint8_t)stages[stage];route_seq_prepare();
                float goal=stages[stage]==ROUTE31_HOSTAGE_TURN_STAGE ? 93.0f : 90.0f;
                CHECK(turn_target_deg()==goal && sequence_start_stage()==0);
                unsigned zeros=(unsigned)zero_calls;
                host_yaw=goal-5.0f;yaw_elapsed(20u);
                CHECK(s_round==R_RUN && last_w>0.0f && !last_x && !last_y &&
                      (unsigned)zero_calls==zeros); /* Five degrees short is not arrival. */
                host_yaw=goal+0.5f;yaw_elapsed(20u);
                CHECK(s_round==R_RUN && last_w<0.0f && !last_x && !last_y);
                if (phase) {
                    host_yaw=goal;yaw_elapsed(20u);
                    CHECK(s_round==R_BRAKE && yaw_stopped());
                    yaw_elapsed(399u);CHECK(s_round==R_BRAKE);
                }
                run_cmd(stops[key]);CHECK(s_seq_state==SQ_STOPPED && yaw_stopped());
                unsigned dispatch=host_precise_calls;
                yaw_elapsed(1000u);
                CHECK(s_seq_state==SQ_STOPPED && s_seq_stage==stages[stage] &&
                      yaw_stopped() && host_precise_calls==dispatch && (unsigned)zero_calls==zeros);
            }
    puts("31 pre-cross right90/hostage right93:goal-5 cannot finish,goal+0.5 reverses;12 g/a/0 RUN/BRAKE cancellations never advance/restart/zero passed");
    return 0;
}

static int check_route31_no_progress_boost_and_resets(void)
{
    static const float signs[] = { -1.0f, 1.0f };
    for (unsigned n = 0u; n < sizeof signs / sizeof signs[0]; ++n) {
        float sign = signs[n];
        CHECK(yaw_start_route_align(31u, 6u, sign * 0.6f) == 0);
        unsigned before = (unsigned)zero_calls;
        CHECK(s_dist_yaw_progress.active && !s_dist_yaw_progress.boosted &&
              fabsf(last_w + sign * 0.30f) < 0.000001f);
        yaw_elapsed(399u);
        CHECK(!s_dist_yaw_progress.boosted && fabsf(last_w + sign * 0.30f) < 0.000001f);
        yaw_elapsed(1u);
        CHECK(s_dist_yaw_progress.boosted && fabsf(last_w + sign * 0.60f) < 0.000001f);
        /* Motion away from the goal is not improvement and cannot erase boost. */
        host_yaw = sign * 0.7f; yaw_elapsed(20u);
        CHECK(s_dist_yaw_progress.boosted && fabsf(last_w + sign * 0.60f) < 0.000001f);
        /* Cumulative improvement is measured from the original reference0.6. */
        host_yaw = sign * 0.551f; yaw_elapsed(20u);
        CHECK(s_dist_yaw_progress.boosted);
        host_yaw = sign * 0.55f; yaw_elapsed(20u);
        CHECK(!s_dist_yaw_progress.boosted && s_dist_yaw_progress.since == host_tick &&
              fabsf(last_w + sign * 0.30f) < 0.000001f);
        yaw_elapsed(400u); CHECK(s_dist_yaw_progress.boosted);
        /* A sign change restores the normal floor immediately, not after a pulse. */
        host_yaw = -sign * 0.6f; yaw_elapsed(20u);
        CHECK(!s_dist_yaw_progress.boosted && s_dist_yaw_progress.since == host_tick &&
              fabsf(last_w - sign * 0.30f) < 0.000001f);
        yaw_elapsed(400u); CHECK(s_dist_yaw_progress.boosted);
        host_yaw = sign * 0.29f; yaw_elapsed(20u);
        CHECK(!s_dist_yaw_progress.active && !s_dist_yaw_progress.boosted && s_dist_align_hold && yaw_stopped());
        yaw_elapsed(399u); CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
        yaw_elapsed(1u); CHECK(yaw_check31_next_leg(before) == 0);
        CHECK(sequence_start_stage() == 0 && s_seq_stage == 7u);
        CHECK(yaw_arrive(sign * 0.6f) == 0);
        yaw_elapsed(T_DIST_STILL_MS); yaw_elapsed(0u);
        CHECK(s_round == R_ALIGN && !s_dist_yaw_progress.boosted &&
              fabsf(last_w + sign * 0.30f) < 0.000001f);
        run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && yaw_stopped());
    }
    static const char *const stops[] = { "g", "a", "0" };
    for (unsigned n = 0u; n < sizeof stops / sizeof stops[0]; ++n) {
        CHECK(yaw_start_route_align(31u, 6u, 0.41f) == 0);
        yaw_elapsed(400u); CHECK(s_dist_yaw_progress.boosted && last_w == -0.60f);
        unsigned before = (unsigned)zero_calls;
        run_cmd(stops[n]); CHECK(s_seq_state == SQ_STOPPED && yaw_stopped());
        unsigned dispatch = host_precise_calls;
        yaw_elapsed(800u);
        CHECK(s_seq_state == SQ_STOPPED && host_precise_calls == dispatch &&
              (unsigned)zero_calls == before && yaw_stopped());
        /* Reselect without reboot; the next episode must wait a fresh400ms. */
        run_cmd("31"); run_cmd("g");
        s_seq_qr[0] = 1; s_seq_qr[1] = 2; s_seq_qr[2] = 3;
        s_seq_stage = 6u; route_seq_prepare(); CHECK(sequence_start_stage() == 0);
        CHECK(yaw_arrive(0.41f) == 0);
        yaw_elapsed(T_DIST_STILL_MS); yaw_elapsed(0u);
        CHECK(!s_dist_yaw_progress.boosted && last_w == -0.30f);
        yaw_elapsed(399u); CHECK(!s_dist_yaw_progress.boosted && last_w == -0.30f);
        yaw_elapsed(1u); CHECK(s_dist_yaw_progress.boosted && last_w == -0.60f);
    }
    /*43 remains a route-owned0.4-degree endpoint, but keeps700ms/no boost. */
    CHECK(yaw_start_route_align(43u, 6u, 0.41f) == 0);
    CHECK(dist_align_tolerance_deg() == 0.4f && dist_align_stable_ms() == 700u && last_w == -0.18f);
    yaw_elapsed(400u); CHECK(!s_dist_yaw_progress.active && last_w == -0.18f);
    yaw_elapsed(400u); CHECK(!s_dist_yaw_progress.boosted && last_w == -0.18f);
    unsigned before = (unsigned)zero_calls;
    host_yaw = 0.29f; yaw_elapsed(20u); CHECK(s_dist_align_hold && yaw_stopped());
    yaw_elapsed(699u); CHECK(s_round == R_ALIGN && (unsigned)zero_calls == before);
    yaw_elapsed(1u);
    CHECK((unsigned)zero_calls == before + 1u && s_seq_mode == 43u &&
          s_seq_stage == 7u && s_seq_state == SQ_STEP_WAIT && yaw_stopped());
    puts("post-yaw:31 actual .30->.60 only after400ms insufficient progress;wrong-way/0.049 preserve boost,0.05/sign/in-band/newleg/reselect reset;g/a/0 stop boosted command;43 retains.18/max.30/700ms/no boost passed");
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
    CHECK(check_route31_continuous_zero_scope() == 0);
    CHECK(check_route31_strict_bound_continuity_min_and_cap() == 0);
    CHECK(check_route31_continuous_hold_restarts() == 0);
    CHECK(check_route31_new_leg_reset_and_reselect() == 0);
    CHECK(check_route31_continuous_cancel_and_faults() == 0);
    CHECK(check_route31_two_second_timeout_acceptance() == 0);
    CHECK(check_route31_wrapped_side_and_turn_isolation() == 0);
    CHECK(check_non31_post_yaw_tolerance_preserved() == 0);
    CHECK(check_turn_body_timeout_is_not_endpoint_cap() == 0);
    CHECK(check_route31_turn_endpoint_strength_scope() == 0);
    CHECK(check_route31_both_right90_cancel_scope() == 0);
    CHECK(check_route31_no_progress_boost_and_resets() == 0);
    puts("translation_post_yaw_test: all host checks passed; physical yaw/road projection remains unverified");
    return 0;
}
