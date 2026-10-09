/* Real App/test.c contact owner/state machine, with host-only IMU snapshots.
 * These checks prove software gating, not that physical tilt guarantees
 * both sides of a chassis have squared against a board. */
#define main retained_g_fixture_main
#include "g_command_stop_test.c"
#undef main

static int board_stopped(void)
{
    return last_x == 0.0f && last_y == 0.0f && last_w == 0.0f;
}

static void board_sample(float pitch, float roll, uint32_t elapsed)
{
    host_pitch = pitch; host_roll = roll; host_imu_age = 0u;
    host_tick += elapsed;
    test_poll();
}

static void board_finish_guard(void)
{
    uint32_t elapsed = (uint32_t)(host_tick - s_meas_t0);
    if (elapsed < ROUTE31_BOARD_CONTACT_START_GUARD_MS)
        board_sample(host_pitch, host_roll, ROUTE31_BOARD_CONTACT_START_GUARD_MS - elapsed);
}

static int board_start(unsigned owner, float pitch, float roll)
{
    char command[8];
    reset_fixture();
    host_pitch = pitch; host_roll = roll;
    snprintf(command, sizeof command, "%u", owner); run_cmd(command); run_cmd("g");
    s_seq_stage = 4u; route_seq_prepare();
    CHECK(s_seq_state == SQ_STILL && s_d == -1.0f && !s_board_contact.active);
    CHECK(sequence_start_stage() == 0);
    CHECK(route31_contact_active() && s_board_contact.active && s_board_contact.zeroed && s_msel == 15 &&
          s_dist_target == 0.0f && s_v == 40.0f && s_dist_heading_kp == 0.0f &&
          s_dist_ff_ratio == 0.0f && !s_dist_align_enabled &&
          last_x == 40.0f && last_y == 0.0f && last_w == 0.0f);
    CHECK(s_board_contact.pitch0 == pitch && s_board_contact.roll0 == roll &&
          s_board_contact.hits == 0u && !s_board_contact.confirming &&
          s_board_contact.sample_ms == host_tick);
    return 0;
}

static int board_nudge_start(unsigned owner, unsigned post, unsigned zeros)
{
    CHECK(s_seq_stage == 4u && s_seq_contact_post == post && !s_board_contact.active &&
          board_stopped() && (unsigned)zero_calls == zeros);
    if (owner == 43u) {
        CHECK(s_seq_state == SQ_STEP_WAIT && s_route43_pending == R43_PENDING_PREP);
        host_tick += 1000u; test_poll();
        CHECK(s_seq_state == SQ_STEP_WAIT && board_stopped() && (unsigned)zero_calls == zeros);
        run_cmd("g");
    }
    CHECK(s_seq_state == SQ_STILL && s_msel == (post == 1u ? 16 : 15) &&
          s_d == (post == 1u ? 10.0f : 15.0f) && s_v == 40.0f &&
          !route_seq_leg()->heading_hold);
    host_tick += T_DIST_STILL_MS - 1u; test_poll();
    CHECK(s_seq_state == SQ_STILL && board_stopped() && (unsigned)zero_calls == zeros);
    host_counts[1]++; host_tick++; test_poll();
    CHECK(s_seq_state == SQ_STILL && board_stopped() && (unsigned)zero_calls == zeros);
    host_tick += T_DIST_STILL_MS - 1u; test_poll();
    CHECK(s_seq_state == SQ_STILL && board_stopped() && (unsigned)zero_calls == zeros);
    host_tick++; test_poll();
    CHECK(s_seq_state == SQ_RUN && s_round == R_RUN && !s_board_contact.active &&
          s_dist_target == (post == 1u ? -10.0f : 15.0f) &&
          s_dist_heading_kp == 0.0f && s_dist_ff_ratio == 0.0f && !s_dist_align_enabled &&
          last_x == (post == 1u ? -40.0f : 40.0f) && last_y == 0.0f && last_w == 0.0f &&
          (unsigned)zero_calls == zeros); /* No zero or 750ms WAIT between nudges. */
    return 0;
}

static int board_nudge_finish(unsigned zeros)
{
    unsigned post = s_seq_contact_post;
    CHECK(post == 1u || post == 2u);
    CHECK(s_dist_target == (post == 1u ? -10.0f : 15.0f));
    host_yaw = 5.0f;
    host_fore = s_dist_odo0 + (post == 1u ? -9.99f : 14.99f);
    host_tick += 20u; test_poll();
    CHECK(s_round == R_RUN && s_seq_state == SQ_RUN && s_seq_stage == 4u &&
          s_seq_contact_post == post && !s_board_contact.active &&
          (post == 1u ? last_x < 0.0f : last_x > 0.0f) &&
          last_y == 0.0f && last_w == 0.0f && host_yaw == 5.0f &&
          (unsigned)zero_calls == zeros);
    host_fore = s_dist_odo0 + (post == 1u ? -10.0f : 15.0f);
    host_tick += 20u; test_poll();
    CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && board_stopped() &&
          s_dist_reason == 1u && s_seq_contact_post == post && !s_board_contact.active &&
          host_yaw == 5.0f && (unsigned)zero_calls == zeros);
    host_tick += T_DIST_STILL_MS - 1u; test_poll();
    CHECK(s_round == R_BRAKE && s_seq_state == SQ_RUN && (unsigned)zero_calls == zeros);
    host_tick++; test_poll();
    CHECK(board_stopped() && host_yaw == 5.0f && (unsigned)zero_calls == zeros);
    return 0;
}

static int board_success_finish(unsigned owner)
{
    CHECK(s_round == R_BRAKE && s_dist_reason == 1u && board_stopped());
    unsigned zeros = (unsigned)zero_calls;
    host_tick += 249u; test_poll();
    CHECK(s_seq_stage == 4u && s_seq_state == SQ_RUN && s_round == R_BRAKE &&
          (unsigned)zero_calls == zeros);
    host_counts[2]++; host_tick += 1u; test_poll();
    CHECK(s_seq_stage == 4u && s_round == R_BRAKE && (unsigned)zero_calls == zeros);
    host_tick += 249u; test_poll();
    CHECK(s_seq_stage == 4u && s_round == R_BRAKE && (unsigned)zero_calls == zeros);
    host_tick += 1u; test_poll();
    CHECK(s_seq_stage == 4u && s_seq_contact_post == 1u && !s_board_contact.active && board_stopped() &&
          (unsigned)zero_calls == zeros && strstr(host_messages, "result=TILT_REACHED"));
    CHECK(strstr(host_messages, "SEQ phase=BOARD_CONTACT_DONE next_BACK10_v40 no_yaw_or_ff=1"));
    CHECK(board_nudge_start(owner, 1u, zeros) == 0 && board_nudge_finish(zeros) == 0);
    CHECK(s_seq_stage == 4u && s_seq_contact_post == 2u);
    CHECK(strstr(host_messages, "SEQ phase=BOARD_BACK10_DONE next_FORWARD15_v40 no_yaw_or_ff=1"));
    CHECK(board_nudge_start(owner, 2u, zeros) == 0 && board_nudge_finish(zeros) == 0);
    CHECK(s_seq_stage == 5u && !s_seq_contact_post);
    CHECK(strstr(host_messages, "SEQ phase=BOARD_FORWARD15_DONE next_still_rezero_settle_then_BACK190"));
    if (owner == 43u) {
        CHECK(s_seq_state == SQ_STEP_WAIT && s_route43_pending == R43_PENDING_PREP);
        host_tick += 1000u; test_poll();
        CHECK(s_seq_state == SQ_STEP_WAIT && board_stopped() && (unsigned)zero_calls == zeros);
        run_cmd("g");
    }
    CHECK(s_seq_state == SQ_STILL && s_d == 190.0f && s_msel == 16 && s_v == 200.0f);
    host_tick += T_DIST_STILL_MS - 1u; test_poll();
    CHECK(s_seq_state == SQ_STILL && board_stopped() && (unsigned)zero_calls == zeros);
    host_tick++; test_poll();
    CHECK(s_seq_state == SQ_WAIT && board_stopped() && (unsigned)zero_calls == zeros + 1u && host_yaw == 0.0f);
    host_tick += NAV_SETTLE_MS - 1u; test_poll();
    CHECK(s_seq_state == SQ_WAIT && board_stopped() && (unsigned)zero_calls == zeros + 1u);
    host_tick++; test_poll();
    CHECK(s_seq_state == SQ_RUN && s_seq_stage == 5u && !s_board_contact.active &&
          s_dist_heading_kp == 0.3f && s_dist_ff_ratio == 0.00625f && last_x == -200.0f);
    return 0;
}

static int check_stopped_rezero_and_rolling_restart(void)
{
    for (unsigned owner = 31u; owner <= 43u; owner += 12u) {
        char command[8];
        reset_fixture(); snprintf(command, sizeof command, "%u", owner); run_cmd(command); run_cmd("g");
        host_pitch = 1.0f; host_roll = 2.0f; s_seq_stage = 4u; route_seq_prepare();
        host_tick += T_DIST_STILL_MS - 1u; test_poll();
        CHECK(s_seq_state == SQ_STILL && board_stopped() && !s_board_contact.zeroed);
        host_tick++; test_poll();
        CHECK(s_seq_state == SQ_WAIT && board_stopped() && !s_board_contact.zeroed);
        unsigned zeros = (unsigned)zero_calls;
        host_tick += 500u; host_counts[1]++; test_poll();
        CHECK(s_seq_state == SQ_STILL && board_stopped() && !s_board_contact.zeroed);
        host_tick += T_DIST_STILL_MS - 1u; test_poll();
        CHECK(s_seq_state == SQ_STILL && !s_board_contact.zeroed);
        host_tick++; test_poll();
        CHECK(s_seq_state == SQ_WAIT && (unsigned)zero_calls == zeros + 1u);
        host_pitch = 7.0f; host_roll = -8.0f;
        host_tick += route_contact_zero_wait_ms() - 1u; test_poll();
        CHECK(s_seq_state == SQ_WAIT && board_stopped() && !s_board_contact.zeroed);
        host_tick++; test_poll();
        CHECK(s_seq_state == SQ_RUN && s_round == R_RUN && s_board_contact.zeroed && s_board_contact.active &&
              s_board_contact.pitch0 == 7.0f && s_board_contact.roll0 == -8.0f &&
              s_board_contact.sample_ms == host_tick && last_x == 40.0f);
        board_sample(9.0f, -10.0f, 20u);
        CHECK(s_round == R_RUN && !s_board_contact.hits && !s_board_contact.confirming &&
              s_board_contact.pitch0 == 7.0f && s_board_contact.roll0 == -8.0f);
        board_sample(7.0f, -8.0f, ROUTE31_BOARD_CONTACT_START_GUARD_MS);
        CHECK(s_round == R_RUN && !s_board_contact.hits);
    }
    puts("board tilt:31/43 wheel-still250 then300/1000ms stopped rezero, wheel motion restarts wait, frozen baseline survives startup changes passed");
    return 0;
}

static int check_startup_log_and_guard(void)
{
    CHECK(board_start(31u, 1.71f, 1.72f) == 0);
    /* User log: +0.56degrees at100ms and1.7mm previously caused immediate
     * success. It is below1.5 and inside the startup guard now. */
    host_fore = s_dist_odo0 + 1.7f;
    board_sample(2.27f, 1.73f, 80u); board_sample(2.27f, 1.73f, 20u);
    CHECK(s_round == R_RUN && !s_board_contact.hits && last_x == 40.0f);
    board_sample(2.27f, 1.73f, ROUTE31_BOARD_CONTACT_START_GUARD_MS);
    for (unsigned i = 0u; i < 6u; ++i) board_sample(2.27f, 1.73f, 20u);
    CHECK(s_round == R_RUN && !s_board_contact.hits && s_board_contact.pitch0 == 1.71f);
    CHECK(board_start(31u, 0.0f, 0.0f) == 0);
    float raised = ROUTE31_BOARD_CONTACT_TILT_DEG + 0.1f;
    board_sample(raised, 0.0f, 100u); board_sample(raised, 0.0f, 100u);
    CHECK(s_round == R_RUN && !s_board_contact.hits && !s_board_contact.confirming);
    board_sample(0.0f, 0.0f, 100u);
    board_sample(raised, 0.0f, 20u);
    CHECK(s_round == R_RUN && s_board_contact.hits == 1u && s_board_contact.confirming);
    board_sample(raised, 0.0f, ROUTE31_BOARD_CONTACT_CONFIRM_MS - 1u);
    CHECK(s_round == R_RUN && s_board_contact.hits >= 2u);
    board_sample(raised, 0.0f, 1u); CHECK(s_round == R_BRAKE && s_dist_reason == 1u);
    puts("board tilt:user-log100ms/+0.56/1.7mm rejected; startup guard300ms and sustained100ms after first fresh qualifying frame passed");
    return 0;
}

static int check_threshold_and_axes(void)
{
    for (unsigned owner = 31u; owner <= 43u; owner += 12u) {
        for (unsigned axis = 0u; axis < 2u; ++axis) {
            for (int sign = -1; sign <= 1; sign += 2) {
                CHECK(board_start(owner, 0.0f, 0.0f) == 0); board_finish_guard();
                float near = (float)sign * (ROUTE31_BOARD_CONTACT_TILT_DEG - 0.01f);
                float at = (float)sign * ROUTE31_BOARD_CONTACT_TILT_DEG;
                board_sample(axis ? 0.0f : near, axis ? near : 0.0f, 20u);
                board_sample(axis ? 0.0f : near, axis ? near : 0.0f, 20u);
                CHECK(s_round == R_RUN && s_board_contact.hits == 0u && last_x == 40.0f);
                board_sample(axis ? 0.0f : at, axis ? at : 0.0f, 20u);
                CHECK(s_round == R_RUN && s_board_contact.hits == 1u);
                for (unsigned elapsed = 20u; elapsed < ROUTE31_BOARD_CONTACT_CONFIRM_MS; elapsed += 20u) {
                    board_sample(axis ? 0.0f : at, axis ? at : 0.0f, 20u); CHECK(s_round == R_RUN);
                }
                board_sample(axis ? 0.0f : at, axis ? at : 0.0f, 20u);
                CHECK(s_round == R_BRAKE && s_dist_reason == 1u && s_board_contact.hits >= 2u);
                CHECK(board_success_finish(owner) == 0);
            }
        }
    }
    puts("board tilt:31/43 +/-pitch/roll1.49reject/1.50 accept ->BACK10/FWD15 fixed40 no yaw/FF/zero/750wait;rolling resets250;43 freshg each;BACK190 only then zero+750 passed");
    return 0;
}

static int check_baseline_wrap_and_consecutive_frames(void)
{
    static const float from[] = {12.5f, -12.5f, 0.2f, 179.9f, -179.9f, 179.25f};
    static const float to[] = {14.0f, -14.0f, 1.7f, -178.6f, 178.6f, -179.25f};
    for (unsigned i = 0u; i < sizeof from / sizeof from[0]; ++i) {
        CHECK(board_start(31u, from[i], -9.0f) == 0); board_finish_guard();
        board_sample(from[i], -9.0f, 20u); CHECK(s_round == R_RUN && !s_board_contact.hits);
        board_sample(to[i], -9.0f, 20u); CHECK(s_round == R_RUN && s_board_contact.hits == 1u);
        board_sample(to[i], -9.0f, ROUTE31_BOARD_CONTACT_CONFIRM_MS);
        CHECK(s_round == R_BRAKE && s_dist_reason == 1u);
    }
    CHECK(board_start(31u, 0.0f, 0.0f) == 0); board_finish_guard();
    float raised = ROUTE31_BOARD_CONTACT_TILT_DEG + 0.1f;
    board_sample(raised, 0.0f, 20u); CHECK(s_board_contact.hits == 1u);
    uint32_t sample_ms = host_tick;
    for (unsigned i = 0u; i < 5u; ++i) {
        host_tick += 20u; host_imu_age = host_tick - sample_ms; test_poll();
        CHECK(s_round == R_RUN && s_board_contact.sample_ms == sample_ms);
    }
    board_sample(raised, 0.0f, 20u); CHECK(s_round == R_RUN && s_board_contact.hits == 1u);
    board_sample(raised, 0.0f, ROUTE31_BOARD_CONTACT_CONFIRM_MS - 1u); CHECK(s_round == R_RUN);
    board_sample(raised, 0.0f, 1u); CHECK(s_round == R_BRAKE);
    CHECK(board_start(31u, 0.0f, 0.0f) == 0); board_finish_guard();
    board_sample(raised, 0.0f, 20u); board_sample(0.0f, 0.0f, 20u);
    CHECK(!s_board_contact.hits && !s_board_contact.confirming && s_round == R_RUN);
    board_sample(-raised, 0.0f, 20u); CHECK(s_board_contact.hits == 1u);
    board_sample(-raised, 0.0f, ROUTE31_BOARD_CONTACT_CONFIRM_MS + 1u);
    CHECK(s_round == R_RUN && s_board_contact.hits == 1u);
    board_sample(-raised, 0.0f, ROUTE31_BOARD_CONTACT_CONFIRM_MS);
    CHECK(s_round == R_BRAKE && s_dist_reason == 1u);
    puts("board tilt:nonzero frozen baseline,decimal/wrap,duplicate timestamp cannot confirm,low frame and over100ms gap restart sustained interval passed");
    return 0;
}

static int check_not_distance_or_yaw_and_timeout(void)
{
    for (unsigned owner = 31u; owner <= 43u; owner += 12u) {
        CHECK(board_start(owner, 0.0f, 0.0f) == 0); unsigned zeros = (unsigned)zero_calls;
        host_fore = s_dist_odo0 + 40.0f; host_yaw = 2.0f; board_sample(0.0f, 0.0f, 20u);
        host_fore = s_dist_odo0 + 1000.0f; host_yaw = -5.0f; board_sample(0.0f, 0.0f, 20u);
        CHECK(s_round == R_RUN && last_x == 40.0f && last_y == 0.0f && last_w == 0.0f &&
              (unsigned)zero_calls == zeros && s_board_contact.hits == 0u);
        host_tick = s_meas_t0 + 9999u; host_imu_age = 0u; test_poll(); CHECK(s_round == R_RUN);
        host_tick++; test_poll();
        CHECK(s_round == R_BRAKE && s_dist_reason == 5u && s_seq_stage == 4u && board_stopped());
        host_tick += T_DIST_STILL_MS; test_poll();
        CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 4u && !s_board_contact.active &&
              (unsigned)zero_calls == zeros && board_stopped() && strstr(host_messages, "CONTACT_TIMEOUT"));
        board_sample(2.0f, 0.0f, 20u); run_cmd("g"); CHECK(s_seq_state == SQ_STOPPED && board_stopped());
    }
    CHECK(board_start(31u, 0.0f, 0.0f) == 0);
    s_meas_t0 = UINT32_MAX - 5000u; host_tick = s_meas_t0 + 9999u;
    board_sample(0.0f, 0.0f, 0u); CHECK(s_round == R_RUN);
    board_sample(0.0f, 0.0f, 1u); CHECK(s_round == R_BRAKE && s_dist_reason == 5u);
    static const uint32_t first_hit_ms[] = {9900u, 9990u};
    static const uint32_t second_hit_ms[] = {10000u, 10090u};
    for (unsigned i = 0u; i < sizeof second_hit_ms / sizeof second_hit_ms[0]; ++i) {
        CHECK(board_start(31u, 0.0f, 0.0f) == 0);
        host_tick = s_meas_t0 + first_hit_ms[i]; board_sample(2.0f, 0.0f, 0u);
        CHECK(s_round == R_RUN && s_board_contact.hits == 1u);
        host_tick = s_meas_t0 + second_hit_ms[i]; board_sample(2.0f, 0.0f, 0u);
        CHECK(s_round == R_BRAKE && s_dist_reason == 5u && s_seq_stage == 4u);
        host_tick += T_DIST_STILL_MS; test_poll(); CHECK(s_seq_state == SQ_STOPPED && board_stopped());
    }
    puts("board tilt:encoder40/1000 and yaw cannot finish;10s timeout wins over sustained contact,tick wrap,noadvance/noresume passed");
    return 0;
}

static int check_invalid_and_manual_cancel(void)
{
    for (unsigned kind = 0u; kind < 5u; ++kind) {
        CHECK(board_start(31u, 0.0f, 0.0f) == 0); unsigned zeros = (unsigned)zero_calls;
        if (kind == 0u) host_pitch = NAN;
        else if (kind == 1u) host_roll = INFINITY;
        else if (kind == 2u) host_pitch = 181.0f;
        else if (kind == 3u) host_roll = -181.0f;
        else host_imu_age = 200u;
        host_tick += 20u; test_poll(); CHECK(s_round == R_BRAKE && s_dist_reason == 2u && board_stopped());
        host_tick += T_DIST_STILL_MS; test_poll();
        CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 4u && !s_board_contact.active &&
              (unsigned)zero_calls == zeros && strstr(host_messages, "IMUERR"));
    }
    CHECK(board_start(31u, 0.0f, 0.0f) == 0); host_imu_valid = 0; host_tick += 20u; test_poll();
    CHECK(s_seq_state == SQ_STOPPED && board_stopped() && !s_board_contact.active);
    static const char *keys[] = {"g", "a", "0"};
    for (unsigned owner = 31u; owner <= 43u; owner += 12u) {
        for (unsigned key = 0u; key < 3u; ++key) {
            for (unsigned phase = 0u; phase < 3u; ++phase) {
                CHECK(board_start(owner, 0.0f, 0.0f) == 0);
                if (phase > 0u) { board_finish_guard(); board_sample(2.0f, 0.0f, 20u); }
                if (phase > 1u) board_sample(2.0f, 0.0f, ROUTE31_BOARD_CONTACT_CONFIRM_MS);
                unsigned zeros = (unsigned)zero_calls; run_cmd(keys[key]);
                CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 4u && board_stopped() && !s_board_contact.active);
                board_sample(2.0f, 0.0f, 1000u); run_cmd("g");
                CHECK(s_seq_state == SQ_STOPPED && board_stopped() && (unsigned)zero_calls == zeros);
            }
            char command[8]; reset_fixture(); snprintf(command, sizeof command, "%u", owner);
            run_cmd(command); run_cmd("g"); s_seq_stage = 4u; route_seq_prepare();
            host_tick += T_DIST_STILL_MS; test_poll(); CHECK(s_seq_state == SQ_WAIT);
            run_cmd(keys[key]); host_tick += ROUTE31_BOARD_CONTACT_ZERO_WAIT_MS; test_poll();
            CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 4u && board_stopped() && !s_board_contact.active);
        }
    }
    reset_fixture(); run_cmd("31"); run_cmd("g"); s_seq_stage = 4u; route_seq_prepare();
    host_tick += T_DIST_STILL_MS; test_poll(); host_imu_age = 200u;
    host_tick += ROUTE31_BOARD_CONTACT_ZERO_WAIT_MS; test_poll();
    CHECK(s_seq_state == SQ_STOPPED && board_stopped() && !s_board_contact.active);
    puts("board tilt:invalid/stale IMU abort in startup/run/rezero,31/43 g/a/0 during wait/run/one-hit/braking never advance passed");
    return 0;
}

static int board31_at_nudge(unsigned post)
{
    CHECK(board_start(31u, 0.0f, 0.0f) == 0); board_finish_guard();
    board_sample(2.0f, 0.0f, 20u);
    board_sample(2.0f, 0.0f, ROUTE31_BOARD_CONTACT_CONFIRM_MS);
    CHECK(s_round == R_BRAKE);
    unsigned zeros = (unsigned)zero_calls;
    host_tick += T_DIST_STILL_MS; test_poll();
    CHECK(s_seq_stage == 4u && s_seq_contact_post == 1u && s_seq_state == SQ_STILL &&
          board_stopped() && (unsigned)zero_calls == zeros);
    if (post == 2u) {
        CHECK(board_nudge_start(31u, 1u, zeros) == 0 && board_nudge_finish(zeros) == 0);
        CHECK(s_seq_stage == 4u && s_seq_contact_post == 2u && s_seq_state == SQ_STILL);
    }
    return 0;
}

static int check_route31_nudge_cancellations(void)
{
    const char *const keys[] = {"g", "a", "0"};
    for (unsigned post = 1u; post <= 2u; ++post) {
        for (unsigned phase = 0u; phase < 3u; ++phase) {
            for (unsigned key = 0u; key < 3u; ++key) {
                CHECK(board31_at_nudge(post) == 0);
                unsigned zeros = (unsigned)zero_calls;
                if (phase > 0u) CHECK(board_nudge_start(31u, post, zeros) == 0);
                if (phase > 1u) {
                    host_fore = s_dist_odo0 + s_dist_target; test_poll();
                    CHECK(s_seq_state == SQ_RUN && s_round == R_BRAKE && board_stopped());
                }
                run_cmd(keys[key]);
                CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 4u && !s_seq_contact_post &&
                      !s_board_contact.active && board_stopped() && (unsigned)zero_calls == zeros);
                board_sample(2.0f, 0.0f, 1000u); run_cmd("g");
                CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 4u && !s_seq_contact_post &&
                      board_stopped() && (unsigned)zero_calls == zeros);
            }
        }
        for (unsigned fault = 0u; fault < 2u; ++fault) {
            CHECK(board31_at_nudge(post) == 0);
            unsigned zeros = (unsigned)zero_calls;
            CHECK(board_nudge_start(31u, post, zeros) == 0);
            if (fault == 0u) host_imu_valid = 0;
            else host_abort = 1;
            host_tick += 20u; test_poll();
            CHECK(s_seq_state == SQ_STOPPED && s_seq_stage == 4u && !s_seq_contact_post &&
                  !s_board_contact.active && board_stopped() && (unsigned)zero_calls == zeros);
            host_imu_valid = 1; host_abort = 0;
            host_tick += 1000u; test_poll(); run_cmd("g");
            CHECK(s_seq_state == SQ_STOPPED && board_stopped() && (unsigned)zero_calls == zeros);
        }
    }
    puts("31 board nudges:BACK10/FWD15 g/a/0 in preparation/run/braking abort without zero/advance/resume;IMU/abort running stops both passed");
    return 0;
}

static int check_legacy_and_noncontact_isolation(void)
{
    static const unsigned owners[] = {34u, 36u, 37u};
    for (unsigned i = 0u; i < sizeof owners / sizeof owners[0]; ++i) {
        char command[8]; reset_fixture(); snprintf(command, sizeof command, "%u", owners[i]); run_cmd(command); run_cmd("g");
        s_seq_stage = owners[i] == 37u ? 1u : 4u; route_seq_prepare();
        CHECK(sequence_start_stage() == 0 && !s_board_contact.active &&
              s_dist_target == 80.0f && s_v == 20.0f && last_x == 20.0f);
        board_sample(4.0f, -4.0f, 20u); board_sample(4.0f, -4.0f, 20u);
        CHECK(s_round == R_RUN && !s_board_contact.active);
        host_fore = s_dist_odo0 + 80.0f; test_poll(); CHECK(s_round == R_BRAKE && s_dist_reason == 1u);
    }
    reset_fixture(); run_cmd("15"); run_cmd("v40"); run_cmd("d80"); run_cmd("g"); test_poll();
    CHECK(s_round == R_RUN && !s_board_contact.active && s_dist_target == 80.0f);
    board_sample(4.0f, -4.0f, 20u); board_sample(4.0f, -4.0f, 20u); CHECK(s_round == R_RUN && !s_board_contact.active);
    CHECK(board_start(43u, 0.0f, 0.0f) == 0);
    run_cmd("g"); run_cmd("43"); s_seq_stage = 4u; s_route43_pending = R43_PENDING_PREP;
    s_seq_state = SQ_STEP_WAIT; s_msel = 43; unsigned distance = s_route43_tune.road_mm[4]; run_cmd("d40");
    CHECK(s_seq_state == SQ_STEP_WAIT && s_route43_tune.road_mm[4] == distance &&
          !s_board_contact.active && board_stopped() && strstr(last_message, "R43_NEXT_IS_NOT_DISTANCE"));
    puts("board tilt:legacy34/36/37 retain80mm/v20/NAV750,manual15 encoder endpoint,43 d rejects sensor-only contact passed");
    return 0;
}

int main(void)
{
    if (check_stopped_rezero_and_rolling_restart() || check_startup_log_and_guard() ||
        check_threshold_and_axes() || check_baseline_wrap_and_consecutive_frames() ||
        check_not_distance_or_yaw_and_timeout() || check_invalid_and_manual_cancel() ||
        check_route31_nudge_cancellations() ||
        check_legacy_and_noncontact_isolation()) return 1;
    puts("route31/43 board tilt wait300/1000ms/guard300ms/confirm100ms/threshold1.5 host-only regression passed");
    return 0;
}
