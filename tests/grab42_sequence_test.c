/* Real mode42/router with the shared host-only TIM7/peripheral model.
 * These assertions verify software sequencing, not physical travel/grip/home. */
#define main legacy_g_fixture_main
#include "g_command_stop_test.c"
#undef main

#undef CHECK
#define CHECK(expr) do { if (!(expr)) { fprintf(stderr, "grab42 line %d: %s\n", __LINE__, #expr); return 1; } } while (0)

static void grab_cmd(const char *line)
{
    while (*line) test_feed((uint8_t)*line++);
    test_feed('\r'); test_feed('\n');
    test_poll();
}

static int grab_inert(void)
{
    CHECK(last_x == 0.0f && last_y == 0.0f && last_w == 0.0f);
    CHECK(!host_precise_calls && !host_integer_calls && !host_laser_on_calls);
    CHECK(!host_scene_calls && !host_target_calls && !s_go && !start_calls);
    return 0;
}

static int grab_boot(void)
{
    reset_fixture(); grab_cmd("42");
    CHECK(T_GRAB_MODE == 42 && T_MODE_MAX == 43 && s_msel == 42);
    CHECK(s_grab42_phase == G42_READY && s_round == R_READY);
    CHECK(ARM_SERVO_GRIP_US == 1700u && T_GRAB_GRIP_DEFAULT_US == 1900u);
    CHECK(s_grab42_extend == 2500u && s_grab42_down == 0u && s_grab42_u == 1900u);
    CHECK(s_grab42_pps == 500u && !pulse_calls && !servo_calls && !host_timer_active);
    char current[32];
    snprintf(current, sizeof current, "current_u=%u", ARM_SERVO_START_US);
    CHECK(ARM_SERVO_START_US == 1150u && host_servo == ARM_SERVO_START_US);
    CHECK(strstr(host_messages, "status=SELECT") && strstr(host_messages, "u=1900") &&
          strstr(host_messages, current));
    CHECK(grab_inert() == 0);
    return 0;
}

static int grab_wait(unsigned ms)
{
    CHECK(!host_timer_active);
    host_tick += ms; test_poll();
    return 0;
}

static int grab_pulses(unsigned count)
{
    int goal = pulse_calls + (int)count;
    unsigned budget = count + 50u;
    while (pulse_calls < goal && budget--) CHECK(jog_host_timer_event() == 0);
    CHECK(pulse_calls == goal);
    test_poll();
    return 0;
}

static int grab_finish_job(unsigned count, int axis, int dir, unsigned wait_phase)
{
    CHECK(host_timer_active && host_jog_axis == axis && host_jog_dir == dir);
    CHECK(s_jog_clock.remaining == count && host_timer_pps == s_grab42_pps);
    CHECK(grab_pulses(count) == 0);
    CHECK(!host_timer_active && s_jog_clock.remaining == 0u);
    CHECK(s_grab42_phase == wait_phase);
    CHECK(grab_inert() == 0);
    return 0;
}

static int check_default_grip_and_origin(void)
{
    CHECK(grab_boot() == 0);
    /* Both default cycles use the private1900 grip and retain f500; no u/f
     * commands or reselect between cycles may hide a repeat-default leak. */
    for (unsigned run = 1u; run <= 2u; ++run) {
        int pulses = pulse_calls, servos = servo_calls;
        grab_cmd("g");
        CHECK(s_grab42_phase == G42_EXTEND && pulse_calls == pulses &&
              servo_calls == servos && host_timer_active && s_active_test == run);
        CHECK(s_grab42_origin_u == ARM_SERVO_START_US && s_grab42_u == 1900u);
        CHECK(grab_finish_job(2500u, 0, 0, G42_EXTEND_WAIT) == 0);
        CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_DOWN_WAIT && !host_timer_active);
        CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_GRIP_HOLD && host_servo == 1900u);
        CHECK(grab_wait(2000u) == 0 && s_grab42_phase == G42_SERVO_RESET_WAIT && host_servo == ARM_SERVO_START_US);
        CHECK(grab_wait(2000u) == 0 && s_grab42_phase == G42_UP_WAIT && !host_timer_active);
        CHECK(grab_wait(250u) == 0 && grab_finish_job(2500u, 0, 1, G42_RETRACT_WAIT) == 0);
        CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_READY && servo_calls == (int)(2u * run));
        CHECK(pulse_calls == (int)(5000u * run) && host_jog_pulses[1][0] == 0u && host_jog_pulses[1][1] == 0u);
        CHECK(s_grab42_u == 1900u && s_grab42_origin_u == ARM_SERVO_START_US && s_grab42_pps == 500u);
        CHECK(grab_wait(60000u) == 0 && s_grab42_phase == G42_READY && grab_inert() == 0);
    }

    CHECK(grab_boot() == 0);
    /* Keep the implementation's unset-u guard defensive even though normal
     * selection now always initializes the private default1900. */
    s_grab42_u = 0u;
    grab_cmd("g");
    CHECK(s_grab42_phase == G42_READY && !pulse_calls && !servo_calls && !host_timer_active);
    CHECK(strstr(last_message, "GRAB42_SET_U_FIRST"));
    grab_cmd("su1600");
    CHECK(host_servo == 1600u && servo_calls == 1 && !pulse_calls);
    grab_cmd("param"); CHECK(strstr(last_message, "current_u=1600"));
    grab_cmd("u1000");
    CHECK(host_servo == 1600u && servo_calls == 1 && s_grab42_u == 1000u);
    grab_cmd("g");
    CHECK(s_grab42_phase == G42_EXTEND && s_grab42_origin_u == 1600u);
    CHECK(!pulse_calls && servo_calls == 1 && host_timer_active);
    grab_cmd("g");
    CHECK(s_grab42_phase == G42_STOPPED && host_servo == 1600u && servo_calls == 1);
    CHECK(grab_inert() == 0);

    CHECK(grab_boot() == 0); grab_cmd("u1000"); host_servo = 0u;
    grab_cmd("g");
    CHECK(strstr(last_message, "GRAB42_ORIGIN_U_UNKNOWN"));
    CHECK(s_grab42_phase == G42_READY && !pulse_calls && !servo_calls && !host_timer_active);
    puts("grab42 selection/start: private default e2500 h0 u1900 f500 completes and repeats on g; global grip1700/start1150 unchanged; defensive internal-u0 rejection, explicit override, idle su origin and unknown-origin rejection passed");
    return 0;
}

static int check_full_recipe_and_repeat(void)
{
    CHECK(grab_boot() == 0);
    grab_cmd("e7"); grab_cmd("h11"); grab_cmd("u1000"); grab_cmd("f333");
    for (unsigned run = 1u; run <= 2u; ++run) {
        grab_cmd("g");
        CHECK(s_active_test == run && s_grab42_phase == G42_EXTEND);
        CHECK(s_grab42_origin_u == ARM_SERVO_START_US && host_servo == ARM_SERVO_START_US);
        CHECK(grab_finish_job(7u, 0, 0, G42_EXTEND_WAIT) == 0);
        CHECK(s_grab42_out == 7u && s_grab42_lowered == 0u);
        CHECK(grab_wait(249u) == 0 && s_grab42_phase == G42_EXTEND_WAIT);
        CHECK(grab_wait(1u) == 0 && s_grab42_phase == G42_DOWN);
        CHECK(grab_finish_job(11u, 1, 1, G42_DOWN_WAIT) == 0);
        CHECK(s_grab42_lowered == 11u && host_servo == ARM_SERVO_START_US);
        CHECK(grab_wait(249u) == 0 && s_grab42_phase == G42_DOWN_WAIT);
        CHECK(grab_wait(1u) == 0 && s_grab42_phase == G42_GRIP_HOLD);
        CHECK(host_servo == 1000u && servo_calls == (int)(2u * run - 1u));
        CHECK(grab_wait(1999u) == 0 && host_servo == 1000u && s_grab42_phase == G42_GRIP_HOLD);
        CHECK(grab_wait(1u) == 0 && host_servo == ARM_SERVO_START_US && s_grab42_phase == G42_SERVO_RESET_WAIT);
        CHECK(servo_calls == (int)(2u * run));
        CHECK(grab_wait(1999u) == 0 && s_grab42_phase == G42_SERVO_RESET_WAIT);
        CHECK(grab_wait(1u) == 0 && s_grab42_phase == G42_UP);
        CHECK(grab_finish_job(11u, 1, 0, G42_UP_WAIT) == 0);
        CHECK(s_grab42_up == 11u);
        CHECK(grab_wait(249u) == 0 && s_grab42_phase == G42_UP_WAIT);
        CHECK(grab_wait(1u) == 0 && s_grab42_phase == G42_RETRACT);
        CHECK(grab_finish_job(7u, 0, 1, G42_RETRACT_WAIT) == 0);
        CHECK(s_grab42_back == 7u);
        CHECK(grab_wait(249u) == 0 && s_grab42_phase == G42_RETRACT_WAIT);
        CHECK(grab_wait(1u) == 0 && s_grab42_phase == G42_READY && s_round == R_READY);
        CHECK(strstr(last_message, "READY repeat_g=1"));
        CHECK(s_grab42_extend == 7u && s_grab42_down == 11u && s_grab42_u == 1000u && s_grab42_pps == 333u);
        CHECK(host_jog_pulses[0][0] == 7u * run && host_jog_pulses[0][1] == 7u * run);
        CHECK(host_jog_pulses[1][1] == 11u * run && host_jog_pulses[1][0] == 11u * run);
        CHECK(grab_wait(60000u) == 0 && s_grab42_phase == G42_READY);
        CHECK(grab_inert() == 0);
    }
    grab_cmd("42");
    CHECK(s_grab42_extend == 2500u && s_grab42_down == 0u && s_grab42_u == 1900u && s_grab42_pps == 500u);
    puts("grab42 full/repeat: extend-DIR0,250ms,down-DIR1,250ms,u,2s,prior-u,2s,up-DIR0,250ms,retract-DIR1,250ms; exact emitted returns and retained params repeat passed");
    return 0;
}

static int check_global_claw_default_isolation(void)
{
    CHECK(grab_boot() == 0);
    grab_cmd("cc");
    CHECK(ARM_SERVO_GRIP_US == 1700u && host_servo == 1700u);
    CHECK(s_grab42_u == 1900u && s_grab42_pps == 500u && s_grab42_phase == G42_READY);
    grab_cmd("e0"); grab_cmd("h0"); grab_cmd("g");
    CHECK(s_grab42_origin_u == 1700u && s_grab42_phase == G42_EXTEND_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_DOWN_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_GRIP_HOLD && host_servo == 1900u);
    CHECK(grab_wait(2000u) == 0 && s_grab42_phase == G42_SERVO_RESET_WAIT && host_servo == 1700u);
    CHECK(grab_wait(2000u) == 0 && s_grab42_phase == G42_UP_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_RETRACT_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_READY && !pulse_calls && !host_timer_start_calls);
    grab_cmd("42");
    CHECK(s_grab42_u == 1900u && s_grab42_pps == 500u && host_servo == 1700u);
    CHECK(grab_inert() == 0);
    puts("grab42 isolation: global cc remains1700; private default1900 grips then returns to captured cc1700 origin, reselect retains physical command/f500 passed");
    return 0;
}

static int check_zero_and_max_counts(void)
{
    CHECK(grab_boot() == 0); grab_cmd("e0"); grab_cmd("h0"); grab_cmd("u2500");
    grab_cmd("g");
    CHECK(s_grab42_phase == G42_EXTEND_WAIT && !host_timer_active);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_DOWN_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_GRIP_HOLD && host_servo == 2500u);
    CHECK(grab_wait(2000u) == 0 && s_grab42_phase == G42_SERVO_RESET_WAIT && host_servo == ARM_SERVO_START_US);
    CHECK(grab_wait(2000u) == 0 && s_grab42_phase == G42_UP_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_RETRACT_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_READY);
    CHECK(!pulse_calls && !host_timer_start_calls && servo_calls == 2);

    CHECK(grab_boot() == 0); grab_cmd("nl100000"); grab_cmd("h100000"); grab_cmd("f20000"); grab_cmd("u500");
    grab_cmd("g"); CHECK(grab_finish_job(100000u, 0, 0, G42_EXTEND_WAIT) == 0);
    CHECK(grab_wait(250u) == 0 && grab_finish_job(100000u, 1, 1, G42_DOWN_WAIT) == 0);
    CHECK(grab_wait(250u) == 0 && host_servo == 500u);
    CHECK(grab_wait(2000u) == 0 && host_servo == ARM_SERVO_START_US);
    CHECK(grab_wait(2000u) == 0 && grab_finish_job(100000u, 1, 0, G42_UP_WAIT) == 0);
    CHECK(grab_wait(250u) == 0 && grab_finish_job(100000u, 0, 1, G42_RETRACT_WAIT) == 0);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_READY && pulse_calls == 400000);
    CHECK(s_grab42_out == 100000u && s_grab42_lowered == 100000u && s_grab42_up == 100000u && s_grab42_back == 100000u);
    CHECK(grab_inert() == 0);
    puts("grab42 limits: zero axes skip all clocks; nl alias and two 100000-step axes at f20000 return exactly400000 total pulses passed");
    return 0;
}

static int check_custom_pre_run_servo_origin(void)
{
    static const unsigned origins[] = {1600u, 900u};
    char command[24];
    CHECK(grab_boot() == 0); grab_cmd("e0"); grab_cmd("h0"); grab_cmd("u1000");
    for (unsigned i = 0u; i < sizeof origins / sizeof origins[0]; ++i) {
        snprintf(command, sizeof command, "su%u", origins[i]); grab_cmd(command);
        CHECK(host_servo == origins[i]);
        grab_cmd("g");
        CHECK(s_grab42_origin_u == origins[i] && s_grab42_origin_u != ARM_SERVO_START_US);
        CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_DOWN_WAIT);
        CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_GRIP_HOLD && host_servo == 1000u);
        CHECK(grab_wait(1999u) == 0 && host_servo == 1000u);
        CHECK(grab_wait(1u) == 0 && s_grab42_phase == G42_SERVO_RESET_WAIT && host_servo == origins[i]);
        CHECK(grab_wait(2000u) == 0 && s_grab42_phase == G42_UP_WAIT);
        CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_RETRACT_WAIT);
        CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_READY && host_servo == origins[i]);
        CHECK(!pulse_calls && !host_timer_start_calls && s_grab42_u == 1000u && s_active_test == i + 1u);
    }
    CHECK(grab_inert() == 0);
    puts("grab42 origin: startup1150 independent; complete trials return to pre-g su1600 then su900, not hard-coded startup/open/close passed");
    return 0;
}

static int check_parameter_parser(void)
{
    static const char *const invalid[] = {
        "e-0","e-1","e100001","e","e1x","e1.5","e2147483648","e4294967296",
        "h-0","h-1","h100001","h","h1x","h1.5","h2147483648","h4294967296",
        "nl-1","nl100001","nl","nl1x","nl1.5","nl4294967296",
        "u499","u2501","u-1","u","u1000x","u1000.5","u4294967296",
        "f0","f20001","f-1","f","f500x","f500.5","f4294967296"
    };
    CHECK(grab_boot() == 0); grab_cmd("e8"); grab_cmd("h9"); grab_cmd("u1600"); grab_cmd("f1000");
    for (unsigned i = 0; i < sizeof invalid / sizeof invalid[0]; ++i) {
        grab_cmd(invalid[i]);
        CHECK(strstr(last_message, "ERR GRAB42_RANGE"));
        CHECK(s_grab42_extend == 8u && s_grab42_down == 9u && s_grab42_u == 1600u && s_grab42_pps == 1000u);
        CHECK(!pulse_calls && !servo_calls && s_grab42_phase == G42_READY);
    }
    static const char *const valid[] = {"e0","e1","e100000","h0","h1","h100000","nl0","nl1","nl100000","u500","u2500","f1","f20000"};
    for (unsigned i = 0; i < sizeof valid / sizeof valid[0]; ++i) {
        grab_cmd(valid[i]); CHECK(strstr(last_message, "status=PARAM_SET"));
        CHECK(!pulse_calls && !servo_calls && !host_timer_active);
    }
    CHECK(grab_inert() == 0);
    puts("grab42 parser: zero/one/max counts,u endpoints,f endpoints,negative-zero/malformed/range/overflow rejections and no write-time actuation passed");
    return 0;
}

/* Reach every active phase by actual commands and clock/poll events. */
static int grab_reach(unsigned phase)
{
    CHECK(grab_boot() == 0); grab_cmd("e7"); grab_cmd("h11"); grab_cmd("u1000"); grab_cmd("g");
    unsigned budget = 20u;
    while (s_grab42_phase != phase && budget--) {
        if (host_timer_active) {
            unsigned remaining = s_jog_clock.remaining;
            CHECK(grab_pulses(remaining) == 0);
        } else {
            unsigned ms = (s_grab42_phase == G42_GRIP_HOLD || s_grab42_phase == G42_SERVO_RESET_WAIT) ? 2000u : 250u;
            CHECK(grab_wait(ms) == 0);
        }
    }
    CHECK(s_grab42_phase == phase && grab42_active());
    return 0;
}

static int check_phase_stops_and_no_resume(void)
{
    static const char *const stops[] = {"g", "a", "0"};
    for (unsigned phase = G42_EXTEND; phase <= G42_RETRACT_WAIT; ++phase)
        for (unsigned key = 0; key < 3u; ++key) {
            CHECK(grab_reach(phase) == 0);
            if (host_timer_active) CHECK(grab_pulses(1u) == 0);
            int pulses = pulse_calls, servos = servo_calls;
            uint16_t retained = host_servo;
            uint32_t test = s_active_test;
            unsigned stops_before = host_timer_stop_calls;
            grab_cmd(stops[key]);
            CHECK(s_grab42_phase == G42_STOPPED && !grab42_active() && !host_timer_active && !s_jog_clock.remaining);
            CHECK(host_timer_stop_calls > stops_before && pulse_calls == pulses && servo_calls == servos && host_servo == retained);
            CHECK(strstr(last_message, "no_auto_return=1 servo_PWM_retained=1"));
            test_stepper_timer_irq();
            CHECK(grab_wait(60000u) == 0 && pulse_calls == pulses && servo_calls == servos);
            grab_cmd("g");
            CHECK(strstr(last_message, "GRAB42_RESTORE_ORIGINS_THEN_SELECT42"));
            CHECK(s_grab42_phase == G42_STOPPED && s_active_test == test && pulse_calls == pulses && servo_calls == servos);
            grab_cmd("u1500");
            CHECK(strstr(last_message, "GRAB42_RESTORE_ORIGINS_THEN_SELECT42"));
            grab_cmd("42");
            CHECK(s_grab42_phase == G42_READY && s_grab42_u == 1900u && host_servo == retained && servo_calls == servos);
            grab_cmd("g");
            CHECK(s_grab42_phase == G42_EXTEND && s_active_test == test + 1u && pulse_calls == pulses);
            CHECK(s_grab42_origin_u == retained && servo_calls == servos && host_timer_active);
            grab_cmd("g"); CHECK(s_grab42_phase == G42_STOPPED && !host_timer_active && pulse_calls == pulses);
            CHECK(grab_inert() == 0);
        }
    puts("grab42 stops:30 g/a/0 cancellations across10 active phases, immediate staleIRQ-safe clock cancellation, no extra servo/reversal/resume; reselect restores privateu1900 and permits only a new run passed");
    return 0;
}

static int check_ownership_and_frozen_parameters(void)
{
    static const char *const writes[] = {
        "15","24","31","32","34","35","36","37","38","39","40","41","42",
        "r1","e1","h1","nl1","nr1","n1","u1500","f2000","su1500","co","cc",
        "v300","d1000","ykp10","fff0","b1d1000","route","vision"
    };
    static const char *const reads[] = {"?", "diag", "param"};
    for (unsigned phase = G42_EXTEND; phase <= G42_RETRACT_WAIT; ++phase) {
        CHECK(grab_reach(phase) == 0);
        int pulses = pulse_calls, servos = servo_calls;
        uint32_t remaining = s_jog_clock.remaining, t0 = s_grab42_t0;
        uint16_t pulse = host_servo;
        for (unsigned n = 0; n < sizeof writes / sizeof writes[0]; ++n) {
            grab_cmd(writes[n]); CHECK(strstr(last_message, "ERR GRAB42_ACTIVE"));
            CHECK(s_msel == 42 && s_grab42_phase == phase && s_grab42_t0 == t0);
            CHECK(s_grab42_extend == 7u && s_grab42_down == 11u && s_grab42_u == 1000u && s_grab42_pps == 500u);
            CHECK(s_jog_clock.remaining == remaining && pulse_calls == pulses && servo_calls == servos && host_servo == pulse);
        }
        for (unsigned n = 0; n < sizeof reads / sizeof reads[0]; ++n) {
            grab_cmd(reads[n]); CHECK(s_grab42_phase == phase && s_msel == 42);
            CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == pulse);
        }
        CHECK(grab_inert() == 0); grab_cmd("g");
    }
    puts("grab42 ownership:31 mode/tune/arm writes blocked in each active phase; only ?/diag/param read-only, frozen params and no chassis/QR/vision/laser-on passed");
    return 0;
}

static int check_clock_failure_abort_and_rollover(void)
{
    static const unsigned before_jobs[] = {G42_READY, G42_EXTEND_WAIT, G42_SERVO_RESET_WAIT, G42_UP_WAIT};
    for (unsigned i = 0; i < 4u; ++i) {
        if (before_jobs[i] == G42_READY) {
            CHECK(grab_boot() == 0); grab_cmd("e7"); grab_cmd("h11"); grab_cmd("u1000");
        } else CHECK(grab_reach(before_jobs[i]) == 0);
        int pulses = pulse_calls, servos = servo_calls;
        uint16_t retained = host_servo;
        host_timer_start_fail = 1;
        if (before_jobs[i] == G42_READY) grab_cmd("g");
        else CHECK(grab_wait(before_jobs[i] == G42_SERVO_RESET_WAIT ? 2000u : 250u) == 0);
        CHECK(s_grab42_phase == G42_STOPPED && !host_timer_active && !s_jog_clock.remaining);
        CHECK(strstr(host_messages, "CLOCK_START_ERROR"));
        CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == retained);
        CHECK(grab_wait(60000u) == 0 && pulse_calls == pulses && servo_calls == servos);
        CHECK(grab_inert() == 0);
    }
    CHECK(grab_reach(G42_DOWN) == 0); CHECK(grab_pulses(2u) == 0);
    run_abort(); test_poll(); CHECK(s_grab42_phase == G42_STOPPED && !host_timer_active && pulse_calls == 9);
    CHECK(grab_wait(60000u) == 0 && pulse_calls == 9 && !servo_calls);

    CHECK(grab_boot() == 0); grab_cmd("e0"); grab_cmd("h0"); grab_cmd("u1000");
    host_tick = UINT32_MAX - 100u;
    grab_cmd("g"); CHECK(s_grab42_phase == G42_EXTEND_WAIT);
    CHECK(grab_wait(249u) == 0 && s_grab42_phase == G42_EXTEND_WAIT);
    CHECK(grab_wait(1u) == 0 && s_grab42_phase == G42_DOWN_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_GRIP_HOLD);
    CHECK(grab_wait(2000u) == 0 && host_servo == ARM_SERVO_START_US);
    CHECK(grab_wait(2000u) == 0 && s_grab42_phase == G42_UP_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_RETRACT_WAIT);
    CHECK(grab_wait(250u) == 0 && s_grab42_phase == G42_READY && !pulse_calls);
    puts("grab42 failure/timing: four motor-job clock-start failures abort without later action; externalabort cancels and HAL tick rollover preserves waits passed");
    return 0;
}

static int check_partial_job_count_faults(void)
{
    static const unsigned phases[] = {G42_EXTEND, G42_DOWN, G42_UP, G42_RETRACT};
    for (unsigned i = 0u; i < 4u; ++i) {
        CHECK(grab_reach(phases[i]) == 0);
        CHECK(grab_pulses(2u) == 0);
        int pulses = pulse_calls, servos = servo_calls;
        uint16_t retained = host_servo;
        /* Deliberate software fault injection: another owner clears a partial
         * TIM7 job. This is not a model for physical stalls/lost motor steps. */
        JogClock interrupted;
        CHECK(step_clock_snapshot(&interrupted, 1) == 0u && interrupted.completed == 2u);
        test_poll();
        CHECK(s_grab42_phase == G42_STOPPED && !host_timer_active && !s_jog_clock.remaining);
        CHECK(strstr(host_messages, "status=COUNT_ERROR"));
        CHECK(pulse_calls == pulses && servo_calls == servos && host_servo == retained);
        CHECK(grab_wait(60000u) == 0 && pulse_calls == pulses && servo_calls == servos);
        test_stepper_timer_irq(); CHECK(pulse_calls == pulses);
        grab_cmd("g"); CHECK(strstr(last_message, "GRAB42_RESTORE_ORIGINS_THEN_SELECT42"));
        CHECK(grab_inert() == 0);
    }
    puts("grab42 countfault: externally cancelled partial jobs in all4 motor phases STOPPED without later grip/return; software-only fault injection passed");
    return 0;
}

int main(void)
{
    CHECK(check_default_grip_and_origin() == 0);
    CHECK(check_full_recipe_and_repeat() == 0);
    CHECK(check_global_claw_default_isolation() == 0);
    CHECK(check_zero_and_max_counts() == 0);
    CHECK(check_custom_pre_run_servo_origin() == 0);
    CHECK(check_parameter_parser() == 0);
    CHECK(check_phase_stops_and_no_resume() == 0);
    CHECK(check_ownership_and_frozen_parameters() == 0);
    CHECK(check_clock_failure_abort_and_rollover() == 0);
    CHECK(check_partial_job_count_faults() == 0);
    puts("grab42 public Bluetooth router/sequence host regression passed; no physical acceptance claimed");
    return 0;
}
