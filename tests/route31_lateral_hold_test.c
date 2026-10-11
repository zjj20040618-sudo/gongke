/* Actual BLE router and route executor -> captured body velocity -> actual
 * precise IK/four-wheel targets. Inert sensors are deliberately not a vehicle
 * plant: passing this test does not prove traction or ground accuracy. */
#define main retained_g_command_stop_main
#include "g_command_stop_test.c"
#undef main

static float lateral_rpm[4];
static unsigned lateral_mask;
void ctrl_stop_all(void) { memset(lateral_rpm, 0, sizeof lateral_rpm); }
void ctrl_set_speed_precise(int m, float rpm)
{ lateral_rpm[m] = rpm; lateral_mask |= 1u << (unsigned)m; }
void ctrl_set_speed_creep(int m, float rpm) { ctrl_set_speed_precise(m,rpm); }

#define motion_init lateral_motion_init
#define motion_ik lateral_motion_ik
#define motion_vel_set lateral_motion_vel_set
#define motion_vel_set_precise lateral_motion_vel_set_precise
#define motion_vel_set_creep lateral_motion_vel_set_creep
#define motion_brake lateral_motion_brake
#define motion_pose lateral_motion_pose
#define motion_pose_update lateral_motion_pose_update
#define motion_odo_mm lateral_motion_odo_mm
#define motion_lateral_odo_mm lateral_motion_lateral_odo_mm
#define motion_ramp_init lateral_motion_ramp_init
#define motion_ramp_step lateral_motion_ramp_step
#define motion_profile_get lateral_motion_profile_get
#define motion_profile_set lateral_motion_profile_set
#define motion_profile_config_missing lateral_motion_profile_config_missing
#define motion_linear_ramp_init lateral_motion_linear_ramp_init
#define motion_linear_profile_step lateral_motion_linear_profile_step
#define motion_linear_ramp_step lateral_motion_linear_ramp_step
void lateral_motion_brake(void);
#include "../App/motion.c"
#undef motion_init
#undef motion_ik
#undef motion_vel_set
#undef motion_vel_set_precise
#undef motion_vel_set_creep
#undef motion_brake
#undef motion_pose
#undef motion_pose_update
#undef motion_odo_mm
#undef motion_lateral_odo_mm
#undef motion_ramp_init
#undef motion_ramp_step
#undef motion_profile_get
#undef motion_profile_set
#undef motion_profile_config_missing
#undef motion_linear_ramp_init
#undef motion_linear_profile_step
#undef motion_linear_ramp_step

#undef CHECK
#define CHECK(expr) do { if (!(expr)) { \
    fprintf(stderr, "route31 lateral hold line %d: %s (last=%s)\n", __LINE__, #expr,last_message); \
    return 1; \
} } while (0)

static int lateral_near(float a, float b) { return fabsf(a-b) < 0.00001f; }

static int lateral_check_ik(void)
{
    float expected[4], without_ff[4];
    motion_ik_precise(last_x,last_y,last_w,expected);
    motion_ik_precise(0.0f,last_y,last_w,without_ff);
    lateral_mask=0u;
    lateral_motion_vel_set_precise(last_x,last_y,last_w);
    CHECK(lateral_mask==15u);
    for (int m=0;m<4;++m) CHECK(lateral_near(lateral_rpm[m],expected[m]));
    /* Fore/aft feedforward must add an equal value to every wheel, not
     * differential steering. Its independent rotational component is zero. */
    float delta[4];
    for (int m=0;m<4;++m) delta[m]=lateral_rpm[m]-without_ff[m];
    CHECK(fabsf(delta[0]-delta[1]-delta[2]+delta[3]) < 0.0001f);
    for (int m=1;m<4;++m) CHECK(fabsf(delta[m]-delta[0]) < 0.0001f);
    return 0;
}

static void lateral_select(unsigned owner)
{ char command[16]; snprintf(command,sizeof command,"%u",owner);run_cmd(command); }

static int lateral_start(unsigned owner,unsigned stage,int offset)
{
    lateral_select(owner);run_cmd("g");
    /* Executor fixture: stage injections stand for already-completed earlier
     * route actions. Real pair transition/ACK gates remain checked by scope. */
    s_seq_stage=(uint8_t)stage;
    if (offset==1) s_seq_pair_offset=1u;
    if (offset==2) s_seq_hostage_offset=s_route31_hostage_offset_done=1u;
    route_seq_prepare();
    CHECK(sequence_start_stage()==0);
    CHECK(s_round==R_RUN && s_seq_state==SQ_RUN);
    return 0;
}

static int check_seeds_signs_and_all_left_jobs(void)
{
    static const float yaw[] = {-6.35f,-1.0f,0.0f,1.0f,6.35f};
    CHECK(ROUTE31_LEFT_YAW_KP_SEED==3.0f && ROUTE31_LEFT_FF_SEED==0.01f);
    for (unsigned offset=0;offset<3;++offset) {
        reset_fixture();
        CHECK(lateral_start(31u,offset==1u?ROUTE31_PAIR_STAGE:offset==2u?ROUTE31_HOSTAGE_STAGE:0u,(int)offset)==0);
        CHECK(s_msel==17 && s_dist_heading_kp==3.0f && s_dist_heading_profile==0u);
        CHECK(s_dist_target==(offset==1u?-15.0f:offset==2u?-30.0f:-575.0f) &&
              s_v==(offset==1u?80.0f:250.0f));
        CHECK(lateral_near(s_dist_ff_ratio,0.01f) && s_dist_align_enabled);
        CHECK(last_y==-s_v && lateral_near(last_x,-0.01f*s_v));
        for (unsigned i=0;i<sizeof yaw/sizeof yaw[0];++i) {
            host_yaw=s_dist_heading0+yaw[i];tick();
            CHECK(s_round==R_RUN && s_seq_state==SQ_RUN && !s_dist_mid_pause);
            CHECK(lateral_near(last_w,-3.0f*yaw[i]*0.0174533f));
            CHECK(last_y==-s_v && last_x<0.0f && lateral_near(last_x,0.01f*last_y));
            CHECK(host_precise_calls && !host_integer_calls && lateral_check_ik()==0);
        }
        CHECK(!pulse_calls && !servo_calls && !laser_state);
    }
    puts("31 lateral: R1LEFT575/v250,pairLEFT15/cap80,hostageLEFT30/v250 use lkp3, lff+.01 gives rearward vx; both yaw signs correct, no added mid-yaw stop; real4wheel IK/zero FF yaw verified");
    return 0;
}

static int check_parameter_ranges_reports_and_active_locks(void)
{
    static const char *const invalid[] = {"lkp-0.001","lkp5.001","lkp","lkpnan","lkpinf","lkp1e-3","lkp2x","lkp--2",
        "lff-.05001","lff.05001","lff","lffnan","lffinf","lff1e-3","lff.01x"};
    reset_fixture();run_cmd("31");run_cmd("lkp0");CHECK(s_route31_left_yaw_kp==0.0f);
    run_cmd("lkp5");CHECK(s_route31_left_yaw_kp==5.0f);
    run_cmd("lkp2.2");run_cmd("lff0.015");
    for(unsigned i=0;i<sizeof invalid/sizeof invalid[0];++i) {
        run_cmd(invalid[i]);CHECK(strstr(last_message,"ERR")!=NULL);
        CHECK(lateral_near(s_route31_left_yaw_kp,2.2f)&&lateral_near(s_route31_left_ff_ratio,.015f));
    }
    host_messages[0]='\0';run_cmd("param");
    CHECK(strstr(host_messages,"PARAM ") && strstr(host_messages,"lff=0.0150"));
    CHECK(strstr(host_messages,"ROUTE31_LEFT_YAW lkp=2.200 left_only; lff=0.01500"));
    const char *line=strstr(host_messages,"ROUTE_PROFILE mode=31");
    CHECK(line && strstr(line,"lff=0.01500") && strstr(line,"lff=0.01500")<strchr(line,'\n'));
    run_cmd("lff-0.05");CHECK(lateral_near(s_route31_left_ff_ratio,-.05f));
    run_cmd("lff0.05");CHECK(lateral_near(s_route31_left_ff_ratio,.05f));
    run_cmd("lff0.015");run_cmd("g");
    run_cmd("lkp4");CHECK(strstr(last_message,"ERR ROUTE_SEQ_ACTIVE"));
    CHECK(lateral_near(s_route31_left_yaw_kp,2.2f));
    CHECK(sequence_start_stage()==0);tick();
    CHECK(lateral_near(s_dist_heading_kp,2.2f)&&lateral_near(s_dist_ff_ratio,.015f));
    run_cmd("lff0");CHECK(strstr(last_message,"ERR ROUTE_SEQ_ACTIVE"));
    run_cmd("lkp0");CHECK(strstr(last_message,"ERR ROUTE_SEQ_ACTIVE"));
    CHECK(lateral_near(s_route31_left_yaw_kp,2.2f)&&lateral_near(s_route31_left_ff_ratio,.015f));
    CHECK(lateral_near(s_dist_heading_kp,2.2f)&&lateral_near(s_dist_ff_ratio,.015f));
    run_cmd("g");run_cmd("lkp4");CHECK(s_route31_left_yaw_kp==4.0f);
    puts("31 lateral: BLE boundaries, malformed/NaN/Inf rejection, actual PARAM/route report and preparation/running write lock verified");
    return 0;
}

static int check_feedforward_signed_range_and_zero(void)
{
    const float ratios[]={-.05f,0.0f,.01f,.05f};
    for(unsigned i=0;i<sizeof ratios/sizeof ratios[0];++i) {
        char command[32];reset_fixture();run_cmd("31");
        snprintf(command,sizeof command,"lff%.5f",ratios[i]);run_cmd(command);
        CHECK(lateral_start(31u,0u,0)==0);
        CHECK(lateral_near(s_dist_ff_ratio,ratios[i]) && last_y==-250.0f);
        CHECK(lateral_near(last_x,ratios[i]*last_y) && last_w==0.0f);
        CHECK(i==0u ? last_x>0.0f : i==1u ? last_x==0.0f : last_x<0.0f);
        CHECK(lateral_check_ik()==0);
    }
    reset_fixture();run_cmd("31");run_cmd("lkp0");
    CHECK(lateral_start(31u,0u,0)==0);
    host_yaw=6.0f;tick();CHECK(s_dist_heading_kp==0.0f&&last_w==0.0f);
    puts("31 lateral: signed LFF boundaries/zero and lkp0 execute through real command/start/IK, not just report storage");
    return 0;
}

static int check_other_directions_owners_and_manual_profiles(void)
{
    const unsigned owners[]={34u,36u,37u,43u};
    reset_fixture();run_cmd("17");run_cmd("v100");run_cmd("ykp1.5");
    run_cmd("v300");run_cmd("ykp4.5");run_cmd("lff-.04");
    run_cmd("31");run_cmd("lkp2.2");run_cmd("lff.025");run_cmd("ykp1.2");
    CHECK(lateral_start(31u,0u,0)==0 && lateral_near(s_dist_heading_kp,2.2f));
    CHECK(lateral_near(s_dist_ff_ratio,.025f));run_cmd("g");
    CHECK(lateral_start(31u,1u,0)==0 && lateral_near(s_dist_heading_kp,1.2f));run_cmd("g");
    lateral_select(31u);run_cmd("g");s_seq_qr[0]=1;s_seq_qr[1]=2;s_seq_qr[2]=3;
    s_seq_stage=ROUTE31_TARGET_CORNER_STAGE;route_seq_prepare();
    CHECK(sequence_start_stage()==0 && s_msel==15 && lateral_near(s_dist_heading_kp,1.2f));
    run_cmd("g");
    CHECK(lateral_start(31u,6u,0)==0 && s_dist_heading_kp==3.0f && s_dist_ff_ratio==0.0f);run_cmd("g");
    for(unsigned i=0;i<sizeof owners/sizeof owners[0];++i) {
        lateral_select(owners[i]);run_cmd("lkp1");CHECK(strstr(last_message,"ERR"));
        CHECK(lateral_near(s_route31_left_yaw_kp,2.2f));
        if (owners[i]!=43u)run_cmd("lff-.02");
        CHECK(lateral_near(s_route31_left_ff_ratio,.025f));
        if (owners[i]!=37u) {
            CHECK(lateral_start(owners[i],0u,0)==0 && s_msel==17);
            CHECK(lateral_near(s_dist_heading_kp,1.2f));
            CHECK(lateral_near(s_dist_ff_ratio,-.02f));run_cmd("g");
        }
    }
    run_cmd("17");run_cmd("v100");int profile=0;
    CHECK(dist_heading_kp_get(&profile)==1.5f&&profile==1);
    run_cmd("d500");run_cmd("g");tick();CHECK(s_dist_ff_ratio==0.0f);run_cmd("g");
    host_tick+=T_DIST_STILL_MS;tick();run_cmd("v300");
    CHECK(dist_heading_kp_get(&profile)==4.5f&&profile==1);
    run_cmd("g");tick();CHECK(lateral_near(s_dist_ff_ratio,-.04f));run_cmd("g");
    run_cmd("31");CHECK(lateral_near(s_route31_left_yaw_kp,2.2f)&&lateral_near(route_left_ff_ratio(),.025f));
    CHECK(step_heading_kp_deg()==.3f && !pulse_calls && !servo_calls && !laser_state);
    run_cmd("0");test_init();
    CHECK(s_route31_left_yaw_kp==3.0f&&lateral_near(s_route31_left_ff_ratio,.01f));
    CHECK(s_route_left_ff_ratio==0.0f&&lateral_near(s_left_ff_ratio,35.0f/1500.0f)&&s_route_heading_kp==.3f);
    puts("31 lateral: straight ykp/right xkp unchanged;34/36/37/43 shared FF and manual17 speed profiles isolated; RAM retained across selection and init restores separate seeds");
    return 0;
}

static int check_g_a_zero_cancellation(void)
{
    const char *const stops[]={"g","a","0"};
    for(unsigned offset=0;offset<3;++offset)for(unsigned i=0;i<3;++i) {
        reset_fixture();CHECK(lateral_start(31u,offset==1u?ROUTE31_PAIR_STAGE:offset==2u?ROUTE31_HOSTAGE_STAGE:0u,(int)offset)==0);
        host_yaw=3.0f;tick();CHECK(last_y<0.0f&&last_x<0.0f&&last_w<0.0f);
        run_cmd(stops[i]);
        CHECK(s_seq_state==SQ_STOPPED && !last_x&&!last_y&&!last_w&&!laser_state);
        CHECK(!s_seq_pair_offset&&!s_seq_hostage_offset&&!vision_align_test_active()&&!host_timer_active);
        for(unsigned j=0;j<10;++j){host_tick+=20u;test_poll();}
        CHECK(s_seq_state==SQ_STOPPED && !last_x&&!last_y&&!last_w);
        CHECK(s_route31_left_yaw_kp==3.0f&&lateral_near(s_route31_left_ff_ratio,.01f));
    }
    puts("31 lateral: g/a/0 stop all3 left jobs immediately, cancel pair/hostage/task owners and never auto-resume");
    return 0;
}

int main(void)
{
    CHECK(check_seeds_signs_and_all_left_jobs()==0);
    CHECK(check_parameter_ranges_reports_and_active_locks()==0);
    CHECK(check_feedforward_signed_range_and_zero()==0);
    CHECK(check_other_directions_owners_and_manual_profiles()==0);
    CHECK(check_g_a_zero_cancellation()==0);
    puts("route31_lateral_hold_test: all host contracts passed; installed-load traction/heading/cross-track remain physical tests");
    return 0;
}
