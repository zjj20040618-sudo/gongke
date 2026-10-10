/* Real IK/control code, injected encoder/PI state only. No motor-plant claim. */
#define main inherited_precise_main
#include "precise_velocity_test.c"
#undef main

static int check_small_pwm_and_four_targets(void)
{
    for (int sign = -1; sign <= 1; sign += 2) {
        host_reset();
        motion_vel_set_precise(sign*20.0f,0.0f,0.0f); host_tick();
        for (int m=0;m<4;m++) CHECK(host_motor[m].duty==0 && !s_creep[m]);
        host_reset();
        motion_vel_set_creep(sign*20.0f,0.0f,0.0f); host_tick();
        for (int m=0;m<4;m++) {
            CHECK(s_creep[m] && sign*s_target[m]>0.0f);
            CHECK(host_motor[m].duty==25 && host_motor[m].dir==(sign>0?BP_DIR_FWD:BP_DIR_REV));
        }
        float history=s_ei[0];
        motion_vel_set_creep(sign*20.0f,0.0f,0.0f);
        CHECK(s_ei[0]==history); /* Polling again must not reset integral. */
        host_tick(); CHECK(sign*s_ei[0]>sign*history);
        motion_vel_set_creep(-sign*20.0f,0.0f,0.0f);
        CHECK(s_ei[0]==0.0f && s_e_prev[0]==0.0f);
        host_tick();
        CHECK(host_motor[0].duty==25 && host_motor[0].dir==(sign>0?BP_DIR_REV:BP_DIR_FWD));
    }
    puts("creep:real v+/-20 four targets/nonzero PI fractional duty gets existing25 floor on firsttick; legacy firsttick0 unchanged; repeated commands retain integral,explicit reversal clears it passed");
    return 0;
}

static int check_sparse_feedback_does_not_reverse_drive(void)
{
    for (int sign=-1;sign<=1;sign+=2) {
        host_reset(); motion_vel_set_precise(sign*20.0f,0.0f,0.0f);
        s_rpm_lp[0]=sign*40.0f; ctrl_run_wheel(0);
        CHECK(host_motor[0].duty==25 && host_motor[0].dir==(sign>0?BP_DIR_REV:BP_DIR_FWD));
        host_reset(); motion_vel_set_creep(sign*20.0f,0.0f,0.0f);
        s_rpm_lp[0]=sign*40.0f; ctrl_run_wheel(0);
        CHECK(host_motor[0].duty==0 && host_motor[0].dir==(sign>0?BP_DIR_FWD:BP_DIR_REV));
        CHECK(s_ei[0]==0.0f);
        for (int m=1;m<4;m++) {
            ctrl_run_wheel(m);
            CHECK(host_motor[m].duty==25 && host_motor[m].dir==(sign>0?BP_DIR_FWD:BP_DIR_REV));
        }
        s_rpm_lp[0]=0.0f;ctrl_run_wheel(0);
        CHECK(host_motor[0].duty==25 && host_motor[0].dir==(sign>0?BP_DIR_FWD:BP_DIR_REV));
    }
    host_reset(); motion_vel_set_creep(20.0f,0.0f,1.0f); host_tick();
    for (int m=0;m<4;m++) CHECK(host_motor[m].duty>=25 &&
        host_motor[m].dir==(s_target[m]>0.0f?BP_DIR_FWD:BP_DIR_REV));
    CHECK(s_target[0]>0.0f && s_target[1]<0.0f); /* Legitimate IK wheel directions, not body-vx sign. */
    puts("creep:real overspeed PI in both directions reduces drive to0 instead of opposite torque; otherwheel targets intact; recovery and legit IK mixed signs preserved passed");
    return 0;
}

static int check_scope_fault_stop_and_saturation(void)
{
    host_reset();motion_vel_set_creep(20.0f,0.0f,0.0f);host_tick();
    motion_vel_set_precise(20.0f,0.0f,0.0f);
    for(int m=0;m<4;m++) CHECK(!s_creep[m]);
    motion_vel_set_creep(20.0f,0.0f,0.0f);ctrl_set_duty_open(0,-45);host_tick();
    CHECK(!s_creep[0] && host_motor[0].duty==45 && host_motor[0].dir==BP_DIR_REV);
    motion_brake();host_tick();
    for(int m=0;m<4;m++) CHECK(!s_creep[m] && s_target[m]==0.0f && host_motor[m].action==HOST_BRAKE);
    motion_vel_set_creep(20.0f,0.0f,0.0f);ctrl_coast_all();host_tick();
    for(int m=0;m<4;m++) CHECK(!s_creep[m] && host_motor[m].action==HOST_COAST);
    motion_vel_set_creep(0.0f,0.0f,0.0f);host_tick();
    for(int m=0;m<4;m++) CHECK(s_target[m]==0.0f && host_motor[m].action==HOST_BRAKE);
    const float bad[]={NAN,INFINITY,-INFINITY};
    for(unsigned n=0;n<3;n++) {
        motion_vel_set_creep(20.0f,0.0f,0.0f);motion_vel_set_creep(bad[n],0.0f,0.0f);host_tick();
        for(int m=0;m<4;m++) CHECK(!s_creep[m] && s_target[m]==0.0f && host_motor[m].action==HOST_BRAKE);
    }
    motion_vel_set_creep(20.0f,0.0f,0.0f);ctrl_set_speed_creep(0,NAN);host_tick();
    for(int m=0;m<4;m++) CHECK(!s_creep[m] && host_motor[m].action==HOST_BRAKE);
    motion_vel_set_creep(20.0f,0.0f,0.0f);s_ei[0]=20000.0f;ctrl_run_wheel(0);
    CHECK(host_motor[0].duty<=(int)MOTOR_PWM_PERIOD);
    puts("creep:normal precise/open/brake/coast isolation,zero target brake,nonfinite all-stop,original PWM limit retained passed");
    return 0;
}
int main(void)
{
    CHECK(check_small_pwm_and_four_targets()==0);
    CHECK(check_sparse_feedback_does_not_reverse_drive()==0);
    CHECK(check_scope_fault_stop_and_saturation()==0);
    CHECK(check_legacy_dispatch()==0 && check_integer_controller_compatibility()==0);
    puts("route31_creep_drive_test:host code risks fixed,not physical synchronization/grip/trajectory acceptance");
    return 0;
}
