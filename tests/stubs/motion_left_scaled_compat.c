/* Compatibility only for older host fixtures which stub chassis commands.
 * These fixtures do not assert a wheel-scale contract. The revised31/44
 * replay tests cover the actual motion.c IK and atomic wheel publication.
 * Do NOT link this unit into firmware or those actual-motion replay tests.
 */
extern void motion_vel_set_precise(float vx, float vy, float w);
void motion_vel_set_left_scaled(float vx, float vy, float w, float left_scale)
{
    (void)left_scale;
    motion_vel_set_precise(vx, vy, w);
}
/* Link compatibility only; actual rear/front targets are verified in revised31. */
void motion_vel_set_board_push(float vx)
{
    motion_vel_set_precise(vx, 0.0f, 0.0f);
}
/* Link-only for old chassis fixtures. No arc kinematics claim: the dedicated
 * route31_arc_transition test uses actual motion.c pair-to-wheel publication. */
void motion_vel_set_arc_pair(float left_mms, float right_mms)
{
    motion_vel_set_precise(0.5f * (left_mms + right_mms), 0.0f, 0.0f);
}
