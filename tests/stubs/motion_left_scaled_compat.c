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
