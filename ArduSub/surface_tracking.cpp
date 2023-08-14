#include "Sub.h"

#if RANGEFINDER_ENABLED == ENABLED

/*
 * Sub::SurfaceTracking -- use a rangefinder sensor to track the seafloor
 *
 * The real work is handled by AC_PosControl.
 *
 * Surface tracking starts in the "reset" state (sub.surface_tracking.reset_target = true). In this state we are waiting
 * for a good rangefinder reading.
 *
 * When we get a good rangefinder reading, we exit the reset state:
 * -- We set the target rangefinder to the current rangefinder.
 * -- We clear the target offset: sub.pos_control.set_pos_offset_target_z_cm(0).
 * -- We clear the offset: sub.pos_control.set_pos_offset_z_cm(0) to clear the offset.
 *
 * During normal operation, SurfaceTracking is using rangefinder readings to set the target offset, and AC_PosControl
 * does the rest.
 *
 * Callers can put SurfaceTracking back into the reset state when something goes wrong or the pilot takes control.
 *
 * We generally do not want to reset SurfaceTracking if the rangefinder glitches, since that will result in a new
 * target rangefinder. E.g., if a pilot is tracking at a target of 1m above the seafloor, there is a glitch, then the
 * next rangefinder reading shows 1.1m, the desired behavior is to move 10cm closer to the seafloor, vs setting a new
 * target of 1.1m above the seafloor.
 */

void Sub::SurfaceTracking::enable(bool _enabled)
{
    enabled = _enabled;
    reset();

    // let pilot know that we're waiting for a rangefinder reading
    if (enabled && !sub.rangefinder_alt_ok()) {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "holding depth, waiting for a rangefinder reading");
    }
}

void Sub::SurfaceTracking::reset()
{
    reset_target = true;
}

void Sub::SurfaceTracking::set_target_rangefinder_cm(float new_target_cm)
{
    target_rangefinder_cm = new_target_cm;
    sub.pos_control.set_pos_offset_z_cm(0);
    sub.pos_control.set_pos_offset_target_z_cm(0);
    sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target is %g m", target_rangefinder_cm * 0.01f);
    reset_target = false;
}

void Sub::SurfaceTracking::update_surface_offset()
{
    if (enabled) {
        if (sub.rangefinder_alt_ok()) {
            // handle first reading or controller reset
            if (reset_target) {
                set_target_rangefinder_cm(sub.rangefinder_state.alt_cm_filt.get());
            }

            // calculate the offset target that will keep a constant distance above the seafloor
            // use a PID controller to handle long-ish data delays
            const float offset_target_z_cm = sub.pos_control.get_pos_offset_z_cm() +
                    sub.surface_tracking.pid_rangefinder.update_error(
                            target_rangefinder_cm - sub.rangefinder_state.alt_cm_filt.get(), 0.01);

            // set the offset target, AC_PosControl will do the rest
            sub.pos_control.set_pos_offset_target_z_cm(offset_target_z_cm);
        }
    } else {
        sub.pos_control.set_pos_offset_z_cm(0);
        sub.pos_control.set_pos_offset_target_z_cm(0);
    }
}

#endif
