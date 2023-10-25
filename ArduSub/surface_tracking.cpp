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
 * We generally do not want to reset SurfaceTracking if the rangefinder glitches, since that will result in a new
 * target rangefinder. E.g., if a pilot is tracking at a target of 1m above the seafloor, there is a glitch, then the
 * next rangefinder reading shows 1.1m, the desired behavior is to move 10cm closer to the seafloor, vs setting a new
 * target of 1.1m above the seafloor.
 *
 * If the pilot takes control and moves to a new depth there are 2 ways to update the rangefinder target:
 * 1. Call reset(). The next call to update_surface_offset() will set a new rangefinder target. This works well if the
 *    rangefinder readings the depth readings track each other. If the rangefinder readings lag the depth readings, this
 *    can lead to the sub "bouncing back" to a previous location.
 * 2. The caller can track the change in depth, and call apply_delta_or_reset(delta) when the pilot releases control.
 *    The rangefinder target will be adjusted by the delta if possible.
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
    target_rangefinder_cm = -1;
}

void Sub::SurfaceTracking::set_target_rangefinder_cm(float new_target_cm)
{
    // do not set a target if the rangefinder is unhealthy
    if (sub.rangefinder_alt_ok()) {
        target_rangefinder_cm = new_target_cm;
        sub.pos_control.set_pos_offset_z_cm(0);
        sub.pos_control.set_pos_offset_target_z_cm(0);
        sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target is %g m", target_rangefinder_cm * 0.01f);
    } else {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder not ok, rangefinder target not set");
    }
}

void Sub::SurfaceTracking::apply_delta_cm_or_reset(float delta_cm)
{
    auto new_target_cm = target_rangefinder_cm + delta_cm;
    if (new_target_cm < (float)sub.rangefinder_state.min_cm) {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target below minimum, holding depth");
        reset();
    } else if (new_target_cm > (float)sub.rangefinder_state.max_cm) {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "rangefinder target above maximum, holding depth");
        reset();
    } else {
        sub.gcs().send_text(MAV_SEVERITY_INFO, "delta applied %g m", delta_cm * 0.01f);
        set_target_rangefinder_cm(new_target_cm);
    }
}

void Sub::SurfaceTracking::update_surface_offset()
{
    if (enabled) {
        if (sub.rangefinder_alt_ok()) {
            // handle first reading or controller reset
            if (!has_target_rangefinder()) {
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
