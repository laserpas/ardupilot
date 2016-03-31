// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Plane.h"

#define PARACHUTE_CHECK_TRIGGER_MS         1000  // 1 second of loss of control triggers the parachute
#define PARACHUTE_EMERGENCY_DURATION_MS    2000  // Stay for 2 seconds in emergency mode if not resolved


/* 
   call parachute library update
*/
void Plane::parachute_check()
{
    parachute.update();

    // check if there is an emer
    parachute_emergency_check();
}

/*
  parachute_release - trigger the release of the parachute
*/
void Plane::parachute_release()
{
    if (parachute.released()) {
        return;
    }
    
    // send message to gcs and dataflash
    gcs_send_text(MAV_SEVERITY_CRITICAL,"Parachute: Released");

    // release parachute
    parachute.release();
}

/*
  parachute_manual_release - trigger the release of the parachute,
  after performing some checks for pilot error checks if the vehicle
  is landed
*/
bool Plane::parachute_manual_release()
{
    // exit immediately if parachute is not enabled
    if (!parachute.enabled() || parachute.released()) {
        return false;
    }

    // do not release if vehicle is not flying
    if (!is_flying()) {
        // warn user of reason for failure
        gcs_send_text(MAV_SEVERITY_WARNING,"Parachute: Not flying");
        return false;
    }

    // if we get this far release parachute
    parachute_release();

    return true;
}

/*
  parachute_emergency_check - trigger the release of the parachute
  automatically if a critical situation is detected
*/
void Plane::parachute_emergency_check()
{
    uint32_t now = millis();
    bool emergency_mode = false;

    // exit immediately if parachute or automatic release is not enabled, or already released
    if (!parachute.auto_enabled() || parachute.release_initiated()) {
        parachute.control_loss_ms(0);
        return;
    }

    // only automatically release in AUTO mode
    if (control_mode != AUTO) {
        parachute.control_loss_ms(0);
        return;
    }

    // do not release if taking off or landing
    if (auto_state.takeoff_complete == false ||
        auto_state.land_complete == true ||
        mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND) {

        parachute.control_loss_ms(0);
        return;
    }

    if (labs(ahrs.roll_sensor) < g.roll_limit_cd + parachute.emergency_roll_margin() &&
        ahrs.pitch_sensor > aparm.pitch_limit_min_cd - parachute.emergency_pitch_margin() &&
        auto_state.sink_rate < parachute.emergency_sink_rate()) {
        // none of the advanced emergency triggers worked
    } else {
        // some of the advanced emergency triggers worked
        gcs_send_text_fmt(MAV_SEVERITY_WARNING, "Emergency: Roll %d, Pitch %d, Sink %.2f", labs(ahrs.roll_sensor), ahrs.pitch_sensor, auto_state.sink_rate);
    }

    // do not release if emergency altitude is not reached
    if (relative_altitude() > parachute.emergency_alt_threshold()) {
        parachute.control_loss_ms(0);
        return;
    }

    // at this point we consider control lost
    if (parachute.control_loss_ms() == 0) {
        gcs_send_text(MAV_SEVERITY_WARNING, "Emergency: Starting to lose control");
        parachute.control_loss_ms(now);
    }

    if (now - parachute.control_loss_ms() > PARACHUTE_CHECK_TRIGGER_MS) {
        // we have not been in a good state for a while, enter emergency mode
        emergency_mode = true;
        parachute.emergency_start_ms(now);
    }

    if (emergency_mode && now - parachute.emergency_start_ms() < PARACHUTE_EMERGENCY_DURATION_MS) {
        // try to release parachute whenever in emergency mode
        if (relative_altitude() > parachute.alt_min() &&
            (parachute.alt_max() < 0 || relative_altitude() < parachute.alt_max())) {
            // altitude suitable for parachute deployment
            parachute_release();
        }
    } else if (emergency_mode) {
        // emergency mode expired (neither continuous loss of control, nor parachute deployed)
        gcs_send_text(MAV_SEVERITY_WARNING, "Emergency: Control restored");
        emergency_mode = false;
    }
}
