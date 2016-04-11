// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "AP_Parachute.h"
#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Parachute::var_info[] = {

    // @Param: ENABLED
    // @DisplayName: Parachute release enabled or disabled
    // @Description: Parachute release enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("ENABLED", 0, AP_Parachute, _enabled, 0),

    // @Param: TYPE
    // @DisplayName: Parachute release mechanism type (relay or servo)
    // @Description: Parachute release mechanism type (relay or servo)
    // @Values: 0:First Relay,1:Second Relay,2:Third Relay,3:Fourth Relay,10:Servo
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_Parachute, _release_type, AP_PARACHUTE_TRIGGER_TYPE_RELAY_0),

    // @Param: SERVO_ON
    // @DisplayName: Parachute Servo ON PWM value
    // @Description: Parachute Servo PWM value when parachute is released
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_ON", 2, AP_Parachute, _servo_on_pwm, AP_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: Parachute Servo PWM value when parachute is not released
    // @Range: 1000 2000
    // @Units: pwm
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF", 3, AP_Parachute, _servo_off_pwm, AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: Parachute min altitude in meters above home
    // @Description: Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
    // @Range: 0 32000
    // @Units: Meters
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 4, AP_Parachute, _alt_min, AP_PARACHUTE_ALT_MIN_DEFAULT),

    // @Param: ALT_MAX
    // @DisplayName: Parachute max altitude in meters above home
    // @Description: Parachute max altitude above home.  Parachute will not be released above this altitude.  -1 to disable alt check.
    // @Range: -1 32000
    // @Units: Meters
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MAX", 5, AP_Parachute, _alt_max, AP_PARACHUTE_ALT_MAX_DEFAULT),

    // @Param: DELAY_MS
    // @DisplayName: Parachute release delay
    // @Description: Delay in millseconds between motor stop and chute release
    // @Range: 0 5000
    // @Units: Milliseconds
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DELAY_MS", 6, AP_Parachute, _delay_ms, AP_PARACHUTE_RELEASE_DELAY_MS),

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
    // @Param: AUTO_ON
    // @DisplayName: Parachute automatic emergency release
    // @Description: Parachute automatic emergency release enabled or disabled.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("AUTO_ON", 7, AP_Parachute, _auto_enabled, AP_PARACHUTE_AUTO_ON_DEFAULT),

    // @Param: ROLL_MRGN
    // @DisplayName: Roll deviation margin on top of LIM_ROLL_CD for automatic parachute release
    // @Description: Roll deviation margin on top of LIM_ROLL_CD at which to release parachute if in AUTO and CHUTE_AUTO_ON.
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ROLL_MRGN", 8, AP_Parachute, _emergency_roll_margin, AP_PARACHUTE_ROLL_MRGN_DEFAULT),

    // @Param: PITCH_MRGN
    // @DisplayName: Pitch deviation margin below LIM_PITCH_MIN for automatic parachute release
    // @Description: Pitch deviation margin below LIM_PITCH_MIN at which to release parachute if in AUTO and CHUTE_AUTO_ON.
    // @Units: centi-Degrees
    // @Range: 0 9000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("PITCH_MRGN", 9, AP_Parachute, _emergency_pitch_margin, AP_PARACHUTE_PITCH_MRGN_DEFAULT),

    // @Param: SINK_RATE
    // @DisplayName: Sink rate for automatic parachute release
    // @Description: Sink rate at which to release parachute if in AUTO and CHUTE_AUTO_ON.
    // @Units: m/s
    // @Range: 0.0 20.0
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SINK_RATE", 10, AP_Parachute, _emergency_sink_rate, AP_PARACHUTE_SINK_RATE_DEFAULT),


    // @Param: ALT_THRESH
    // @DisplayName: Altitude threshold for automatic parachute release
    // @Description: Altitude above home at which to release parachute if in AUTO and CHUTE_AUTO_ON.
    // @Units: m
    // @Range: 0 32000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_THRESH", 11, AP_Parachute, _emergency_alt_threshold, AP_PARACHUTE_ALT_THRESH_DEFAULT),
#endif

    AP_GROUPEND
};

/// enabled - enable or disable parachute release
void AP_Parachute::enabled(bool on_off)
{
    _enabled = on_off;

    // clear release_time
    _release_time = 0;
}

/// release - release parachute
void AP_Parachute::release()
{
    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    // set release time to current system time
    if (_release_time == 0) {
        _release_time = AP_HAL::millis();
    }

    _release_initiated = true;

    // update AP_Notify
    AP_Notify::flags.parachute_release = 1;
}

/// update - shuts off the trigger should be called at about 10hz
void AP_Parachute::update()
{
    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0) {
        return;
    }

    // calc time since release
    uint32_t time_diff = AP_HAL::millis() - _release_time;
    uint32_t delay_ms = _delay_ms<=0 ? 0: (uint32_t)_delay_ms;

    // check if we should release parachute
    if ((_release_time != 0) && !_release_in_progress) {
        if (time_diff >= delay_ms) {
            if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
                // move servo
                RC_Channel_aux::set_radio(RC_Channel_aux::k_parachute_release, _servo_on_pwm);
            }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
                // set relay
                _relay.on(_release_type);
            }
            _release_in_progress = true;
            _released = true;
        }
    }else if ((_release_time == 0) || time_diff >= delay_ms + AP_PARACHUTE_RELEASE_DURATION_MS) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            RC_Channel_aux::set_radio(RC_Channel_aux::k_parachute_release, _servo_off_pwm);
        }else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            _relay.off(_release_type);
        }
        // reset released flag and release_time
        _release_in_progress = false;
        _release_time = 0;
        // update AP_Notify
        AP_Notify::flags.parachute_release = 0;
    }
}

#if APM_BUILD_TYPE(APM_BUILD_ArduPlane)
void AP_Parachute::control_loss_ms(uint32_t time)
{
    _control_loss_ms = time;
}

void AP_Parachute::emergency_start_ms(uint32_t time)
{
    _emergency_start_ms = time;
}
#endif
