#pragma once

/// @file    AC_ArcNav.h
/// @brief   Constant-speed circular arc trajectory generator.
///
/// SCurve blends straight legs, so the speed at a corner falls to v*cos(theta/2)
/// and the blend also needs each leg to last long enough to open up.  A field
/// U-turn between adjacent spray lines is a 180 degree reversal over a radius of
/// half the swath, which is the worst case for both: measured in SITL, an
/// agricultural hexacopter loses 83% of its speed there at 2 m/s, and no amount
/// of WPNAV_ACCEL or WPNAV_JERK recovers it.
///
/// This generator sidesteps the blend.  It emits the position, velocity and
/// acceleration of a circular arc flown at constant tangential speed and feeds
/// them straight to AC_PosControl, the same way AC_WPNav does.  The same SITL
/// case then holds speed to within 1%.
///
/// The arc is only flyable if the vehicle can produce the centripetal
/// acceleration it needs, so set_arc() refuses a request it cannot fly rather
/// than accepting it and quietly slowing down.
///
/// The reference advances at the rate the vehicle can actually keep up with,
/// not on wall-clock time, using the same along-track error governor as
/// AC_WPNav::advance_wp_target_along_track().  Advancing on time alone lets the
/// reference run away from a lagging vehicle: the position error grows, the
/// controller saturates, and the vehicle ends up flying the right circle at a
/// fraction of the commanded speed.

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AC_AttitudeControl/AC_PosControl.h>

// Fraction of the position controller's lean angle limit that an arc is allowed
// to consume.  The remainder is headroom for tracking corrections and wind.
// Measured in SITL: an arc sized to the full limit degrades to a 31% speed loss,
// while the same arc sized to 70% of it holds speed to within 5%.
#ifndef AC_ARCNAV_TILT_FRACTION
 # define AC_ARCNAV_TILT_FRACTION   0.7f
#endif

class AC_ArcNav {
public:
    AC_ArcNav() {}

    CLASS_NO_COPY(AC_ArcNav);

    /// Set up an arc to be flown at constant tangential speed.
    ///
    /// centre_ne_m    circle centre, NE metres in the EKF origin frame
    /// radius_m       arc radius, metres
    /// start_ne_m     where the arc begins; its bearing from the centre sets the
    ///                start angle, and its distance is expected to match radius_m
    /// sweep_rad      angle to travel, positive for counter-clockwise seen from
    ///                above; +/-PI gives the half circle used by a U-turn
    /// speed_ms       tangential speed to hold for the whole arc
    /// alt_u_m        altitude to hold, metres up in the EKF origin frame
    ///
    /// Returns false and leaves the object inactive if the arc is not flyable,
    /// which is the case when v^2/r needs more lean than the budget allows.
    /// Callers should fall back to a wider radius or a lower speed.
    bool set_arc(const AC_PosControl& pos_control,
                 const Vector2f& centre_ne_m, float radius_m,
                 const Vector2f& start_ne_m, float sweep_rad,
                 float speed_ms, float alt_u_m);

    /// Advance the arc by dt and push the new targets into AC_PosControl.
    /// Returns false once the sweep is finished or if no arc is set, in which
    /// case the caller should hand control back to its normal navigation.
    bool update(AC_PosControl& pos_control, float dt);

    /// True while an arc is set and has not finished.
    bool active() const { return _active; }

    /// Along-track governor value in use, 0 to 1.  Exposed so callers can log
    /// it: a value stuck near its 0.05 floor means the reference has stalled
    /// and the vehicle is following it down rather than holding speed.
    float dt_scalar() const { return _dt_scalar; }

    /// Fraction of the sweep already flown, 0 to 1.
    float progress() const { return _sweep_rad > 0 ? constrain_float(_travelled_rad / _sweep_rad, 0.0f, 1.0f) : 1.0f; }

    /// Where the arc ends, so the caller can line up whatever follows it.
    Vector2f exit_position_ne_m() const;
    Vector2f exit_velocity_ne_ms() const;

    /// Lean angle this arc needs, radians.  Meaningful once set_arc succeeds,
    /// and also useful to a caller sizing a radius before committing to it.
    float required_lean_angle_rad() const { return _required_lean_rad; }

    /// Largest speed that the given radius can be flown at within the budget.
    /// Use it to pick a speed for a radius that geometry has already fixed.
    float max_speed_for_radius_ms(const AC_PosControl& pos_control, float radius_m) const;

    /// Smallest radius that the given speed can be flown at within the budget.
    /// Use it to pick a turn radius, e.g. how many swath lines to skip.
    float min_radius_for_speed_m(const AC_PosControl& pos_control, float speed_ms) const;

    /// Tangential speed the arc was set up to hold.
    float commanded_speed_ms() const { return _speed_ms; }

    void stop() { _active = false; }

private:
    // Lean angle an arc may consume, radians.
    static float tilt_budget_rad(const AC_PosControl& pos_control);

    // Position on the arc at the given angle, NE metres.
    Vector2f position_at_rad(float angle_rad) const;

    bool     _active = false;
    Vector2f _centre_ne_m;
    float    _radius_m = 0.0f;
    float    _start_angle_rad = 0.0f;   // bearing of the start point from the centre
    float    _sweep_rad = 0.0f;         // magnitude of the sweep
    float    _direction = 1.0f;         // +1 counter-clockwise, -1 clockwise
    float    _speed_ms = 0.0f;
    float    _alt_u_m = 0.0f;
    float    _travelled_rad = 0.0f;
    float    _required_lean_rad = 0.0f;
    float    _dt_scalar = 1.0f;         // along-track governor, 0 to 1
};
