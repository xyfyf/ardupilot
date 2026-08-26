#pragma once

/// @file    AC_ArcNav.h
/// @brief   Constant-speed turn generator with clothoid transitions.
///
/// SCurve blends straight legs, so the speed at a corner falls to v*cos(theta/2)
/// and the blend also needs each leg to last long enough to open up.  A field
/// U-turn between adjacent spray lines is a 180 degree reversal over a radius of
/// half the swath, which is the worst case for both: measured in SITL, an
/// agricultural hexacopter loses 83% of its speed there at 2 m/s, and no amount
/// of WPNAV_ACCEL or WPNAV_JERK recovers it.
///
/// This generator sidesteps the blend.  It emits the position, velocity and
/// acceleration of a turn flown at constant tangential speed and feeds them
/// straight to AC_PosControl, the same way AC_WPNav does.  The same SITL case
/// then holds speed to within 1%.
///
/// The turn is not a bare circular arc.  Joining a straight leg to a circle
/// steps the curvature from 0 to 1/r in one sample, which steps the lateral
/// acceleration and the tangent's rotation rate with it: measured in SITL,
/// 0 to 1.33 m/s2 and 0 to 38 deg/s at r=3 m and 2 m/s.  Nothing physical can
/// follow a step, so the vehicle lags and the nose trails the tangent by a mean
/// of 9.6 degrees through the turn.  This is the same defect that makes a
/// Dubins path only theoretically constant-speed, and the fix ground vehicles
/// have used on headland turns, and road builders long before them, is to
/// insert a transition curve where the curvature ramps linearly with distance.
/// Both the lateral acceleration and the yaw rate then become ramps instead of
/// steps, and the vehicle can actually track them.
///
/// The profile flown is therefore: entry spiral, constant-curvature arc, exit
/// spiral.  The spiral length follows from the jerk the position controller is
/// shaping to, L_s = v^3 / (r * jerk), and the arc takes up the remainder,
/// L_c = r * sweep - L_s.  Turn a curvature ramp into a heading and it is a
/// Fresnel integral, so the path is integrated numerically rather than
/// evaluated in closed form; the cost is a handful of trig calls per loop.
///
/// The turn is only flyable if the vehicle can produce the centripetal
/// acceleration the tightest part of it needs, so set_arc() refuses a request
/// it cannot fly rather than accepting it and quietly slowing down.
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

// Fraction of the position controller's lean angle limit that a turn is allowed
// to consume.  The remainder is headroom for tracking corrections and wind.
// Measured in SITL: an arc sized to the full limit degrades to a 31% speed loss,
// while the same arc sized to 70% of it holds speed to within 5%.
#ifndef AC_ARCNAV_TILT_FRACTION
 # define AC_ARCNAV_TILT_FRACTION   0.7f
#endif

// Steps used to pre-integrate the path when set_arc() works out where the turn
// ends.  The integral is a Fresnel integral with no closed form in floats; 512
// midpoint steps put the exit position inside a millimetre for any turn a
// vehicle of this class flies, and it is paid once per turn, not per loop.
#ifndef AC_ARCNAV_PREINTEGRATE_STEPS
 # define AC_ARCNAV_PREINTEGRATE_STEPS  512
#endif

// Seconds of path the heading command looks ahead by.  The yaw loop cannot
// follow its demand instantly: measured in SITL on the tight U-turn (r=3 m,
// 2 m/s, tangent turning at 38 deg/s), the rate controller asks for 105 deg/s
// and the airframe delivers 50, so the nose trails the tangent by 21 degrees
// through the first third of the turn and settles around 5.  That lag is close
// to first order, so commanding the tangent the vehicle will need one lag time
// from now cancels it to first order.
//
// The look-ahead is taken along the *planned* path, never from the measured
// velocity: the nose still tracks where the trajectory is going, so a sprayer
// in a crosswind keeps crabbing to hold its swath line instead of turning into
// the wind and walking off it.
#ifndef AC_ARCNAV_HEADING_LEAD_S
 # define AC_ARCNAV_HEADING_LEAD_S  0.5f
#endif

class AC_ArcNav {
public:
    AC_ArcNav() {}

    CLASS_NO_COPY(AC_ArcNav);

    /// Set up a turn to be flown at constant tangential speed.
    ///
    /// centre_ne_m    centre of the nominal circle, NE metres in the EKF origin
    ///                frame.  It fixes where the turn starts and which way it
    ///                curves; the flown path leaves that circle by the small
    ///                offset the transition spirals introduce, so ask
    ///                exit_position_ne_m() where the turn actually ends rather
    ///                than assuming a point on the circle.
    /// radius_m       radius of the constant-curvature part, metres.  This is
    ///                the tightest the path ever gets, so it is what sets the
    ///                lean angle.
    /// start_ne_m     where the turn begins; its bearing from the centre sets
    ///                the start tangent, and its distance is expected to match
    ///                radius_m
    /// sweep_rad      total heading change, positive for counter-clockwise seen
    ///                from above; +/-PI gives the half circle used by a U-turn
    /// speed_ms       tangential speed to hold for the whole turn
    /// alt_u_m        altitude to hold, metres up in the EKF origin frame
    ///
    /// Returns false and leaves the object inactive if the turn is not flyable,
    /// which is the case when v^2/r needs more lean than the budget allows.
    /// Callers should fall back to a wider radius or a lower speed.
    bool set_arc(const AC_PosControl& pos_control,
                 const Vector2f& centre_ne_m, float radius_m,
                 const Vector2f& start_ne_m, float sweep_rad,
                 float speed_ms, float alt_u_m);

    /// Advance the turn by dt and push the new targets into AC_PosControl.
    /// Returns false once the sweep is finished or if no turn is set, in which
    /// case the caller should hand control back to its normal navigation.
    bool update(AC_PosControl& pos_control, float dt);

    /// True while a turn is set and has not finished.
    bool active() const { return _active; }

    /// Along-track governor value in use, 0 to 1.  Exposed so callers can log
    /// it: a value stuck near its 0.05 floor means the reference has stalled
    /// and the vehicle is following it down rather than holding speed.
    float dt_scalar() const { return _dt_scalar; }

    /// Fraction of the turn already flown by distance, 0 to 1.
    float progress() const { return is_positive(_total_len_m) ? constrain_float(_s_m / _total_len_m, 0.0f, 1.0f) : 1.0f; }

    /// Length of each transition spiral, metres.  Zero means the turn is a bare
    /// circular arc, which happens when the sweep is too short to fit a spiral
    /// or the controller reports no jerk limit to size one from.
    float spiral_length_m() const { return _spiral_len_m; }

    /// Total path length of the turn, metres.
    float total_length_m() const { return _total_len_m; }

    /// Where the turn ends, so the caller can line up whatever follows it.
    /// This is the integrated end of the flown path, spirals included, not a
    /// point on the nominal circle.
    Vector2f exit_position_ne_m() const { return _exit_pos_ne_m; }
    Vector2f exit_velocity_ne_ms() const;

    /// Lean angle this turn needs at its tightest, radians.  Meaningful once
    /// set_arc succeeds, and also useful to a caller sizing a radius before
    /// committing to it.
    float required_lean_angle_rad() const { return _required_lean_rad; }

    /// Largest speed that the given radius can be flown at within the budget.
    /// Use it to pick a speed for a radius that geometry has already fixed.
    float max_speed_for_radius_ms(const AC_PosControl& pos_control, float radius_m) const;

    /// Smallest radius that the given speed can be flown at within the budget.
    /// Use it to pick a turn radius, e.g. how many swath lines to skip.
    float min_radius_for_speed_m(const AC_PosControl& pos_control, float speed_ms) const;

    /// Tangential speed the turn was set up to hold.
    float commanded_speed_ms() const { return _speed_ms; }

    /// Heading of the *desired* tangent at the reference point, radians NED.
    /// This is derived from the turn's own parameter, not from any measurement:
    /// the nose follows the planned track, and the position controller is left
    /// free to crab into a crosswind to hold that track.  Commanding heading
    /// from the measured velocity instead would make the vehicle turn into the
    /// wind and leave the swath line.
    float track_heading_rad() const { return heading_at_m(_s_m + _speed_ms * _heading_lead_s); }

    /// Seconds of path the heading command leads the reference point by, to
    /// cover the yaw loop's own lag.  Zero commands the tangent at the
    /// reference point itself.
    void set_heading_lead_s(float lead_s) { _heading_lead_s = MAX(lead_s, 0.0f); }
    float heading_lead_s() const { return _heading_lead_s; }

    /// Tangent bearing at an arbitrary distance along the path, radians NED.
    /// Curvature is piecewise linear, so its integral is closed form; before
    /// the start and past the end the path is straight and the heading holds.
    float heading_at_m(float s_m) const;

    /// Rate the tangent rotates at, rad/s, signed by the direction of travel:
    /// omega = v * curvature.  The spirals make this ramp from zero rather than
    /// step to v/r, but its peak is still v/r, so a caller checking whether the
    /// vehicle can yaw fast enough should check against peak_heading_rate_rads().
    float track_heading_rate_rads() const;

    /// Peak tangent rotation rate over the whole turn, rad/s, unsigned.
    /// On a tight, slow turn this binds before the lean angle does, so callers
    /// should check it against their yaw rate limit before committing.
    float peak_heading_rate_rads() const { return _speed_ms / MAX(_radius_m, 0.01f); }

    void stop() { _active = false; }

private:
    // Lean angle a turn may consume, radians.
    static float tilt_budget_rad(const AC_PosControl& pos_control);

    // Curvature at distance s along the path, 1/m, unsigned.  Ramps 0 -> 1/r
    // over the entry spiral, holds 1/r, ramps back to 0 over the exit spiral.
    float curvature_at_m(float s_m) const;

    // Walk the whole path once to find where it ends.  Called by set_arc.
    Vector2f integrate_exit_position() const;

    bool     _active = false;
    Vector2f _centre_ne_m;
    float    _radius_m = 0.0f;
    float    _sweep_rad = 0.0f;         // magnitude of the total heading change
    float    _direction = 1.0f;         // +1 counter-clockwise, -1 clockwise
    float    _speed_ms = 0.0f;
    float    _alt_u_m = 0.0f;
    float    _required_lean_rad = 0.0f;
    float    _dt_scalar = 1.0f;         // along-track governor, 0 to 1

    // path shape, all metres of arc length
    float    _spiral_len_m = 0.0f;      // each transition spiral
    float    _arc_len_m = 0.0f;         // constant-curvature part between them
    float    _total_len_m = 0.0f;       // 2 * spiral + arc

    // integration state, stepped by update()
    float    _s_m = 0.0f;               // distance flown along the path
    float    _heading_rad = 0.0f;       // tangent bearing at _s_m
    float    _heading_lead_s = AC_ARCNAV_HEADING_LEAD_S;
    Vector2f _pos_ne_m;                 // reference position at _s_m

    float    _start_heading_rad = 0.0f;
    Vector2f _start_pos_ne_m;
    Vector2f _exit_pos_ne_m;
};
