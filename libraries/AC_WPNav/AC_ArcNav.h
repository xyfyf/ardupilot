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

// Fraction of the available yaw rate the turn is allowed to ask for.  Holding
// the nose on the tangent needs v/r continuously, so a turn sized to the full
// limit leaves nothing to recover a transient with and the nose never catches
// up.  Measured in SITL: r=3 m at 2 m/s needs 38 deg/s against 50-58 deg/s
// available, and the nose still trails 15 degrees at its worst.
// Ratio of a smoothstep's peak slope to its average.  d/du (3u^2 - 2u^3) peaks
// at 1.5 in the middle of the ramp, so a transition shaped this way needs to be
// 1.5x as long as a straight ramp to keep the same peak yaw acceleration.
#define AC_ARCNAV_SMOOTHSTEP_PEAK  1.5f

// How far the actual radial distance may sit from the commanded radius before
// the turn is refused.  The generator starts the path at the vehicle's position
// but curves it at the commanded radius, so the circle it flies is centred
// wherever that puts it - not necessarily at the commanded centre.  A small
// mismatch is unavoidable; a large one means the caller and the generator
// disagree about where the circle is.
#ifndef AC_ARCNAV_RADIUS_TOL_FRAC
 # define AC_ARCNAV_RADIUS_TOL_FRAC  0.20f
#endif
#ifndef AC_ARCNAV_RADIUS_TOL_MIN_M
 # define AC_ARCNAV_RADIUS_TOL_MIN_M 1.0f
#endif

// Minimum entry speed *along the entry tangent*, as a fraction of the arc
// speed.  Projecting rather than taking the magnitude tests direction and
// magnitude at once: a vehicle hovering, or moving the other way round the
// circle, both fail it, and both would otherwise have the reference set off at
// working speed from a standstill.
#ifndef AC_ARCNAV_ENTRY_SPEED_FRACTION
 # define AC_ARCNAV_ENTRY_SPEED_FRACTION  0.7f
#endif

#ifndef AC_ARCNAV_YAW_RATE_FRACTION
 # define AC_ARCNAV_YAW_RATE_FRACTION  0.5f
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
    /// sweep_rad      total heading change, positive for CLOCKWISE seen from
    ///                above; +/-PI gives the half circle used by a U-turn.
    ///                Heading here is the NE bearing atan2(East, North), which
    ///                increases N->E->S->W, i.e. clockwise on a map.
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

    /// Everything the pre-flight feasibility math reads.  Two numbers the
    /// position controller owns, two the yaw channel does, and the heading
    /// look-ahead; nothing else about the vehicle enters the decision.
    ///
    /// Naming them is not tidying.  Whether a turn is flyable is decidable from
    /// a mission file and a parameter file, with no vehicle present - but a
    /// signature that takes an AC_PosControl can only be called from inside a
    /// flying one, which is why the only caller today is a leg away from the
    /// turn.  Splitting the five scalars out lets the same arithmetic answer
    /// the question on the ground and in the air, instead of being written a
    /// second time somewhere else and drifting from this one.
    struct Limits {
        float lean_angle_max_rad;   // pos_control.get_lean_angle_max_rad()
        float jerk_ne_msss;         // pos_control.get_shaping_jerk_NE_msss()

        // Yaw rate the airframe can actually hold, at the lean angle it will be
        // working at.  This is a measured capability and not ATC_SLEW_YAW: a
        // limit parameter says what the vehicle is *allowed* to ask for, which
        // is not what it *can do*.  Set above what the airframe can deliver and
        // this check waves through turns it cannot fly.  Lean angle also costs
        // yaw authority - roll and pitch take the mixer headroom first - so the
        // hover figure is an upper bound, not the working one.  Zero leaves the
        // limit unchecked.
        float yaw_rate_max_rads;
        float yaw_accel_max_radss;  // zero leaves it unchecked
        float heading_lead_s;
    };

    /// The limits this generator is currently working to.  Combines what the
    /// position controller reports with what set_yaw_limits() was told.
    Limits limits(const AC_PosControl& pos_control) const;

    /// Everything about a turn that can be decided before the vehicle reaches
    /// it: whether the lean angle and the yaw rate it needs fit inside what the
    /// airframe has.  Both depend only on radius, speed and the limits, so they
    /// are knowable while the *previous* leg is still being planned - which is
    /// when the mission has to decide whether to aim past the entry point and
    /// carry speed in, or to slow down for an ordinary circle.
    ///
    /// Deciding it in two places was the bug this exists to prevent: the leg
    /// before the turn used to commit to a fast entry on the turn count alone,
    /// and only afterwards would set_arc() work out that the turn was not
    /// flyable - by which time the leg geometry had already been changed and
    /// the fallback circle no longer had its standard entry.
    ///
    /// On success lead_in_m is how far past the entry point the previous leg
    /// should aim, taken from the transition length the generator will really
    /// use.  Requires set_yaw_limits() to have been called.
    ///
    /// Note what lead_in_m is and is not.  It is a *requirement placed on the
    /// caller*: the previous leg has to be at least this long and must not
    /// decelerate over it.  Nothing here checks that it is.  A short working
    /// leg with a wide turn at the end of it gets true back from this function
    /// and still cannot fly the planned arc, because the run-up it was promised
    /// does not physically exist.  Whoever lays out the mission has to compare
    /// lead_in_m against the leg it belongs to; see AC_ArcNav::Limits for why
    /// that comparison belongs on the ground.
    bool plan_feasible(const AC_PosControl& pos_control, float radius_m,
                       float speed_ms, float& lead_in_m) const;

    /// Transition length the turn needs before the sweep is allowed to shorten
    /// it.  Exposed so the lead-in can be sized from it rather than guessed.
    float required_spiral_len_m(const AC_PosControl& pos_control,
                                float radius_m, float speed_ms) const;

    /// Largest speed that the given radius can be flown at within the budget.
    /// Use it to pick a speed for a radius that geometry has already fixed.
    float max_speed_for_radius_ms(const AC_PosControl& pos_control, float radius_m) const;

    /// Smallest radius that the given speed can be flown at within the budget.
    /// Use it to pick a turn radius, e.g. how many swath lines to skip.
    float min_radius_for_speed_m(const AC_PosControl& pos_control, float speed_ms) const;

    /// The same four decisions, off explicit limits rather than a live
    /// controller.  These are the primitives; the members above are wrappers
    /// that read the limits off pos_control and call straight through, so a
    /// ground check and the vehicle cannot answer differently.
    static bool  plan_feasible(const Limits& lim, float radius_m,
                               float speed_ms, float& lead_in_m);
    static float required_spiral_len_m(const Limits& lim, float radius_m, float speed_ms);
    static float max_speed_for_radius_ms(const Limits& lim, float radius_m);
    static float min_radius_for_speed_m(const Limits& lim, float speed_ms);

    /// Tangential speed the turn was set up to hold.
    float commanded_speed_ms() const { return _speed_ms; }

    /// Heading of the *desired* tangent at the reference point, radians NED.
    /// This is derived from the turn's own parameter, not from any measurement:
    /// the nose follows the planned track, and the position controller is left
    /// free to crab into a crosswind to hold that track.  Commanding heading
    /// from the measured velocity instead would make the vehicle turn into the
    /// wind and leave the swath line.
    float track_heading_rad() const;

    /// Seconds of path the heading command leads the reference point by, to
    /// cover the yaw loop's own lag.  Zero commands the tangent at the
    /// reference point itself.
    ///
    /// The look-ahead is faded in across the entry transition and back out
    /// across the exit, rather than applied at full value throughout.  Applying
    /// it flat defeats the transition it is supposed to work with: at the start
    /// of the turn the look-ahead point is already a full lead-distance in, so
    /// the commanded yaw rate starts at its maximum instead of at zero, and the
    /// ramp the transition exists to create never reaches the controller.
    /// Measured in SITL, that made the commanded rate a step from 0 to 28.6
    /// deg/s on the first sample and put a 17.9 degree spike in the heading
    /// error right after entry.
    void set_heading_lead_s(float lead_s) { _heading_lead_s = MAX(lead_s, 0.0f); }
    float heading_lead_s() const { return _heading_lead_s; }

    /// Tangent bearing at an arbitrary distance along the path, radians NED.
    /// Curvature is piecewise linear, so its integral is closed form; before
    /// the start and past the end the path is straight and the heading holds.
    float heading_at_m(float s_m) const;

    /// Distance the heading command currently looks ahead by, metres.  Not a
    /// constant: see the note on set_heading_lead_s().
    float heading_lead_m() const;

    /// Rate the tangent rotates at, rad/s, signed by the direction of travel:
    /// omega = v * curvature.  The spirals make this ramp from zero rather than
    /// step to v/r, but its peak is still v/r, so a caller checking whether the
    /// vehicle can yaw fast enough should check against peak_heading_rate_rads().
    float track_heading_rate_rads() const;

    /// Peak tangent rotation rate over the whole turn, rad/s, unsigned.
    /// On a tight, slow turn this binds before the lean angle does, so callers
    /// should check it against their yaw rate limit before committing.
    float peak_heading_rate_rads() const { return _speed_ms / MAX(_radius_m, 0.01f); }

    /// Tell the generator what the yaw channel can do, before calling set_arc.
    ///
    /// Binding the nose to the tangent makes yaw a function of the position
    /// trajectory rather than the free flat output it normally is: the tangent
    /// turns at v*curvature, so its rate of change is v^2 * d(curvature)/ds.
    /// Both of those then have to fit inside what the yaw channel can deliver,
    /// which is the weakest axis on a multirotor - it works against rotor drag
    /// torque, not against a lever arm.
    ///
    /// rate_max_rads   yaw rate the vehicle can hold, e.g. ATC_RATE_Y_MAX
    /// accel_max_radss yaw angular acceleration available, e.g. ATC_ACCEL_Y_MAX
    ///
    /// Zero for either leaves that limit unchecked.  Without this call set_arc
    /// only checks the lean angle, which on a tight turn is not the binding
    /// constraint.
    void set_yaw_limits(float rate_max_rads, float accel_max_radss) {
        _yaw_rate_max_rads = MAX(rate_max_rads, 0.0f);
        _yaw_accel_max_radss = MAX(accel_max_radss, 0.0f);
    }

    /// A U-turn planned from what the yaw channel can do, rather than from the
    /// swath.  Everything here follows from the two yaw limits and the speed.
    struct UTurnPlan {
        float radius_m;             // radius of the constant-curvature part
        float spiral_len_m;         // each transition, metres
        float duration_s;           // time to complete the whole turn
        float peak_yaw_rate_rads;   // highest tangent rotation rate reached
        bool  rate_limited;         // true if the sweep is long enough to reach
                                    // peak_yaw_rate; false means the turn is
                                    // two ramps with no constant part
    };

    /// Plan a turn from the yaw channel's capability instead of from geometry.
    ///
    /// Sizing a turn from the swath and then checking whether the yaw can keep
    /// up gets the dependency backwards: the swath is negotiable (skip a row,
    /// swing wide into the headland) while the yaw capability is not.  Start
    /// from what the vehicle can actually do and the radius falls out, along
    /// with the time the turn takes - which is what the operator cares about,
    /// since it is dead time between spray runs.
    ///
    /// With the nose on the tangent the whole turn is one yaw manoeuvre through
    /// sweep_rad, flown at constant speed.  Shaping that as a trapezoidal yaw
    /// rate profile gives, directly:
    ///
    ///   radius     = v / yaw_rate          the constant-curvature part
    ///   spiral     = v * yaw_rate / yaw_accel
    ///   duration   = yaw_rate/yaw_accel + sweep/yaw_rate
    ///
    /// If the sweep is too short to reach yaw_rate_max the profile degenerates
    /// to two ramps and the peak rate is sqrt(sweep * yaw_accel) instead.
    ///
    /// Pass the limits already derated for the working tilt angle: measured in
    /// SITL, a hover step reaches 78 deg/s but the same airframe holds only
    /// 55 deg/s at the 20 degrees of lean a turn needs, because roll and pitch
    /// take the mixer headroom first.
    static UTurnPlan plan_from_yaw_capability(float speed_ms, float sweep_rad,
                                              float yaw_rate_max_rads,
                                              float yaw_accel_max_radss);

    /// Yaw angular acceleration the transitions ask for, rad/s/s.  Constant
    /// through each spiral, because curvature ramps linearly there:
    /// d(v * curvature)/dt = v^2 * (1/r) / L_s.
    float required_yaw_accel_radss() const;

    void stop() { _active = false; }

private:
    // Lean angle a turn may consume, radians.
    static float tilt_budget_rad(const Limits& lim);

    // Curvature at distance s along the path, 1/m, unsigned.  Ramps 0 -> 1/r
    // over the entry spiral, holds 1/r, ramps back to 0 over the exit spiral.
    float curvature_at_m(float s_m) const;

    // Walk the whole path once to find where it ends.  Called by set_arc.
    Vector2f integrate_exit_position() const;

    bool     _active = false;
    Vector2f _centre_ne_m;
    float    _radius_m = 0.0f;
    float    _sweep_rad = 0.0f;         // magnitude of the total heading change
    float    _direction = 1.0f;         // +1 clockwise, -1 counter-clockwise
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
    float    _yaw_rate_max_rads = 0.0f;
    float    _yaw_accel_max_radss = 0.0f;
    Vector2f _pos_ne_m;                 // reference position at _s_m

    float    _start_heading_rad = 0.0f;
    Vector2f _start_pos_ne_m;
    Vector2f _exit_pos_ne_m;
};
