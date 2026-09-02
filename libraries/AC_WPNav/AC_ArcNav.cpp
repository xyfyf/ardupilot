#include "AC_ArcNav.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

// Curvature shape across a transition, 0 to 1 over the normalised length.
// Zero slope at both ends is the whole point: that is what makes the yaw
// acceleration - and so the yaw torque - continuous rather than stepped.
static float smoothstep(float u)
{
    u = constrain_float(u, 0.0f, 1.0f);
    return sq(u) * (3.0f - 2.0f * u);
}

// Integral of smoothstep from 0 to u, i.e. u^3 - u^4/2.  Over a whole
// transition (u = 1) this is 1/2, so a transition of length L turns through
// L/(2r) - exactly what a linear ramp would.
static float smoothstep_integral(float u)
{
    u = constrain_float(u, 0.0f, 1.0f);
    const float u3 = u * sq(u);
    return u3 - 0.5f * u3 * u;
}

float AC_ArcNav::tilt_budget_rad(const AC_PosControl& pos_control)
{
    return pos_control.get_lean_angle_max_rad() * constrain_float(AC_ARCNAV_TILT_FRACTION, 0.1f, 1.0f);
}

float AC_ArcNav::max_speed_for_radius_ms(const AC_PosControl& pos_control, float radius_m) const
{
    if (!is_positive(radius_m)) {
        return 0.0f;
    }
    // a = g*tan(tilt), and holding a circle needs a = v^2 / r
    return safe_sqrt(GRAVITY_MSS * tanf(tilt_budget_rad(pos_control)) * radius_m);
}

float AC_ArcNav::min_radius_for_speed_m(const AC_PosControl& pos_control, float speed_ms) const
{
    const float a_max = GRAVITY_MSS * tanf(tilt_budget_rad(pos_control));
    if (!is_positive(a_max)) {
        return 0.0f;
    }
    return sq(speed_ms) / a_max;
}

// Curvature at distance s along the path, 1/m and unsigned.  This is the whole
// shape of the turn: a linear ramp up, a flat top, a linear ramp down.  Every
// other quantity the generator emits is an integral or a product of it.
float AC_ArcNav::curvature_at_m(float s_m) const
{
    if (!is_positive(_radius_m)) {
        return 0.0f;
    }
    if (s_m <= 0.0f || s_m >= _total_len_m) {
        // outside the turn the path is straight, which the look-ahead relies on
        return 0.0f;
    }
    const float inv_r = 1.0f / _radius_m;
    if (!is_positive(_spiral_len_m)) {
        // no room for transitions, so the path is a bare arc
        return inv_r;
    }
    // Smoothstep rather than a straight ramp.  A linear curvature ramp is the
    // classic clothoid and it bounds the yaw acceleration, but that
    // acceleration still steps at both ends of the ramp - and yaw acceleration
    // is yaw torque, which comes from spinning rotors up and down against their
    // own inertia.  A torque step is no more followable than the rate step the
    // clothoid was introduced to remove, just one derivative down.  Smoothstep
    // has zero slope at both ends, so the torque comes on and goes off
    // continuously.
    //
    // It integrates to the same total turn as the linear ramp, L_s/(2r), so the
    // geometry of the turn is unchanged; what it costs is a 1.5x higher peak
    // slope for the same length, which set_arc pays for by lengthening L_s.
    if (s_m <= _spiral_len_m) {
        return inv_r * smoothstep(s_m / _spiral_len_m);
    }
    const float exit_start_m = _spiral_len_m + _arc_len_m;
    if (s_m >= exit_start_m) {
        return inv_r * smoothstep((_total_len_m - s_m) / _spiral_len_m);
    }
    return inv_r;
}

// Heading change accumulated between the start of the path and s, unsigned.
// The curvature ramps are triangles and the arc a rectangle, so the integral is
// closed form.  Using it rather than accumulating the per-step integration
// keeps the commanded heading exact and lets it be evaluated ahead of the
// reference point, which is what the look-ahead needs.
float AC_ArcNav::heading_at_m(float s_m) const
{
    float swept_rad;
    if (!is_positive(_radius_m) || s_m <= 0.0f) {
        // before the turn starts the path is the straight leg
        swept_rad = 0.0f;
    } else if (s_m >= _total_len_m) {
        // past the end it is the next straight leg
        swept_rad = _sweep_rad;
    } else if (!is_positive(_spiral_len_m)) {
        swept_rad = s_m / _radius_m;
    } else if (s_m <= _spiral_len_m) {
        // entry transition: integral of smoothstep, which is u^3 - u^4/2
        swept_rad = _spiral_len_m * smoothstep_integral(s_m / _spiral_len_m) / _radius_m;
    } else if (s_m <= _spiral_len_m + _arc_len_m) {
        // a whole transition turns through L_s/(2r), same as a linear ramp
        swept_rad = _spiral_len_m / (2.0f * _radius_m) + (s_m - _spiral_len_m) / _radius_m;
    } else {
        // exit transition: the whole sweep less what the remaining tail turns
        const float remaining_m = _total_len_m - s_m;
        swept_rad = _sweep_rad - _spiral_len_m * smoothstep_integral(remaining_m / _spiral_len_m) / _radius_m;
    }
    return wrap_PI(_start_heading_rad + _direction * swept_rad);
}

// A curvature that varies with distance integrates to a Fresnel integral, which
// has no closed form to evaluate here, so walk the path once instead.  Midpoint
// steps because the error then goes as the square of the step: at 512 steps the
// end point lands inside a millimetre, and this runs once when the turn is set.
Vector2f AC_ArcNav::integrate_exit_position() const
{
    Vector2f pos = _start_pos_ne_m;
    float heading = _start_heading_rad;
    const float ds = _total_len_m / AC_ARCNAV_PREINTEGRATE_STEPS;
    float s = 0.0f;
    for (uint16_t i = 0; i < AC_ARCNAV_PREINTEGRATE_STEPS; i++) {
        const float k_mid = curvature_at_m(s + ds * 0.5f);
        const float heading_mid = heading + _direction * k_mid * ds * 0.5f;
        pos += Vector2f{cosf(heading_mid), sinf(heading_mid)} * ds;
        heading += _direction * k_mid * ds;
        s += ds;
    }
    return pos;
}

AC_ArcNav::UTurnPlan AC_ArcNav::plan_from_yaw_capability(float speed_ms, float sweep_rad,
                                                         float yaw_rate_max_rads,
                                                         float yaw_accel_max_radss)
{
    UTurnPlan plan {};
    sweep_rad = fabsf(sweep_rad);
    if (!is_positive(speed_ms) || !is_positive(sweep_rad) ||
        !is_positive(yaw_rate_max_rads) || !is_positive(yaw_accel_max_radss)) {
        return plan;
    }

    // Reaching the rate limit and coming back down costs rate^2/accel of the
    // sweep.  If the sweep is smaller than that the profile never gets there.
    // A smoothstep ramp takes 1.5x as long as a linear one to reach the same
    // rate under the same acceleration limit, because its peak slope is 1.5x
    // its average.  That is the price of a continuous torque.
    const float ramp_sweep_rad = AC_ARCNAV_SMOOTHSTEP_PEAK * sq(yaw_rate_max_rads) / yaw_accel_max_radss;
    plan.rate_limited = ramp_sweep_rad <= sweep_rad;

    if (plan.rate_limited) {
        plan.peak_yaw_rate_rads = yaw_rate_max_rads;
        // ramp up, hold, ramp down
        plan.duration_s = AC_ARCNAV_SMOOTHSTEP_PEAK * yaw_rate_max_rads / yaw_accel_max_radss
                          + sweep_rad / yaw_rate_max_rads;
    } else {
        // two ramps meeting at the peak: sweep = peak^2/accel
        plan.peak_yaw_rate_rads = safe_sqrt(sweep_rad * yaw_accel_max_radss / AC_ARCNAV_SMOOTHSTEP_PEAK);
        plan.duration_s = 2.0f * AC_ARCNAV_SMOOTHSTEP_PEAK * plan.peak_yaw_rate_rads / yaw_accel_max_radss;
    }

    // The path follows from the yaw profile: curvature is yaw rate over speed,
    // so the tightest part has radius v/peak_rate, and the ramp covers the
    // distance flown while the rate builds.
    plan.radius_m = speed_ms / plan.peak_yaw_rate_rads;
    plan.spiral_len_m = AC_ARCNAV_SMOOTHSTEP_PEAK * speed_ms * plan.peak_yaw_rate_rads / yaw_accel_max_radss;
    return plan;
}

// Length the entry and exit transitions have to be, before the sweep is allowed
// to shorten them.  Pulled out of set_arc() so the mission planner can size the
// lead-in leg from the same number the generator will actually use: a fixed
// lead distance is either wasteful or too short the moment speed, radius or the
// jerk limit changes.
//
// Depends on the yaw limits, so set_yaw_limits() must have run first.
bool AC_ArcNav::plan_feasible(const AC_PosControl& pos_control, float radius_m,
                              float speed_ms, float& lead_in_m) const
{
    lead_in_m = 0.0f;
    if (!is_positive(radius_m) || !is_positive(speed_ms)) {
        return false;
    }
    // Lean.  The constant-curvature part is the tightest the path gets, so
    // radius_m is what has to be checked - same test set_arc() applies.
    if (speed_ms > max_speed_for_radius_ms(pos_control, radius_m)) {
        return false;
    }
    // Yaw rate.  Binding the nose to the tangent costs v/r for the whole arc,
    // and on a multirotor yaw is the weakest axis, so this bites first on a
    // tight turn.  Half the budget is left as headroom for the entry transient.
    if (is_positive(_yaw_rate_max_rads) &&
        speed_ms / radius_m > _yaw_rate_max_rads * AC_ARCNAV_YAW_RATE_FRACTION) {
        return false;
    }
    // Aim one transition past the entry point, plus the look-ahead the heading
    // uses.  That is the distance over which the path is still straightening
    // out, so it is exactly the distance the previous leg must not decelerate
    // over.
    lead_in_m = required_spiral_len_m(pos_control, radius_m, speed_ms)
                + speed_ms * _heading_lead_s;
    return true;
}

float AC_ArcNav::required_spiral_len_m(const AC_PosControl& pos_control,
                                       float radius_m, float speed_ms) const
{
    if (!is_positive(radius_m) || !is_positive(speed_ms)) {
        return 0.0f;
    }
    const float jerk = pos_control.get_shaping_jerk_NE_msss();
    float spiral_len_m = 0.0f;
    if (is_positive(jerk)) {
        spiral_len_m = (sq(speed_ms) * speed_ms) / (radius_m * jerk);
    }
    // The transition must also be at least as long as the heading look-ahead,
    // or the look-ahead point skips straight over the ramp into the constant
    // arc and the yaw command steps to its full rate on the first sample -
    // measured in SITL, exactly what happened with a 1.0 m look-ahead over a
    // 0.53 m spiral.  Sizing the ramp to cover the look-ahead keeps the yaw
    // command a ramp, which is the whole point of the transition.
    spiral_len_m = MAX(spiral_len_m, speed_ms * _heading_lead_s);
    // With the nose on the tangent, yaw accelerates at v^2/(r*L_s) through a
    // spiral.  A short spiral therefore asks for a yaw acceleration the vehicle
    // may not have; lengthening it is the only way to lower that demand without
    // slowing down or opening the radius, both of which are ruled out here -
    // the speed is fixed by the spray rate and the radius by the swath.
    if (is_positive(_yaw_accel_max_radss)) {
        // The commanded heading rate is v*curvature evaluated at the look-ahead
        // point, so its rate of change carries two factors:
        //
        //   yaw_accel = v^2 * curvature'(s) * (1 + lead'(s))
        //
        // curvature' peaks at k/(r*L_s), and the look-ahead fades in across the
        // transition so lead'(s) = v*tau/L_s there.  Ignoring that second factor
        // under-sizes the transition: it is what turns a ramp the vehicle could
        // have followed into one it cannot.  Requiring the peak to fit gives a
        // quadratic in L_s, whose positive root is the shortest transition that
        // keeps the commanded yaw acceleration inside what the vehicle has.
        const float a = AC_ARCNAV_SMOOTHSTEP_PEAK * sq(speed_ms) / (radius_m * _yaw_accel_max_radss);
        const float lead_m = speed_ms * _heading_lead_s;
        const float min_len_m = 0.5f * (a + safe_sqrt(sq(a) + 4.0f * a * lead_m));
        spiral_len_m = MAX(spiral_len_m, min_len_m);
    }
    // Each spiral turns through spiral/(2r), so the pair uses spiral/r of the
    // sweep and the arc takes what is left.  A sweep too small to fit both
    // spirals gets as much transition as it can and no constant arc at all.
    return spiral_len_m;
}

bool AC_ArcNav::set_arc(const AC_PosControl& pos_control,
                        const Vector2f& centre_ne_m, float radius_m,
                        const Vector2f& start_ne_m, float sweep_rad,
                        float speed_ms, float alt_u_m)
{
    _active = false;

    if (!is_positive(radius_m) || !is_positive(speed_ms) || is_zero(sweep_rad)) {
        return false;
    }

    // Refuse a turn the vehicle cannot hold speed on.  Accepting it would just
    // move the failure into flight, where it shows up as the speed loss this
    // generator exists to remove.  The tightest part of the path is the
    // constant-curvature arc, so radius_m is what has to be checked.
    const float accel_needed = sq(speed_ms) / radius_m;
    _required_lean_rad = atanf(accel_needed / GRAVITY_MSS);
    if (_required_lean_rad > tilt_budget_rad(pos_control)) {
        return false;
    }

    // Binding the nose to the tangent costs a yaw rate of v/r for the whole
    // constant-curvature part.  Multirotor yaw works against rotor drag torque
    // rather than a lever arm, so it is the weakest axis and this constraint
    // bites long before the lean angle does on a tight turn.  Leave half the
    // available rate as headroom: a turn that asks for all of it has nothing
    // left to recover the entry transient with.
    if (is_positive(_yaw_rate_max_rads) &&
        speed_ms / radius_m > _yaw_rate_max_rads * AC_ARCNAV_YAW_RATE_FRACTION) {
        return false;
    }

    const Vector2f offset = start_ne_m - centre_ne_m;
    const float offset_len_m = offset.length();
    if (offset_len_m < 0.1f) {
        // start point sits on the centre, so it defines no bearing
        return false;
    }

    // The commanded centre has to agree with where the vehicle actually is.
    // Only the bearing of `offset` is used below; the path then curves at
    // radius_m from the vehicle's own position, so if the two disagree the
    // circle that gets flown is not the one that was asked for and nothing
    // downstream ever notices.
    if (fabsf(offset_len_m - radius_m) >
        MAX(AC_ARCNAV_RADIUS_TOL_FRAC * radius_m, AC_ARCNAV_RADIUS_TOL_MIN_M)) {
        return false;
    }

    // Entry state.  Checked here rather than in each caller because the caller
    // that had no check at all was AUTO, where LOITER_TURNS can follow a hover,
    // a delay or a takeoff - the reference would then step to working speed
    // from a standstill and the vehicle would spend the entry transition
    // catching up with it.  Projecting the velocity on the entry tangent covers
    // direction too: going the other way round the circle fails the same test.
    const float dir = is_positive(sweep_rad) ? 1.0f : -1.0f;
    const float entry_heading_rad = wrap_PI(atan2f(offset.y, offset.x) + dir * M_PI_2);
    const Vector2f tangent{cosf(entry_heading_rad), sinf(entry_heading_rad)};
    const Vector2f vel_ne_ms = pos_control.get_vel_estimate_NEU_ms().xy();
    if ((vel_ne_ms * tangent) < speed_ms * AC_ARCNAV_ENTRY_SPEED_FRACTION) {
        return false;
    }

    _centre_ne_m = centre_ne_m;
    _radius_m = radius_m;
    _direction = is_positive(sweep_rad) ? 1.0f : -1.0f;
    _sweep_rad = fabsf(sweep_rad);
    _speed_ms = speed_ms;
    _alt_u_m = alt_u_m;

    // The tangent at the start is the radius turned a quarter circle the way
    // the vehicle is going.
    _start_pos_ne_m = start_ne_m;
    _start_heading_rad = entry_heading_rad;

    // Size the transitions from the jerk the position controller is already
    // shaping to, so the turn asks for no more rate of change of acceleration
    // than the straight legs do.  Ramping the lateral acceleration from 0 to
    // v^2/r at that jerk takes v^2/(r*jerk) seconds, hence v^3/(r*jerk) metres.
    // Same sizing the planner used to place the lead-in point; see
    // required_spiral_len_m().
    float spiral_len_m = required_spiral_len_m(pos_control, radius_m, speed_ms);

    const float nominal_len_m = radius_m * _sweep_rad;
    spiral_len_m = MIN(spiral_len_m, nominal_len_m);

    _spiral_len_m = spiral_len_m;
    _arc_len_m = nominal_len_m - spiral_len_m;
    _total_len_m = 2.0f * spiral_len_m + _arc_len_m;

    _s_m = 0.0f;
    _heading_rad = _start_heading_rad;
    _pos_ne_m = _start_pos_ne_m;
    _dt_scalar = 1.0f;
    _exit_pos_ne_m = integrate_exit_position();

    _active = true;
    return true;
}

// Look-ahead distance in use right now.  It fades in over the entry transition
// and out over the exit so that the commanded heading rate still starts and ends
// at zero - the turn joins a straight leg at both ends, and a straight leg has
// no curvature, so anything else is a step.  Through the constant-curvature part
// the full lead applies, which is where the yaw lag actually needs covering.
float AC_ArcNav::heading_lead_m() const
{
    const float full_m = _speed_ms * _heading_lead_s;
    if (!is_positive(_spiral_len_m)) {
        return full_m;
    }
    if (_s_m < _spiral_len_m) {
        return full_m * constrain_float(_s_m / _spiral_len_m, 0.0f, 1.0f);
    }
    const float exit_start_m = _spiral_len_m + _arc_len_m;
    if (_s_m > exit_start_m) {
        return full_m * constrain_float((_total_len_m - _s_m) / _spiral_len_m, 0.0f, 1.0f);
    }
    return full_m;
}

float AC_ArcNav::track_heading_rad() const
{
    return heading_at_m(_s_m + heading_lead_m());
}

float AC_ArcNav::required_yaw_accel_radss() const
{
    if (!is_positive(_spiral_len_m) || !is_positive(_radius_m)) {
        // a bare arc steps the tangent rate, which is an unbounded demand
        return FLT_MAX;
    }
    // smoothstep peaks at 1.5x the average slope, at the middle of the ramp
    return AC_ARCNAV_SMOOTHSTEP_PEAK * sq(_speed_ms) / (_radius_m * _spiral_len_m);
}

float AC_ArcNav::track_heading_rate_rads() const
{
    // Taken at the look-ahead point too, so the rate feedforward stays the
    // derivative of the heading it is paired with.
    return _direction * _speed_ms * curvature_at_m(_s_m + heading_lead_m());
}

Vector2f AC_ArcNav::exit_velocity_ne_ms() const
{
    // The spirals change where the path ends but not which way it points: the
    // curvature ramps integrate to exactly the sweep that was asked for.
    const float heading = wrap_PI(_start_heading_rad + _direction * _sweep_rad);
    return Vector2f{cosf(heading), sinf(heading)} * _speed_ms;
}

bool AC_ArcNav::update(AC_PosControl& pos_control, float dt)
{
    if (!_active || !is_positive(dt)) {
        return false;
    }

    // Advance the reference at the rate the vehicle is actually keeping up with,
    // the same governor AC_WPNav uses.  track_error is the position error
    // projected onto the direction of travel and track_velocity the vehicle's
    // speed along it; when the vehicle falls behind, the scalar drops and the
    // reference waits for it instead of running away.
    float dt_scalar = 1.0f;
    const Vector3f target_vel = pos_control.get_vel_desired_NEU_ms();
    const float target_speed = target_vel.length();
    if (is_positive(target_speed)) {
        const Vector3f dir = target_vel / target_speed;
        const float track_error_m = pos_control.get_pos_error_NEU_m().dot(dir);
        const float track_vel_ms = pos_control.get_vel_estimate_NEU_ms().dot(dir);
        dt_scalar = constrain_float(0.05f +
                                    (track_vel_ms - pos_control.get_pos_NE_p().kP() * track_error_m) / target_speed,
                                    0.0f, 1.0f);
    }
    // filter the scalar so the advance rate does not chatter; the time constant
    // is the one the position controller would take to change acceleration
    const float accel_arc = sq(_speed_ms) / _radius_m;
    const float jerk = pos_control.get_shaping_jerk_NE_msss();
    const float tc = is_positive(jerk) ? MAX(accel_arc / jerk, 0.05f) : 0.3f;
    _dt_scalar += (dt_scalar - _dt_scalar) * constrain_float(dt / tc, 0.0f, 1.0f);

    // Step along the path by the distance the governor allows, integrating the
    // curvature into a heading and the heading into a position.  Stepping in
    // distance rather than in angle is what lets the curvature vary: on the
    // spirals there is no single centre to sweep an angle about.
    //
    // Position and heading are two independent plans sharing one time axis, and
    // what the governor slows down is that shared axis - not one of the plans -
    // so the nose keeps pointing at the tangent of the point the vehicle is
    // actually at.  Distance stands in for that axis here only because the turn
    // is flown at constant speed, where s = v*tau.  A turn that varied its speed
    // would need the time axis carried explicitly.
    float ds = _speed_ms * dt * _dt_scalar;
    bool last_step = false;
    if (_s_m + ds >= _total_len_m) {
        ds = _total_len_m - _s_m;
        last_step = true;
    }
    if (is_positive(ds)) {
        const float k_mid = curvature_at_m(_s_m + ds * 0.5f);
        const float heading_mid = _heading_rad + _direction * k_mid * ds * 0.5f;
        _pos_ne_m += Vector2f{cosf(heading_mid), sinf(heading_mid)} * ds;
        _s_m += ds;
        _heading_rad = heading_at_m(_s_m);
    }

    // Velocity is the tangent at constant magnitude, which is the property
    // SCurve blending cannot provide.  Acceleration is purely centripetal
    // because the speed never changes, and its magnitude v^2*k now ramps with
    // the curvature instead of stepping at the ends of the turn.
    const float k = curvature_at_m(_s_m);
    const float ch = cosf(_heading_rad);
    const float sh = sinf(_heading_rad);
    const Vector2f vel_ne = Vector2f{ch, sh} * _speed_ms;
    const Vector2f accel_ne = Vector2f{-sh, ch} * (_direction * sq(_speed_ms) * k);

    const Vector3p pos_neu{_pos_ne_m.x, _pos_ne_m.y, _alt_u_m};
    const Vector3f vel_neu{vel_ne.x, vel_ne.y, 0.0f};
    const Vector3f accel_neu{accel_ne.x, accel_ne.y, 0.0f};
    pos_control.set_pos_vel_accel_NEU_m(pos_neu, vel_neu, accel_neu);

    if (last_step) {
        _active = false;
        return false;
    }
    return true;
}
