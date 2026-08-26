#include "AC_ArcNav.h"

#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

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

bool AC_ArcNav::set_arc(const AC_PosControl& pos_control,
                        const Vector2f& centre_ne_m, float radius_m,
                        const Vector2f& start_ne_m, float sweep_rad,
                        float speed_ms, float alt_u_m)
{
    _active = false;

    if (!is_positive(radius_m) || !is_positive(speed_ms) || is_zero(sweep_rad)) {
        return false;
    }

    // Refuse an arc the vehicle cannot hold speed on.  Accepting it would just
    // move the failure into flight, where it shows up as the speed loss this
    // generator exists to remove.
    const float accel_needed = sq(speed_ms) / radius_m;
    _required_lean_rad = atanf(accel_needed / GRAVITY_MSS);
    if (_required_lean_rad > tilt_budget_rad(pos_control)) {
        return false;
    }

    const Vector2f offset = start_ne_m - centre_ne_m;
    if (offset.length() < 0.1f) {
        // start point sits on the centre, so it defines no bearing
        return false;
    }

    _centre_ne_m = centre_ne_m;
    _radius_m = radius_m;
    _start_angle_rad = atan2f(offset.y, offset.x);
    _direction = is_positive(sweep_rad) ? 1.0f : -1.0f;
    _sweep_rad = fabsf(sweep_rad);
    _speed_ms = speed_ms;
    _alt_u_m = alt_u_m;
    _travelled_rad = 0.0f;
    _active = true;
    return true;
}

Vector2f AC_ArcNav::position_at_rad(float angle_rad) const
{
    return _centre_ne_m + Vector2f{cosf(angle_rad), sinf(angle_rad)} * _radius_m;
}

Vector2f AC_ArcNav::exit_position_ne_m() const
{
    return position_at_rad(_start_angle_rad + _direction * _sweep_rad);
}

Vector2f AC_ArcNav::exit_velocity_ne_ms() const
{
    const float a = _start_angle_rad + _direction * _sweep_rad;
    // tangent, taken in the direction of travel
    return Vector2f{-sinf(a), cosf(a)} * (_speed_ms * _direction);
}

bool AC_ArcNav::update(AC_PosControl& pos_control, float dt)
{
    if (!_active || !is_positive(dt)) {
        return false;
    }

    // Angular rate follows from holding the tangential speed: omega = v / r.
    const float omega = _speed_ms / _radius_m;
    _travelled_rad += omega * dt;

    bool last_step = false;
    if (_travelled_rad >= _sweep_rad) {
        _travelled_rad = _sweep_rad;
        last_step = true;
    }

    const float angle = _start_angle_rad + _direction * _travelled_rad;
    const float c = cosf(angle);
    const float s = sinf(angle);

    // Position on the circle, velocity along the tangent, acceleration toward
    // the centre.  The velocity magnitude is _speed_ms at every point, which is
    // the property SCurve blending cannot provide.
    const Vector2f pos_ne = _centre_ne_m + Vector2f{c, s} * _radius_m;
    const Vector2f vel_ne = Vector2f{-s, c} * (_speed_ms * _direction);
    const Vector2f accel_ne = Vector2f{-c, -s} * (sq(_speed_ms) / _radius_m);

    const Vector3p pos_neu{pos_ne.x, pos_ne.y, _alt_u_m};
    const Vector3f vel_neu{vel_ne.x, vel_ne.y, 0.0f};
    const Vector3f accel_neu{accel_ne.x, accel_ne.y, 0.0f};
    pos_control.set_pos_vel_accel_NEU_m(pos_neu, vel_neu, accel_neu);

    if (last_step) {
        _active = false;
        return false;
    }
    return true;
}
