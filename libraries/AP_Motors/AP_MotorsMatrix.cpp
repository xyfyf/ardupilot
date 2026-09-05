/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_MotorsMatrix.h"
#include <GCS_MAVLink/GCS.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Math/AP_Math.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsMatrix::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // record requested frame class and type
    _active_frame_class = frame_class;
    _active_frame_type = frame_type;

    if (frame_class == MOTOR_FRAME_SCRIPTING_MATRIX) {
        // if Scripting frame class, do nothing scripting must call its own dedicated init function
        return;
    }

    // setup the motors
    setup_motors(frame_class, frame_type);

    // enable fast channels or instant pwm
    set_update_rate(_speed_hz);
}

#if AP_SCRIPTING_ENABLED
// dedicated init for lua scripting
bool AP_MotorsMatrix::init(uint8_t expected_num_motors)
{
    if (_active_frame_class != MOTOR_FRAME_SCRIPTING_MATRIX) {
        // not the correct class
        return false;
    }

    // Make sure the correct number of motors have been added
    uint8_t num_motors = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            num_motors++;
        }
    }

    set_initialised_ok(expected_num_motors == num_motors);

    if (!initialised_ok()) {
        _mav_type = MAV_TYPE_GENERIC;
        return false;
    }

    switch (num_motors) {
        case 3:
            _mav_type = MAV_TYPE_TRICOPTER;
            break;
        case 4:
            _mav_type = MAV_TYPE_QUADROTOR;
            break;
        case 6:
            _mav_type = MAV_TYPE_HEXAROTOR;
            break;
        case 8:
            _mav_type = MAV_TYPE_OCTOROTOR;
            break;
        case 10:
            _mav_type = MAV_TYPE_DECAROTOR;
            break;
        case 12:
            _mav_type = MAV_TYPE_DODECAROTOR;
            break;
        default:
            _mav_type = MAV_TYPE_GENERIC;
    }

    normalise_rpy_factors();

    set_update_rate(_speed_hz);

    return true;
}

// Set throttle factor from scripting
bool AP_MotorsMatrix::set_throttle_factor(int8_t motor_num, float throttle_factor)
{
    if ((_active_frame_class != MOTOR_FRAME_SCRIPTING_MATRIX) ) {
        // not the correct class
        return false;
    }

    if (initialised_ok() || !motor_enabled[motor_num]) {
        // Already setup or given motor is not enabled
        return false;
    }

    _throttle_factor[motor_num] = throttle_factor;
    return true;
}

#endif // AP_SCRIPTING_ENABLED

// set update rate to motors - a value in hertz
void AP_MotorsMatrix::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    uint32_t mask = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            mask |= 1U << i;
        }
    }
    rc_set_freq(mask, _speed_hz);
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsMatrix::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // exit immediately if armed or no change
    if (armed() || (frame_class == _active_frame_class && _active_frame_type == frame_type)) {
        return;
    }
    _active_frame_class = frame_class;
    _active_frame_type = frame_type;

    init(frame_class, frame_type);

}

void AP_MotorsMatrix::output_to_motors()
{
    int8_t i;

    switch (_spool_state) {
        case SpoolState::SHUT_DOWN: {
            // no output
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled_mask(i)) {
                    _actuator[i] = 0.0f;
                }
            }
            break;
        }
        case SpoolState::GROUND_IDLE:
            // sends output to motors when armed but not flying
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], actuator_spin_up_to_ground_idle());
                }
            }
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // set motor output based on thrust requests
            for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    set_actuator_with_slew(_actuator[i], thr_lin.thrust_to_actuator(_thrust_rpyt_out[i]));
                }
            }
            break;
    }

    // convert output to PWM and send to each motor
    for (i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            rc_write(i, output_to_pwm(_actuator[i]));
        }
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsMatrix::get_motor_mask()
{
    uint32_t motor_mask = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            motor_mask |= 1U << i;
        }
    }
    uint32_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// helper to return value scaled between boost and normal based on the value of _thrust_boost_ratio
// _thrust_boost_ratio of 1 -> return = boost_value
// _thrust_boost_ratio of 0 -> return = normal_value
float AP_MotorsMatrix::boost_ratio(float boost_value, float normal_value) const
{
    return _thrust_boost_ratio * boost_value + (1.0 - _thrust_boost_ratio) * normal_value;
}

// output_armed - sends commands to the motors
// includes new scaling stability patch
void AP_MotorsMatrix::output_armed_stabilizing()
{
    update_failure_detection();

    // Act on a motor the detector (or a ground test) has flagged as failed.
    // Done here rather than at the parameter write so it takes effect on the
    // very next mixer pass, and only once - the degradation is not reversible.
    const int8_t fail_idx = _fail_motor_idx.get();
    if (fail_idx > 0 && _failed_motor < 0) {
        if (set_motor_failed(uint8_t(fail_idx - 1))) {
            gcs().send_text(MAV_SEVERITY_CRITICAL,
                            "Motor %d failed: mixer degraded", int(fail_idx));
        }
    }

    // apply voltage and air pressure compensation
    const float compensation_gain = thr_lin.get_compensation_gain(); // compensation for battery voltage and altitude

    // pitch thrust input value, +/- 1.0
    const float roll_thrust = (_roll_in + _roll_in_ff) * compensation_gain;

    // pitch thrust input value, +/- 1.0
    const float pitch_thrust = (_pitch_in + _pitch_in_ff) * compensation_gain;

    // yaw thrust input value, +/- 1.0
    float yaw_thrust = (_yaw_in + _yaw_in_ff) * compensation_gain;

    // throttle thrust input value, 0.0 - 1.0
    float throttle_thrust = get_throttle() * compensation_gain;

    // throttle thrust average maximum value, 0.0 - 1.0
    float throttle_avg_max = _throttle_avg_max * compensation_gain;

    // throttle thrust maximum value, 0.0 - 1.0, If thrust boost is active then do not limit maximum thrust
    const float throttle_thrust_max = boost_ratio(1.0, _throttle_thrust_max * compensation_gain);

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= throttle_thrust_max) {
        throttle_thrust = throttle_thrust_max;
        limit.throttle_upper = true;
    }

    // ensure that throttle_avg_max is between the input throttle and the maximum throttle
    throttle_avg_max = constrain_float(throttle_avg_max, throttle_thrust, throttle_thrust_max);

    // throttle providing maximum roll, pitch and yaw range
    // calculate the highest allowed average thrust that will provide maximum control range
    float throttle_thrust_best_rpy = MIN(0.5f, throttle_avg_max);

    // calculate throttle that gives most possible room for yaw which is the lower of:
    //      1. 0.5f - (rpy_low+rpy_high)/2.0 - this would give the maximum possible margin above the highest motor and below the lowest
    //      2. the higher of:
    //            a) the pilot's throttle input
    //            b) the point _throttle_rpy_mix between the pilot's input throttle and hover-throttle
    //      Situation #2 ensure we never increase the throttle above hover throttle unless the pilot has commanded this.
    //      Situation #2b allows us to raise the throttle above what the pilot commanded but not so far that it would actually cause the copter to rise.
    //      We will choose #1 (the best throttle for yaw control) if that means reducing throttle to the motors (i.e. we favor reducing throttle *because* it provides better yaw control)
    //      We will choose #2 (a mix of pilot and hover throttle) only when the throttle is quite low.  We favor reducing throttle instead of better yaw control because the pilot has commanded it

    // Under the motor lost condition we remove the highest motor output from our calculations and let that motor go greater than 1.0
    // To ensure control and maximum righting performance Hex and Octo have some optimal settings that should be used
    // Y6               : MOT_YAW_HEADROOM = 350, ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.375, ATC_RAT_PIT_IMAX = 0.375, ATC_RAT_YAW_IMAX = 0.375
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.75,  ATC_RAT_PIT_IMAX = 0.75,  ATC_RAT_YAW_IMAX = 0.375
    // Usable minimums below may result in attitude offsets when motors are lost. Hex aircraft are only marginal and must be handles with care
    // Hex              : MOT_YAW_HEADROOM = 0,   ATC_RAT_RLL_IMAX = 1.0,   ATC_RAT_PIT_IMAX = 1.0,   ATC_RAT_YAW_IMAX = 0.5
    // Octo-Quad (x8) x : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.25,  ATC_RAT_PIT_IMAX = 0.25,  ATC_RAT_YAW_IMAX = 0.25
    // Octo-Quad (x8) + : MOT_YAW_HEADROOM = 300, ATC_RAT_RLL_IMAX = 0.5,   ATC_RAT_PIT_IMAX = 0.5,   ATC_RAT_YAW_IMAX = 0.25
    // Quads cannot make use of motor loss handling because it doesn't have enough degrees of freedom.

    // calculate amount of yaw we can fit into the throttle range
    // this is always equal to or less than the requested yaw from the pilot or rate controller
    float yaw_allowed = 1.0f; // amount of yaw we can fit in
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            // calculate the thrust outputs for roll and pitch
            _thrust_rpyt_out[i] = roll_thrust * _roll_factor[i] + pitch_thrust * _pitch_factor[i];

            // Check the maximum yaw control that can be used on this channel
            // Exclude any lost motors if thrust boost is enabled
            if (!is_zero(_yaw_factor[i]) && (!_thrust_boost || i != _motor_lost_index)) {
                const float thrust_rp_best_throttle = throttle_thrust_best_rpy + _thrust_rpyt_out[i];
                float motor_room;
                if (is_positive(yaw_thrust * _yaw_factor[i])) {
                    // room to upper limit
                    motor_room = 1.0 - thrust_rp_best_throttle;
                } else {
                    // room to lower limit
                    motor_room = thrust_rp_best_throttle;
                }
                const float motor_yaw_allowed = MAX(motor_room, 0.0)/fabsf(_yaw_factor[i]);
                yaw_allowed = MIN(yaw_allowed, motor_yaw_allowed);
            }
        }
    }

    // calculate the maximum yaw control that can be used
    // todo: make _yaw_headroom 0 to 1
    float yaw_allowed_min = (float)_yaw_headroom * 0.001f;

    // increase yaw headroom to 50% if thrust boost enabled
    yaw_allowed_min = boost_ratio(0.5, yaw_allowed_min);

    // Let yaw access minimum amount of head room
    yaw_allowed = MAX(yaw_allowed, yaw_allowed_min);

    // Include the lost motor scaled by _thrust_boost_ratio to smoothly transition this motor in and out of the calculation
    if (_thrust_boost && motor_enabled[_motor_lost_index]) {
        // Check the maximum yaw control that can be used on this channel
        // Exclude any lost motors if thrust boost is enabled
        if (!is_zero(_yaw_factor[_motor_lost_index])){
            const float thrust_rp_best_throttle = throttle_thrust_best_rpy + _thrust_rpyt_out[_motor_lost_index];
            float motor_room;
            if (is_positive(yaw_thrust * _yaw_factor[_motor_lost_index])) {
                motor_room = 1.0 - thrust_rp_best_throttle;
            } else {
                motor_room = thrust_rp_best_throttle;
            }
            const float motor_yaw_allowed = MAX(motor_room, 0.0)/fabsf(_yaw_factor[_motor_lost_index]);
            yaw_allowed = boost_ratio(yaw_allowed, MIN(yaw_allowed, motor_yaw_allowed));
        }
    }

    if (fabsf(yaw_thrust) > yaw_allowed) {
        // not all commanded yaw can be used
        yaw_thrust = constrain_float(yaw_thrust, -yaw_allowed, yaw_allowed);
        limit.yaw = true;
    }

    // add yaw control to thrust outputs
    float rpy_low = 1.0f;   // lowest thrust value
    float rpy_high = -1.0f; // highest thrust value
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out[i] = _thrust_rpyt_out[i] + yaw_thrust * _yaw_factor[i];

            // record lowest roll + pitch + yaw command
            if (_thrust_rpyt_out[i] < rpy_low) {
                rpy_low = _thrust_rpyt_out[i];
            }
            // record highest roll + pitch + yaw command
            // Exclude any lost motors if thrust boost is enabled
            if (_thrust_rpyt_out[i] > rpy_high && (!_thrust_boost || i != _motor_lost_index)) {
                rpy_high = _thrust_rpyt_out[i];
            }
        }
    }
    // Include the lost motor scaled by _thrust_boost_ratio to smoothly transition this motor in and out of the calculation
    if (_thrust_boost) {
        // record highest roll + pitch + yaw command
        if (_thrust_rpyt_out[_motor_lost_index] > rpy_high && motor_enabled[_motor_lost_index]) {
            rpy_high = boost_ratio(rpy_high, _thrust_rpyt_out[_motor_lost_index]);
        }
    }

    // calculate any scaling needed to make the combined thrust outputs fit within the output range
    float rpy_scale = 1.0f;
    if (rpy_high - rpy_low > 1.0f) {
        rpy_scale = 1.0f / (rpy_high - rpy_low);
    }
    if (throttle_avg_max + rpy_low < 0) {
        rpy_scale = MIN(rpy_scale, -throttle_avg_max / rpy_low);
    }

    // calculate how close the motors can come to the desired throttle
    rpy_high *= rpy_scale;
    rpy_low *= rpy_scale;
    throttle_thrust_best_rpy = -rpy_low;
    float thr_adj = throttle_thrust - throttle_thrust_best_rpy;
    if (rpy_scale < 1.0f) {
        // Full range is being used by roll, pitch, and yaw.
        limit.set_rpy(true);
        if (thr_adj > 0.0f) {
            limit.throttle_upper = true;
        }
        thr_adj = 0.0f;
    } else if (thr_adj < 0.0f) {
        // Throttle can't be reduced to desired value
        // todo: add lower limit flag and ensure it is handled correctly in altitude controller
        thr_adj = 0.0f;
    } else if (thr_adj > 1.0f - (throttle_thrust_best_rpy + rpy_high)) {
        // Throttle can't be increased to desired value
        thr_adj = 1.0f - (throttle_thrust_best_rpy + rpy_high);
        limit.throttle_upper = true;
    }

    // add scaled roll, pitch, constrained yaw and throttle for each motor
    const float throttle_thrust_best_plus_adj = throttle_thrust_best_rpy + thr_adj;

    // With a motor removed, redistribute instead of mixing forward.
    //
    // The forward path above builds a linear combination and then rescales or
    // clips whatever falls outside the thrust limits.  That is fine while there
    // is margin on both sides, but a hexacopter that has lost a rotor trims
    // with one survivor sitting on the lower limit, so the clipping is the
    // normal case rather than the exception - and clipping silently breaks the
    // moment balance the combination was built to produce.  Solving with the
    // limits as explicit constraints keeps the balance the solver can still
    // reach, and gives up only what it cannot.
    //
    // Yaw is left out of the demand on purpose; see allocate_redistributed().
    bool redistributed = false;
    _alloc_active = false;
    if (_failed_motor >= 0 && _fail_alloc_mode > 0) {
        const float demand[4] = { throttle_thrust_best_plus_adj, roll_thrust, pitch_thrust, yaw_thrust };
        float alloc[AP_MOTORS_MAX_NUM_MOTORS];
        if (allocate_redistributed(demand, false, alloc)) {
            for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                if (motor_enabled[i]) {
                    _thrust_rpyt_out[i] = alloc[i];
                }
            }
            redistributed = true;
            set_limits_from_allocation(demand, false, alloc);
        }
    }

    if (!redistributed) {
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (motor_enabled[i]) {
                _thrust_rpyt_out[i] = (throttle_thrust_best_plus_adj * _throttle_factor[i]) + (rpy_scale * _thrust_rpyt_out[i]);
            }
        }
    }

    // determine throttle thrust for harmonic notch
    // compensation_gain can never be zero
    _throttle_out = throttle_thrust_best_plus_adj / compensation_gain;

    // check for failed motor
    check_for_failed_motor(throttle_thrust_best_plus_adj);
}

// check for failed motor
//   should be run immediately after output_armed_stabilizing
//   first argument is the sum of:
//      a) throttle_thrust_best_rpy : throttle level (from 0 to 1) providing maximum roll, pitch and yaw range without climbing
//      b) thr_adj: the difference between the pilot's desired throttle and throttle_thrust_best_rpy
//   records filtered motor output values in _thrust_rpyt_out_filt array
//   sets thrust_balanced to true if motors are balanced, false if a motor failure is detected
//   sets _motor_lost_index to index of failed motor
void AP_MotorsMatrix::check_for_failed_motor(float throttle_thrust_best_plus_adj)
{
    // record filtered and scaled thrust output for motor loss monitoring purposes
    float alpha = _dt_s / (_dt_s + 0.5f);
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            _thrust_rpyt_out_filt[i] += alpha * (_thrust_rpyt_out[i] - _thrust_rpyt_out_filt[i]);
        }
    }

    float rpyt_high = 0.0f;
    float rpyt_sum = 0.0f;
    uint8_t number_motors = 0.0f;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            number_motors += 1;
            rpyt_sum += _thrust_rpyt_out_filt[i];
            // record highest filtered thrust command
            if (_thrust_rpyt_out_filt[i] > rpyt_high) {
                rpyt_high = _thrust_rpyt_out_filt[i];
                // hold motor lost index constant while thrust boost is active
                if (!_thrust_boost) {
                    _motor_lost_index = i;
                }
            }
        }
    }

    float thrust_balance = 1.0f;
    if (rpyt_sum > 0.1f) {
        thrust_balance = rpyt_high * number_motors / rpyt_sum;
    }
    // ensure thrust balance does not activate for multirotors with less than 6 motors
    if (number_motors >= 6 && thrust_balance >= 1.5f && _thrust_balanced) {
        _thrust_balanced = false;
    }
    if (thrust_balance <= 1.25f && !_thrust_balanced) {
        _thrust_balanced = true;
    }

    // check to see if thrust boost is using more throttle than _throttle_thrust_max
    if ((_throttle_thrust_max * thr_lin.get_compensation_gain() > throttle_thrust_best_plus_adj) && (rpyt_high < 0.9f) && _thrust_balanced) {
        _thrust_boost = false;
    }
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsMatrix::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // loop through all the possible orders spinning any motors that match that description
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i] && _test_order[i] == motor_seq) {
            // turn on this motor
            rc_write(i, pwm);
        }
    }
}

// output_test_num - spin a motor connected to the specified output channel
//  (should only be performed during testing)
//  If a motor output channel is remapped, the mapped channel is used.
//  Returns true if motor output is set, false otherwise
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
bool AP_MotorsMatrix::output_test_num(uint8_t output_channel, int16_t pwm)
{
    if (!armed()) {
        return false;
    }

    // Is channel in supported range?
    if (output_channel > AP_MOTORS_MAX_NUM_MOTORS - 1) {
        return false;
    }

    // Is motor enabled?
    if (!motor_enabled[output_channel]) {
        return false;
    }

    rc_write(output_channel, pwm); // output
    return true;
}

// add_motor
void AP_MotorsMatrix::add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order, float throttle_factor)
{
    if (initialised_ok()) {
        // do not allow motors to be set if the current frame type has init correctly
        return;
    }

    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {

        // enable motor
        motor_enabled[motor_num] = true;

        // set roll, pitch, yaw and throttle factors
        _roll_factor[motor_num] = roll_fac;
        _pitch_factor[motor_num] = pitch_fac;
        _yaw_factor[motor_num] = yaw_fac;
        _throttle_factor[motor_num] = throttle_factor;

        // set order that motor appears in test
        _test_order[motor_num] = testing_order;

        // call parent class method
        add_motor_num(motor_num);
    }
}

// add_motor using just position and prop direction - assumes that for each motor, roll and pitch factors are equal
void AP_MotorsMatrix::add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor(motor_num, angle_degrees, angle_degrees, yaw_factor, testing_order);
}

// add_motor using position and prop direction. Roll and Pitch factors can differ (for asymmetrical frames)
void AP_MotorsMatrix::add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order)
{
    add_motor_raw(
        motor_num,
        cosf(radians(roll_factor_in_degrees + 90)),
        cosf(radians(pitch_factor_in_degrees)),
        yaw_factor,
        testing_order);
}

// remove_motor - disabled motor and clears all roll, pitch, throttle factors for this motor
void AP_MotorsMatrix::remove_motor(int8_t motor_num)
{
    // ensure valid motor number is provided
    if (motor_num >= 0 && motor_num < AP_MOTORS_MAX_NUM_MOTORS) {
        // disable the motor, set all factors to zero
        motor_enabled[motor_num] = false;
        _roll_factor[motor_num] = 0.0f;
        _pitch_factor[motor_num] = 0.0f;
        _yaw_factor[motor_num] = 0.0f;
        _throttle_factor[motor_num] = 0.0f;
    }
}

// Below this command the square root turns telemetry noise into a huge
// swing in k, so the reading is not a usable reference however low
// MOT_FAIL_THST has been set.
#define AP_MOTORS_SHED_MIN_THRUST 0.05f

// Time constant for learning each motor's own k, expressed as a share of the
// fleet.  Long compared with the confirm window, so a thrown prop shows up as
// a step the detector catches long before the reference has moved to meet it.
#define AP_MOTORS_SHED_LEARN_TAU_S 10.0f

// How much flight a motor's reference needs before it is worth judging
// against.  Below this the reference is still the first few samples.
#define AP_MOTORS_SHED_LEARN_MIN_S 5.0f

void AP_MotorsMatrix::update_failure_detection()
{
#if HAL_WITH_ESC_TELEM
    const bool stop_check = is_positive(_fail_rpm_min);
    const bool shed_check = is_positive(_fail_shed_ratio);

    if (!armed()) {
        // Relearn from scratch each flight.  Carrying a reference across a
        // disarm would judge this flight's motors against the last one's -
        // and a stale low reference is not self-correcting, because the
        // suspicion it creates freezes the very learning that would fix it.
        //
        // Clearing the learn time is enough to reset the whole state: it makes
        // `ready` false, which sends the next pass down the else branch, and
        // that branch re-seeds _shed_ref from the first fresh reading and zeros
        // both timers.  Writing the other three arrays here as well cost 1.1 kB
        // of flash on a budget with 4 kB in it.
        memset(_shed_learn_s, 0, sizeof(_shed_learn_s));
        return;
    }

    // One degradation per flight: the change is irreversible, and a second
    // pass could only remove a motor the vehicle still needs.
    if (_failed_motor >= 0 || (!stop_check && !shed_check)) {
        return;
    }

    const float dt = get_dt_s();
    const float confirm_s = MAX(_fail_time_ms.get(), 0) * 0.001f;
    AP_ESC_Telem &telem = AP::esc_telem();

    uint8_t suspect_count = 0;
    int8_t suspect = -1;
    bool suspect_shed = false;

    // Per-motor load factor k = rpm / sqrt(thrust), gathered in the same pass
    // as the stopped check.  Thrust goes as rpm squared, so every healthy
    // motor swinging the same prop in the same air shares one k whatever the
    // airframe's rpm constant happens to be.  Judging each motor against the
    // fleet median of k therefore needs no calibration, and rides out battery
    // sag and air density, which move all of them together.
    float k[AP_MOTORS_MAX_NUM_MOTORS];
    bool k_valid[AP_MOTORS_MAX_NUM_MOTORS] {};

    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (!motor_enabled[i]) {
            continue;
        }
        float rpm;
        // A motor coasting at low command is expected to turn slowly, so only
        // judge one that is actually being asked for thrust.  Without this the
        // check would fire on every descent and on every disarm.
        if (_thrust_rpyt_out[i] < _fail_thrust_min ||
            !telem.get_rpm(i, rpm)) {
            _fail_timer_s[i] = 0.0f;
            _shed_timer_s[i] = 0.0f;
            continue;
        }
        if (_thrust_rpyt_out[i] >= AP_MOTORS_SHED_MIN_THRUST) {
            k[i] = rpm / sqrtf(_thrust_rpyt_out[i]);
            k_valid[i] = true;
        }

        if (!stop_check) {
            _fail_timer_s[i] = 0.0f;
        } else if (rpm < _fail_rpm_min) {
            _fail_timer_s[i] += dt;
            if (_fail_timer_s[i] >= confirm_s) {
                suspect_count++;
                suspect = i;
            }
        } else {
            _fail_timer_s[i] = 0.0f;
        }
    }

    // A thrown propeller leaves the motor spinning faster, not slower, so the
    // check above cannot see it - it is looking for the opposite sign.  The
    // signature is a motor whose k has climbed away from where that motor
    // normally sits: same command, no load, higher rpm.  What the mixer has to
    // do about it is the same either way, because either way that point makes
    // no thrust, so this ends at the same set_motor_failed().
    //
    // Judged against the motor's own history, not against the fleet.  Every
    // motor sits at its own k even on a healthy airframe - measured in SITL,
    // where there is no manufacturing spread at all, the six spread over
    // 1.33:1, because geometry alone puts each motor at a different operating
    // point.  A fixed ratio to the fleet median therefore fires on whichever
    // motor naturally rides high while staying blind to the one that rides
    // low; on a real airframe, with build tolerance on top, it is worse.  What
    // is learned is each motor's k as a share of the fleet's, so the reference
    // still rides out battery sag and air density - those move every motor
    // together and cancel in the ratio - while the per-motor offset that broke
    // the fleet comparison is exactly what the reference absorbs.
    //
    // Only when the stopped check found nothing: it has already decided, and
    // one degradation per flight means there is nothing left to add.
    if (shed_check && suspect_count == 0) {
        float sorted[AP_MOTORS_MAX_NUM_MOTORS];
        uint8_t n = 0;
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            if (!k_valid[i]) {
                continue;
            }
            uint8_t m = n++;
            while (m > 0 && sorted[m - 1] > k[i]) {
                sorted[m] = sorted[m - 1];
                m--;
            }
            sorted[m] = k[i];
        }
        // Under four readings the median is not a fleet consensus - with three
        // it sits on one motor, so a single bad reading becomes the reference
        // every other motor is judged against.  Hold off rather than guess.
        if (n >= 4) {
            const float median = (n & 1) ? sorted[n / 2]
                                         : 0.5f * (sorted[n / 2 - 1] + sorted[n / 2]);
            if (is_positive(median)) {
                for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
                    if (!k_valid[i]) {
                        continue;
                    }
                    const float share = k[i] / median;
                    const bool ready = _shed_learn_s[i] >= AP_MOTORS_SHED_LEARN_MIN_S;
                    if (ready && share > _shed_ref[i] * _fail_shed_ratio) {
                        // Suspect: hold the reference still.  Letting it keep
                        // learning here would walk it up to meet the fault and
                        // clear the very condition being confirmed.
                        _shed_timer_s[i] += dt;
                        if (_shed_timer_s[i] >= confirm_s) {
                            suspect_count++;
                            suspect = i;
                            suspect_shed = true;
                        }
                    } else {
                        _shed_timer_s[i] = 0.0f;
                        if (_shed_learn_s[i] <= 0.0f) {
                            // First reading: start at it rather than ramping
                            // up from zero, which would read as a motor
                            // climbing and be indistinguishable from the fault.
                            _shed_ref[i] = share;
                        } else {
                            _shed_ref[i] += (share - _shed_ref[i]) *
                                            (dt / AP_MOTORS_SHED_LEARN_TAU_S);
                        }
                        _shed_learn_s[i] += dt;
                    }
                }
            }
        }
    }

    if (suspect_count == 1) {
        if (set_motor_failed(uint8_t(suspect))) {
            if (suspect_shed) {
                gcs().send_text(MAV_SEVERITY_CRITICAL,
                                "Motor %d lost its prop: mixer degraded", int(suspect) + 1);
            } else {
                gcs().send_text(MAV_SEVERITY_CRITICAL,
                                "Motor %d stopped: mixer degraded", int(suspect) + 1);
            }
        }
    } else if (suspect_count > 1) {
        // Several motors failing at once is far more likely to be the
        // telemetry link or a channel mapping error than a simultaneous
        // multiple failure - and removing motors on that basis would cause the
        // crash it is meant to prevent.  Warn, do not act.
        //
        // Warning without acting means _failed_motor is never set, so the early
        // return at the top of this function never engages and this branch is
        // reached on every mixer pass - 400 Hz on this board - for as long as
        // the condition lasts.  Send on the rising edge, then no more often
        // than once a second, or the message floods the GCS queue and buries
        // whatever else the vehicle is trying to say.
        const uint32_t now_ms = AP_HAL::millis();
        if (_fail_warn_count == 0 || now_ms - _fail_warn_last_ms >= 1000U) {
            _fail_warn_last_ms = now_ms;
            if (suspect_shed) {
                gcs().send_text(MAV_SEVERITY_WARNING,
                                "Motor: %u read unloaded, check ESC telem", suspect_count);
            } else {
                gcs().send_text(MAV_SEVERITY_WARNING,
                                "Motor: %u read stopped, check ESC telem", suspect_count);
            }
        }
        _fail_warn_count = suspect_count;
    } else if (_fail_warn_count != 0) {
        // Cleared.  Say so - a warning that simply stops leaves the operator
        // unable to tell recovery from a lost link.
        _fail_warn_count = 0;
        gcs().send_text(MAV_SEVERITY_INFO, "Motor: ESC telem readings recovered");
    }
#endif // HAL_WITH_ESC_TELEM
}

// Relative residual (squared) above which a solve is treated as too
// ill-conditioned to use.  1e-4 = 1% relative error.
#define AP_MOTORS_ALLOC_RESIDUAL_REL_SQ 1.0e-4f

bool AP_MotorsMatrix::allocate_redistributed(const float demand[4], bool include_yaw,
                                             float thrust_out[AP_MOTORS_MAX_NUM_MOTORS]) const
{
    const uint8_t rows = include_yaw ? 4 : 3;
    // Effector matrix rows are [throttle, roll, pitch, yaw]; one column per
    // enabled motor.
    uint8_t idx[AP_MOTORS_MAX_NUM_MOTORS];
    uint8_t n = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        thrust_out[i] = 0.0f;
        if (motor_enabled[i]) {
            idx[n++] = i;
        }
    }
    _alloc_result = AllocResult::NOT_RUN;
    _alloc_passes = 0;
    _alloc_clamp_mask = 0;
    if (n < rows) {
        // Fewer effectors than demands: nothing to redistribute between.
        _alloc_result = AllocResult::TOO_FEW_MOTORS;
        return false;
    }

    bool freed[AP_MOTORS_MAX_NUM_MOTORS];
    float thrust[AP_MOTORS_MAX_NUM_MOTORS];
    for (uint8_t k = 0; k < n; k++) {
        freed[k] = true;
        thrust[k] = 0.0f;
    }
    // Whether any pass has produced and verified a complete assignment.  Every
    // exit below is a `break`, and thrust[] starts at zero, so without this the
    // failure of the very first solve is indistinguishable from success and
    // hands the caller six zeroed motors as a valid answer.
    bool have_solution = false;

    // At most one clamp per motor, so this terminates.
    for (uint8_t pass = 0; pass <= n; pass++) {
        _alloc_passes = pass + 1;
        // Demand still to be met by the motors that are free, after removing
        // what the already-clamped ones contribute.
        float rem[4];
        for (uint8_t r = 0; r < rows; r++) {
            rem[r] = demand[r];
        }
        // The throttle demand is per motor, not a total to divide up: forward
        // mixing gives every motor throttle_thrust * _throttle_factor[i].  The
        // rows here form a linear system whose throttle row *sums* the motors,
        // so the demand has to be scaled by how many there are or the solve
        // delivers 1/n of the lift that was asked for.  Getting this wrong is
        // not subtle in the air but is easy to miss on paper: with six motors
        // trimmed at 0.161 and one gone, each survivor needs 0.193, the solver
        // handed out 0.039, and the vehicle sank at 2.5 m/s with the throttle
        // pinned at 1.0 - which still hovers, because 1.0/5 = 0.2 lands just
        // above 0.193, so it read as a controlled hover at a lower altitude
        // rather than as a thrust shortfall.
        rem[0] *= (float)n;
        for (uint8_t k = 0; k < n; k++) {
            if (freed[k]) {
                continue;
            }
            const uint8_t m = idx[k];
            rem[0] -= thrust[k] * _throttle_factor[m];
            rem[1] -= thrust[k] * _roll_factor[m];
            rem[2] -= thrust[k] * _pitch_factor[m];
            if (include_yaw) {
                rem[3] -= thrust[k] * _yaw_factor[m];
            }
        }

        // Minimum-norm solution on the free motors: T = B^T (B B^T)^-1 rem.
        //
        // With yaw out of the demand there are more free motors than rows, so
        // the solution is not unique - a null space is left over, and every
        // point in it delivers the same throttle, roll and pitch. Minimum norm
        // picks one of them without regard to how much yaw moment it leaves
        // behind, and on a hexacopter down one motor that choice is a poor one:
        // it lands on a part of the trade-off that is strictly dominated, with
        // both more residual yaw and less roll authority than a neighbouring
        // solution. MOT_FAIL_YSUP steers the pick by minimising
        //   |T|^2 + ysup * (yaw moment)^2
        // over that same null space, which by Sherman-Morrison is the plain
        // minimum-norm solve with (I - alpha*y*y^T) folded in.
        //
        // Note this suppresses the *parasitic* yaw moment - the target is zero,
        // not the pilot's yaw demand. It is not MOT_FAIL_YAW, which puts yaw
        // back into the demand and asks the controller to chase it.
        const float ysup = (rows == 3) ? constrain_float(_fail_yaw_suppress, 0.0f, 1.0f) : 0.0f;
        uint8_t n_free = 0;
        float syaw[4] = {};     // s_a = sum over free motors of B[a][j] * yaw_j
        float bbt[16] = {};
        for (uint8_t k = 0; k < n; k++) {
            if (!freed[k]) {
                continue;
            }
            const uint8_t m = idx[k];
            const float col[4] = { _throttle_factor[m], _roll_factor[m],
                                   _pitch_factor[m], _yaw_factor[m] };
            n_free++;
            for (uint8_t a = 0; a < rows; a++) {
                syaw[a] += col[a] * _yaw_geom[m];
                for (uint8_t b = 0; b < rows; b++) {
                    bbt[a * rows + b] += col[a] * col[b];
                }
            }
        }
        // Yaw factors are +-1, so y^T y is just the number of free motors.
        const float alpha = (is_positive(ysup) && n_free > 0) ? ysup / (float)n_free : 0.0f;
        if (is_positive(alpha)) {
            for (uint8_t a = 0; a < rows; a++) {
                for (uint8_t b = 0; b < rows; b++) {
                    bbt[a * rows + b] -= alpha * syaw[a] * syaw[b];
                }
            }
        }
        // Rank test.  B B^T is singular exactly when the free columns of B are
        // rank deficient, so this is the real check; counting motors above only
        // rules out the trivially impossible.
        float bbt_inv[16];
        if (!mat_inverse(bbt, bbt_inv, rows)) {
            _alloc_result = AllocResult::SINGULAR;
            break;
        }
        // MOT_FAIL_YTRK moves the target of that same term off zero and onto
        // the yaw demand: the objective becomes
        //   |T|^2 + ysup * (yaw moment - ytrk * yaw demand)^2
        // Working the Lagrangian through, the only thing a non-zero target
        // changes is one scalar.  With the substitution the code already makes
        // (alpha = ysup / n_free absorbing the Sherman-Morrison denominator) it
        // comes out as gamma = alpha * target, entering in two places: the
        // right-hand side of the solve loses gamma*s, and each thrust gains
        // gamma*y.
        //
        // The residual check below still validates against the unmodified rem,
        // and that is not a slip.  Substituting back,
        //   B*t = (B B^T - alpha*s*s^T)*lambda + gamma*s
        //       = (rem - gamma*s) + gamma*s
        //       = rem
        // so the identity the check relies on survives the change untouched.
        //
        // Note what this does NOT do: throttle, roll and pitch stay as hard
        // equality constraints in B, so yaw is served only out of the null
        // space they leave behind.  It cannot trade attitude away for heading -
        // which is exactly what MOT_FAIL_YAW does, and why that one crashed the
        // airframe in 2 m/s of wind while this cannot.  When the null space
        // runs out, motors clamp, the active set shrinks, and the solve falls
        // back toward the plain suppression case on its own.
        const float gamma = (is_positive(alpha) && rows == 3)
                            ? alpha * constrain_float(_fail_yaw_track, 0.0f, 1.0f) * demand[3]
                            : 0.0f;

        float lambda[4] = {};
        for (uint8_t a = 0; a < rows; a++) {
            for (uint8_t b = 0; b < rows; b++) {
                lambda[a] += bbt_inv[a * rows + b] * (rem[b] - gamma * syaw[b]);
            }
        }

        float slam = 0.0f;
        if (is_positive(alpha)) {
            for (uint8_t a = 0; a < rows; a++) {
                slam += syaw[a] * lambda[a];
            }
        }

        // Stage the solution, then check it before committing.
        //
        // inverse3x3()/inverse4x4() reject only an exactly zero determinant
        // (is_zero(), i.e. below FLT_EPSILON), so a merely ill-conditioned
        // matrix is inverted happily and yields entries of order 1/det.  The
        // resulting thrusts are nonsense but finite, every one of them clamps,
        // and the result reads as a valid degraded allocation.
        //
        // The cheap honest test is to substitute back.  t = B^T*lambda -
        // alpha*y*(s^T*lambda) satisfies B*t == rem identically: expanding it
        // gives (B B^T - alpha*s*s^T)*lambda, which is the very system that was
        // solved.  That holds whether or not yaw suppression is active and
        // regardless of what the clamping does afterwards, so any real
        // discrepancy is the conditioning showing through and nothing else.
        float t_raw[AP_MOTORS_MAX_NUM_MOTORS];
        float check[4] = {};
        bool finite = true;
        for (uint8_t k = 0; k < n && finite; k++) {
            if (!freed[k]) {
                continue;
            }
            const uint8_t m = idx[k];
            float t = _throttle_factor[m] * lambda[0] + _roll_factor[m] * lambda[1]
                    + _pitch_factor[m] * lambda[2];
            if (include_yaw) {
                t += _yaw_factor[m] * lambda[3];
            }
            t -= alpha * _yaw_geom[m] * slam;
            t += gamma * _yaw_geom[m];
            if (!isfinite(t)) {
                // constrain_float() turns a NaN into (low+high)/2, i.e. half
                // throttle on every motor - louder than zero and just as wrong.
                finite = false;
                break;
            }
            t_raw[k] = t;
            const float col[4] = { _throttle_factor[m], _roll_factor[m],
                                   _pitch_factor[m], _yaw_factor[m] };
            for (uint8_t r = 0; r < rows; r++) {
                check[r] += col[r] * t;
            }
        }
        if (!finite) {
            _alloc_result = AllocResult::NON_FINITE;
            break;
        }
        float err2 = 0.0f, mag2 = 0.0f;
        for (uint8_t r = 0; r < rows; r++) {
            const float d = check[r] - rem[r];
            err2 += d * d;
            mag2 += rem[r] * rem[r];
        }
        // Deliberately loose - a well-conditioned solve lands many orders of
        // magnitude inside this, while rejecting a good solve would drop the
        // vehicle back to forward mixing, which is the failure this allocator
        // exists to avoid.
        if (err2 > AP_MOTORS_ALLOC_RESIDUAL_REL_SQ * MAX(mag2, 1.0f)) {
            _alloc_result = AllocResult::RESIDUAL;
            break;
        }

        bool clamped_any = false;
        for (uint8_t k = 0; k < n; k++) {
            if (!freed[k]) {
                continue;
            }
            const float t = t_raw[k];
            if (t < 0.0f || t > 1.0f) {
                thrust[k] = constrain_float(t, 0.0f, 1.0f);
                freed[k] = false;
                clamped_any = true;
                _alloc_clamp_mask |= (uint8_t)(1U << (idx[k] & 7));
            } else {
                thrust[k] = t;
            }
        }
        if (!clamped_any) {
            // Only a pass that needed no clamping counts as a solution.
            //
            // This is an active-set iteration: clamped_any means this round's
            // assignment is *not* usable and the next round must redistribute
            // with the clamped motors' contributions removed from the demand.
            // Setting the flag on such a round made a later degeneration commit
            // it anyway - the clamped motors would hold their clamped values
            // while the free ones still held figures computed on the assumption
            // that those same motors were putting out their unclamped, out of
            // range values.  B*t no longer equals the demand and the moment
            // balance is gone, yet the caller is told the allocation succeeded.
            //
            // It is reachable, and not by numerical accident: with yaw out of
            // the demand (rows = 3), once clamping has frozen all but two
            // motors, B is 3x2, so B*B^T is 3x3 of rank at most 2 and its
            // determinant is identically zero - mat_inverse() fails and the
            // loop breaks.  Three successive clamps in a saturated case is all
            // it takes.
            //
            // No snapshot is needed: this branch breaks immediately, so
            // thrust[] at the exit is exactly the assignment being blessed.
            have_solution = true;
            _alloc_result = AllocResult::OK;
            break;
        }
    }

    if (!have_solution) {
        if (_alloc_result == AllocResult::NOT_RUN) {
            // Loop ran out of passes with every round still clamping.
            _alloc_result = AllocResult::NO_SOLUTION;
        }
        // Never verified a solve.  thrust_out[] is still the zeroed array from
        // the top; reporting success on it commanded zero thrust on every motor
        // and dropped the vehicle.  Say so instead and let the caller fall back
        // to forward mixing, which at least flies.
        return false;
    }

    for (uint8_t k = 0; k < n; k++) {
        thrust_out[idx[k]] = constrain_float(thrust[k], 0.0f, 1.0f);
    }
    return true;
}

// Relative shortfall above which an axis counts as saturated.  The solver meets
// the demand exactly whenever nothing clamps - the residual check inside
// allocate_redistributed() guarantees that - so anything beyond arithmetic noise
// came from clamping, i.e. from running out of motor.  1% keeps float noise out
// while still catching the first real millimetre of shortfall.
#define AP_MOTORS_ALLOC_LIMIT_REL     0.01f
#define AP_MOTORS_ALLOC_LIMIT_ABS     1.0e-4f

void AP_MotorsMatrix::set_limits_from_allocation(const float demand[4], bool include_yaw,
                                                 const float thrust[AP_MOTORS_MAX_NUM_MOTORS])
{
    // What the committed thrusts actually produce.  Same B as the solver used:
    // the throttle row sums the motors, the moment rows weight them by their
    // geometry.
    float achieved[4] = {};
    uint8_t n = 0;
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (!motor_enabled[i]) {
            continue;
        }
        n++;
        achieved[0] += thrust[i] * _throttle_factor[i];
        achieved[1] += thrust[i] * _roll_factor[i];
        achieved[2] += thrust[i] * _pitch_factor[i];
        // Yaw uses the *geometric* factors, not _yaw_factor[]: with the default
        // MOT_FAIL_YAW=0 the live factors are scaled to zero at failure, so this
        // sum was identically zero in every degraded flight - the log said the
        // allocation produced no yaw moment while the rotors were producing
        // enough of it to sweep the nose right round. The mixer stops
        // *commanding* yaw; the rotors keep *making* it. Still a normalised
        // allocation estimate, not a measured torque.
        achieved[3] += thrust[i] * (_failed_motor >= 0 ? _yaw_geom[i] : _yaw_factor[i]);
    }

    // The throttle demand is per motor while the throttle row is a sum, the
    // same scaling allocate_redistributed() applies to rem[0].
    const float thr_demand = demand[0] * (float)n;

    // limit.roll/pitch/yaw are consumed by AC_PID::update_i(), which still lets
    // the integrator shrink when the error opposes it and only blocks growth.
    // So the honest signal is simply "this axis did not get what it asked for",
    // with no need to work out a direction here.
    const auto short_of = [](float want, float got) {
        return fabsf(want - got) > MAX(AP_MOTORS_ALLOC_LIMIT_ABS,
                                       AP_MOTORS_ALLOC_LIMIT_REL * fabsf(want));
    };

    // Snapshot for the log before the flags collapse it to booleans.
    _alloc_demand[0] = thr_demand;
    for (uint8_t r = 1; r < 4; r++) {
        _alloc_demand[r] = demand[r];
    }
    for (uint8_t r = 0; r < 4; r++) {
        _alloc_achieved[r] = achieved[r];
    }
    _alloc_active = true;

    limit.roll  = short_of(demand[1], achieved[1]);
    limit.pitch = short_of(demand[2], achieved[2]);

    // Yaw is deliberately absent from the demand today (see
    // allocate_redistributed), so the allocator is not even trying to track it.
    // Leaving limit.yaw as the forward mixer left it invites the yaw PID to keep
    // integrating against a demand nothing is serving, and a wound-up yaw
    // integrator eats the mixer headroom roll and pitch still need.  Report it
    // as limited for as long as yaw is out of the solve.
    //
    // Keyed off include_yaw rather than hardcoded so that if MOT_FAIL_YAW ever
    // does put yaw back into the demand (R-03), this follows automatically
    // instead of silently reporting a tracked axis as saturated.
    limit.yaw = include_yaw ? short_of(demand[3], achieved[3]) : true;

    // Throttle is directional: too little lift is the upper limit biting, too
    // much is the lower one.
    if (thr_demand - achieved[0] > MAX(AP_MOTORS_ALLOC_LIMIT_ABS,
                                       AP_MOTORS_ALLOC_LIMIT_REL * fabsf(thr_demand))) {
        limit.throttle_upper = true;
    } else if (achieved[0] - thr_demand > MAX(AP_MOTORS_ALLOC_LIMIT_ABS,
                                              AP_MOTORS_ALLOC_LIMIT_REL * fabsf(thr_demand))) {
        limit.throttle_lower = true;
    }
}

int8_t AP_MotorsMatrix::find_opposite_motor(uint8_t motor_num) const
{
    if (motor_num >= AP_MOTORS_MAX_NUM_MOTORS || !motor_enabled[motor_num]) {
        return -1;
    }
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (i == motor_num || !motor_enabled[i]) {
            continue;
        }
        if (fabsf(_roll_factor[i] + _roll_factor[motor_num]) < 0.01f &&
            fabsf(_pitch_factor[i] + _pitch_factor[motor_num]) < 0.01f) {
            return int8_t(i);
        }
    }
    return -1;
}

bool AP_MotorsMatrix::set_motor_failed(uint8_t motor_num, bool surrender_yaw)
{
    if (motor_num >= AP_MOTORS_MAX_NUM_MOTORS || !motor_enabled[motor_num]) {
        return false;
    }

    remove_motor(motor_num);

    // Optionally shut down the motor opposite the failed one as well.
    //
    // On a hexacopter this restores a symmetric four-rotor layout: the four
    // survivors are two counter-rotating pairs again, and the hover solution
    // spreads thrust evenly across them instead of driving one to zero, which
    // is what the five-motor solution does.  Even thrust means every motor
    // keeps its full range of adjustment.
    //
    // What it costs is an axis.  Those four rotors have yaw factors equal to
    // -2x their roll factors, so the yaw column is a multiple of the roll
    // column and the allocation drops to rank 3: roll and yaw can no longer be
    // commanded independently.  Rolling produces yaw whether or not it is
    // asked for.
    if (_fail_stop_opposite) {
        const int8_t opposite = find_opposite_motor(motor_num);
        if (opposite >= 0) {
            remove_motor(opposite);
        }
    }

    // Snapshot the geometric yaw factors before surrender_yaw scales them.
    // Zeroing _yaw_factor stops the mixer asking for yaw; it does not stop the
    // remaining rotors from making it.  MOT_FAIL_YSUP needs the real geometry.
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _yaw_geom[i] = motor_enabled[i] ? _yaw_factor[i] : 0.0f;
    }

    if (surrender_yaw) {
        // Scale every yaw factor, not just the failed motor's.
        //
        // Holding yaw *exactly* is degenerate, and worth understanding before
        // reading the default.  Solve the hover constraints for a hexacopter
        // with one rotor gone - total thrust, zero roll, zero pitch, zero yaw -
        // and the roll and yaw rows together force the motor opposite the
        // failed one to exactly zero thrust.  That costs a second motor, and
        // since thrust cannot go negative it leaves that motor unable to trim
        // downward either: four motors for four constraints, nothing spare to
        // make control moments with.
        //
        // The mixer is never asked for an exact solution, though.  It serves
        // throttle, roll and pitch before yaw, so yaw is squeezed on its own,
        // exactly as far as the remaining authority requires - the same
        // priority order PX4 applies in its sequential desaturation, where yaw
        // is likewise the first axis given up.  In still air that is enough to
        // buy most of the heading back: about 2 deg/s of residual rotation
        // instead of 23, for roll error rising from 15 to 20 degrees.
        //
        // It does not survive wind.  The authority spent holding heading is the
        // same authority needed to trim against a crosswind, and one rotor down
        // there is not enough for both: measured on the worst-case motor with
        // 2 m/s of wind, keeping yaw drove roll overshoot to 68 degrees and the
        // vehicle crashed, while giving yaw up held roll to 2.4 degrees and it
        // landed.  At 4 m/s the same split held - 100 degrees against 3.6.
        //
        // So the default gives yaw up.  Attitude is the axis that has to be
        // held; heading is the one that can be spent.  MOT_FAIL_YAW exists for
        // an airframe with margin to spare, and should only be raised with wind
        // in the test.
        const float keep = constrain_float(_fail_yaw_keep, 0.0f, 1.0f);
        for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
            _yaw_factor[i] *= keep;
        }
    }

    // The remaining motors must be rescaled: with one contributor gone the
    // surviving factors no longer span the same range, and without this the
    // effective roll and pitch gains change underneath the attitude controller.
    normalise_rpy_factors();

    // Deliberately *not* touching the rate integrators here.  It is tempting to
    // - the mixing matrix just changed, so the accumulated trim looks stale -
    // but the integrators hold a demand in generalised moments, not in motor
    // commands: how much roll, pitch and yaw torque this airframe needs to stay
    // balanced against gravity, centre-of-mass offset and mounting-angle error.
    // None of that changes when a rotor is removed; only the mapping from
    // torque to motors does, and normalise_rpy_factors() has just fixed that.
    // Relaxing them measurably made things worse: roll overshoot 20 -> 48 deg
    // and the vehicle crashed, because the trim it had already built was thrown
    // away and had to be rebuilt at the worst possible moment.

    _failed_motor = motor_num;
    return true;
}

void AP_MotorsMatrix::add_motors(const struct MotorDef *motors, uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        const auto &motor = motors[i];
        add_motor(i, motor.angle_degrees, motor.yaw_factor, motor.testing_order);
    }
}
void AP_MotorsMatrix::add_motors_raw(const struct MotorDefRaw *motors, uint8_t num_motors)
{
    for (uint8_t i=0; i<num_motors; i++) {
        const auto &m = motors[i];
        add_motor_raw(i, m.roll_fac, m.pitch_fac, m.yaw_fac, m.testing_order);
    }
}
#if AP_MOTORS_FRAME_QUAD_ENABLED
bool AP_MotorsMatrix::setup_quad_matrix(motor_frame_type frame_type)
{
    _frame_class_string = "QUAD";
    _mav_type = MAV_TYPE_QUADROTOR;
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            {   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
#if APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    case MOTOR_FRAME_TYPE_NYT_PLUS: {
        _frame_type_string = "NYT_PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  90, 0,  2 },
            { -90, 0,  4 },
            {   0, 0,  1 },
            { 180, 0,  3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_NYT_X: {
        _frame_type_string = "NYT_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, 0,  1 },
            { -135, 0,  3 },
            {  -45, 0,  4 },
            {  135, 0,  2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
#endif //APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    case MOTOR_FRAME_TYPE_BF_X: {
        // betaflight quad X order
        // see: https://fpvfrenzy.com/betaflight-motor-order/
        _frame_type_string = "BF_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_BF_X_REV: {
        // betaflight quad X order, reversed motors
        _frame_type_string = "X_REV";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_DJI_X: {
        // DJI quad X order
        // see https://forum44.djicdn.com/data/attachment/forum/201711/26/172348bppvtt1ot1nrtp5j.jpg
        _frame_type_string = "DJI_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_CW_X: {
        // "clockwise X" motor order. Motors are ordered clockwise from front right
        // matching test order
        _frame_type_string = "CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_V: {
        _frame_type_string = "V";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45,   0.7981f,   1 },
            { -135,   1.0000f,   3 },
            {  -45,  -0.7981f,   4 },
            {  135,  -1.0000f,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_H: {
        // H frame set-up - same as X but motors spin in opposite directions
        _frame_type_string = "H";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_VTAIL: {
        /*
            Tested with: Lynxmotion Hunter Vtail 400
            - inverted rear outward blowing motors (at a 40 degree angle)
            - should also work with non-inverted rear outward blowing motors
            - no roll in rear motors
            - no yaw in front motors
            - should fly like some mix between a tricopter and X Quadcopter

            Roll control comes only from the front motors, Yaw control only from the rear motors.
            Roll & Pitch factor is measured by the angle away from the top of the forward axis to each arm.

            Note: if we want the front motors to help with yaw,
                motors 1's yaw factor should be changed to sin(radians(40)).  Where "40" is the vtail angle
                motors 3's yaw factor should be changed to -sin(radians(40))
        */
        _frame_type_string = "VTAIL";
        add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
        add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 3);
        add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
        add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2);
        break;
    }
    case MOTOR_FRAME_TYPE_ATAIL:
        /*
            The A-Shaped VTail is the exact same as a V-Shaped VTail, with one difference:
            - The Yaw factors are reversed, because the rear motors are facing different directions

            With V-Shaped VTails, the props make a V-Shape when spinning, but with
            A-Shaped VTails, the props make an A-Shape when spinning.
            - Rear thrust on a V-Shaped V-Tail Quad is outward
            - Rear thrust on an A-Shaped V-Tail Quad is inward

            Still functions the same as the V-Shaped VTail mixing below:
            - Yaw control is entirely in the rear motors
            - Roll is is entirely in the front motors
        */
        _frame_type_string = "ATAIL";
        add_motor(AP_MOTORS_MOT_1, 60, 60, 0, 1);
        add_motor(AP_MOTORS_MOT_2, 0, -160, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3);
        add_motor(AP_MOTORS_MOT_3, -60, -60, 0, 4);
        add_motor(AP_MOTORS_MOT_4, 0, 160, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2);
        break;
    case MOTOR_FRAME_TYPE_PLUSREV: {
        // plus with reversed motor directions
        _frame_type_string = "PLUSREV";
        static const AP_MotorsMatrix::MotorDef motors[] {
            { 90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
            { 0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_Y4:
        _frame_type_string = "Y4";
        // Y4 motor definition with right front CCW, left front CW
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f,  1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            {  1.0f,  1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    default:
        // quad frame class does not support this frame type
        return false;
    }
    return true;
}
#endif //AP_MOTORS_FRAME_QUAD_ENABLED
#if AP_MOTORS_FRAME_HEXA_ENABLED
bool AP_MotorsMatrix::setup_hexa_matrix(motor_frame_type frame_type)
{
    _frame_class_string = "HEXA";
    _mav_type = MAV_TYPE_HEXAROTOR;
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            { -120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            {   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6 },
            {  120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_H: {
        // H is same as X except middle motors are closer to center
        _frame_type_string = "H";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f, 0.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 2 },
            { 1.0f, 0.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
            { 1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 6 },
            { -1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            { -1.0f, 1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            { 1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW, 4 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_DJI_X: {
        _frame_type_string = "DJI_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_CW_X: {
        _frame_type_string = "CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // hexa frame class does not support this frame type
        return false;
    } //hexa
    return true;
}
#endif ////AP_MOTORS_FRAME_HEXA_ENABLED
#if AP_MOTORS_FRAME_OCTA_ENABLED
bool AP_MotorsMatrix::setup_octa_matrix(motor_frame_type frame_type)
{
    _frame_class_string = "OCTA";
    _mav_type = MAV_TYPE_OCTOROTOR;
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
        };

        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            { -157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            {   67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {  157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            {  -22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
            { -112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6 },
            {  -67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            {  112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_V: {
        _frame_type_string = "V";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            {  0.83f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 },
            { -0.67f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
            {  0.67f, -0.32f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            { -0.50f, -1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {  1.00f,  1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
            { -0.83f,  0.34f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            { -1.00f,  1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            {  0.50f, -1.00f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_H: {
        _frame_type_string = "H";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            {  1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
            { -1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            { -1.0f,   -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {  1.0f,    1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
            {  1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            {  1.0f,  0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 },
            { -1.0f, -0.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_I: {
        _frame_type_string = "I";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { 0.333f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            { -0.333f,  1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            {    1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            {  0.333f,  1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
            { -0.333f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {   -1.0f,  1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            {   -1.0f, -1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
            {    1.0f,  1.0f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_DJI_X: {
        _frame_type_string = "DJI_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  -22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8 },
            {  -67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7 },
            { -112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            { -157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            {  112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {   67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_CW_X: {
        _frame_type_string = "CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {   67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            { -157.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            { -112.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            {  -67.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7 },
            {  -22.5f,  AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // octa frame class does not support this frame type
        return false;
    } // octa frame type
    return true;
}
#endif //AP_MOTORS_FRAME_OCTA_ENABLED
#if AP_MOTORS_FRAME_OCTAQUAD_ENABLED
bool AP_MotorsMatrix::setup_octaquad_matrix(motor_frame_type frame_type)
{
    _mav_type = MAV_TYPE_OCTOROTOR;
    _frame_class_string = "OCTAQUAD";
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            { -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
            {   0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            { 180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_V: {
        _frame_type_string = "V";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45,   0.7981f,  1 },
            {  -45,  -0.7981f,  7 },
            { -135,   1.0000f,  5 },
            {  135,  -1.0000f,  3 },
            {  -45,   0.7981f,  8 },
            {   45,  -0.7981f,  2 },
            {  135,   1.0000f,  4 },
            { -135,  -1.0000f,  6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_H: {
        // H frame set-up - same as X but motors spin in opposite directions
        _frame_type_string = "H";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   1 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  7 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   5 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  3 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   8 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  2 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   4 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  6 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_CW_X: {
        _frame_type_string = "CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    // BF/X cinelifters using two 4-in-1 ESCs are quite common
    // see: https://fpvfrenzy.com/betaflight-motor-order/
    case MOTOR_FRAME_TYPE_BF_X: {
        _frame_type_string = "BF_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  7 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 8 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_BF_X_REV: {
    // betaflight octa quad X order, reversed motors
        _frame_type_string = "X_REV";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 7 },
            {  135, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
            {   45, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            { -135, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            {  -45, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  8 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // octaquad frame class does not support this frame type
        return false;
    } //octaquad
    return true;
}
#endif // AP_MOTORS_FRAME_OCTAQUAD_ENABLED
#if AP_MOTORS_FRAME_DODECAHEXA_ENABLED
bool AP_MotorsMatrix::setup_dodecahexa_matrix(motor_frame_type frame_type)
{
    _mav_type = MAV_TYPE_DODECAROTOR;
    _frame_class_string = "DODECAHEXA";
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  1 }, // forward-top
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   2 }, // forward-bottom
            {   60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   3 }, // forward-right-top
            {   60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  4 }, // forward-right-bottom
            {  120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  5 }, // back-right-top
            {  120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   6 }, // back-right-bottom
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   7 }, // back-top
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  8 }, // back-bottom
            { -120, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  9 }, // back-left-top
            { -120, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10 }, // back-left-bottom
            {  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11 }, // forward-left-top
            {  -60, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12 }, // forward-left-bottom
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X: {
        _frame_type_string = "X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   1 }, // forward-right-top
            {   30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    2 }, // forward-right-bottom
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    3 }, // right-top
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   4 }, // right-bottom
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   5 }, // back-right-top
            {  150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    6 }, // back-right-bottom
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    7 }, // back-left-top
            { -150, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   8 }, // back-left-bottom
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   9 }, // left-top
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10 }, // left-bottom
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   11 }, // forward-left-top
            {  -30, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,  12 }, // forward-left-bottom
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // dodeca-hexa frame class does not support this frame type
        return false;
    } //dodecahexa
    return true;
}
#endif //AP_MOTORS_FRAME_DODECAHEXA_ENABLED
#if AP_MOTORS_FRAME_Y6_ENABLED
bool AP_MotorsMatrix::setup_y6_matrix(motor_frame_type frame_type)
{
    _mav_type = MAV_TYPE_HEXAROTOR;
    _frame_class_string = "Y6";
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_Y6B: {
        // Y6 motor definition with all top motors spinning clockwise, all bottom motors counter clockwise
        _frame_type_string = "Y6B";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            { -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  3 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 4 },
            {  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
            {  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_Y6F: {
        // Y6 motor layout for FireFlyY6
        _frame_type_string = "Y6F";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
            { -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 1 },
            {  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 5 },
            {  0.0f, -1.000f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
            { -1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  2 },
            {  1.0f,  0.500f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  6 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    default: {
        _frame_type_string = "default";
        static const AP_MotorsMatrix::MotorDefRaw motors[] {
            { -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 2 },
            {  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  5 },
            {  1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 6 },
            {  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  4 },
            { -1.0f,  0.666f, AP_MOTORS_MATRIX_YAW_FACTOR_CW,  1 },
            {  0.0f, -1.333f, AP_MOTORS_MATRIX_YAW_FACTOR_CCW, 3 },
        };
        add_motors_raw(motors, ARRAY_SIZE(motors));
        break;
    }
    } //y6
    return true;
}
#endif // AP_MOTORS_FRAME_Y6_ENABLED
#if AP_MOTORS_FRAME_DECA_ENABLED
bool AP_MotorsMatrix::setup_deca_matrix(motor_frame_type frame_type)
{
    _mav_type = MAV_TYPE_DECAROTOR;
    _frame_class_string = "DECA";
    switch (frame_type) {
    case MOTOR_FRAME_TYPE_PLUS: {
        _frame_type_string = "PLUS";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {    0, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   1 },
            {   36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    2 },
            {   72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   3 },
            {  108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    4 },
            {  144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   5 },
            {  180, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    6 },
            { -144, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   7 },
            { -108, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    8 },
            {  -72, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   9 },
            {  -36, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    case MOTOR_FRAME_TYPE_X:
    case MOTOR_FRAME_TYPE_CW_X: {
        _frame_type_string = "X/CW_X";
        static const AP_MotorsMatrix::MotorDef motors[] {
            {   18, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   1 },
            {   54, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    2 },
            {   90, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   3 },
            {  126, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    4 },
            {  162, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   5 },
            { -162, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    6 },
            { -126, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   7 },
            {  -90, AP_MOTORS_MATRIX_YAW_FACTOR_CW,    8 },
            {  -54, AP_MOTORS_MATRIX_YAW_FACTOR_CCW,   9 },
            {  -18, AP_MOTORS_MATRIX_YAW_FACTOR_CW,   10 },
        };
        add_motors(motors, ARRAY_SIZE(motors));
        break;
    }
    default:
        // deca frame class does not support this frame type
        return false;
    } //deca
    return true;
}
#endif // AP_MOTORS_FRAME_DECA_ENABLED

void AP_MotorsMatrix::setup_motors(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // remove existing motors
    for (int8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        remove_motor(i);
    }
    set_initialised_ok(false);
    bool success = true;

    switch (frame_class) {
#if AP_MOTORS_FRAME_QUAD_ENABLED
    case MOTOR_FRAME_QUAD:
        success = setup_quad_matrix(frame_type);
        break;  // quad
#endif //AP_MOTORS_FRAME_QUAD_ENABLED
#if AP_MOTORS_FRAME_HEXA_ENABLED
    case MOTOR_FRAME_HEXA:
        success = setup_hexa_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_HEXA_ENABLED
#if AP_MOTORS_FRAME_OCTA_ENABLED
    case MOTOR_FRAME_OCTA:
        success = setup_octa_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_OCTA_ENABLED
#if AP_MOTORS_FRAME_OCTAQUAD_ENABLED
    case MOTOR_FRAME_OCTAQUAD:
        success = setup_octaquad_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_OCTAQUAD_ENABLED
#if AP_MOTORS_FRAME_DODECAHEXA_ENABLED
    case MOTOR_FRAME_DODECAHEXA:
        success = setup_dodecahexa_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_DODECAHEXA_ENABLED
#if AP_MOTORS_FRAME_Y6_ENABLED
    case MOTOR_FRAME_Y6:
        success = setup_y6_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_Y6_ENABLED
#if AP_MOTORS_FRAME_DECA_ENABLED
    case MOTOR_FRAME_DECA:
        success = setup_deca_matrix(frame_type);
        break;
#endif //AP_MOTORS_FRAME_DECA_ENABLED
    default:
        // matrix doesn't support the configured class
        success = false;
        _mav_type = MAV_TYPE_GENERIC;
        break;
    } // switch frame_class

    // normalise factors to magnitude 0.5
    normalise_rpy_factors();

    if (!success) {
        _frame_class_string = "UNSUPPORTED";
    }
    set_initialised_ok(success);
}

// normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
// normalizes throttle factors so max value is 1 and no value is less than 0
void AP_MotorsMatrix::normalise_rpy_factors()
{
    float roll_fac = 0.0f;
    float pitch_fac = 0.0f;
    float yaw_fac = 0.0f;
    float throttle_fac = 0.0f;

    // find maximum roll, pitch and yaw factors
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            roll_fac = MAX(roll_fac,fabsf(_roll_factor[i]));
            pitch_fac = MAX(pitch_fac,fabsf(_pitch_factor[i]));
            yaw_fac = MAX(yaw_fac,fabsf(_yaw_factor[i]));
            throttle_fac = MAX(throttle_fac,MAX(0.0f,_throttle_factor[i]));
        }
    }

    // scale factors back to -0.5 to +0.5 for each axis
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        if (motor_enabled[i]) {
            if (!is_zero(roll_fac)) {
                _roll_factor[i] = 0.5f * _roll_factor[i] / roll_fac;
            }
            if (!is_zero(pitch_fac)) {
                _pitch_factor[i] = 0.5f * _pitch_factor[i] / pitch_fac;
            }
            if (!is_zero(yaw_fac)) {
                _yaw_factor[i] = 0.5f * _yaw_factor[i] / yaw_fac;
            }
            if (!is_zero(throttle_fac)) {
                _throttle_factor[i] = MAX(0.0f,_throttle_factor[i] / throttle_fac);
            }
        }
    }
}


/*
  call vehicle supplied thrust compensation if set. This allows
  vehicle code to compensate for vehicle specific motor arrangements
  such as tiltrotors or tiltwings
*/
void AP_MotorsMatrix::thrust_compensation(void)
{
    if (_thrust_compensation_callback) {
        _thrust_compensation_callback(_thrust_rpyt_out, AP_MOTORS_MAX_NUM_MOTORS);
    }
}

/*
  disable the use of motor torque to control yaw. Used when an
  external mechanism such as vectoring is used for yaw control
*/
void AP_MotorsMatrix::disable_yaw_torque(void)
{
    for (uint8_t i = 0; i < AP_MOTORS_MAX_NUM_MOTORS; i++) {
        _yaw_factor[i] = 0;
    }
}

#if APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
// examples can pull values direct
float AP_MotorsMatrix::get_thrust_rpyt_out(uint8_t i) const
{
    if (i < AP_MOTORS_MAX_NUM_MOTORS) {
        return _thrust_rpyt_out[i];
    }
    return 0.0;
}

bool AP_MotorsMatrix::get_factors(uint8_t i, float &roll, float &pitch, float &yaw, float &throttle, uint8_t &testing_order) const
{
    if ((i < AP_MOTORS_MAX_NUM_MOTORS) && motor_enabled[i]) {
        roll = _roll_factor[i];
        pitch = _pitch_factor[i];
        yaw = _yaw_factor[i];
        throttle = _throttle_factor[i];
        testing_order = _test_order[i];
        return true;
    }
    return false;
}
#endif

// singleton instance
AP_MotorsMatrix *AP_MotorsMatrix::_singleton;
