/// @file	AP_MotorsMatrix.h
/// @brief	Motor control class for Matrixcopters
#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMulticopter.h"

#define AP_MOTORS_MATRIX_YAW_FACTOR_CW   -1
#define AP_MOTORS_MATRIX_YAW_FACTOR_CCW   1

/// @class      AP_MotorsMatrix
class AP_MotorsMatrix : public AP_MotorsMulticopter {
public:

    /// Constructor
    AP_MotorsMatrix(uint16_t speed_hz = AP_MOTORS_SPEED_DEFAULT) :
        AP_MotorsMulticopter(speed_hz)
    {
        if (_singleton != nullptr) {
            AP_HAL::panic("AP_MotorsMatrix must be singleton");
        }
        _singleton = this;
    };

    // get singleton instance
    static AP_MotorsMatrix *get_singleton() {
        return _singleton;
    }

    // init
    virtual void        init(motor_frame_class frame_class, motor_frame_type frame_type) override;

#if AP_SCRIPTING_ENABLED
    // Init to be called from scripting
    virtual bool        init(uint8_t expected_num_motors);

    // Set throttle factor from scripting
    bool                set_throttle_factor(int8_t motor_num, float throttle_factor);

#endif // AP_SCRIPTING_ENABLED

    // set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
    void                set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type) override;

    // set update rate to motors - a value in hertz
    // you must have setup_motors before calling this
    void                set_update_rate(uint16_t speed_hz) override;

    // output_test_num - spin a motor connected to the specified output channel
    //  (should only be performed during testing)
    //  If a motor output channel is remapped, the mapped channel is used.
    //  Returns true if motor output is set, false otherwise
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    bool                output_test_num(uint8_t motor, int16_t pwm);

    // output_to_motors - sends minimum values out to the motors
    virtual void        output_to_motors() override;

    // get_motor_mask - returns a bitmask of which outputs are being used for motors (1 means being used)
    //  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
    uint32_t            get_motor_mask() override;

    // return number of motor that has failed.  Should only be called if get_thrust_boost() returns true
    uint8_t             get_lost_motor() const override { return _motor_lost_index; }

    // return the roll factor of any motor, this is used for tilt rotors and tail sitters
    // using copter motors for forward flight
    float               get_roll_factor(uint8_t i) override { return _roll_factor[i]; }
    // return the pitch factor of any motor
    float               get_pitch_factor(uint8_t i) override { return _pitch_factor[i]; }

    // disable the use of motor torque to control yaw. Used when an external mechanism such
    // as vectoring is used for yaw control
    void                disable_yaw_torque(void) override;

    // add_motor using raw roll, pitch, throttle and yaw factors
    void                add_motor_raw(int8_t motor_num, float roll_fac, float pitch_fac, float yaw_fac, uint8_t testing_order, float throttle_factor = 1.0f);

    // same structure, but with floats.
    struct MotorDef {
        float angle_degrees;
        float yaw_factor;
        uint8_t testing_order;
    };

    // method to add many motors specified in a structure:
    void add_motors(const struct MotorDef *motors, uint8_t num_motors);

    // structure used for initialising motors that add have separate
    // roll/pitch/yaw factors.  Note that this does *not* include
    // the final parameter for the add_motor_raw call - throttle
    // factor as that is only used in the scripting binding, not in
    // the static motors at the moment.
    struct MotorDefRaw {
        float roll_fac;
        float pitch_fac;
        float yaw_fac;
        uint8_t testing_order;
    };
    void add_motors_raw(const struct MotorDefRaw *motors, uint8_t num_motors);

    // pull values direct, (examples only)
    float get_thrust_rpyt_out(uint8_t i) const;
    bool get_factors(uint8_t i, float &roll, float &pitch, float &yaw, float &throttle, uint8_t &testing_order) const;

    // Degrade the mixer after one motor has stopped.
    //
    // Without this the controller has no idea the motor is gone: it keeps
    // commanding it, and then drives the opposite motor down to balance thrust
    // that is not being produced.  Measured in SITL on a hexacopter, stopping
    // one motor pushed its own command to the top of its range and the opposite
    // motor to the bottom - one failure turned into two - and the vehicle rolled
    // over and crashed 17 s later with the remaining motors nowhere near
    // saturation.
    //
    // surrender_yaw scales the yaw factors by MOT_FAIL_YAW, which defaults to
    // keeping them.  Five motors cannot satisfy roll, pitch, yaw and throttle
    // *exactly* - the allocation is rank deficient once the counter-rotating
    // pairs stop matching - but the mixer never demands exact: it serves yaw
    // last, so yaw alone absorbs the shortfall.  Setting MOT_FAIL_YAW to 0
    // gives yaw up outright and lets the vehicle rotate freely.
    //
    // Returns false if the motor number is invalid or already removed.  The
    // change is not reversible, which matches the failure it models.
    bool                set_motor_failed(uint8_t motor_num, bool surrender_yaw = true);

    // motor removed by set_motor_failed(), or -1
    int8_t              get_failed_motor() const { return _failed_motor; }

    // Last redistributed allocation, for the MALC log message.  Without demand
    // against achieved there is no way to tell, after the fact, whether a poor
    // post-failure response was the controller asking for the wrong thing or the
    // allocator failing to deliver what it asked.  Valid only while
    // alloc_active() is true.
    bool                alloc_active() const { return _alloc_active; }
    const float        *get_alloc_demand() const { return _alloc_demand; }
    const float        *get_alloc_achieved() const { return _alloc_achieved; }

    // Why the last solve ended the way it did.  Logged unconditionally while
    // degraded, because a solve that fails logs nothing else: the caller falls
    // back to forward mixing and the flight continues looking ordinary.  A gap
    // in MALC and a healthy allocation are indistinguishable without this.
    enum class AllocResult : uint8_t {
        OK              = 0,
        TOO_FEW_MOTORS  = 1,   // fewer effectors than rows
        SINGULAR        = 2,   // B*B^T not invertible - rank deficient
        NON_FINITE      = 3,   // NaN/Inf came out of the solve
        RESIDUAL        = 4,   // B*t != rem beyond tolerance - ill conditioned
        NO_SOLUTION     = 5,   // every pass needed clamping, none verified
        NOT_RUN         = 6,   // allocator disabled or no failed motor
    };
    AllocResult         alloc_result() const { return _alloc_result; }
    uint8_t             alloc_passes() const { return _alloc_passes; }
    uint8_t             alloc_clamp_mask() const { return _alloc_clamp_mask; }

    // Redistributed-pseudoinverse control allocation.
    //
    // Solves for per-motor thrusts whose resulting moments come as close as
    // possible to the demand, while respecting 0 <= thrust <= 1.  Iterates:
    // pseudoinverse solve, clamp whatever left the limits, re-solve on the
    // motors still free.
    //
    // The point is that the allocator *knows about the limits*.  The fixed
    // forward mixing this class normally uses does not: it computes a linear
    // combination, and anything that comes out negative is simply clipped -
    // which silently destroys the moment balance the combination was built to
    // produce.  With a rotor gone the hover solution sits right on that limit,
    // so the clipping is not an edge case, it is the normal state.
    //
    // include_yaw selects whether heading is part of the demand at all.  On a
    // hexacopter with a rotor gone it should be false: the hover trim then sits
    // exactly on the lower thrust limit, where the motor pinned at zero can
    // only be raised, never lowered - one direction of disturbance has no
    // control authority left.  A solution exists, but the equilibrium is not
    // controllable, which is the published result for this class of airframe.
    // Dropping yaw from the demand frees that dimension and the remaining five
    // motors regain margin for attitude.
    //
    // demand is [throttle, roll, pitch, yaw]; returns false if the geometry is
    // degenerate.  See Durham, Bordignon & Beck, "Aircraft Control Allocation".
    // Yaw factors as the airframe's geometry gives them, captured when a motor
    // is removed and before surrender_yaw scales the live ones toward zero.
    // The mixer stops *commanding* yaw, but the rotors keep *producing* it, so
    // the suppression term in allocate_redistributed() has to work from these.
    float               _yaw_geom[AP_MOTORS_MAX_NUM_MOTORS];

    // Snapshot of the last redistributed solve: what was asked for, and what the
    // committed thrusts actually produce.  [throttle, roll, pitch, yaw], with the
    // throttle entry already summed over motors so the two are comparable.
    float               _alloc_demand[4];
    float               _alloc_achieved[4];
    bool                _alloc_active;

    // Diagnostics from the last solve.  mutable because
    // allocate_redistributed() is const - it computes thrusts and must not
    // change control state, but it is the only place that knows why it gave up.
    mutable AllocResult _alloc_result = AllocResult::NOT_RUN;
    mutable uint8_t     _alloc_passes;        // active-set iterations run
    mutable uint8_t     _alloc_clamp_mask;    // bit i = motor i hit a limit

    bool                allocate_redistributed(const float demand[4], bool include_yaw,
                                               float thrust_out[AP_MOTORS_MAX_NUM_MOTORS]) const;

    // Report what the redistributed allocation actually achieved back into
    // limit.*, so the rate PIDs get anti-windup that matches the mixer that
    // really ran.  Without this the flags still describe the forward mixer,
    // which the allocator has just replaced wholesale.
    void                set_limits_from_allocation(const float demand[4], bool include_yaw,
                                                   const float thrust[AP_MOTORS_MAX_NUM_MOTORS]);

    // Index of the motor diametrically opposite the given one - the one whose
    // roll and pitch factors are both its negation - or -1 if there is none.
    int8_t              find_opposite_motor(uint8_t motor_num) const;

    // Watch ESC rpm for a motor that has stopped, and degrade the mixer when
    // one is confirmed.  Detects a stopped motor only: a thrown propeller
    // leaves the motor spinning *faster* under no load, so it shows up as
    // over-speed rather than under-speed and is deliberately out of scope here.
    void                update_failure_detection();



protected:
    // output - sends commands to the motors
    void                output_armed_stabilizing() override;

    // check for failed motor
    void                check_for_failed_motor(float throttle_thrust_best);

    // add_motor using just position and yaw_factor (or prop direction)
    void                add_motor(int8_t motor_num, float angle_degrees, float yaw_factor, uint8_t testing_order);

    // add_motor using separate roll and pitch factors (for asymmetrical frames) and prop direction
    void                add_motor(int8_t motor_num, float roll_factor_in_degrees, float pitch_factor_in_degrees, float yaw_factor, uint8_t testing_order);

    // remove_motor
    void                remove_motor(int8_t motor_num);





    // configures the motors for the defined frame_class and frame_type
    virtual void        setup_motors(motor_frame_class frame_class, motor_frame_type frame_type);

    // normalizes the roll, pitch and yaw factors so maximum magnitude is 0.5
    void                normalise_rpy_factors();

    int8_t              _failed_motor = -1;

    // seconds each motor has been commanded but not turning
    float               _fail_timer_s[AP_MOTORS_MAX_NUM_MOTORS];

    // Separate from _fail_timer_s: the stopped and shed checks look for
    // opposite signs of the same reading, so one sharing the other's timer
    // would reset exactly what the other is trying to accumulate.
    float               _shed_timer_s[AP_MOTORS_MAX_NUM_MOTORS];

    // Each motor's own k as a share of the fleet's, learned in flight, and how
    // long it has been learning.  The share is what makes the reference
    // immune to battery sag and air density while still absorbing the
    // per-motor offset that a fleet comparison cannot see past.
    // Zero-initialised: the object is plain new'd, so without this the arrays
    // start indeterminate.  Garbage that happens to read as a learned
    // reference would be judged against on the very first pass and could
    // remove a working motor before any real reading has been taken.
    float               _shed_ref[AP_MOTORS_MAX_NUM_MOTORS] {};
    float               _shed_learn_s[AP_MOTORS_MAX_NUM_MOTORS] {};
    // Rate limiting for the multi-motor telemetry warning.  That branch warns
    // without acting, so _failed_motor stays -1 and the early return in
    // update_failure_detection() never engages: unthrottled, the message
    // repeats at the mixer rate for as long as the condition lasts.
    uint32_t            _fail_warn_last_ms;
    uint8_t             _fail_warn_count = 0;

    // call vehicle supplied thrust compensation if set
    void                thrust_compensation(void) override;

    const char*         _get_frame_string() const override { return _frame_class_string; }
    const char*         get_type_string() const override { return _frame_type_string; }

    // output_test_seq - spin a motor at the pwm value specified
    //  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
    //  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
    virtual void        _output_test_seq(uint8_t motor_seq, int16_t pwm) override;

    float               _roll_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to roll
    float               _pitch_factor[AP_MOTORS_MAX_NUM_MOTORS]; // each motors contribution to pitch
    float               _yaw_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to yaw (normally 1 or -1)
    float               _throttle_factor[AP_MOTORS_MAX_NUM_MOTORS];  // each motors contribution to throttle 0~1
    float               _thrust_rpyt_out[AP_MOTORS_MAX_NUM_MOTORS]; // combined roll, pitch, yaw and throttle outputs to motors in 0~1 range
    uint8_t             _test_order[AP_MOTORS_MAX_NUM_MOTORS];  // order of the motors in the test sequence

    // motor failure handling
    float               _thrust_rpyt_out_filt[AP_MOTORS_MAX_NUM_MOTORS];    // filtered thrust outputs with 1 second time constant
    uint8_t             _motor_lost_index;  // index number of the lost motor

    motor_frame_class   _active_frame_class; // active frame class (i.e. quad, hexa, octa, etc)
    motor_frame_type    _active_frame_type;  // active frame type (i.e. plus, x, v, etc)

    const char*         _frame_class_string = ""; // string representation of frame class
    const char*         _frame_type_string = "";  //  string representation of frame type

private:

    // helper to return value scaled between boost and normal based on the value of _thrust_boost_ratio
    float boost_ratio(float boost_value, float normal_value) const;

    // setup motors matrix
    bool setup_quad_matrix(motor_frame_type frame_type);
    bool setup_hexa_matrix(motor_frame_type frame_type);
    bool setup_octa_matrix(motor_frame_type frame_type);
    bool setup_deca_matrix(motor_frame_type frame_type);
    bool setup_dodecahexa_matrix(motor_frame_type frame_type);
    bool setup_y6_matrix(motor_frame_type frame_type);
    bool setup_octaquad_matrix(motor_frame_type frame_type);

    static AP_MotorsMatrix *_singleton;
};
