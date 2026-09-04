/*
 * Unit tests for the part of AC_ArcNav that decides, before the vehicle is
 * anywhere near a turn, whether that turn can be flown at all.
 *
 * These functions have a second consumer besides the flight code: a ground
 * check that reads a mission file and a parameter file and reports which turns
 * in the mission are flyable.  That consumer cannot link this library, so it
 * implements the same arithmetic itself - which means there are two copies of
 * it, and two copies drift.
 *
 * Hence --dump-table below.  It writes the exact inputs and outputs of every
 * one of these functions over a grid, and the ground check's own test suite
 * runs this binary and compares against what it computes.  The table is
 * deliberately not a file in the repository: a checked-in table stops being
 * evidence the moment this file changes and nobody regenerates it, and it
 * fails silently, still looking like an external reference.  Generated on
 * demand it cannot go stale.
 */

#include <AP_gtest.h>

#include <AC_WPNav/AC_ArcNav.h>

#include <stdio.h>
#include <string.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// A working set of limits, near what an agricultural hexacopter runs with.
// Lean 40 degrees, the position controller's shaping jerk, and a yaw capability
// measured at working lean angle rather than read off ATC_SLEW_YAW.
static AC_ArcNav::Limits working_limits()
{
    AC_ArcNav::Limits lim {};
    lim.lean_angle_max_rad  = radians(40.0f);
    lim.jerk_ne_msss        = 5.0f;
    lim.yaw_rate_max_rads   = radians(57.0f);
    lim.yaw_accel_max_radss = radians(120.0f);
    lim.heading_lead_s      = 0.5f;
    return lim;
}

TEST(ArcNavFeasibility, LeanBudgetIsSeventyPercentOfTheLimit)
{
    AC_ArcNav::Limits lim = working_limits();

    // The turn may use AC_ARCNAV_TILT_FRACTION of the lean limit; the rest is
    // headroom for tracking and wind.  Check the speed that comes back is the
    // one that arithmetic gives, not merely something plausible.
    const float budget_rad = radians(40.0f) * 0.7f;
    const float expect_ms = sqrtf(GRAVITY_MSS * tanf(budget_rad) * 9.0f);
    EXPECT_NEAR(expect_ms, AC_ArcNav::max_speed_for_radius_ms(lim, 9.0f), 1e-4f);

    // Sizing a radius for a speed and sizing a speed for that radius have to be
    // the same statement read in two directions.  They are used that way: the
    // ground check offers both "fly it this slow" and "open it out this far" as
    // fixes for the same rejected turn, and if they disagreed one of the two
    // fixes would not actually work.
    const float v = 4.0f;
    const float r = AC_ArcNav::min_radius_for_speed_m(lim, v);
    EXPECT_NEAR(v, AC_ArcNav::max_speed_for_radius_ms(lim, r), 1e-3f);
}

TEST(ArcNavFeasibility, ZeroAndNegativeInputsAreRefusedNotComputed)
{
    AC_ArcNav::Limits lim = working_limits();
    float lead_in_m = -1.0f;

    EXPECT_FALSE(AC_ArcNav::plan_feasible(lim, 0.0f, 3.0f, lead_in_m));
    EXPECT_FLOAT_EQ(0.0f, lead_in_m);
    EXPECT_FALSE(AC_ArcNav::plan_feasible(lim, 5.0f, 0.0f, lead_in_m));
    EXPECT_FLOAT_EQ(0.0f, lead_in_m);
    EXPECT_FALSE(AC_ArcNav::plan_feasible(lim, -5.0f, 3.0f, lead_in_m));

    EXPECT_FLOAT_EQ(0.0f, AC_ArcNav::max_speed_for_radius_ms(lim, 0.0f));
    EXPECT_FLOAT_EQ(0.0f, AC_ArcNav::required_spiral_len_m(lim, 0.0f, 3.0f));
    EXPECT_FLOAT_EQ(0.0f, AC_ArcNav::required_spiral_len_m(lim, 5.0f, 0.0f));
}

TEST(ArcNavFeasibility, YawRateBindsBeforeLeanOnATightTurn)
{
    AC_ArcNav::Limits lim = working_limits();
    float lead_in_m = 0.0f;

    // A 2 m radius at 3 m/s.  Lean needs atan(v^2/r/g) = 24.6 degrees, well
    // inside the 28 degree budget, so on lean alone this turn passes.  Holding
    // the nose on the tangent needs v/r = 85.9 deg/s, against half of 57, so it
    // must be refused.  This is the whole reason the yaw check exists: on the
    // turns this project actually flies, lean is not the binding constraint.
    EXPECT_LT(degrees(atanf(sq(3.0f) / 2.0f / GRAVITY_MSS)), 40.0f * 0.7f);
    EXPECT_FALSE(AC_ArcNav::plan_feasible(lim, 2.0f, 3.0f, lead_in_m));

    // Same turn with the yaw limit left unset is accepted, which is what makes
    // supplying a real measured capability a requirement and not a nicety.
    lim.yaw_rate_max_rads = 0.0f;
    EXPECT_TRUE(AC_ArcNav::plan_feasible(lim, 2.0f, 3.0f, lead_in_m));
}

TEST(ArcNavFeasibility, HalfTheYawRateIsTheBoundary)
{
    AC_ArcNav::Limits lim = working_limits();
    float lead_in_m = 0.0f;

    // Only half the yaw rate may be spent on the turn; the other half is what
    // recovers the entry transient.  Sit either side of that line and check the
    // answer changes there, so a change to AC_ARCNAV_YAW_RATE_FRACTION shows up
    // here rather than silently in flight.
    const float v = 3.0f;
    const float r_at_limit = v / (lim.yaw_rate_max_rads * 0.5f);
    EXPECT_TRUE(AC_ArcNav::plan_feasible(lim, r_at_limit * 1.02f, v, lead_in_m));
    EXPECT_FALSE(AC_ArcNav::plan_feasible(lim, r_at_limit * 0.98f, v, lead_in_m));
}

TEST(ArcNavFeasibility, LeadInCoversTheTransitionAndTheLookAhead)
{
    AC_ArcNav::Limits lim = working_limits();
    float lead_in_m = 0.0f;

    const float r = 12.0f;
    const float v = 5.0f;
    ASSERT_TRUE(AC_ArcNav::plan_feasible(lim, r, v, lead_in_m));

    // lead_in is the transition plus the heading look-ahead, and the transition
    // is itself never shorter than the look-ahead, so the run-up is at least
    // two look-ahead distances.  A caller that aims a fixed distance past the
    // entry point is guessing; this is the number the generator will really use.
    const float spiral_m = AC_ArcNav::required_spiral_len_m(lim, r, v);
    EXPECT_NEAR(spiral_m + v * lim.heading_lead_s, lead_in_m, 1e-4f);
    EXPECT_GE(spiral_m, v * lim.heading_lead_s);
}

TEST(ArcNavFeasibility, TransitionGrowsWhenYawAccelIsScarce)
{
    AC_ArcNav::Limits lim = working_limits();
    const float r = 12.0f;
    const float v = 5.0f;

    const float generous_m = AC_ArcNav::required_spiral_len_m(lim, r, v);
    lim.yaw_accel_max_radss = radians(20.0f);
    const float scarce_m = AC_ArcNav::required_spiral_len_m(lim, r, v);

    // A weak yaw axis cannot be helped by slowing down or opening the radius -
    // spray rate fixes one and the swath fixes the other - so the only lever
    // left is a longer transition.  It must move in that direction.
    EXPECT_GT(scarce_m, generous_m);

    // And an unset yaw acceleration must not be read as "infinitely capable
    // and therefore no transition needed"; the jerk and look-ahead floors still
    // apply.
    lim.yaw_accel_max_radss = 0.0f;
    EXPECT_GE(AC_ArcNav::required_spiral_len_m(lim, r, v), v * lim.heading_lead_s);
}

TEST(ArcNavFeasibility, FeasibleTurnStillDemandsARunUpNobodyChecks)
{
    AC_ArcNav::Limits lim = working_limits();
    float lead_in_m = 0.0f;

    // This is the shape of the defect the ground check exists to catch, pinned
    // here so the contract is not mistaken for a guarantee.  The turn is
    // accepted, and the acceptance carries a requirement - the leg before it
    // must be at least lead_in_m long and must not decelerate over that
    // distance.  Nothing in this library checks that, because nothing here is
    // given the leg.  A short working row ending in a wide turn gets true back
    // and still cannot fly the planned arc.
    ASSERT_TRUE(AC_ArcNav::plan_feasible(lim, 12.0f, 5.0f, lead_in_m));
    EXPECT_GT(lead_in_m, 5.0f);
}

TEST(ArcNavFeasibility, PlanFromYawCapabilityMatchesItsOwnPlan)
{
    // Planning backwards from what the yaw channel can do, rather than forwards
    // from the swath.  The result has to survive the forward check, or the two
    // directions disagree and the ground report would recommend a turn its own
    // feasibility test rejects.
    AC_ArcNav::Limits lim = working_limits();
    const float v = 3.0f;
    const float sweep = M_PI;

    // Plan against half the rate, which is the budget plan_feasible enforces.
    const AC_ArcNav::UTurnPlan plan =
        AC_ArcNav::plan_from_yaw_capability(v, sweep,
                                            lim.yaw_rate_max_rads * 0.5f,
                                            lim.yaw_accel_max_radss);
    ASSERT_GT(plan.radius_m, 0.0f);
    EXPECT_NEAR(v / plan.peak_yaw_rate_rads, plan.radius_m, 1e-4f);

    float lead_in_m = 0.0f;
    EXPECT_TRUE(AC_ArcNav::plan_feasible(lim, plan.radius_m * 1.001f, v, lead_in_m));
}

TEST(ArcNavFeasibility, PlanFromYawCapabilityRefusesDegenerateInput)
{
    EXPECT_FLOAT_EQ(0.0f, AC_ArcNav::plan_from_yaw_capability(0.0f, M_PI, 1.0f, 2.0f).radius_m);
    EXPECT_FLOAT_EQ(0.0f, AC_ArcNav::plan_from_yaw_capability(3.0f, 0.0f, 1.0f, 2.0f).radius_m);
    EXPECT_FLOAT_EQ(0.0f, AC_ArcNav::plan_from_yaw_capability(3.0f, M_PI, 0.0f, 2.0f).radius_m);
    EXPECT_FLOAT_EQ(0.0f, AC_ArcNav::plan_from_yaw_capability(3.0f, M_PI, 1.0f, 0.0f).radius_m);
}

// ---------------------------------------------------------------------------
// Cross-check table
//
// CSV on stdout, one row per (limits, radius, speed).  %.9g round-trips a
// float exactly, so the ground check can compare bit for bit rather than to a
// tolerance it would have to justify.
// ---------------------------------------------------------------------------

static void dump_row(const AC_ArcNav::Limits& lim, float radius_m, float speed_ms)
{
    float lead_in_m = 0.0f;
    const bool feasible = AC_ArcNav::plan_feasible(lim, radius_m, speed_ms, lead_in_m);
    const AC_ArcNav::UTurnPlan plan =
        AC_ArcNav::plan_from_yaw_capability(speed_ms, M_PI,
                                            lim.yaw_rate_max_rads * 0.5f,
                                            lim.yaw_accel_max_radss);

    printf("%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d\n",
           lim.lean_angle_max_rad, lim.jerk_ne_msss,
           lim.yaw_rate_max_rads, lim.yaw_accel_max_radss, lim.heading_lead_s,
           radius_m, speed_ms,
           feasible ? 1 : 0,
           lead_in_m,
           AC_ArcNav::required_spiral_len_m(lim, radius_m, speed_ms),
           AC_ArcNav::max_speed_for_radius_ms(lim, radius_m),
           AC_ArcNav::min_radius_for_speed_m(lim, speed_ms),
           plan.radius_m, plan.spiral_len_m, plan.duration_s,
           plan.rate_limited ? 1 : 0);
}

static void dump_table()
{
    // Limit sets spanning what a parameter file can produce: a slack yaw limit
    // (the ATC_SLEW_YAW mistake), a measured one, a weak one, and the unchecked
    // case.  Jerk and lean vary because both feed the transition length.
    const float lean_deg[]      = { 30.0f, 40.0f, 45.0f };
    const float jerk[]          = { 3.0f, 5.0f, 20.0f };
    const float yaw_rate_deg[]  = { 0.0f, 30.0f, 57.0f, 78.4f, 120.0f };
    const float yaw_accel_deg[] = { 0.0f, 20.0f, 120.0f };
    const float lead_s[]        = { 0.0f, 0.5f };
    const float radius_m[]      = { 1.0f, 2.0f, 2.5f, 3.0f, 3.5f, 6.0f, 7.5f, 12.0f, 20.0f };
    const float speed_ms[]      = { 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f };

    printf("lean_angle_max_rad,jerk_ne_msss,yaw_rate_max_rads,yaw_accel_max_radss,"
           "heading_lead_s,radius_m,speed_ms,"
           "feasible,lead_in_m,required_spiral_len_m,max_speed_for_radius_ms,"
           "min_radius_for_speed_m,plan_radius_m,plan_spiral_len_m,plan_duration_s,"
           "plan_rate_limited\n");

    for (float ld : lean_deg) {
        for (float jk : jerk) {
            for (float yr : yaw_rate_deg) {
                for (float ya : yaw_accel_deg) {
                    for (float ls : lead_s) {
                        AC_ArcNav::Limits lim {};
                        lim.lean_angle_max_rad  = radians(ld);
                        lim.jerk_ne_msss        = jk;
                        lim.yaw_rate_max_rads   = radians(yr);
                        lim.yaw_accel_max_radss = radians(ya);
                        lim.heading_lead_s      = ls;
                        for (float r : radius_m) {
                            for (float v : speed_ms) {
                                dump_row(lim, r, v);
                            }
                        }
                    }
                }
            }
        }
    }
}

int main(int argc, char *argv[])
{
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dump-table") == 0) {
            dump_table();
            return 0;
        }
    }
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
