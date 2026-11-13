// Unit tests for LowLevelSafety (no ROS, no DDS).

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "hardware_unitree_mujoco/LowLevelSafety.h"

using namespace hardware_unitree_mujoco;

namespace
{
    constexpr double kDt = 0.001; // 1 kHz

    // Standing-like state inside every joint range.
    JointStateSample nominalState()
    {
        JointStateSample s;
        for (std::size_t leg = 0; leg < 4; ++leg)
        {
            s.q[leg * 3 + 0] = 0.0;
            s.q[leg * 3 + 1] = 0.8;
            s.q[leg * 3 + 2] = -1.5;
        }
        s.temperature.fill(30.0);
        return s;
    }

    // Command that holds the nominal state with moderate gains.
    JointCommand nominalCommand()
    {
        JointCommand c;
        const auto s = nominalState();
        c.q = s.q;
        c.kp.fill(20.0);
        c.kd.fill(1.0);
        c.tau.fill(1.0);
        return c;
    }

    bool isDamping(const JointCommand& c, const double kd)
    {
        for (std::size_t i = 0; i < kNumJoints; ++i)
        {
            if (c.q[i] != 0.0 || c.dq[i] != 0.0 || c.kp[i] != 0.0 || c.tau[i] != 0.0 || c.kd[i] != kd) return false;
        }
        return true;
    }

    // Safety with a valid state and command already received at t = 0.
    struct Fixture : ::testing::Test
    {
        LowLevelSafety safety{SafetyParams::go2Defaults()};
        double t = 0.0;

        void SetUp() override
        {
            safety.updateState(nominalState(), t, t);
            JointCommand c = nominalCommand();
            safety.filterCommand(c, true, t);
            ASSERT_FALSE(safety.stopped());
        }

        // One control cycle: fresh state and fresh command at time t (+dt).
        JointCommand step(const JointStateSample& s, JointCommand c, const bool fresh = true)
        {
            t += kDt;
            safety.updateState(s, t, t);
            safety.filterCommand(c, fresh, t);
            return c;
        }
    };
}

TEST(Sentinel, DistinguishesUnwrittenFromNaN)
{
    EXPECT_TRUE(std::isnan(commandSentinel()));
    EXPECT_TRUE(isCommandSentinel(commandSentinel()));
    EXPECT_FALSE(isCommandSentinel(std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(isCommandSentinel(-std::numeric_limits<double>::quiet_NaN()));
    EXPECT_FALSE(isCommandSentinel(0.0));

    JointArray buffer;
    buffer.fill(commandSentinel());
    JointArray held;
    held.fill(5.0);
    EXPECT_FALSE(takeCommand(buffer, held));
    EXPECT_EQ(held[0], 5.0); // held value kept

    buffer[3] = 1.5;
    EXPECT_TRUE(takeCommand(buffer, held));
    EXPECT_EQ(held[3], 1.5);
    EXPECT_EQ(held[0], 5.0);
    EXPECT_TRUE(isCommandSentinel(buffer[3])); // reset after take

    // A NaN written by a controller is taken (and later rejected by filterCommand)
    buffer[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_TRUE(takeCommand(buffer, held));
    EXPECT_TRUE(std::isnan(held[0]));
}

TEST(Remote, ButtonLayout)
{
    std::uint8_t raw[40] = {};
    // byte 2: R1 L1 Start Select R2 L2 f1 f2 (bit 0..7), byte 3: A B X Y up right down left
    raw[2] = 1u << 5; // L2
    raw[3] = 1u << 1; // B
    EXPECT_EQ(remote::buttons(raw), remote::L2 | remote::B);
}

TEST_F(Fixture, NominalCommandPassesUnchanged)
{
    const JointCommand in = nominalCommand();
    const JointCommand out = step(nominalState(), in);
    EXPECT_FALSE(safety.stopped());
    for (std::size_t i = 0; i < kNumJoints; ++i)
    {
        EXPECT_DOUBLE_EQ(out.q[i], in.q[i]);
        EXPECT_DOUBLE_EQ(out.kp[i], in.kp[i]);
        EXPECT_DOUBLE_EQ(out.kd[i], in.kd[i]);
        EXPECT_DOUBLE_EQ(out.tau[i], in.tau[i]);
    }
}

TEST_F(Fixture, NaNCommandStops)
{
    JointCommand c = nominalCommand();
    c.tau[4] = std::numeric_limits<double>::quiet_NaN();
    const JointCommand out = step(nominalState(), c);
    EXPECT_TRUE(safety.stopped());
    EXPECT_EQ(safety.stopInfo().reason, StopReason::NON_FINITE_COMMAND);
    EXPECT_EQ(safety.stopInfo().joint, 4);
    EXPECT_TRUE(isDamping(out, safety.params().damping_kd));
}

TEST_F(Fixture, InfGainStops)
{
    JointCommand c = nominalCommand();
    c.kp[0] = std::numeric_limits<double>::infinity();
    step(nominalState(), c);
    EXPECT_EQ(safety.stopInfo().reason, StopReason::NON_FINITE_COMMAND);
}

TEST_F(Fixture, NaNStateStops)
{
    auto s = nominalState();
    s.dq[7] = std::nan("");
    step(s, nominalCommand());
    EXPECT_EQ(safety.stopInfo().reason, StopReason::NON_FINITE_STATE);
}

TEST_F(Fixture, TotalTorqueIsLimitedByReducingTau)
{
    auto s = nominalState();
    JointCommand c = nominalCommand();
    // thigh (index 1): tau 10 + kp 50 * (q_cmd - q = 0.3) = 25 Nm > 23.7 * 0.7 = 16.59
    c.kp[1] = 50.0;
    c.q[1] = s.q[1] + 0.3;
    c.tau[1] = 10.0;
    // calf (index 2): pure feed-forward far beyond the limit, negative
    c.tau[2] = -100.0;
    const JointCommand out = step(s, c);
    EXPECT_FALSE(safety.stopped());

    const double lim_thigh = 23.7 * 0.7;
    const double lim_calf = 45.43 * 0.7;
    EXPECT_NEAR(safety.totalTorque(out, 1), lim_thigh, 1e-9);
    EXPECT_DOUBLE_EQ(out.kp[1], 50.0); // gains and q_cmd untouched
    EXPECT_DOUBLE_EQ(out.q[1], s.q[1] + 0.3);
    EXPECT_NEAR(out.tau[1], lim_thigh - 50.0 * 0.3, 1e-9);
    EXPECT_NEAR(safety.totalTorque(out, 2), -lim_calf, 1e-9);
    EXPECT_GT(safety.torqueClampCount(), 0u);
}

TEST_F(Fixture, TotalTorqueIncludesDamping)
{
    auto s = nominalState();
    s.dq[5] = -10.0; // moving fast
    JointCommand c = nominalCommand();
    c.kd[5] = 4.0; // kd * (0 - (-10)) = 40 Nm
    c.tau[5] = 0.0;
    const JointCommand out = step(s, c);
    EXPECT_NEAR(safety.totalTorque(out, 5), 45.43 * 0.7, 1e-9);
    EXPECT_LT(out.tau[5], 0.0);
}

TEST_F(Fixture, GainsAndPositionCommandAreClamped)
{
    JointCommand c = nominalCommand();
    c.kp[0] = 1000.0;
    c.kd[0] = 50.0;
    c.kp[1] = -5.0;
    c.q[2] = -3.0; // calf below -2.7227
    c.q[0] = 2.0; // hip above 1.0472
    const JointCommand out = step(nominalState(), c);
    const auto& p = safety.params();
    EXPECT_DOUBLE_EQ(out.kp[0], p.kp_max);
    EXPECT_DOUBLE_EQ(out.kd[0], p.kd_max);
    EXPECT_DOUBLE_EQ(out.kp[1], 0.0);
    EXPECT_DOUBLE_EQ(out.q[2], -2.7227 + p.q_cmd_margin);
    EXPECT_DOUBLE_EQ(out.q[0], 1.0472 - p.q_cmd_margin);
    EXPECT_FALSE(safety.stopped());
}

TEST_F(Fixture, SustainedTauEstStops)
{
    auto s = nominalState();
    s.tau_est[8] = 0.95 * 45.43 * 0.7; // calf, above 90 % of the limit
    const double t0 = t;
    while (!safety.stopped() && t - t0 < 1.0)
    {
        step(s, nominalCommand());
    }
    ASSERT_TRUE(safety.stopped());
    EXPECT_EQ(safety.stopInfo().reason, StopReason::TAU_EST_SUSTAINED);
    EXPECT_EQ(safety.stopInfo().joint, 8);
    EXPECT_NEAR(t - t0, 0.2 + kDt, 2 * kDt);
}

TEST_F(Fixture, ShortTauEstPeaksDoNotStop)
{
    auto high = nominalState();
    high.tau_est[1] = -1.2 * 23.7 * 0.7;
    // 150 ms over, 1 ms under, repeated: never 200 ms continuously
    for (int k = 0; k < 10; ++k)
    {
        for (int i = 0; i < 150; ++i) step(high, nominalCommand());
        step(nominalState(), nominalCommand());
    }
    EXPECT_FALSE(safety.stopped());
}

TEST_F(Fixture, JointOutOfRangeStops)
{
    auto s = nominalState();
    s.q[5] = -2.7227 - safety.params().q_monitor_tolerance - 0.01;
    step(s, nominalCommand());
    EXPECT_EQ(safety.stopInfo().reason, StopReason::JOINT_OUT_OF_RANGE);
    EXPECT_EQ(safety.stopInfo().joint, 5);
}

TEST_F(Fixture, SmallRangeExcessWithinToleranceIsOk)
{
    auto s = nominalState();
    s.q[2] = -2.724; // lying pose in MuJoCo, 1.3 mrad below the range
    step(s, nominalCommand());
    EXPECT_FALSE(safety.stopped());
}

TEST_F(Fixture, JointVelocityStops)
{
    auto s = nominalState();
    s.dq[11] = 0.81 * 15.70;
    step(s, nominalCommand());
    EXPECT_EQ(safety.stopInfo().reason, StopReason::JOINT_VELOCITY);
    EXPECT_EQ(safety.stopInfo().joint, 11);
}

TEST_F(Fixture, TemperatureOnlyWarnsByDefault)
{
    EXPECT_TRUE(std::isinf(safety.params().temperature_stop));
    auto s = nominalState();
    s.temperature[3] = 120.0;
    step(s, nominalCommand());
    EXPECT_FALSE(safety.stopped());
    EXPECT_EQ(safety.takeTemperatureWarning(), 3);
}

TEST(Temperature, WarnThenStopWhenEnabled)
{
    auto p = SafetyParams::go2Defaults();
    p.temperature_stop = 85.0;
    LowLevelSafety safety(p);
    double t = 0.0;
    auto s = nominalState();
    JointCommand c = nominalCommand();
    safety.updateState(s, t, t);
    safety.filterCommand(c, true, t);

    s.temperature[3] = p.temperature_warn;
    safety.updateState(s, t, t);
    EXPECT_FALSE(safety.stopped());
    EXPECT_EQ(safety.takeTemperatureWarning(), 3);
    EXPECT_EQ(safety.takeTemperatureWarning(), -1); // only once
    safety.updateState(s, t, t);
    EXPECT_EQ(safety.takeTemperatureWarning(), -1);

    s.temperature[3] = 85.0;
    safety.updateState(s, t, t);
    EXPECT_EQ(safety.stopInfo().reason, StopReason::MOTOR_TEMPERATURE);
}

TEST(Defaults, Go2OfficialLimits)
{
    const auto p = SafetyParams::go2Defaults();
    EXPECT_DOUBLE_EQ(p.torque_max[2], 45.43);
    EXPECT_DOUBLE_EQ(p.dq_max[2], 15.70);
    // front thigh (FR = 1, FL = 4) vs rear thigh (RR = 7, RL = 10)
    for (const std::size_t i : {1u, 4u})
    {
        EXPECT_DOUBLE_EQ(p.q_min[i], -1.5708);
        EXPECT_DOUBLE_EQ(p.q_max[i], 3.4907);
    }
    for (const std::size_t i : {7u, 10u})
    {
        EXPECT_DOUBLE_EQ(p.q_min[i], -0.5236);
        EXPECT_DOUBLE_EQ(p.q_max[i], 4.5379);
    }
}

TEST_F(Fixture, RearThighRangeIsPerLeg)
{
    // -0.8 rad is inside the front thigh range but below the rear one (-0.5236 - 0.05)
    auto s = nominalState();
    s.q[1] = -0.8; // FR thigh: ok
    step(s, nominalCommand());
    EXPECT_FALSE(safety.stopped());
    s.q[7] = -0.8; // RR thigh: out of range
    step(s, nominalCommand());
    EXPECT_EQ(safety.stopInfo().reason, StopReason::JOINT_OUT_OF_RANGE);
    EXPECT_EQ(safety.stopInfo().joint, 7);
}

TEST_F(Fixture, TorqueClampCountPerJoint)
{
    JointCommand c = nominalCommand();
    c.tau[2] = 100.0;
    for (int i = 0; i < 3; ++i) step(nominalState(), c);
    EXPECT_EQ(safety.torqueClampCount(2), 3u);
    EXPECT_EQ(safety.torqueClampCount(0), 0u);
    EXPECT_EQ(safety.torqueClampCount(), 3u);
}

TEST_F(Fixture, LowStateCrcFailures)
{
    const unsigned n = safety.params().lowstate_crc_max_failures;
    safety.updateCrcFailures(n - 1, t);
    EXPECT_FALSE(safety.stopped());
    safety.updateCrcFailures(0, t); // a good message resets the count (done by the caller)
    safety.updateCrcFailures(n, t);
    EXPECT_EQ(safety.stopInfo().reason, StopReason::LOWSTATE_CRC);
}

TEST_F(Fixture, CommandWatchdog)
{
    // controller at 500 Hz under a 1 kHz loop: every other cycle not fresh -> fine
    for (int i = 0; i < 100; ++i)
    {
        step(nominalState(), nominalCommand(), i % 2 == 0);
    }
    EXPECT_FALSE(safety.stopped());

    // controller stops writing
    const double t0 = t;
    JointCommand out;
    while (!safety.stopped() && t - t0 < 1.0)
    {
        out = step(nominalState(), nominalCommand(), false);
    }
    ASSERT_TRUE(safety.stopped());
    EXPECT_EQ(safety.stopInfo().reason, StopReason::COMMAND_TIMEOUT);
    EXPECT_NEAR(t - t0, 0.02, 2 * kDt);
    EXPECT_TRUE(isDamping(out, safety.params().damping_kd));
}

TEST(Watchdog, NotArmedBeforeFirstCommand)
{
    LowLevelSafety safety;
    double t = 0.0;
    for (int i = 0; i < 1000; ++i, t += kDt)
    {
        safety.updateState(nominalState(), t, t);
        JointCommand c;
        safety.filterCommand(c, false, t);
    }
    EXPECT_FALSE(safety.stopped());
}

TEST(StateTimeout, DampsBeforeFirstStateAndStopsWhenStateStops)
{
    LowLevelSafety safety;
    JointCommand c = nominalCommand();
    safety.filterCommand(c, true, 0.0);
    EXPECT_FALSE(safety.stopped()); // not latched
    EXPECT_TRUE(isDamping(c, safety.params().damping_kd)); // but no command passes without a state

    double t = 0.0;
    safety.updateState(nominalState(), t, t);
    for (int i = 0; i < 49; ++i)
    {
        t += kDt;
        c = nominalCommand();
        safety.filterCommand(c, true, t); // no new state
    }
    EXPECT_FALSE(safety.stopped());
    t += 0.002;
    c = nominalCommand();
    safety.filterCommand(c, true, t);
    EXPECT_EQ(safety.stopInfo().reason, StopReason::STATE_TIMEOUT);
}

TEST_F(Fixture, RemoteEstop)
{
    auto s = nominalState();
    s.remote_buttons = remote::L2; // L2 alone: nothing
    step(s, nominalCommand());
    EXPECT_FALSE(safety.stopped());
    s.remote_buttons = remote::L2 | remote::B | remote::UP;
    step(s, nominalCommand());
    EXPECT_EQ(safety.stopInfo().reason, StopReason::REMOTE_ESTOP);
}

TEST_F(Fixture, StopIsLatched)
{
    auto s = nominalState();
    s.dq[0] = 100.0;
    step(s, nominalCommand());
    ASSERT_TRUE(safety.stopped());
    const StopInfo first = safety.stopInfo();
    EXPECT_TRUE(safety.takeStopEvent());
    EXPECT_FALSE(safety.takeStopEvent()); // reported once

    // everything back to normal for 2 s: still damping, reason unchanged
    JointCommand out;
    for (int i = 0; i < 2000; ++i) out = step(nominalState(), nominalCommand());
    EXPECT_TRUE(safety.stopped());
    EXPECT_TRUE(isDamping(out, safety.params().damping_kd));
    EXPECT_EQ(safety.stopInfo().reason, first.reason);
    EXPECT_EQ(safety.stopInfo().joint, first.joint);
    EXPECT_FALSE(safety.takeStopEvent());
}

TEST_F(Fixture, ExternalStopLatches)
{
    safety.externalStop(StopReason::LOOP_STALL, 0.15, 0.1, t);
    const JointCommand out = step(nominalState(), nominalCommand());
    EXPECT_TRUE(safety.stopped());
    EXPECT_EQ(safety.stopInfo().reason, StopReason::LOOP_STALL);
    EXPECT_TRUE(isDamping(out, safety.params().damping_kd));
}

TEST(Disabled, PassesEverything)
{
    auto p = SafetyParams::go2Defaults();
    p.enabled = false;
    LowLevelSafety safety(p);
    JointCommand c = nominalCommand();
    c.tau[0] = 1000.0;
    safety.filterCommand(c, true, 0.0);
    EXPECT_EQ(c.tau[0], 1000.0);
    EXPECT_FALSE(safety.stopped());
}
