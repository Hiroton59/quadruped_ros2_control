//
// Low-level safety layer for Unitree quadrupeds (Go2 defaults).
//
// ROS-independent, header-only. Called every control cycle (500 Hz - 1 kHz) from
// HardwareUnitree::read()/write(), so it never allocates and never throws.
//
//   - filterCommand(): limits applied to every command (non-finite -> stop, kp/kd clamp,
//     q_cmd clamp to the joint range minus a margin, total motor torque clamp by reducing tau),
//     plus the command watchdog.
//   - updateState(): monitors on the measured state (sustained tau_est, joint range, joint
//     velocity, motor temperature, LowState timeout, remote-controller e-stop).
//
// Any monitor failure switches to DAMPING and latches (only a restart clears it):
//   tau = 0, kp = 0, kd = damping_kd, q = 0, dq = 0 for every joint.
//

#ifndef HARDWARE_UNITREE_MUJOCO_LOWLEVELSAFETY_H
#define HARDWARE_UNITREE_MUJOCO_LOWLEVELSAFETY_H

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

namespace hardware_unitree_mujoco
{
    constexpr std::size_t kNumJoints = 12;
    using JointArray = std::array<double, kNumJoints>;

    // ---------------------------------------------------------------------------------------
    // Command sentinel (used for the watchdog).
    //
    // After every write() the hardware fills its command buffers with this NaN. ros2_control
    // controllers write command interfaces with set_value(), which copies the double bit for
    // bit, so a buffer that still holds this exact bit pattern at the next write() was not
    // written by any controller in that cycle. A NaN produced by a controller's arithmetic has
    // the default payload (0x7ff8000000000000 / 0xfff8000000000000), never this one, so it is
    // still detected as a non-finite command.
    // ---------------------------------------------------------------------------------------
    constexpr std::uint64_t kCommandSentinelBits = 0x7ff8'5afe'c0de'0001ULL;

    inline double commandSentinel()
    {
        double d;
        std::memcpy(&d, &kCommandSentinelBits, sizeof(d));
        return d;
    }

    inline bool isCommandSentinel(const double value)
    {
        std::uint64_t bits;
        std::memcpy(&bits, &value, sizeof(bits));
        return bits == kCommandSentinelBits;
    }

    /// Take the values written by a controller since the last call.
    /// For every element: if it is not the sentinel, copy it to `held` and report fresh.
    /// Then reset the buffer to the sentinel. Elements a controller did not write keep the
    /// previously held value (a 500 Hz controller under a 1 kHz controller_manager writes
    /// every other cycle).
    inline bool takeCommand(JointArray& buffer, JointArray& held)
    {
        bool fresh = false;
        for (std::size_t i = 0; i < kNumJoints; ++i)
        {
            if (!isCommandSentinel(buffer[i]))
            {
                held[i] = buffer[i];
                fresh = true;
            }
            buffer[i] = commandSentinel();
        }
        return fresh;
    }

    // ---------------------------------------------------------------------------------------
    // Remote controller (LowState_.wireless_remote, 40 bytes).
    //
    // Layout from unitree_sdk2 include/unitree/dds_wrapper/common/unitree_joystick.hpp
    // (REMOTE_DATA_RX / BtnDataStruct / BtnUnion), the same struct the SDK examples memcpy
    // wireless_remote into (example/state_machine/robot_controller.hpp,
    // example/g1/low_level/g1_ankle_swing_example.cpp):
    //   bytes 0-1  head[2]
    //   bytes 2-3  btn (uint16 little endian, bit fields from bit 0):
    //              R1, L1, Start, Select, R2, L2, f1, f2, A, B, X, Y, up, right, down, left
    //   bytes 4-23 lx, rx, ry, L2(analog), ly (float)
    // unitree_mujoco fills the same struct (go2_pub.h: memcpy of joystick->combine()).
    // ---------------------------------------------------------------------------------------
    namespace remote
    {
        constexpr std::uint16_t R1 = 1u << 0;
        constexpr std::uint16_t L1 = 1u << 1;
        constexpr std::uint16_t START = 1u << 2;
        constexpr std::uint16_t SELECT = 1u << 3;
        constexpr std::uint16_t R2 = 1u << 4;
        constexpr std::uint16_t L2 = 1u << 5;
        constexpr std::uint16_t F1 = 1u << 6;
        constexpr std::uint16_t F2 = 1u << 7;
        constexpr std::uint16_t A = 1u << 8;
        constexpr std::uint16_t B = 1u << 9;
        constexpr std::uint16_t X = 1u << 10;
        constexpr std::uint16_t Y = 1u << 11;
        constexpr std::uint16_t UP = 1u << 12;
        constexpr std::uint16_t RIGHT = 1u << 13;
        constexpr std::uint16_t DOWN = 1u << 14;
        constexpr std::uint16_t LEFT = 1u << 15;

        /// Button bits from the raw wireless_remote bytes (needs at least 4 bytes).
        inline std::uint16_t buttons(const std::uint8_t* wireless_remote)
        {
            return static_cast<std::uint16_t>(wireless_remote[2] |
                (static_cast<std::uint16_t>(wireless_remote[3]) << 8));
        }
    }

    // ---------------------------------------------------------------------------------------

    struct SafetyParams
    {
        bool enabled = true;

        // Joint limits, joint order FR, FL, RR, RL x (hip, thigh, calf) = Unitree motor order.
        // Defaults: go2_description/xacro/const.xacro.
        JointArray torque_max{}; ///< [Nm] rated limit before scaling
        JointArray q_min{}, q_max{}; ///< [rad]
        JointArray dq_max{}; ///< [rad/s]

        double torque_limit_scale = 0.7; ///< effective torque limit = torque_max * scale
        double q_cmd_margin = 0.05; ///< [rad] q_cmd is clamped to [q_min + m, q_max - m]
        double q_monitor_tolerance = 0.05; ///< [rad] measured q may exceed the range this much
        double kp_max = 200.0;
        double kd_max = 10.0;
        double damping_kd = 3.0;

        double tau_est_ratio = 0.9; ///< of the effective torque limit
        double tau_est_duration = 0.2; ///< [s]
        double dq_ratio = 0.8; ///< of dq_max

        double temperature_warn = 70.0; ///< [deg C] (MotorState_.temperature)
        double temperature_stop = 85.0; ///< [deg C]

        double command_timeout = 0.02; ///< [s] watchdog
        double state_timeout = 0.05; ///< [s] LowState not received

        std::uint16_t estop_buttons = remote::L2 | remote::B; ///< 0 disables the remote e-stop

        static SafetyParams go2Defaults()
        {
            SafetyParams p;
            for (std::size_t leg = 0; leg < 4; ++leg)
            {
                const std::size_t h = leg * 3, t = h + 1, c = h + 2;
                p.torque_max[h] = 23.7;
                p.torque_max[t] = 23.7;
                p.torque_max[c] = 35.55;
                p.q_min[h] = -1.0472;
                p.q_max[h] = 1.0472;
                p.q_min[t] = -1.5708;
                p.q_max[t] = 3.4907;
                p.q_min[c] = -2.7227;
                p.q_max[c] = -0.83776;
                p.dq_max[h] = 30.1;
                p.dq_max[t] = 30.1;
                p.dq_max[c] = 20.06;
            }
            return p;
        }
    };

    struct JointCommand
    {
        JointArray q{}, dq{}, kp{}, kd{}, tau{};
    };

    struct JointStateSample
    {
        JointArray q{}, dq{}, tau_est{};
        std::array<double, kNumJoints> temperature{};
        std::uint16_t remote_buttons = 0;
    };

    enum class StopReason : std::uint8_t
    {
        NONE = 0,
        NON_FINITE_COMMAND = 1,
        NON_FINITE_STATE = 2,
        TAU_EST_SUSTAINED = 3,
        JOINT_OUT_OF_RANGE = 4,
        JOINT_VELOCITY = 5,
        MOTOR_TEMPERATURE = 6,
        COMMAND_TIMEOUT = 7,
        STATE_TIMEOUT = 8,
        REMOTE_ESTOP = 9,
    };

    inline const char* toString(const StopReason r)
    {
        switch (r)
        {
        case StopReason::NONE: return "none";
        case StopReason::NON_FINITE_COMMAND: return "non-finite command";
        case StopReason::NON_FINITE_STATE: return "non-finite joint state";
        case StopReason::TAU_EST_SUSTAINED: return "sustained high measured torque (tau_est)";
        case StopReason::JOINT_OUT_OF_RANGE: return "joint position out of range";
        case StopReason::JOINT_VELOCITY: return "joint velocity over limit";
        case StopReason::MOTOR_TEMPERATURE: return "motor temperature over stop threshold";
        case StopReason::COMMAND_TIMEOUT: return "command watchdog timeout";
        case StopReason::STATE_TIMEOUT: return "LowState timeout";
        case StopReason::REMOTE_ESTOP: return "remote controller e-stop";
        }
        return "unknown";
    }

    struct StopInfo
    {
        StopReason reason = StopReason::NONE;
        int joint = -1; ///< -1 if not joint specific
        double value = 0.0;
        double limit = 0.0;
        double time = 0.0; ///< [s] time passed to the call that tripped
    };

    class LowLevelSafety
    {
    public:
        explicit LowLevelSafety(const SafetyParams& params = SafetyParams::go2Defaults())
        {
            configure(params);
        }

        void configure(const SafetyParams& params)
        {
            p_ = params;
            for (std::size_t i = 0; i < kNumJoints; ++i)
            {
                torque_limit_[i] = p_.torque_max[i] * p_.torque_limit_scale;
            }
            reset();
        }

        /// Clear the latch and all timers (only for tests / a full restart).
        void reset()
        {
            stopped_ = false;
            stop_ = StopInfo{};
            stop_reported_ = false;
            have_state_ = false;
            have_command_ = false;
            last_state_time_ = 0.0;
            last_command_time_ = 0.0;
            tau_over_since_.fill(-1.0);
            temp_warned_.fill(false);
            temp_warn_reported_.fill(false);
            torque_clamp_count_ = 0;
            state_ = JointStateSample{};
        }

        /// New LowState sample received at `stamp` [s]; called from read().
        /// `now` [s] is the current time on the same clock.
        void updateState(const JointStateSample& s, const double stamp, const double now)
        {
            if (!p_.enabled) return;
            state_ = s;
            last_state_time_ = stamp;
            have_state_ = true;
            if (stopped_) return;

            for (std::size_t i = 0; i < kNumJoints; ++i)
            {
                if (!std::isfinite(s.q[i]) || !std::isfinite(s.dq[i]) || !std::isfinite(s.tau_est[i]))
                {
                    trip(StopReason::NON_FINITE_STATE, static_cast<int>(i), NAN, 0.0, now);
                    return;
                }
            }

            // 10. remote controller e-stop
            if (p_.estop_buttons != 0 && (s.remote_buttons & p_.estop_buttons) == p_.estop_buttons)
            {
                trip(StopReason::REMOTE_ESTOP, -1, s.remote_buttons, p_.estop_buttons, now);
                return;
            }

            for (std::size_t i = 0; i < kNumJoints; ++i)
            {
                const int j = static_cast<int>(i);
                // 6a. joint range
                if (s.q[i] < p_.q_min[i] - p_.q_monitor_tolerance)
                {
                    trip(StopReason::JOINT_OUT_OF_RANGE, j, s.q[i], p_.q_min[i] - p_.q_monitor_tolerance, now);
                    return;
                }
                if (s.q[i] > p_.q_max[i] + p_.q_monitor_tolerance)
                {
                    trip(StopReason::JOINT_OUT_OF_RANGE, j, s.q[i], p_.q_max[i] + p_.q_monitor_tolerance, now);
                    return;
                }
                // 6b. joint velocity
                if (const double dq_lim = p_.dq_ratio * p_.dq_max[i]; std::fabs(s.dq[i]) > dq_lim)
                {
                    trip(StopReason::JOINT_VELOCITY, j, s.dq[i], dq_lim, now);
                    return;
                }
                // 7. temperature (two stages)
                if (s.temperature[i] >= p_.temperature_stop)
                {
                    trip(StopReason::MOTOR_TEMPERATURE, j, s.temperature[i], p_.temperature_stop, now);
                    return;
                }
                if (s.temperature[i] >= p_.temperature_warn)
                {
                    temp_warned_[i] = true;
                }
                // 5. sustained measured torque
                if (const double tau_lim = p_.tau_est_ratio * torque_limit_[i]; std::fabs(s.tau_est[i]) > tau_lim)
                {
                    if (tau_over_since_[i] < 0.0)
                    {
                        tau_over_since_[i] = now;
                    }
                    else if (now - tau_over_since_[i] >= p_.tau_est_duration)
                    {
                        trip(StopReason::TAU_EST_SUSTAINED, j, s.tau_est[i], tau_lim, now);
                        return;
                    }
                }
                else
                {
                    tau_over_since_[i] = -1.0;
                }
            }
        }

        /// Limit the command in place; called from write() every cycle.
        /// `fresh`: a controller wrote at least one command interface in this cycle.
        void filterCommand(JointCommand& cmd, const bool fresh, const double now)
        {
            if (!p_.enabled) return;

            // 9. LowState timeout (armed after the first sample)
            if (!stopped_ && have_state_ && now - last_state_time_ > p_.state_timeout)
            {
                trip(StopReason::STATE_TIMEOUT, -1, now - last_state_time_, p_.state_timeout, now);
            }
            // 8. command watchdog (armed after the first command)
            if (fresh)
            {
                have_command_ = true;
                last_command_time_ = now;
            }
            else if (!stopped_ && have_command_ && now - last_command_time_ > p_.command_timeout)
            {
                trip(StopReason::COMMAND_TIMEOUT, -1, now - last_command_time_, p_.command_timeout, now);
            }

            // 1. non-finite command
            if (!stopped_)
            {
                for (std::size_t i = 0; i < kNumJoints; ++i)
                {
                    const double v[5] = {cmd.q[i], cmd.dq[i], cmd.kp[i], cmd.kd[i], cmd.tau[i]};
                    for (const double x : v)
                    {
                        if (!std::isfinite(x))
                        {
                            trip(StopReason::NON_FINITE_COMMAND, static_cast<int>(i), x, 0.0, now);
                            break;
                        }
                    }
                    if (stopped_) break;
                }
            }

            // No state yet: the controller computed on zeros. Damp without latching.
            if (stopped_ || !have_state_)
            {
                setDamping(cmd);
                return;
            }

            for (std::size_t i = 0; i < kNumJoints; ++i)
            {
                // 3. gains
                cmd.kp[i] = clamp(cmd.kp[i], 0.0, p_.kp_max);
                cmd.kd[i] = clamp(cmd.kd[i], 0.0, p_.kd_max);
                // 4. position command inside the range
                cmd.q[i] = clamp(cmd.q[i], p_.q_min[i] + p_.q_cmd_margin, p_.q_max[i] - p_.q_cmd_margin);
                // 2. total motor torque tau + kp (q_cmd - q) + kd (dq_cmd - dq), limited by changing tau
                const double pd = cmd.kp[i] * (cmd.q[i] - state_.q[i]) + cmd.kd[i] * (cmd.dq[i] - state_.dq[i]);
                const double total = cmd.tau[i] + pd;
                const double lim = torque_limit_[i];
                if (total > lim)
                {
                    cmd.tau[i] = lim - pd;
                    ++torque_clamp_count_;
                }
                else if (total < -lim)
                {
                    cmd.tau[i] = -lim - pd;
                    ++torque_clamp_count_;
                }
            }
        }

        /// Total motor torque the command would produce with the latest measured state.
        [[nodiscard]] double totalTorque(const JointCommand& cmd, const std::size_t i) const
        {
            return cmd.tau[i] + cmd.kp[i] * (cmd.q[i] - state_.q[i]) + cmd.kd[i] * (cmd.dq[i] - state_.dq[i]);
        }

        void setDamping(JointCommand& cmd) const
        {
            cmd.q.fill(0.0);
            cmd.dq.fill(0.0);
            cmd.kp.fill(0.0);
            cmd.tau.fill(0.0);
            cmd.kd.fill(p_.damping_kd);
        }

        [[nodiscard]] bool stopped() const { return stopped_; }
        [[nodiscard]] const StopInfo& stopInfo() const { return stop_; }
        [[nodiscard]] bool hasState() const { return have_state_; }
        [[nodiscard]] const SafetyParams& params() const { return p_; }
        [[nodiscard]] double torqueLimit(const std::size_t i) const { return torque_limit_[i]; }
        [[nodiscard]] std::uint64_t torqueClampCount() const { return torque_clamp_count_; }

        /// True exactly once after a stop, so the caller can log the reason once.
        bool takeStopEvent()
        {
            if (stopped_ && !stop_reported_)
            {
                stop_reported_ = true;
                return true;
            }
            return false;
        }

        /// Joint index whose temperature crossed the warning level (once per joint), or -1.
        int takeTemperatureWarning()
        {
            for (std::size_t i = 0; i < kNumJoints; ++i)
            {
                if (temp_warned_[i] && !temp_warn_reported_[i])
                {
                    temp_warn_reported_[i] = true;
                    return static_cast<int>(i);
                }
            }
            return -1;
        }

        /// Human-readable stop description (no allocation).
        void describeStop(char* buf, const std::size_t size) const
        {
            if (stop_.joint >= 0)
            {
                std::snprintf(buf, size, "%s: joint %d (%s) value %.4f, limit %.4f",
                              toString(stop_.reason), stop_.joint, jointName(stop_.joint), stop_.value,
                              stop_.limit);
            }
            else
            {
                std::snprintf(buf, size, "%s: value %.4f, limit %.4f", toString(stop_.reason), stop_.value,
                              stop_.limit);
            }
        }

        static const char* jointName(const int i)
        {
            static constexpr const char* names[kNumJoints] = {
                "FR_hip", "FR_thigh", "FR_calf", "FL_hip", "FL_thigh", "FL_calf",
                "RR_hip", "RR_thigh", "RR_calf", "RL_hip", "RL_thigh", "RL_calf"
            };
            return (i >= 0 && i < static_cast<int>(kNumJoints)) ? names[i] : "?";
        }

    private:
        static double clamp(const double v, const double lo, const double hi)
        {
            return v < lo ? lo : (v > hi ? hi : v);
        }

        void trip(const StopReason reason, const int joint, const double value, const double limit,
                  const double now)
        {
            if (stopped_) return;
            stopped_ = true;
            stop_ = StopInfo{reason, joint, value, limit, now};
        }

        SafetyParams p_;
        JointArray torque_limit_{};

        bool stopped_ = false;
        bool stop_reported_ = false;
        StopInfo stop_;

        JointStateSample state_;
        bool have_state_ = false;
        double last_state_time_ = 0.0;

        bool have_command_ = false;
        double last_command_time_ = 0.0;

        JointArray tau_over_since_{};
        std::array<bool, kNumJoints> temp_warned_{};
        std::array<bool, kNumJoints> temp_warn_reported_{};
        std::uint64_t torque_clamp_count_ = 0;
    };
} // namespace hardware_unitree_mujoco

#endif // HARDWARE_UNITREE_MUJOCO_LOWLEVELSAFETY_H
