//
// Created by biao on 24-9-9.
//

#include "hardware_unitree_mujoco/HardwareUnitree.h"

#include <chrono>
#include <sstream>
#include <stdexcept>

#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "crc32.h"

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::robot;
using hardware_interface::return_type;
using hardware_unitree_mujoco::JointCommand;
using hardware_unitree_mujoco::JointStateSample;
using hardware_unitree_mujoco::kNumJoints;
using hardware_unitree_mujoco::SafetyParams;
using hardware_unitree_mujoco::StopReason;

namespace
{
    const rclcpp::Logger kLogger = rclcpp::get_logger("unitree_hardware");
}

HardwareUnitree::~HardwareUnitree()
{
    stall_monitor_running_ = false;
    if (stall_monitor_.joinable())
    {
        stall_monitor_.join();
    }
}

double HardwareUnitree::steadyNow()
{
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_init(
    const hardware_interface::HardwareInfo& info)
{
    if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    {
        return CallbackReturn::ERROR;
    }

    // No controller has written yet: buffers hold the sentinel, held commands are zero.
    joint_torque_command_.fill(hardware_unitree_mujoco::commandSentinel());
    joint_position_command_.fill(hardware_unitree_mujoco::commandSentinel());
    joint_velocities_command_.fill(hardware_unitree_mujoco::commandSentinel());
    joint_kp_command_.fill(hardware_unitree_mujoco::commandSentinel());
    joint_kd_command_.fill(hardware_unitree_mujoco::commandSentinel());
    held_command_ = JointCommand{};

    joint_position_.assign(12, 0);
    joint_velocities_.assign(12, 0);
    joint_effort_.assign(12, 0);

    imu_states_.assign(10, 0);
    foot_force_.assign(4, 0);
    high_states_.assign(6, 0);

    for (const auto& joint : info_.joints)
    {
        for (const auto& interface : joint.state_interfaces)
        {
            joint_interfaces[interface.name].push_back(joint.name);
        }
    }


    if (const auto network_interface_param = info.hardware_parameters.find("network_interface"); network_interface_param
        != info.hardware_parameters.end())
    {
        network_interface_ = network_interface_param->second;
    }
    if (const auto domain_param = info.hardware_parameters.find("domain"); domain_param != info.hardware_parameters.
        end())
    {
        domain_ = std::stoi(domain_param->second);
    }
    if (const auto show_foot_force_param = info.hardware_parameters.find("show_foot_force"); show_foot_force_param !=
        info.hardware_parameters.end())
    {
        show_foot_force_ = show_foot_force_param->second == "true";
    }

    RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"), " network_interface: %s, domain: %d", network_interface_.c_str(), domain_);

    if (info_.joints.size() != kNumJoints)
    {
        RCLCPP_ERROR(kLogger, "Expected %zu joints (FR, FL, RR, RL x hip, thigh, calf), got %zu", kNumJoints,
                     info_.joints.size());
        return CallbackReturn::ERROR;
    }
    if (!loadSafetyParams(info))
    {
        return CallbackReturn::ERROR;
    }
    ChannelFactory::Instance()->Init(domain_, network_interface_);

    low_cmd_publisher_ =
        std::make_shared<ChannelPublisher<unitree_go::msg::dds_::LowCmd_>>(
            TOPIC_LOWCMD);
    low_cmd_publisher_->InitChannel();

    lows_tate_subscriber_ =
        std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::LowState_>>(
            TOPIC_LOWSTATE);
    lows_tate_subscriber_->InitChannel(
        [this](auto&& PH1)
        {
            lowStateMessageHandle(std::forward<decltype(PH1)>(PH1));
        },
        1);
    initLowCmd();

    high_state_subscriber_ =
        std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(
            TOPIC_HIGHSTATE);
    high_state_subscriber_->InitChannel(
        [this](auto&& PH1)
        {
            highStateMessageHandle(std::forward<decltype(PH1)>(PH1));
        },
        1);


    return SystemInterface::on_init(info);
}

std::vector<hardware_interface::StateInterface> HardwareUnitree::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        state_interfaces.emplace_back(joint_name, "position", &joint_position_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        state_interfaces.emplace_back(joint_name, "effort", &joint_effort_[ind++]);
    }

    // low-level safety state (also published by joint_state_broadcaster on /dynamic_joint_states)
    state_interfaces.emplace_back("safety", "stop", &safety_stop_state_);
    state_interfaces.emplace_back("safety", "reason", &safety_reason_state_);
    state_interfaces.emplace_back("safety", "lowstate_crc_errors", &lowstate_crc_errors_state_);
    // cumulative number of cycles in which the total torque limit changed tau, per joint
    for (size_t i = 0; i < kNumJoints; ++i)
    {
        state_interfaces.emplace_back(
            "safety", std::string("torque_limit_count_") + hardware_unitree_mujoco::LowLevelSafety::jointName(
                static_cast<int>(i)), &torque_limit_count_state_[i]);
    }

    // export imu sensor state interface
    for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
    {
        state_interfaces.emplace_back(
            info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_states_[i]);
    }

    // export foot force sensor state interface
    if (info_.sensors.size() > 1)
    {
        for (uint i = 0; i < info_.sensors[1].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[1].name, info_.sensors[1].state_interfaces[i].name, &foot_force_[i]);
        }
    }

    // export odometer state interface
    if (info_.sensors.size() > 2)
    {
        // export high state interface
        for (uint i = 0; i < info_.sensors[2].state_interfaces.size(); i++)
        {
            state_interfaces.emplace_back(
                info_.sensors[2].name, info_.sensors[2].state_interfaces[i].name, &high_states_[i]);
        }
    }


    return
        state_interfaces;
}

std::vector<hardware_interface::CommandInterface> HardwareUnitree::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    int ind = 0;
    for (const auto& joint_name : joint_interfaces["position"])
    {
        command_interfaces.emplace_back(joint_name, "position", &joint_position_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["velocity"])
    {
        command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind++]);
    }

    ind = 0;
    for (const auto& joint_name : joint_interfaces["effort"])
    {
        command_interfaces.emplace_back(joint_name, "effort", &joint_torque_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kp", &joint_kp_command_[ind]);
        command_interfaces.emplace_back(joint_name, "kd", &joint_kd_command_[ind]);
        ind++;
    }
    return command_interfaces;
}

return_type HardwareUnitree::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    bool new_sample = false;
    double sample_time = 0.0;
    unsigned crc_consecutive = 0;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        crc_consecutive = low_state_crc_consecutive_;
        lowstate_crc_errors_state_ = static_cast<double>(low_state_crc_total_);
        if (low_state_rx_seq_ != low_state_read_seq_)
        {
            low_state_ = low_state_rx_;
            low_state_read_seq_ = low_state_rx_seq_;
            sample_time = low_state_rx_time_;
            new_sample = true;
        }
        high_state_ = high_state_rx_;
    }

    if (new_sample)
    {
        JointStateSample sample;
        for (size_t i = 0; i < kNumJoints; ++i)
        {
            const auto& m = low_state_.motor_state()[i];
            sample.q[i] = m.q();
            sample.dq[i] = m.dq();
            sample.tau_est[i] = m.tau_est();
            sample.temperature[i] = m.temperature();
        }
        sample.remote_buttons = hardware_unitree_mujoco::remote::buttons(low_state_.wireless_remote().data());
        safety_.updateState(sample, sample_time, steadyNow());
    }
    safety_.updateCrcFailures(crc_consecutive, steadyNow());

    // joint states
    for (int i(0); i < 12; ++i)
    {
        joint_position_[i] = low_state_.motor_state()[i].q();
        joint_velocities_[i] = low_state_.motor_state()[i].dq();
        joint_effort_[i] = low_state_.motor_state()[i].tau_est();
    }

    // imu states
    imu_states_[0] = low_state_.imu_state().quaternion()[0]; // w
    imu_states_[1] = low_state_.imu_state().quaternion()[1]; // x
    imu_states_[2] = low_state_.imu_state().quaternion()[2]; // y
    imu_states_[3] = low_state_.imu_state().quaternion()[3]; // z
    imu_states_[4] = low_state_.imu_state().gyroscope()[0];
    imu_states_[5] = low_state_.imu_state().gyroscope()[1];
    imu_states_[6] = low_state_.imu_state().gyroscope()[2];
    imu_states_[7] = low_state_.imu_state().accelerometer()[0];
    imu_states_[8] = low_state_.imu_state().accelerometer()[1];
    imu_states_[9] = low_state_.imu_state().accelerometer()[2];

    // contact states
    foot_force_[0] = low_state_.foot_force()[0];
    foot_force_[1] = low_state_.foot_force()[1];
    foot_force_[2] = low_state_.foot_force()[2];
    foot_force_[3] = low_state_.foot_force()[3];

    if (show_foot_force_)
    {
        RCLCPP_INFO(rclcpp::get_logger("unitree_hardware"), "foot_force(): %f, %f, %f, %f", foot_force_[0], foot_force_[1], foot_force_[2],
                    foot_force_[3]);
    }

    // high states
    high_states_[0] = high_state_.position()[0];
    high_states_[1] = high_state_.position()[1];
    high_states_[2] = high_state_.position()[2];
    high_states_[3] = high_state_.velocity()[0];
    high_states_[4] = high_state_.velocity()[1];
    high_states_[5] = high_state_.velocity()[2];

    // RCLCPP_INFO(get_logger(), "high state: %f %f %f %f %f %f", high_states_[0], high_states_[1], high_states_[2],
    //             high_states_[3], high_states_[4], high_states_[5]);

    return return_type::OK;
}

return_type HardwareUnitree::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
    using hardware_unitree_mujoco::takeCommand;
    // Values written by a controller in this cycle (non-sentinel) replace the held ones.
    // Bitwise | so that every buffer is taken and reset.
    const bool fresh = takeCommand(joint_position_command_, held_command_.q) |
        takeCommand(joint_velocities_command_, held_command_.dq) |
        takeCommand(joint_kp_command_, held_command_.kp) |
        takeCommand(joint_kd_command_, held_command_.kd) |
        takeCommand(joint_torque_command_, held_command_.tau);

    JointCommand cmd = held_command_;
    const double now = steadyNow();
    {
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        last_write_time_ = now;
        if (loop_stalled_)
        {
            safety_.externalStop(StopReason::LOOP_STALL, loop_stall_duration_, safety_.params().loop_stall_timeout,
                                 now);
        }
        safety_.filterCommand(cmd, fresh, now);
        publishLowCmd(cmd);
    }
    safety_stop_state_ = safety_.stopped() ? 1.0 : 0.0;
    safety_reason_state_ = static_cast<double>(safety_.stopInfo().reason);
    for (size_t i = 0; i < kNumJoints; ++i)
    {
        torque_limit_count_state_[i] = static_cast<double>(safety_.torqueClampCount(i));
    }
    reportSafetyEvents();
    return return_type::OK;
}

void HardwareUnitree::publishLowCmd(const JointCommand& cmd)
{
    for (size_t i = 0; i < kNumJoints; ++i)
    {
        low_cmd_.motor_cmd()[i].mode() = 0x01;
        low_cmd_.motor_cmd()[i].q() = static_cast<float>(cmd.q[i]);
        low_cmd_.motor_cmd()[i].dq() = static_cast<float>(cmd.dq[i]);
        low_cmd_.motor_cmd()[i].kp() = static_cast<float>(cmd.kp[i]);
        low_cmd_.motor_cmd()[i].kd() = static_cast<float>(cmd.kd[i]);
        low_cmd_.motor_cmd()[i].tau() = static_cast<float>(cmd.tau[i]);
    }

    low_cmd_.crc() = crc32_core(reinterpret_cast<uint32_t*>(&low_cmd_),
                                (sizeof(unitree_go::msg::dds_::LowCmd_) >> 2) - 1);
    low_cmd_publisher_->Write(low_cmd_);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    if (safety_.params().enabled && !stall_monitor_.joinable())
    {
        last_write_time_ = 0.0; // armed by the first write()
        stall_monitor_running_ = true;
        stall_monitor_ = std::thread(&HardwareUnitree::stallMonitorLoop, this);
    }
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    publishDampingOnExit("deactivate");
    return CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn HardwareUnitree::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    publishDampingOnExit("shutdown");
    return CallbackReturn::SUCCESS;
}

void HardwareUnitree::publishDampingOnExit(const char* reason)
{
    if (!safety_.params().enabled || !low_cmd_publisher_)
    {
        return;
    }
    // The stall monitor would not get the chance: the process may exit right after this.
    stall_monitor_running_ = false;
    if (stall_monitor_.joinable())
    {
        stall_monitor_.join();
    }
    std::lock_guard<std::mutex> lock(cmd_mutex_);
    JointCommand damping;
    safety_.setDamping(damping);
    // Publish a few times in case one sample is lost.
    for (int k = 0; k < 5; ++k)
    {
        publishLowCmd(damping);
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    RCLCPP_WARN(kLogger, "[LowLevelSafety] hardware %s: published damping (kd=%.2f)", reason,
                safety_.params().damping_kd);
}

void HardwareUnitree::stallMonitorLoop()
{
    // safety_.params() is constant after on_init, so reading it here is safe.
    const double timeout = safety_.params().loop_stall_timeout;
    while (stall_monitor_running_)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        if (last_write_time_ <= 0.0 || steadyNow() - last_write_time_ <= timeout)
        {
            continue;
        }
        std::lock_guard<std::mutex> lock(cmd_mutex_);
        const double gap = steadyNow() - last_write_time_;
        if (gap <= timeout)
        {
            continue; // write() ran meanwhile
        }
        if (!loop_stalled_)
        {
            loop_stall_duration_ = gap;
            loop_stalled_ = true;
            RCLCPP_ERROR(kLogger,
                         "[LowLevelSafety] SAFETY STOP (latched): control loop stall, write() not called for %.3f s "
                         "(limit %.3f s). Publishing damping from the monitor thread. "
                         "(Expected when ros2_control_node is shutting down.)", gap, timeout);
        }
        JointCommand damping;
        safety_.setDamping(damping);
        publishLowCmd(damping);
    }
}

void HardwareUnitree::initLowCmd()
{
    low_cmd_.head()[0] = 0xFE;
    low_cmd_.head()[1] = 0xEF;
    low_cmd_.level_flag() = 0xFF;
    low_cmd_.gpio() = 0;

    for (int i = 0; i < 20; i++)
    {
        low_cmd_.motor_cmd()[i].mode() =
            0x01; // motor switch to servo (PMSM) mode
        low_cmd_.motor_cmd()[i].q() = 0;
        low_cmd_.motor_cmd()[i].kp() = 0;
        low_cmd_.motor_cmd()[i].dq() = 0;
        low_cmd_.motor_cmd()[i].kd() = 0;
        low_cmd_.motor_cmd()[i].tau() = 0;
    }
}

void HardwareUnitree::lowStateMessageHandle(const void* messages)
{
    // DDS receive thread
    const double now = steadyNow();
    const auto* msg = static_cast<const unitree_go::msg::dds_::LowState_*>(messages);
    // Same CRC as the sender (unitree_sdk2 dds_wrapper go2_pub.h, and LowCmd in write()):
    // crc32 over the message words except the trailing crc field.
    const bool crc_ok = !lowstate_crc_check_ ||
        crc32_core(reinterpret_cast<const uint32_t*>(msg), (sizeof(unitree_go::msg::dds_::LowState_) >> 2) - 1) ==
        msg->crc();
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!crc_ok)
    {
        ++low_state_crc_consecutive_;
        ++low_state_crc_total_;
        return; // not used
    }
    low_state_crc_consecutive_ = 0;
    low_state_rx_ = *msg;
    low_state_rx_time_ = now;
    ++low_state_rx_seq_;
}

void HardwareUnitree::highStateMessageHandle(const void* messages)
{
    // DDS receive thread
    std::lock_guard<std::mutex> lock(state_mutex_);
    high_state_rx_ = *static_cast<const unitree_go::msg::dds_::SportModeState_*>(messages);
}

void HardwareUnitree::reportSafetyEvents()
{
    if (safety_.takeStopEvent())
    {
        char buf[256];
        safety_.describeStop(buf, sizeof(buf));
        RCLCPP_ERROR(kLogger, "[LowLevelSafety] SAFETY STOP (latched, damping kd=%.2f; restart the node to clear): %s",
                     safety_.params().damping_kd, buf);
    }
    if (const int j = safety_.takeTemperatureWarning(); j >= 0)
    {
        RCLCPP_WARN(kLogger, "[LowLevelSafety] motor temperature warning: joint %d (%s) %d >= %.0f", j,
                    hardware_unitree_mujoco::LowLevelSafety::jointName(j),
                    static_cast<int>(low_state_.motor_state()[j].temperature()), safety_.params().temperature_warn);
    }
    // Report how often the total torque limit was active (at most every 5 s).
    if (const double now = steadyNow(); now - last_clamp_report_time_ > 5.0)
    {
        if (const auto count = safety_.torqueClampCount(); count != last_clamp_count_)
        {
            RCLCPP_INFO(kLogger, "[LowLevelSafety] total torque limit applied %lu joint-cycles in the last %.1f s",
                        static_cast<unsigned long>(count - last_clamp_count_), now - last_clamp_report_time_);
            last_clamp_count_ = count;
        }
        last_clamp_report_time_ = now;
    }
}

bool HardwareUnitree::loadSafetyParams(const hardware_interface::HardwareInfo& info)
{
    SafetyParams p = SafetyParams::go2Defaults();
    const auto& hw = info.hardware_parameters;
    std::string bad;

    auto get_double = [&](const char* name, double& value)
    {
        if (const auto it = hw.find(name); it != hw.end())
        {
            try
            {
                size_t pos = 0;
                value = std::stod(it->second, &pos);
                if (pos != it->second.size()) throw std::invalid_argument("trailing characters");
            }
            catch (const std::exception&)
            {
                bad += std::string(" ") + name + "='" + it->second + "'";
            }
        }
    };
    // Index 0 = hip, 1 = thigh, 2 = calf. "<name>" sets every leg, then "front_<name>" (FR, FL)
    // and "rear_<name>" (RR, RL) override it, e.g. rear_thigh_position_min.
    auto get_per_joint = [&](const char* name, hardware_unitree_mujoco::JointArray& array, size_t offset)
    {
        const std::string n(name);
        for (const auto& [prefix, first_leg] : {std::pair<std::string, size_t>{"", 0}, {"front_", 0}, {"rear_", 2}})
        {
            if (hw.find(prefix + n) == hw.end()) continue;
            double v = 0.0;
            get_double((prefix + n).c_str(), v);
            const size_t last_leg = prefix.empty() ? 4 : first_leg + 2;
            for (size_t leg = first_leg; leg < last_leg; ++leg) array[leg * 3 + offset] = v;
        }
    };
    auto get_unsigned = [&](const char* name, unsigned& value)
    {
        double v = value;
        get_double(name, v);
        if (v < 0.0 || v != static_cast<double>(static_cast<unsigned>(v))) bad += std::string(" ") + name;
        else value = static_cast<unsigned>(v);
    };

    if (const auto it = hw.find("safety_enabled"); it != hw.end())
    {
        p.enabled = it->second != "false" && it->second != "0";
    }
    get_double("torque_limit_scale", p.torque_limit_scale);
    get_per_joint("hip_torque_max", p.torque_max, 0);
    get_per_joint("thigh_torque_max", p.torque_max, 1);
    get_per_joint("calf_torque_max", p.torque_max, 2);
    get_per_joint("hip_position_min", p.q_min, 0);
    get_per_joint("thigh_position_min", p.q_min, 1);
    get_per_joint("calf_position_min", p.q_min, 2);
    get_per_joint("hip_position_max", p.q_max, 0);
    get_per_joint("thigh_position_max", p.q_max, 1);
    get_per_joint("calf_position_max", p.q_max, 2);
    get_per_joint("hip_velocity_max", p.dq_max, 0);
    get_per_joint("thigh_velocity_max", p.dq_max, 1);
    get_per_joint("calf_velocity_max", p.dq_max, 2);
    get_double("position_command_margin", p.q_cmd_margin);
    get_double("position_monitor_tolerance", p.q_monitor_tolerance);
    get_double("kp_max", p.kp_max);
    get_double("kd_max", p.kd_max);
    get_double("damping_kd", p.damping_kd);
    get_double("tau_est_ratio", p.tau_est_ratio);
    get_double("tau_est_duration", p.tau_est_duration);
    get_double("velocity_ratio", p.dq_ratio);
    get_double("temperature_warn", p.temperature_warn);
    // "none" (default) disables the temperature stop
    if (const auto it = hw.find("temperature_stop"); it != hw.end() && it->second != "none")
    {
        get_double("temperature_stop", p.temperature_stop);
    }
    get_double("command_timeout", p.command_timeout);
    get_double("lowstate_timeout", p.state_timeout);
    get_unsigned("lowstate_crc_max_failures", p.lowstate_crc_max_failures);
    if (const auto it = hw.find("lowstate_crc_check"); it != hw.end())
    {
        lowstate_crc_check_ = it->second != "false" && it->second != "0";
    }
    get_double("loop_stall_timeout", p.loop_stall_timeout);

    // estop_buttons: button names joined by '+', e.g. "L2+B"; "none" disables.
    if (const auto it = hw.find("estop_buttons"); it != hw.end())
    {
        namespace r = hardware_unitree_mujoco::remote;
        static const std::pair<const char*, std::uint16_t> names[] = {
            {"R1", r::R1}, {"L1", r::L1}, {"Start", r::START}, {"Select", r::SELECT}, {"R2", r::R2},
            {"L2", r::L2}, {"F1", r::F1}, {"F2", r::F2}, {"A", r::A}, {"B", r::B}, {"X", r::X},
            {"Y", r::Y}, {"up", r::UP}, {"right", r::RIGHT}, {"down", r::DOWN}, {"left", r::LEFT}
        };
        std::uint16_t mask = 0;
        if (it->second != "none")
        {
            std::stringstream ss(it->second);
            std::string token;
            while (std::getline(ss, token, '+'))
            {
                bool found = false;
                for (const auto& [n, bit] : names)
                {
                    if (token == n)
                    {
                        mask |= bit;
                        found = true;
                    }
                }
                if (!found) bad += " estop_buttons='" + it->second + "'";
            }
        }
        p.estop_buttons = mask;
    }

    // sanity checks
    if (p.torque_limit_scale <= 0.0 || p.torque_limit_scale > 1.0) bad += " torque_limit_scale (0, 1]";
    if (p.damping_kd < 0.0 || p.damping_kd > p.kd_max) bad += " damping_kd [0, kd_max]";
    if (p.command_timeout <= 0.0 || p.state_timeout <= 0.0 || p.loop_stall_timeout <= 0.0 ||
        p.tau_est_duration < 0.0)
        bad += " timeouts > 0";
    if (p.temperature_warn > p.temperature_stop) bad += " temperature_warn <= temperature_stop";
    for (size_t i = 0; i < kNumJoints; ++i)
    {
        if (p.q_min[i] + p.q_cmd_margin >= p.q_max[i] - p.q_cmd_margin) bad += " position range/margin";
        if (p.torque_max[i] <= 0.0 || p.dq_max[i] <= 0.0) bad += " torque/velocity max > 0";
    }
    if (!bad.empty())
    {
        RCLCPP_ERROR(kLogger, "[LowLevelSafety] invalid hardware parameters:%s", bad.c_str());
        return false;
    }

    safety_.configure(p);
    if (!p.enabled)
    {
        RCLCPP_WARN(kLogger, "[LowLevelSafety] DISABLED (safety_enabled=false): commands are sent unchecked");
        return true;
    }
    RCLCPP_INFO(kLogger,
                "[LowLevelSafety] torque limit (scale %.2f) hip %.2f thigh %.2f calf %.2f Nm | kp<=%.1f kd<=%.1f "
                "| q_cmd margin %.3f rad, q tolerance %.3f rad | dq <= %.0f%% | tau_est > %.0f%% for %.3f s "
                "| temp warn %.0f stop %.0f | cmd timeout %.3f s, lowstate timeout %.3f s, loop stall %.3f s "
                "| estop mask 0x%04x | damping kd %.2f | LowState CRC check %s (stop after %u)",
                p.torque_limit_scale, safety_.torqueLimit(0), safety_.torqueLimit(1), safety_.torqueLimit(2),
                p.kp_max, p.kd_max, p.q_cmd_margin, p.q_monitor_tolerance, p.dq_ratio * 100.0,
                p.tau_est_ratio * 100.0, p.tau_est_duration, p.temperature_warn, p.temperature_stop,
                p.command_timeout, p.state_timeout, p.loop_stall_timeout, p.estop_buttons, p.damping_kd,
                lowstate_crc_check_ ? "on" : "OFF", p.lowstate_crc_max_failures);
    for (size_t leg = 0; leg < 4; ++leg)
    {
        const size_t h = leg * 3;
        RCLCPP_INFO(kLogger,
                    "[LowLevelSafety] %.2s: q hip [%.4f, %.4f] thigh [%.4f, %.4f] calf [%.4f, %.4f] rad | "
                    "tau %.2f/%.2f/%.2f Nm | dq max %.2f/%.2f/%.2f rad/s",
                    hardware_unitree_mujoco::LowLevelSafety::jointName(static_cast<int>(h)),
                    p.q_min[h], p.q_max[h], p.q_min[h + 1], p.q_max[h + 1], p.q_min[h + 2], p.q_max[h + 2],
                    safety_.torqueLimit(h), safety_.torqueLimit(h + 1), safety_.torqueLimit(h + 2),
                    p.dq_max[h], p.dq_max[h + 1], p.dq_max[h + 2]);
    }
    return true;
}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    HardwareUnitree, hardware_interface::SystemInterface)
