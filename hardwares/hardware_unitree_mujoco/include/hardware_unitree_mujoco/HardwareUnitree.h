//
// Created by biao on 24-9-9.
//


#ifndef HARDWAREUNITREE_H
#define HARDWAREUNITREE_H

#include <array>
#include <cstdint>
#include <mutex>

#include "hardware_interface/system_interface.hpp"
#include "hardware_unitree_mujoco/LowLevelSafety.h"
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

class HardwareUnitree final : public hardware_interface::SystemInterface {
public:
    CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

    hardware_interface::return_type write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) override;

protected:
    // Command interface buffers. Reset to the watchdog sentinel after every write()
    // (see LowLevelSafety.h, takeCommand()).
    hardware_unitree_mujoco::JointArray joint_torque_command_{};
    hardware_unitree_mujoco::JointArray joint_position_command_{};
    hardware_unitree_mujoco::JointArray joint_velocities_command_{};
    hardware_unitree_mujoco::JointArray joint_kp_command_{};
    hardware_unitree_mujoco::JointArray joint_kd_command_{};

    // Last values written by a controller (held while a controller skips a cycle).
    hardware_unitree_mujoco::JointCommand held_command_{};

    std::vector<double> joint_position_;
    std::vector<double> joint_velocities_;
    std::vector<double> joint_effort_;

    std::vector<double> imu_states_;
    std::vector<double> foot_force_;
    std::vector<double> high_states_;

    std::unordered_map<std::string, std::vector<std::string> > joint_interfaces = {
        {"position", {}},
        {"velocity", {}},
        {"effort", {}}
    };


    void initLowCmd();

    bool loadSafetyParams(const hardware_interface::HardwareInfo& info);

    void reportSafetyEvents();

    static double steadyNow();

    void lowStateMessageHandle(const void *messages);

    void highStateMessageHandle(const void *messages);

    void remoteWirelessHandle(const void *messages);

    unitree_go::msg::dds_::LowCmd_ low_cmd_{}; // default init
    // low_state_ / high_state_ are the control thread's snapshots. The DDS receive threads
    // write the *_rx_ copies under state_mutex_, read() copies them out under the same mutex.
    unitree_go::msg::dds_::LowState_ low_state_{}; // default init
    unitree_go::msg::dds_::SportModeState_ high_state_{}; // default init
    std::mutex state_mutex_;
    unitree_go::msg::dds_::LowState_ low_state_rx_{};
    unitree_go::msg::dds_::SportModeState_ high_state_rx_{};
    std::uint64_t low_state_rx_seq_ = 0; // guarded by state_mutex_
    double low_state_rx_time_ = 0.0; // [s] steady clock, guarded by state_mutex_
    std::uint64_t low_state_read_seq_ = 0; // control thread only

    // Low-level safety
    hardware_unitree_mujoco::LowLevelSafety safety_;
    double safety_stop_state_ = 0.0; // state interface safety/stop (1 = latched damping)
    double safety_reason_state_ = 0.0; // state interface safety/reason (StopReason code)
    std::uint64_t last_clamp_count_ = 0;
    double last_clamp_report_time_ = 0.0;

    std::string network_interface_ = "lo";
    int domain_ = 1;
    bool show_foot_force_ = false;

    /*publisher*/
    unitree::robot::ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_publisher_;
    /*subscriber*/
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lows_tate_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> high_state_subscriber_;
};

#endif //HARDWAREUNITREE_H
