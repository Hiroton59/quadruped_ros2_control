//
// Lie down from OCS2: move the joints from the current pose to down_pos with joint PD, then PASSIVE.
//

#include "ocs2_quadruped_controller/FSM/StateLieDown.h"

#include <algorithm>
#include <cmath>
#include <utility>

namespace ocs2::legged_robot
{
    StateLieDown::StateLieDown(CtrlInterfaces& ctrl_interfaces, Params params)
        : FSMState(FSMStateName::LIEDOWN, "lie down", ctrl_interfaces),
          params_(std::move(params))
    {
    }

    void StateLieDown::enter()
    {
        for (int i = 0; i < 12; i++)
        {
            start_pos_[i] = ctrl_interfaces_.joint_position_state_interface_[i].get().get_value();
            // The last OCS2 (WBC) torque holds the body. Fade it out while the PD takes over,
            // so the body does not drop in the first cycles (the PD error starts at zero).
            start_torque_[i] = ctrl_interfaces_.joint_torque_command_interface_[i].get().get_value();
        }
        elapsed_ = 0.0;
        // Write a command in this cycle (hardware command watchdog).
        writeCommand();
    }

    void StateLieDown::run(const rclcpp::Time& /*time*/, const rclcpp::Duration& period)
    {
        elapsed_ += period.seconds();
        writeCommand();
    }

    void StateLieDown::writeCommand()
    {
        // Smooth (cosine) interpolation: zero velocity at both ends.
        const double s = std::clamp(elapsed_ / params_.duration, 0.0, 1.0);
        const double phase = 0.5 * (1.0 - std::cos(M_PI * s));
        const double phase_rate = s < 1.0 ? 0.5 * M_PI / params_.duration * std::sin(M_PI * s) : 0.0;
        for (int i = 0; i < 12; i++)
        {
            const double diff = params_.down_pos[i] - start_pos_[i];
            ctrl_interfaces_.joint_torque_command_interface_[i].get().set_value((1.0 - phase) * start_torque_[i]);
            ctrl_interfaces_.joint_position_command_interface_[i].get().set_value(start_pos_[i] + phase * diff);
            ctrl_interfaces_.joint_velocity_command_interface_[i].get().set_value(phase_rate * diff);
            ctrl_interfaces_.joint_kp_command_interface_[i].get().set_value(params_.kp);
            ctrl_interfaces_.joint_kd_command_interface_[i].get().set_value(params_.kd);
        }
    }

    void StateLieDown::exit()
    {
    }

    FSMStateName StateLieDown::checkChange()
    {
        if (ctrl_interfaces_.control_inputs_.command == 1)
        {
            return FSMStateName::PASSIVE;
        }
        if (elapsed_ >= params_.duration + params_.hold)
        {
            return FSMStateName::PASSIVE;
        }
        return FSMStateName::LIEDOWN;
    }
}
