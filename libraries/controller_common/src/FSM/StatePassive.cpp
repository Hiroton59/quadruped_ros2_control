//
// Created by tlab-uav on 25-2-27.
//

#include "controller_common/FSM/StatePassive.h"

#include <iostream>

namespace
{
    void writePassiveCommand(const CtrlInterfaces& ctrl)
    {
        for (auto i : ctrl.joint_torque_command_interface_)
        {
            i.get().set_value(0);
        }
        for (auto i : ctrl.joint_position_command_interface_)
        {
            i.get().set_value(0);
        }
        for (auto i : ctrl.joint_velocity_command_interface_)
        {
            i.get().set_value(0);
        }
        for (auto i : ctrl.joint_kp_command_interface_)
        {
            i.get().set_value(0);
        }
        for (auto i : ctrl.joint_kd_command_interface_)
        {
            i.get().set_value(1);
        }
    }
}

StatePassive::StatePassive(CtrlInterfaces& ctrl_interfaces) : FSMState(
    FSMStateName::PASSIVE, "passive", ctrl_interfaces)
{
}

void StatePassive::enter()
{
    writePassiveCommand(ctrl_interfaces_);
    ctrl_interfaces_.control_inputs_.command = 0;
}

void StatePassive::run(const rclcpp::Time&/*time*/, const rclcpp::Duration&/*period*/)
{
    // Re-write every cycle: the hardware command watchdog (hardware_unitree_mujoco LowLevelSafety)
    // treats a cycle in which no command interface was written as a missing command.
    writePassiveCommand(ctrl_interfaces_);
}

void StatePassive::exit()
{
}

FSMStateName StatePassive::checkChange()
{
    if (ctrl_interfaces_.control_inputs_.command == 2)
    {
        return FSMStateName::FIXEDDOWN;
    }
    return FSMStateName::PASSIVE;
}
