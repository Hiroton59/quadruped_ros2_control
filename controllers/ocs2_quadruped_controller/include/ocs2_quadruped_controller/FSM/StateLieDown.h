//
// Lie down from OCS2: move the joints from the current pose to down_pos with joint PD, then PASSIVE.
//

#ifndef STATELIEDOWN_H
#define STATELIEDOWN_H

#include <array>
#include <vector>

#include "controller_common/FSM/FSMState.h"

namespace ocs2::legged_robot
{
    class StateLieDown final : public FSMState
    {
    public:
        struct Params
        {
            std::vector<double> down_pos; ///< [rad] 12 joints, same order as the controller's joints
            double kp = 80.0;             ///< same as unitree_guide stand_kp
            double kd = 3.5;              ///< same as unitree_guide stand_kd
            double duration = 1.8;        ///< [s] from the current pose to down_pos
            double hold = 0.2;            ///< [s] keep down_pos before PASSIVE
        };

        StateLieDown(CtrlInterfaces& ctrl_interfaces, Params params);

        void enter() override;

        void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;

        void exit() override;

        /// command 1: PASSIVE at once. Otherwise PASSIVE after duration + hold.
        FSMStateName checkChange() override;

    private:
        void writeCommand();

        Params params_;
        std::array<double, 12> start_pos_{};
        std::array<double, 12> start_torque_{};
        double elapsed_ = 0.0;
    };
}

#endif //STATELIEDOWN_H
