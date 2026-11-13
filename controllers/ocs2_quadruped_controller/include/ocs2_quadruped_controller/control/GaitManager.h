//
// Created by tlab-uav on 24-9-26.
//

#ifndef GAITMANAGER_H
#define GAITMANAGER_H
#include <atomic>
#include <controller_common/CtrlInterfaces.h>
#include <ocs2_legged_robot/gait/GaitSchedule.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float64.hpp>

namespace ocs2::legged_robot
{
    class GaitManager final : public SolverSynchronizedModule
    {
    public:
        GaitManager(CtrlInterfaces& ctrl_interfaces,
                    std::shared_ptr<GaitSchedule> gait_schedule_ptr,
                    rclcpp_lifecycle::LifecycleNode::SharedPtr node);

        void preSolverRun(scalar_t initTime, scalar_t finalTime,
                          const vector_t& currentState,
                          const ReferenceManagerInterface& referenceManager) override;

        void postSolverRun(const PrimalSolution&/**primalSolution**/) override
        {
        }

        void init(const std::string& gait_file);

    private:
        void getTargetGait();

        // gait_period_ > 0 のとき，stance 以外の gait の周期をその値に伸縮する
        ModeSequenceTemplate scaledGait(size_t index) const;

        CtrlInterfaces& ctrl_interfaces_;
        std::shared_ptr<GaitSchedule> gait_schedule_ptr_;
        rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
        rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr period_sub_;

        ModeSequenceTemplate target_gait_;
        int last_command_ = 0;
        size_t current_gait_index_ = 0;
        bool gait_updated_{false};
        bool verbose_{false};
        std::atomic<double> gait_period_{0.0};  // 0：gait.info のまま
        std::atomic<bool> period_updated_{false};
        std::vector<ModeSequenceTemplate> gait_list_;
        std::vector<std::string> gait_name_list_;
    };
}


#endif //GAITMANAGER_H
