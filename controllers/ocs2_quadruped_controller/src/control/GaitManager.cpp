//
// Created by tlab-uav on 24-9-26.
//

#include <utility>

#include "ocs2_quadruped_controller/control/GaitManager.h"

#include <ocs2_core/misc/LoadData.h>
#include <rclcpp/logging.hpp>

namespace ocs2::legged_robot
{
    GaitManager::GaitManager(CtrlInterfaces& ctrl_interfaces,
                             std::shared_ptr<GaitSchedule> gait_schedule_ptr,
                             rclcpp_lifecycle::LifecycleNode::SharedPtr node)
        : ctrl_interfaces_(ctrl_interfaces),
          gait_schedule_ptr_(std::move(gait_schedule_ptr)),
          node_(std::move(node)),
          target_gait_({0.0, 1.0}, {STANCE})
    {
        // The lie down command (Ocs2QuadrupedController) is not a gait
        if (node_->has_parameter("lie_down.command"))
        {
            lie_down_command_ = static_cast<int>(node_->get_parameter("lie_down.command").as_int());
        }
        // 歩行周期 [s]．stance 以外の gait をこの周期に伸縮する（0 以下で gait.info のまま）
        period_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
            "/gait_period", rclcpp::QoS(1).reliable().transient_local(),
            [this](const std_msgs::msg::Float64::SharedPtr msg)
            {
                if (msg->data > 0.0 && (msg->data < 0.2 || msg->data > 1.5))
                {
                    RCLCPP_WARN(rclcpp::get_logger("GaitManager"),
                                "gait period %.3f s is out of range [0.2, 1.5], ignored", msg->data);
                    return;
                }
                gait_period_ = msg->data;
                period_updated_ = true;
                RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Gait period set to %.3f s", msg->data);
            });
    }

    void GaitManager::preSolverRun(const scalar_t initTime, const scalar_t finalTime,
                                   const vector_t& /**currentState**/,
                                   const ReferenceManagerInterface& /**referenceManager**/)
    {
        getTargetGait();
        if (gait_updated_)
        {
            const auto timeHorizon = finalTime - initTime;
            gait_schedule_ptr_->insertModeSequenceTemplate(target_gait_, finalTime,
                                                           timeHorizon);
            gait_updated_ = false;
        }
    }

    void GaitManager::init(const std::string& gait_file)
    {
        gait_name_list_.clear();
        loadData::loadStdVector(gait_file, "list", gait_name_list_, verbose_);

        gait_list_.clear();
        for (const auto& name : gait_name_list_)
        {
            gait_list_.push_back(loadModeSequenceTemplate(gait_file, name, verbose_));
        }

        RCLCPP_INFO(rclcpp::get_logger("gait_manager"), "GaitManager is ready.");
    }

    ModeSequenceTemplate GaitManager::scaledGait(const size_t index) const
    {
        ModeSequenceTemplate gait = gait_list_[index];
        const double period = gait_period_;
        if (period <= 0.0 || gait_name_list_[index] == "stance") return gait;

        const double scale = period / gait.switchingTimes.back();
        for (auto& t : gait.switchingTimes) t *= scale;
        return gait;
    }

    void GaitManager::getTargetGait()
    {
        bool changed = false;

        const int command = ctrl_interfaces_.control_inputs_.command;
        if (command != 0 && command != lie_down_command_ && command != last_command_)
        {
            last_command_ = command;
            const size_t index = std::max(0, command - 2);
            if (index < gait_list_.size())
            {
                current_gait_index_ = index;
                changed = true;
            }
            else
            {
                RCLCPP_WARN(rclcpp::get_logger("GaitManager"), "No gait for command %d", command);
            }
        }
        if (period_updated_.exchange(false)) changed = true;
        if (!changed) return;

        target_gait_ = scaledGait(current_gait_index_);
        RCLCPP_INFO(rclcpp::get_logger("GaitManager"), "Switch to gait: %s (period %.3f s)",
                    gait_name_list_[current_gait_index_].c_str(), target_gait_.switchingTimes.back());
        gait_updated_ = true;
    }
}
