//
// Created by qiayuan on 2022/7/26.
//

#pragma once

#include <cmath>
#include <iostream>

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_mpc/SystemObservation.h>

namespace ocs2::legged_robot {
    struct SafetyCheckerParams {
        double max_roll = 45.0 * M_PI / 180.0; ///< [rad] |roll| above this fails
        double max_pitch = 45.0 * M_PI / 180.0; ///< [rad] |pitch| above this fails
        double min_base_height = 0.15; ///< [m] estimated base z below this fails ...
        /// ... once the base has been above this height since entering the OCS2 state.
        /// The OCS2 state is entered lying down (z ~ 0.1 m) and stands up by itself.
        double base_height_arm = 0.20;
    };

    /// Early failure detection in the OCS2 state. A failed check makes StateOCS2 switch to PASSIVE.
    /// The hardware layer (hardware_unitree_mujoco LowLevelSafety) is the last line of defence;
    /// this only catches a fall earlier.
    class SafetyChecker {
    public:
        using Params = SafetyCheckerParams;

        explicit SafetyChecker(const CentroidalModelInfo &info, const Params &params = Params())
            : info_(info), params_(params) {
        }

        /// Call when the OCS2 state is entered.
        void reset() { height_armed_ = false; }

        [[nodiscard]] bool check(const SystemObservation &observation, const vector_t & /*optimized_state*/,
                                 const vector_t & /*optimized_input*/) {
            // Base pose = [x, y, z, yaw, pitch, roll] (ZYX Euler angles)
            const vector_t pose = centroidal_model::getBasePose(observation.state, info_);
            return checkOrientation(pose) && checkHeight(pose);
        }

        [[nodiscard]] const Params &params() const { return params_; }

    protected:
        [[nodiscard]] bool checkOrientation(const vector_t &pose) const {
            const double pitch = pose(4);
            const double roll = pose(5);
            if (std::abs(roll) > params_.max_roll || std::abs(pitch) > params_.max_pitch) {
                std::cerr << "[SafetyChecker] Orientation safety check failed: roll " << roll * 180.0 / M_PI
                        << " deg, pitch " << pitch * 180.0 / M_PI << " deg (limits "
                        << params_.max_roll * 180.0 / M_PI << ", " << params_.max_pitch * 180.0 / M_PI << " deg)"
                        << std::endl;
                return false;
            }
            return true;
        }

        [[nodiscard]] bool checkHeight(const vector_t &pose) {
            const double z = pose(2);
            if (!height_armed_) {
                height_armed_ = z > params_.base_height_arm;
                return true;
            }
            if (z < params_.min_base_height) {
                std::cerr << "[SafetyChecker] Base height safety check failed: z " << z << " m < "
                        << params_.min_base_height << " m" << std::endl;
                return false;
            }
            return true;
        }

        const CentroidalModelInfo &info_;
        Params params_;
        bool height_armed_ = false;
    };
} // namespace legged
