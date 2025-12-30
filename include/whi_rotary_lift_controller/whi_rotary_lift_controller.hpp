/******************************************************************
rotary lift controller plugin under ROS 2
it is a controller resource layer for ros2_controller

Features:
- rotary lift
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-07-17: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include "whi_rotary_lift_controller/visibility_control.h"
#include "whi_rotary_lift_controller/speed_limiter.hpp"
#include <whi_interfaces/msg/whi_rotary_lift_pose_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_box.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <queue>

namespace whi_rotary_lift_controller
{
    class WhiRotaryLiftController : public controller_interface::ControllerInterface
    {
        using RotaryLiftPose = whi_interfaces::msg::WhiRotaryLiftPoseStamped;

    public:
        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        WhiRotaryLiftController();

        virtual ~WhiRotaryLiftController() = default;

    public:
        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        controller_interface::return_type update(const rclcpp::Time& Time,
            const rclcpp::Duration& Period) override;

        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State& PreState) override;

        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State& PreState) override;

        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State& PreState) override;

        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_cleanup(
            const rclcpp_lifecycle::State& PreState) override;

        WHI_ROTARY_LIFT_CONTROLLER_PUBLIC
        controller_interface::CallbackReturn on_error(
            const rclcpp_lifecycle::State& PreState) override;

    protected:
        bool reset();
        void halt();

        struct RotaryLiftHandle
        {
            std::reference_wrapper<const hardware_interface::LoanedStateInterface> position_sta_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> position_cmd_;
            std::reference_wrapper<hardware_interface::LoanedCommandInterface> velocity_cmd_;
        };
        bool configure(const std::vector<std::string>& JointNames, std::vector<RotaryLiftHandle>& RegisteredHandles);
        CallbackReturn configure(const std::vector<std::string>& RotaryNames, const std::vector<std::string>& LiftNames,
            std::vector<RotaryLiftHandle>& RegisteredHandles);

    protected:
        std::vector<RotaryLiftHandle> registered_handles_;

        std::vector<std::string> rotary_names_;
        std::vector<std::string> lift_names_;
        double rotary_position_coef_{ 1.0 };
        double rotary_velocity_coef_{ 1.0 };
        double lift_position_coef_{ 1.0 };
        double lift_velocity_coef_{ 1.0 };

        realtime_tools::RealtimeBox<std::shared_ptr<RotaryLiftPose>> received_cmd_msg_ptr_{nullptr};

        bool subscriber_is_active_{ false };
        rclcpp::Subscription<RotaryLiftPose>::SharedPtr command_subscriber_ = nullptr;

        std::chrono::milliseconds cmd_vel_timeout_{ 500 };
        bool is_halted{ false };

        // speed limiters
        SpeedLimiter limiter_rotary_;
        SpeedLimiter limiter_lift_;

        // publish rate limiter
        double publish_rate_{ 50.0 };
        rclcpp::Duration publish_period_{ 0, 0 };
        rclcpp::Time previous_publish_timestamp_{ 0 };
    };
}  // namespace whi_rotary_lift_controller
