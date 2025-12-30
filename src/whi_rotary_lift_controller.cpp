/******************************************************************
rotary lift controller plugin under ROS 2
it is a controller resource layer for ros2_controller

Features:
- rotary lift
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_rotary_lift_controller/whi_rotary_lift_controller.hpp"

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>

namespace whi_rotary_lift_controller
{
    using controller_interface::interface_configuration_type;
    using controller_interface::InterfaceConfiguration;

    WhiRotaryLiftController::WhiRotaryLiftController()
        : controller_interface::ControllerInterface() {}

    InterfaceConfiguration WhiRotaryLiftController::command_interface_configuration() const
    {
        std::vector<std::string> confNames;
        for (const auto& name : rotary_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
            confNames.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
        }
        for (const auto& name : lift_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
            confNames.push_back(name + "/" + hardware_interface::HW_IF_VELOCITY);
        }

        return {interface_configuration_type::INDIVIDUAL, confNames};
    }

    InterfaceConfiguration WhiRotaryLiftController::state_interface_configuration() const
    {
        std::vector<std::string> confNames;
        for (const auto& name : rotary_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
        }
        for (const auto& name : lift_names_)
        {
            confNames.push_back(name + "/" + hardware_interface::HW_IF_POSITION);
        }

        return {interface_configuration_type::INDIVIDUAL, confNames};
    }

    controller_interface::return_type WhiRotaryLiftController::update(const rclcpp::Time& Time,
            const rclcpp::Duration& Period)
    {
        auto logger = get_node()->get_logger();

        if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
        {
            if (!is_halted)
            {
                halt();
                is_halted = true;
            }
            return controller_interface::return_type::OK;
        }

        std::shared_ptr<RotaryLiftPose> lastMsg;
        received_cmd_msg_ptr_.get(lastMsg);
        if (lastMsg == nullptr)
        {
            RCLCPP_WARN(logger, "rotary lift command message received was a nullptr");
            return controller_interface::return_type::ERROR;
        }

        const auto dt = Time - lastMsg->header.stamp;
        // brake if cmd_vel has timeout, override the stored command
        if (dt > cmd_vel_timeout_)
        {
            for (auto& it : lastMsg->positions)
            {
                it = 0.0;
            }
            for (auto& it : lastMsg->velocities)
            {
                it = 0.0;
            }
        }

        // command may be limited further by SpeedLimit, without affecting the stored command
        RotaryLiftPose command = *lastMsg;

        for (size_t i = 0; i < rotary_names_.size(); ++i)
        {
            // from the hardware interface
            const double position = registered_handles_[i].position_sta_.get().get_value();
            if (std::isnan(position))
            {
                RCLCPP_ERROR(logger, "position is invalid for index [%zu] on rotary", i);
                return controller_interface::return_type::ERROR;
            }
        }
        for (size_t i = 0; i < lift_names_.size(); ++i)
        {
            // from the hardware interface
            const double position = registered_handles_[i + rotary_names_.size()].position_sta_.get().get_value();
            if (std::isnan(position))
            {
                RCLCPP_ERROR(logger, "position is invalid for index [%zu] on lift", i);
                return controller_interface::return_type::ERROR;
            }
        }

        // set rotary lift positions to hardware interface
        for (auto i = 0; i < rotary_names_.size(); ++i)
        {
            auto found = std::find(command.joints.begin(), command.joints.end(), rotary_names_[i]);
            if (found != command.joints.end())
            {
                auto index = std::distance(command.joints.begin(), found);
                limiter_rotary_.limit_velocity(command.velocities[index]);
                registered_handles_[i].position_cmd_.get().set_value(command.positions[index] * rotary_position_coef_);
                registered_handles_[i].velocity_cmd_.get().set_value(command.velocities[index] * rotary_velocity_coef_);
            }
        }
        for (auto i = 0; i < lift_names_.size(); ++i)
        {
            auto found = std::find(command.joints.begin(), command.joints.end(), lift_names_[i]);
            if (found != command.joints.end())
            {
                auto index = std::distance(command.joints.begin(), found);
                limiter_rotary_.limit_velocity(command.velocities[index]);
                registered_handles_[i + rotary_names_.size()].position_cmd_.get().set_value(command.positions[index] * lift_position_coef_);
                registered_handles_[i + rotary_names_.size()].velocity_cmd_.get().set_value(command.velocities[index] * lift_velocity_coef_);
            }
        }

        return controller_interface::return_type::OK;
    }

    controller_interface::CallbackReturn WhiRotaryLiftController::on_init()
    {
        /// node version and copyright announcement
        std::cout << "\nWHI rotary lift controller VERSION 0.1.1" << std::endl;
        std::cout << "Copyright © 2025-2026 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;

        try
        {
            auto_declare<double>("publish_rate", publish_rate_);

            // with the lifecycle node being initialized, we can declare parameters
            auto_declare<std::vector<std::string>>("rotary_names", std::vector<std::string>());
            auto_declare<double>("rotary_position_coef", rotary_position_coef_);
            auto_declare<double>("rotary_velocity_coef", rotary_velocity_coef_);
            auto_declare<std::vector<std::string>>("lift_names", std::vector<std::string>());
            auto_declare<double>("lift_position_coef", lift_position_coef_);
            auto_declare<double>("lift_velocity_coef", lift_velocity_coef_);

            auto_declare<double>("cmd_vel_timeout", cmd_vel_timeout_.count() / 1000.0);
        }
        catch (const std::exception& e)
        {
            RCLCPP_FATAL(get_node()->get_logger(),
                "\033[1;31mexception thrown during init stage with message: %s\033[0m", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiRotaryLiftController::on_configure(
        const rclcpp_lifecycle::State&)
    {
        auto logger = get_node()->get_logger();

        /// update parameters
        rotary_names_ = get_node()->get_parameter("rotary_names").as_string_array();
        if (rotary_names_.empty())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31rotary names are empty\033[0m");
            return controller_interface::CallbackReturn::ERROR;
        }
        lift_names_ = get_node()->get_parameter("lift_names").as_string_array();
        if (lift_names_.empty())
        {
            RCLCPP_ERROR(logger,
                "\033[1;31lift names are empty\033[0m");
            return controller_interface::CallbackReturn::ERROR;
        }

        // cmd
        cmd_vel_timeout_ = std::chrono::milliseconds{
            static_cast<int>(get_node()->get_parameter("cmd_vel_timeout").as_double() * 1000.0)};
        // speed limit
        try
        {
            limiter_rotary_ = SpeedLimiter(
                get_node()->get_parameter("rotary.has_velocity_limits").as_bool(),
                get_node()->get_parameter("rotary.min_velocity").as_double(),
                get_node()->get_parameter("rotary.max_velocity").as_double());
        }
        catch (const std::runtime_error & e)
        {
            RCLCPP_ERROR(logger,
                "\033[1;31merror configuring rotary speed limiter: %s\033[0m", e.what());
        }
        try
        {
            limiter_lift_ = SpeedLimiter(
                get_node()->get_parameter("lift.has_velocity_limits").as_bool(),
                get_node()->get_parameter("lift.min_velocity").as_double(),
                get_node()->get_parameter("lift.max_velocity").as_double());
        }
        catch (const std::runtime_error & e)
        {
            RCLCPP_ERROR(logger,
                "\033[1;31merror configuring lift speed limiter: %s\033[0m", e.what());
        }

        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }
    
        const RotaryLiftPose emptyRotaryLiftPose;
        received_cmd_msg_ptr_.set(std::make_shared<RotaryLiftPose>(emptyRotaryLiftPose));

        // initialize command subscriber
        command_subscriber_ = get_node()->create_subscription<RotaryLiftPose>(
            "~/cmd_rotary_lift", rclcpp::SystemDefaultsQoS(),
            [this, &logger](const std::shared_ptr<RotaryLiftPose> msg) -> void
            {
                if (!subscriber_is_active_)
                {
                    RCLCPP_WARN(logger, "Can't accept new commands. subscriber is inactive");
                    return;
                }
                if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0))
                {
                    RCLCPP_WARN_ONCE(logger,
                        "received WhiRotaryLiftPoseStamped with zero timestamp, setting it to current "
                        "time, this message will only be shown once");

                    msg->header.stamp = get_node()->get_clock()->now();
                }
                received_cmd_msg_ptr_.set(std::move(msg));
            });

        // limit the publication on the topics /odom and /tf
        publish_rate_ = get_node()->get_parameter("publish_rate").as_double();
        publish_period_ = rclcpp::Duration::from_seconds(1.0 / publish_rate_);
        previous_publish_timestamp_ = get_node()->get_clock()->now();

        // coefs
        rotary_position_coef_ = get_node()->get_parameter("rotary_position_coef").as_double();
        rotary_velocity_coef_ = get_node()->get_parameter("rotary_velocity_coef").as_double();
        lift_position_coef_ = get_node()->get_parameter("lift_position_coef").as_double();
        lift_velocity_coef_ = get_node()->get_parameter("lift_velocity_coef").as_double();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiRotaryLiftController::on_activate(
        const rclcpp_lifecycle::State&)
    {
        const auto result = configure(rotary_names_, lift_names_, registered_handles_);
        if (result == controller_interface::CallbackReturn::ERROR)
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        if (registered_handles_.empty())
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                "\033[1;31mrotary lift interfaces are non existent\033[0m");
            return controller_interface::CallbackReturn::ERROR;
        }

        is_halted = false;
        subscriber_is_active_ = true;

        RCLCPP_INFO(get_node()->get_logger(),
            "\033[1;32msubscriber and publisher of rotary lift controller are now active\033[0m");
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiRotaryLiftController::on_deactivate(
        const rclcpp_lifecycle::State&)
    {
        subscriber_is_active_ = false;
        if (!is_halted)
        {
            halt();
            is_halted = true;
        }

        registered_handles_.clear();

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiRotaryLiftController::on_cleanup(
        const rclcpp_lifecycle::State&)
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        received_cmd_msg_ptr_.set(std::make_shared<RotaryLiftPose>());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn WhiRotaryLiftController::on_error(
        const rclcpp_lifecycle::State&)
    {
        if (!reset())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    bool WhiRotaryLiftController::configure(const std::vector<std::string>& JointNames, std::vector<RotaryLiftHandle>& RegisteredHandles)
    {
        auto logger = get_node()->get_logger();

        for (const auto& name : JointNames)
        {
            const auto stateHandlePos = std::find_if(
                state_interfaces_.cbegin(), state_interfaces_.cend(), [&name](const auto& interface)
                {
                    return interface.get_prefix_name() == name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });

            if (stateHandlePos == state_interfaces_.cend())
            {
                RCLCPP_ERROR(logger,
                    "\033[1;31munable to obtain position state handle for %s\033[0m", name.c_str());
                return false;
            }

            const auto commandHandlePos = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(), [&name](const auto& interface)
                {
                    return interface.get_prefix_name() == name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_POSITION;
                });
            const auto commandHandleVel = std::find_if(
                command_interfaces_.begin(), command_interfaces_.end(), [&name](const auto& interface)
                {
                    return interface.get_prefix_name() == name &&
                        interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY;
                });

            if (commandHandlePos == command_interfaces_.end() || commandHandleVel == command_interfaces_.end())
            {
                RCLCPP_ERROR(logger,
                    "\033[1;31munable to obtain either position command or velocity command handle for %s\033[0m", name.c_str());
                return false;
            }

            RegisteredHandles.emplace_back(RotaryLiftHandle{std::ref(*stateHandlePos),
                std::ref(*commandHandlePos), std::ref(*commandHandleVel)});
        }

        return true;
    }

    controller_interface::CallbackReturn WhiRotaryLiftController::configure(
        const std::vector<std::string>& RotaryNames, const std::vector<std::string>& LiftNames,
        std::vector<RotaryLiftHandle>& RegisteredHandles)
    {
        if (RotaryNames.empty() || LiftNames.empty())
        {
            return controller_interface::CallbackReturn::ERROR;
        }

        // register handles
        RegisteredHandles.reserve(RotaryNames.size() + LiftNames.size());
        if (configure(RotaryNames, RegisteredHandles) && configure(LiftNames, RegisteredHandles))
        {
            return controller_interface::CallbackReturn::SUCCESS;
        }
        else
        {
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    bool WhiRotaryLiftController::reset()
    {
        registered_handles_.clear();

        subscriber_is_active_ = false;
        command_subscriber_.reset();

        received_cmd_msg_ptr_.set(nullptr);
        is_halted = false;

        return true;
    }

    void WhiRotaryLiftController::halt()
    {
        const auto haltRotaryLift = [](auto& Handles)
        {
            for (const auto& it : Handles)
            {
                it.velocity_cmd_.get().set_value(0.0);
            }
        };

        haltRotaryLift(registered_handles_);
    }
}  // namespace whi_rotary_lift_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(whi_rotary_lift_controller::WhiRotaryLiftController,
    controller_interface::ControllerInterface)
