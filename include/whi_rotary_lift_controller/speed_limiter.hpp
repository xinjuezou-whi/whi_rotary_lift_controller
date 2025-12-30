/******************************************************************
speed limiter for rotary lift under ROS 2
it is a controller resource layer for ros2_controller

Features:
- speed limiter
- xxx

Written by Xinjue Zou, xinjue.zou.whi@gmail.com

Apache License Version 2.0, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2025-12-29: Initial version
2025-xx-xx: xxx
******************************************************************/
#pragma once
#include <rcppmath/clamp.hpp>

#include <cmath>
#include <algorithm>
#include <stdexcept>

namespace whi_rotary_lift_controller
{
    class SpeedLimiter
    {
    public:
        /**
        * \brief Constructor
        * \param [in] HasVelocityLimits     if true, applies velocity limits
        * \param [in] MinVelocity Minimum velocity [m/s], usually <= 0
        * \param [in] MaxVelocity Maximum velocity [m/s], usually >= 0
        */
        SpeedLimiter(bool HasVelocityLimits = false,
            double MinVelocity = NAN, double MaxVelocity = NAN)
            : has_velocity_limits_(HasVelocityLimits), min_velocity_(MinVelocity), max_velocity_(MaxVelocity)
        {
            // Check if limits are valid, max must be specified, min defaults to -max if unspecified
            if (has_velocity_limits_)
            {
                if (std::isnan(max_velocity_))
                {
                    throw std::runtime_error("Cannot apply velocity limits if max_velocity is not specified");
                }
                if (std::isnan(min_velocity_))
                {
                    min_velocity_ = -max_velocity_;
                }
            }
        };

        /**
        * \brief Limit the velocity
        * \param [in, out] Velocity Velocity [m/s]
        * \return Limiting factor (1.0 if none)
        */
        double limit_velocity(double& Velocity)
        {
            const double tmp = Velocity;

            if (has_velocity_limits_)
            {
                Velocity = rcppmath::clamp(Velocity, min_velocity_, max_velocity_);
            }

            return tmp != 0.0 ? Velocity / tmp : 1.0;
        };

    private:
        // Enable/Disable velocity
        bool has_velocity_limits_;

        // Velocity limits:
        double min_velocity_;
        double max_velocity_;
    };
}  // namespace whi_rotary_lift_controller
