/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2/Frame/ROS2FrameConfiguration.h>

namespace ROS2
{
    //! Base class to get and set ROS2FrameConfiguration on components that have it.
    class ROSFrameInterface
    {
        AZ_RTTI(ROSFrameInterface, ROSFrameInterfaceTypeId);

    public:
        //! Getter for configuration, always available
        virtual ROS2FrameConfiguration GetConfiguration() const = 0;

        //! Allows setting configuration , only for disabled components
        virtual void SetConfiguration(const ROS2FrameConfiguration& config) = 0;
    };

} // namespace ROS2
