/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>
#include <AzCore/std/string/string.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

namespace ROS2
{
    //! Publishes transforms as standard ROS2 tf2 messages. Static transforms are published once.
    //! @note This class is already used through ROS2FrameComponent.
    class ROS2Transform
    {
    public:
        //! Create a transform between given frames.
        //! @param parentFrame id of parent frame of transformation.
        //! @param childFrame id of child frame of transformation.
        //! @param isDynamic whether the transformation is dynamic (should be computed every frame) or static (only once).
        ROS2Transform(AZStd::string parentFrame, AZStd::string childFrame, bool isDynamic);

        //! Construct and delegate publishing of a transform according to members' values.
        //! @param transform AZ::Transform with current transformation between m_parentFrame and m_childFrame.
        //! @note The actual publishing is done by singleton tf broadcasters.
        void Publish(const AZ::Transform& transform);

    private:
        geometry_msgs::msg::TransformStamped CreateTransformMessage(const AZ::Transform& transform);

        const AZStd::string m_parentFrame;
        const AZStd::string m_childFrame;
        bool m_isPublished = false;
        bool m_isDynamic;
    };
} // namespace ROS2
