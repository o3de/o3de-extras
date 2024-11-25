/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <ROS2/Lidar/PC2PostProcessingBus.h>

namespace ROS2
{
    //! Base component for Point Cloud 2 Post-processing components.
    //! Allows for easier prototyping and development of post-processing components.
    //! Handles bus connections and implements reflection for the post-processing priority.
    //! Components that inherit this base, only need to define the ApplyPostProcessing method override.
    class PC2PostProcessingBaseComponent
        : public AZ::Component
        , public PC2PostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(PC2PostProcessingBaseComponent, "{b21f4ae3-4a94-403d-a5e9-848ab17178dd}", AZ::Component)

        PC2PostProcessingBaseComponent() = default;
        ~PC2PostProcessingBaseComponent() = default;

        static void Reflect(AZ::ReflectContext* context);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

        void ApplyPostProcessing([[maybe_unused]] sensor_msgs::msg::PointCloud2& message) override
        {
        }

    protected:
        AZ::u8 m_priority{ PC2PostProcessing::DefaultPriority };

    private:
        [[nodiscard]] AZ::u8 GetPriority() const override;
    };
} // namespace ROS2
