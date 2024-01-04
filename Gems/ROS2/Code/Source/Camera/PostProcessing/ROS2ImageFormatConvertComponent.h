/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/Component.h"
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <ROS2/Camera/CameraPostProcessingRequestBus.h>

namespace ROS2
{

    //! Change image format
    class ROS2ImageFormatConvert
        : public AZ::Component
        , public CameraPostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2ImageFormatConvert, "12449810-d179-44f1-8f72-22d8d3fa4460");

        ROS2ImageFormatConvert() = default;
        ~ROS2ImageFormatConvert() override = default;

        void Activate() override;
        void Deactivate() override;

        //! CameraPostProcessingRequestBus::Handler overrides
        void ApplyPostProcessing(sensor_msgs::msg::Image& image) override;
        AZ::u8 GetPriority() const override;

        static void Reflect(AZ::ReflectContext* context);

    private:
        AZ::u16 m_priority = CameraPostProcessingRequests::DEFAULT_PRIORITY;
    };
} // namespace ROS2
