/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <ROS2Sensors/Camera/CameraPostProcessingRequestBus.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    enum class ImageEncoding : AZ::u8
    {
        RGBA8,
        RGB8,
        Mono8,
        Mono16,
    };

    struct EncodingConversion
    {
        AZ_TYPE_INFO(EncodingConversion, ROS2Sensors::EncodingConversionTypeId);
        static void Reflect(AZ::ReflectContext* context);
        AZ::Outcome<void, AZStd::string> ValidateInputEncoding(void* newValue, const AZ::Uuid& valueType);
        AZ::Outcome<void, AZStd::string> ValidateOutputEncoding(void* newValue, const AZ::Uuid& valueType);

        bool operator==(const EncodingConversion& rhs) const
        {
            return encodingIn == rhs.encodingIn && encodingOut == rhs.encodingOut;
        }

        ImageEncoding encodingIn = ImageEncoding::RGBA8;
        ImageEncoding encodingOut = ImageEncoding::RGB8;
    };

    //! Change image format
    class ROS2ImageEncodingConversionComponent
        : public AZ::Component
        , public CameraPostProcessingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2ImageEncodingConversionComponent, ROS2Sensors::ROS2ImageEncodingConversionComponentTypeId, AZ::Component);
        static void Reflect(AZ::ReflectContext* context);

        ROS2ImageEncodingConversionComponent() = default;
        ~ROS2ImageEncodingConversionComponent() override = default;

        void Activate() override;
        void Deactivate() override;

        //! CameraPostProcessingRequestBus::Handler overrides
        void ApplyPostProcessing(sensor_msgs::msg::Image& image) override;
        AZ::u8 GetPriority() const override;

    private:
        AZ::u8 m_priority = CameraPostProcessingRequests::DEFAULT_PRIORITY;
        EncodingConversion m_encodingConvertData;
    };
} // namespace ROS2Sensors
