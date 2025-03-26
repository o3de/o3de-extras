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
#include <ROS2/Camera/CameraPostProcessingRequestBus.h>

namespace ROS2
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
        AZ_TYPE_INFO(EncodingConversion, "{db361adc-b339-4a4e-a10b-c6bf6791eda6}");
        static void Reflect(AZ::ReflectContext* context);
        AZ::Outcome<void, AZStd::string> ValidateInputEncoding(void* newValue, const AZ::Uuid& valueType);
        AZ::Outcome<void, AZStd::string> ValidateOutputEncoding(void* newValue, const AZ::Uuid& valueType);

        bool operator==(const ROS2::EncodingConversion& rhs) const
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
        AZ_COMPONENT(ROS2ImageEncodingConversionComponent, "12449810-d179-44f1-8f72-22d8d3fa4460");
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
} // namespace ROS2
