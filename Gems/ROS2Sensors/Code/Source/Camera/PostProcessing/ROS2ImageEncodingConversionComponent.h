/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <Camera/CameraUtilities.h>
#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/TypeInfoSimple.h>
#include <ROS2Sensors/Camera/CameraPostProcessingRequestBus.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{


    struct EncodingConversion
    {

        AZ_TYPE_INFO(EncodingConversion, ROS2Sensors::EncodingConversionTypeId);
        static void Reflect(AZ::ReflectContext* context);

        //! Compares the encoding pair only: the conversion table is keyed by it, whatever the scale factor.
        bool operator==(const EncodingConversion& rhs) const
        {
            return encodingIn == rhs.encodingIn && encodingOut == rhs.encodingOut;
        }

        using ImageEncoding = CameraUtils::ImageEncoding;
        ImageEncoding encodingIn = ImageEncoding::RGBA8;
        ImageEncoding encodingOut = ImageEncoding::RGB8;
        float m_scaleFactor = 1.0f;

    private:
        //! Text of the "Conversion summary" label: whether the selected pair is supported or not.
        AZStd::string GetEncodingUiComment() const;

        //! The scale factor only means anything for depth input, the one conversion that scales samples.
        AZ::Crc32 GetScaleFactorVisibility() const;

        //! Validates if conversion is valid
        AZ::Outcome<void, AZStd::string> ValidateEncodingConversion() const;

    };

    //! Apply an encoding conversion to an image, in place.
    //! @param image image to rewrite; untouched unless the conversion is supported and matches its encoding.
    //! @param conversion encoding pair to apply, carrying the scale factor used when quantizing depth.
    //! @return whether the conversion was applied.
    bool ApplyEncodingConversion(sensor_msgs::msg::Image& image, const EncodingConversion& conversion);

    //! Change image format
    class ROS2ImageEncodingConversionComponent
        : public AZ::Component
        , protected CameraPostProcessingRequestBus::Handler
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
