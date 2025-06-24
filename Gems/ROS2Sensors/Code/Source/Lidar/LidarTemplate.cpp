/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <ROS2Sensors/Lidar/LidarTemplate.h>

namespace ROS2Sensors
{
    void LidarTemplate::NoiseParameters::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<NoiseParameters>()
                ->Version(1)
                ->Field("Angular noise standard deviation", &NoiseParameters::m_angularNoiseStdDev)
                ->Field("Distance noise standard deviation base", &NoiseParameters::m_distanceNoiseStdDevBase)
                ->Field("Distance noise standard deviation slope", &NoiseParameters::m_distanceNoiseStdDevRisePerMeter);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<NoiseParameters>("Noise Parameters", "Noise Parameters")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NoiseParameters::m_angularNoiseStdDev,
                        "Angular noise std dev [Deg]",
                        "Angular noise standard deviation")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NoiseParameters::m_distanceNoiseStdDevBase,
                        "Distance noise std dev base [m]",
                        "Distance noise standard deviation base")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &NoiseParameters::m_distanceNoiseStdDevRisePerMeter,
                        "Distance noise std dev slope [m]",
                        "Distance noise standard deviation slope")
                    ->Attribute(AZ::Edit::Attributes::Min, 0.0f);
            }
        }
    }

    bool LidarTemplate::IsLayersVisible() const
    {
        return !m_is2D;
    }

    void LidarTemplate::Reflect(AZ::ReflectContext* context)
    {
        NoiseParameters::Reflect(context);
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LidarTemplate>()
                ->Version(2)
                ->Field("Name", &LidarTemplate::m_name)
                ->Field("Layers", &LidarTemplate::m_layers)
                ->Field("Points per layer", &LidarTemplate::m_numberOfIncrements)
                ->Field("Min horizontal angle", &LidarTemplate::m_minHAngle)
                ->Field("Max horizontal angle", &LidarTemplate::m_maxHAngle)
                ->Field("Min vertical angle", &LidarTemplate::m_minVAngle)
                ->Field("Max vertical angle", &LidarTemplate::m_maxVAngle)
                ->Field("Min range", &LidarTemplate::m_minRange)
                ->Field("Max range", &LidarTemplate::m_maxRange)
                ->Field("Enable Noise", &LidarTemplate::m_isNoiseEnabled)
                ->Field("Noise Parameters", &LidarTemplate::m_noiseParameters);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<LidarTemplate>("Lidar Template", "Lidar Template")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarTemplate::m_name, "Name", "Custom lidar name")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarTemplate::m_layers, "Layers", "Vertical dimension")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarTemplate::IsLayersVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &LidarTemplate::m_numberOfIncrements, "Points per layer", "Horizontal dimension")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &LidarTemplate::m_minHAngle, "Min horizontal angle [Deg]", "Left-most reach of fov")
                    ->Attribute(AZ::Edit::Attributes::Min, -180.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &LidarTemplate::m_maxHAngle, "Max horizontal angle [Deg]", "Right-most reach of fov")
                    ->Attribute(AZ::Edit::Attributes::Min, -180.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 180.0f)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &LidarTemplate::m_minVAngle, "Min vertical angle [Deg]", "Downwards reach of fov")
                    ->Attribute(AZ::Edit::Attributes::Min, -90.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 90.0f)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarTemplate::IsLayersVisible)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &LidarTemplate::m_maxVAngle, "Max vertical angle [Deg]", "Upwards reach of fov")
                    ->Attribute(AZ::Edit::Attributes::Min, -90.0f)
                    ->Attribute(AZ::Edit::Attributes::Max, 90.0f)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarTemplate::IsLayersVisible)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarTemplate::m_minRange, "Min range", "Minimum beam range [m]")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &LidarTemplate::ValidateMinRange)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &LidarTemplate::m_maxRange, "Max range", "Maximum beam range [m]")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &LidarTemplate::ValidateMaxRange)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LidarTemplate::m_isNoiseEnabled,
                        "Enable noise",
                        "Enable the use of noise and it's configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarTemplate::m_showNoiseConfig)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ::Edit::PropertyRefreshLevels::EntireTree)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &LidarTemplate::m_noiseParameters,
                        "Noise parameters",
                        "Parameters for Noise Configuration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &LidarTemplate::IsNoiseConfigVisible);
            }
        }
    }

    bool LidarTemplate::IsNoiseConfigVisible() const
    {
        return m_showNoiseConfig && m_isNoiseEnabled;
    }

    AZ::Outcome<void, AZStd::string> LidarTemplate::ValidateRange(float minRange, float maxRange)
    {
        if (0.0f <= minRange && minRange <= maxRange)
        {
            return AZ::Success();
        }

        return AZ::Failure(AZStd::string::format(
            "Provided ray range (%.2f, %.2f) was of incorrect value: Max range must be greater than or equal to min range,"
            " and both must be greater than zero.",
            minRange,
            maxRange));
    }

    AZ::Outcome<void, AZStd::string> LidarTemplate::ValidateMinRange(void* newValue, const AZ::TypeId& valueType) const
    {
        if (azrtti_typeid<float>() != valueType)
        {
            AZ_Assert(false, "Unexpected value type");
            return AZ::Failure(AZStd::string("Unexpectedly received a non-float type for the min range!"));
        }

        return ValidateRange(*reinterpret_cast<float*>(newValue), m_maxRange);
    }

    AZ::Outcome<void, AZStd::string> LidarTemplate::ValidateMaxRange(void* newValue, const AZ::TypeId& valueType) const
    {
        if (azrtti_typeid<float>() != valueType)
        {
            AZ_Assert(false, "Unexpected value type");
            return AZ::Failure(AZStd::string("Unexpectedly received a non-float type for the max range!"));
        }

        return ValidateRange(m_minRange, *reinterpret_cast<float*>(newValue));
    }

} // namespace ROS2Sensors
