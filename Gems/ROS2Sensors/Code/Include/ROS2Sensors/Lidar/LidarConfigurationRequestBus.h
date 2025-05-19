/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Math/Vector3.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/string/string.h>
#include <ROS2Sensors/Lidar/LidarSensorConfiguration.h>
#include <ROS2Sensors/Lidar/LidarTemplate.h>

namespace ROS2Sensors
{
    //! Interface that allows to get and set Lidar sensor's configuration.
    class LidarConfigurationRequest : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;

        //! Returns the current configuration of the component.
        virtual const LidarSensorConfiguration GetConfiguration() const = 0;

        //! Sets the configuration of the component.
        //! Each component should handle the configuration change without fully reinitializing the ROS2 publisher.
        //! This will allow to change the configuration of the component at runtime.
        //! @param configuration The new configuration to set.
        virtual void SetConfiguration(const LidarSensorConfiguration& configuration) = 0;

        //! Get the name of the lidar model.
        virtual AZStd::string GetModelName() const = 0;
        //! Set the name of the lidar model. Unknown name will set a custom lidar model.
        virtual void SetModelName(const AZStd::string& name) = 0;

        //! Check if segmentation is enabled in lidar.
        virtual bool IsSegmentationEnabled() const = 0;
        //! Set if segmentation is enabled in lidar.
        virtual void SetSegmentationEnabled(bool enabled) = 0;

        //! Check if the lidar output should include points at maximum range.
        virtual bool IsAddPointsAtMaxEnabled() const = 0;
        //! Set if the lidar output should include points at maximum range.
        virtual void SetAddPointsAtMaxEnabled(bool addPoints) = 0;

        //! Check if the lidar is 2D.
        virtual bool Is2D() const = 0;

        //! Get the minimum horizontal angle (altitude of the ray), in degrees.
        virtual float GetMinHAngle() const = 0;
        //! Set the minimum horizontal angle (altitude of the ray), in degrees.
        virtual void SetMinHAngle(float angle) = 0;

        //! Get the maximum horizontal angle (altitude of the ray), in degrees.
        virtual float GetMaxHAngle() const = 0;
        //! Set the maximum horizontal angle (altitude of the ray), in degrees.
        virtual void SetMaxHAngle(float angle) = 0;

        //! Get the minimum vertical angle (azimuth of the ray), in degrees.
        //! Override for 3D Lidars.
        virtual float GetMinVAngle() const
        {
            AZ_Warning("LidarConfigurationRequestBus", false, "GetMinVAngle() is not implemented.");
            return 0.0f;
        }
        //! Set the minimum vertical angle (azimuth of the ray), in degrees.
        //! Override for 3D Lidars.
        virtual void SetMinVAngle(float angle)
        {
            AZ_Warning("LidarConfigurationRequestBus", false, "SetMinVAngle() is not implemented.");
        }

        //! Get the maximum vertical angle (azimuth of the ray), in degrees.
        //! Override for 3D Lidars.
        virtual float GetMaxVAngle() const
        {
            AZ_Warning("LidarConfigurationRequestBus", false, "GetMaxVAngle() is not implemented.");
            return 0.0f;
        }
        //! Set the maximum vertical angle (azimuth of the ray), in degrees.
        //! Override for 3D Lidars.
        virtual void SetMaxVAngle(float angle)
        {
            AZ_Warning("LidarConfigurationRequestBus", false, "SetMaxVAngle() is not implemented.");
        }

        //! Get the number of laser layers (resolution in horizontal direction).
        //! Override for 3D Lidars.
        virtual unsigned int GetLayers() const
        {
            AZ_Warning("LidarConfigurationRequestBus", false, "GetLayers() is not implemented.");
            return 0;
        }
        //! Set the number of laser layers (resolution in horizontal direction).
        //! Override for 3D Lidars.
        virtual void SetLayers(unsigned int layers)
        {
            AZ_Warning("LidarConfigurationRequestBus", false, "SetLayers() is not implemented.");
        }

        //! Get the resolution in vertical direction.
        virtual unsigned int GetNumberOfIncrements() const = 0;
        //! Set the resolution in vertical direction.
        virtual void SetNumberOfIncrements(unsigned int increments) = 0;

        //! Get the minimum range of the simulated Lidar.
        virtual float GetMinRange() const = 0;
        //! Set the minimum range of the simulated Lidar.
        virtual void SetMinRange(float range) = 0;

        //! Get the maximum range of the simulated Lidar.
        virtual float GetMaxRange() const = 0;
        //! Set the maximum range of the simulated Lidar.
        virtual void SetMaxRange(float range) = 0;

        //! Get the noise parameters.
        virtual const LidarTemplate::NoiseParameters& GetNoiseParameters() const = 0;
        //! Set the noise parameters.
        virtual void SetNoiseParameters(const LidarTemplate::NoiseParameters& params) = 0;
    };

    using LidarConfigurationRequestBus = AZ::EBus<LidarConfigurationRequest>;
} // namespace ROS2Sensors
