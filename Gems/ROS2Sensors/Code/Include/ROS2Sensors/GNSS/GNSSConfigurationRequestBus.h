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
#include <ROS2Sensors/GNSS/GNSSSensorConfiguration.h>

namespace ROS2Sensors
{
    //! Interface that allows to get and set GNSS sensor's configuration.
    class GnssConfigurationRequest : public AZ::EBusTraits
    {
    public:
        using BusIdType = AZ::EntityId;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        //! Returns the current configuration of the component.
        virtual const GnssSensorConfiguration GetConfiguration() = 0;

        //! Sets the configuration of the component.
        //! Each component should handle the configuration change without fully reinitializing the ROS2 publisher.
        //! This will allow to change the configuration of the component at runtime.
        //! Note: this method does not verify the configuration, so it is up to the caller to ensure that the configuration is valid.
        //! @param configuration The new configuration to set.
        virtual void SetConfiguration(const GnssSensorConfiguration& configuration) = 0;

        //! Gets current setting if normal NavSat is to be used
        virtual bool GetUseNavSat() = 0;

        //! Sets if normal NavSat is to be used
        virtual void SetUseNavSat(bool useNavSat) = 0;

        //! Gets current setting if GpsFix is to be used
        virtual bool GetUseGpsFix() = 0;

        //! Sets if GpsFix is to be used
        virtual void SetUseGpsFix(bool useGpsFix) = 0;

        //! Gets current setting if Quaternion is to be used
        virtual bool GetUseQuaternion() = 0;

        //! Sets if normal Quaternion is to be used
        virtual void SetUseQuaternion(bool useQuaternion) = 0;
    };

    using GnssConfigurationRequestBus = AZ::EBus<GnssConfigurationRequest>;
} // namespace ROS2Sensors
