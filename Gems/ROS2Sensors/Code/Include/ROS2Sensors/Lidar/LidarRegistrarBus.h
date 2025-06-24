/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/std/string/string.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    //! Enum bitwise flags used to describe LidarSystem's feature support.
    // clang-format off
    enum LidarSystemFeatures : uint16_t
    {
        None =                  0,
        Noise =                 1,
        CollisionLayers =       1 << 1,
        EntityExclusion =       1 << 2,
        MaxRangePoints =        1 << 3,
        PointcloudPublishing =  1 << 4,
        Intensity =             1 << 5,
        Segmentation =          1 << 6,
        All =                   (1 << 7) - 1, // All feature bits enabled.
    };
    // clang-format on

    //! Structure used to hold LidarSystem's metadata.
    struct LidarSystemMetaData
    {
        AZStd::string m_name;
        AZStd::string m_description;
        LidarSystemFeatures m_features;
    };

    //! Interface class that allows for communication with the LidarRegistrarSystemComponent.
    class LidarRegistrarRequests
    {
    public:
        AZ_RTTI(LidarRegistrarRequests, LidarRegistrarBusTypeId);

        //! Registers a new lidar system under the provided name.
        //! To obtain the busId of a lidarSystem use the AZ_CRC macro as follows.
        //! @code
        //! AZ::Crc32 busId = AZ_CRC(<lidarSystemName>);
        //! @endcode
        //! @param name Name of the newly registered lidar system.
        //! @param description Further information about the lidar system.
        virtual void RegisterLidarSystem(const char* name, const char* description, const LidarSystemFeatures& features) = 0;

        //! Returns A list of all registered lidar systems.
        //! @return A vector of registered lidar systems' names.
        virtual AZStd::vector<AZStd::string> GetRegisteredLidarSystems() const = 0;

        //! Returns metadata of a registered lidar system.
        //! If no lidar system with provided name was found returns nullptr.
        //! @param name Name of a registered lidar system.
        //! @return Pointer to the metadata of a lidar system with the provided name.
        virtual const LidarSystemMetaData* GetLidarSystemMetaData(const AZStd::string& name) const = 0;

    protected:
        ~LidarRegistrarRequests() = default;
    };

    class LidarRegistrarBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using LidarRegistrarRequestBus = AZ::EBus<LidarRegistrarRequests, LidarRegistrarBusTraits>;
    using LidarRegistrarInterface = AZ::Interface<LidarRegistrarRequests>;
} // namespace ROS2Sensors
