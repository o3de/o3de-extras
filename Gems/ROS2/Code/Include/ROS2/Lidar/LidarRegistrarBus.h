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

namespace ROS2
{
    //! Structure used to describe LidarSystem's feature support.
    struct LidarSystemFeatures
    {
    public:
        bool m_noise{ false };
        bool m_collisionLayers{ false };
        bool m_entityExclusion{ false };
        bool m_maxRangePoints{ false };
    };

    //! Structure used to hold LidarSystem's metadata.
    struct LidarSystemMetaData
    {
    public:
        AZStd::string m_name;
        AZStd::string m_description;
        LidarSystemFeatures m_features;
    };

    class LidarRegistrarRequests
    {
    public:
        AZ_RTTI(LidarRegistrarRequests, "{22030dc7-a1db-43bd-b748-0fb9ec43ce2e}");
        virtual ~LidarRegistrarRequests() = default;

        //! Registers a new lidar system under the provided name.
        //! To obtain the busId of a lidarSystem use the AZ_CRC macro as follows.
        //! @code
        //! AZ::Crc32 busId = AZ_CRC(<lidarSystemName>);
        //! @endcode
        //! @param name name of the newly registered lidar system.
        //! @param description further information about the lidar system.
        virtual void RegisterLidarSystem(const char* name, const char* description, const LidarSystemFeatures& features) = 0;

        //! Returns a list of all registered lidar systems.
        //! @return a vector of registered lidar systems' names.
        virtual const AZStd::vector<AZStd::string> GetRegisteredLidarSystems() = 0;

        //! Returns metadata of a registered lidar system.
        //! @param name name of a registered lidar system.
        //! @return metadata of a lidar system with the provided name.
        virtual const LidarSystemMetaData& GetLidarSystemMetaData(const AZStd::string& name) = 0;
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
} // namespace ROS2
