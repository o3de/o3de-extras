/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>
#include <AzCore/Math/Color.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <ROS2Sensors/Lidar/SegmentationClassConfiguration.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2Sensors
{
    static constexpr uint8_t UnknownClassId = 0U;
    static constexpr uint8_t TerrainClassId = 1U;
    using SegmentationClassConfigList = AZStd::vector<SegmentationClassConfiguration>;

    //! Interface class that allows for retrieval of segmentation class information.
    class ClassSegmentationRequests
    {
    public:
        AZ_RTTI(ClassSegmentationRequests, ClassSegmentationBusTypeId);

        //! Returns the color of segmentation class with the provided class ID.
        //! If no segmentation class is found with provided class ID, returns AZ::Colors::White.
        //! @param classId Class ID of the segmentation class.
        //! @return Color of the class with provided ID.
        virtual AZ::Color GetClassColor(uint8_t classId) const = 0;

        //! If segmentation class exists that is associated with provided tag,
        //! returns ID of this class. Otherwise, returns AZStd::nullopt;
        //! @param tag Tag associated with the segmentation class.
        //! @return ID of found class or AZStd::nullopt.
        virtual AZStd::optional<uint8_t> GetClassIdForTag(LmbrCentral::Tag tag) const = 0;

        //! Returns a reference to the segmentation config list.
        virtual const SegmentationClassConfigList& GetClassConfigList() const = 0;

    protected:
        virtual ~ClassSegmentationRequests() = default;
    };

    class ClassSegmentationRequestBusTraits : public AZ::EBusTraits
    {
    public:
        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static constexpr AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////
    };

    using ClassSegmentationRequestBus = AZ::EBus<ClassSegmentationRequests, ClassSegmentationRequestBusTraits>;
    using ClassSegmentationInterface = AZ::Interface<ClassSegmentationRequests>;
} // namespace ROS2Sensors
