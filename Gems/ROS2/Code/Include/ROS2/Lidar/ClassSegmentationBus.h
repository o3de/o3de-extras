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
#include <ROS2/Lidar/SegmentationClassConfiguration.h>

namespace ROS2
{
    static constexpr uint8_t UnknownClassId = 0U;
    static constexpr uint8_t TerrainClassId = 1U;
    using SegmentationClassConfigList = AZStd::vector<SegmentationClassConfiguration>;

    //! Interface class that allows for retrieval of segmentation class information.
    class ClassSegmentationRequests
    {
    public:
        AZ_RTTI(ClassSegmentationRequests, "{69b4109e-25ff-482f-b92e-f19cdf06bce2}");

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

    //! Notification bus for segmentation class global configuration.
    class ClassSegmentationNotifications : public AZ::EBusTraits
    {
    private:
        template<class Bus>
        struct ClassSegmentationConnectionPolicy : public AZ::EBusConnectionPolicy<Bus>
        {
            static void Connect(
                typename Bus::BusPtr& busPtr,
                typename Bus::Context& context,
                typename Bus::HandlerNode& handler,
                typename Bus::Context::ConnectLockGuard& connectLock,
                const typename Bus::BusIdType& id = 0)
            {
                AZ::EBusConnectionPolicy<Bus>::Connect(busPtr, context, handler, connectLock, id);

                if (ClassSegmentationInterface::Get())
                {
                    handler->OnSegmentationClassesReady();
                }
            }
        };

    public:
        template<typename Bus>
        using ConnectionPolicy = ClassSegmentationConnectionPolicy<Bus>;

        //////////////////////////////////////////////////////////////////////////
        // EBusTraits overrides
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Multiple;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        //////////////////////////////////////////////////////////////////////////

        virtual void OnSegmentationClassesReady()
        {
        }
        //////////////////////////////////////////////////////////////////////////
    };
    using ClassSegmentationNotificationBus = AZ::EBus<ClassSegmentationNotifications>;
} // namespace ROS2
