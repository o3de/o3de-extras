/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Frame/ROS2FrameRegistrationBus.h>
#include <ROS2/Frame/ROS2FrameComponentBus.h>
#include <ROS2/Frame/ROS2FrameTrackingBus.h>

namespace ROS2
{
    //! System component that tracks all registered ROS2 frame components.
    //! This component maintains a simple registry of all active frame entities in the scene.
    class ROS2FrameSystemComponent
        : public AZ::Component
        , public AZ::SystemTickBus::Handler
        , public ROS2FrameRegistrationInterface::Registrar
        , public ROS2FrameTrackingInterface::Registrar
    {
    public:
        AZ_COMPONENT(ROS2FrameSystemComponent, "{B8E5F123-4567-89AB-CDEF-123456789ABC}");

        ROS2FrameSystemComponent() = default;
        ~ROS2FrameSystemComponent() = default;

        //////////////////////////////////////////////////////////////////////////
        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // AZ::SystemTickBus::Handler overrides
        void OnSystemTick() override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // ROS2FrameRegistrationInterface::Registrar overrides
        void RegisterFrame(const AZ::EntityId& frameEntityId) override;
        void UnregisterFrame(const AZ::EntityId& frameEntityId) override;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // ROS2FrameTrackingInterface::Registrar overrides
        const AZStd::unordered_set<AZ::EntityId>& GetRegisteredFrames() const override;
        bool IsFrameRegistered(const AZ::EntityId& frameEntityId) const override;
        size_t GetRegisteredFrameCount() const override;
        AZ::EntityId GetFrameEntityByNamespacedId(const AZStd::string& namespacedFrameId) const override;
        AZStd::string GetNamespacedFrameId(const AZ::EntityId& frameEntityId) const override;
        AZStd::unordered_set<AZStd::string> GetAllNamespacedFrameIds() const override;
        //////////////////////////////////////////////////////////////////////////

        static void Reflect(AZ::ReflectContext* context);

    private:
        //! Debug method to print all internal maps
        void DebugPrintMaps() const;

        //! Tick counter for periodic debug printing
        int m_tickCounter = 0;

        //! Set of all registered frame entity IDs
        AZStd::unordered_set<AZ::EntityId> m_registeredFrames;
        
        //! Map from namespaced frame ID to entity ID for fast lookup
        AZStd::unordered_map<AZStd::string, AZ::EntityId> m_namespacedFrameIdToEntity;
        
        //! Map from entity ID to namespaced frame ID for reverse lookup
        AZStd::unordered_map<AZ::EntityId, AZStd::string> m_entityToNamespacedFrameId;
    };
} // namespace ROS2
