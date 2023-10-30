/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/Entity.h"
#include "AzCore/Component/EntityId.h"
#include "AzCore/Component/TransformBus.h"
#include "AzCore/Math/Transform.h"
#include "AzCore/RTTI/RTTIMacros.h"
#include "AzCore/std/containers/vector.h"
#include "AzCore/std/parallel/mutex.h"
#include "AzCore/std/string/string.h"
#include "ROS2/Frame/ROS2FrameSystemBus.h"
#include <AzCore/Component/Component.h>
#include <AzCore/std/containers/map.h>
#include <string>

namespace ROS2
{
    class ROS2FrameSystemTransformHandler : public AZ::TransformNotificationBus::Handler
    {
    public:
        AZ_RTTI(ROS2FrameSystemTransformHandler, "{0b4324bb-4c36-4a86-8827-a044a6ac44e8}");

        ROS2FrameSystemTransformHandler(AZStd::mutex& frameSystemMutex, const AZ::EntityId& watchedEntityId);

        void OnParentChanged(AZ::EntityId oldParent, AZ::EntityId newParent) override;

        void AddFrameEntity(const AZ::EntityId& frameEntityId);
        void RemoveFrameEntity(const AZ::EntityId& frameEntityId);

        unsigned int GetFrameAmount();

    private:
        AZStd::mutex& m_frameSystemMutex;
        AZStd::set<AZ::EntityId> m_frameEntities;
    };

    class ROS2FrameSystemComponent
        : public AZ::Component
        , public ROS2FrameSystemInterface::Registrar
    {
    public:
        AZ_COMPONENT(ROS2FrameSystemComponent, "{360c4b45-ac02-42d2-9e1a-1d77eb22a054}");
        static void Reflect(AZ::ReflectContext* context);

        // static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        // static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        // static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        void Activate() override;
        void Deactivate() override;

        void RegisterFrame(AZ::EntityId frameEntityId) override;
        void UnregisterFrame(AZ::EntityId frameEntityId) override;

        void MoveFrame(AZ::EntityId frameEntityId, AZ::EntityId newParent) override;

        void NotifyChange(AZ::EntityId frameEntityId) override;

        bool IsTopLevel(AZ::EntityId frameEntityId) const override;
        AZ::EntityId GetParentEntityId(AZ::EntityId frameEntityId) const override;

        ROS2FrameSystemComponent();

        ~ROS2FrameSystemComponent();

    private:
        void RegisterFrame(AZ::EntityId frameEntityId, AZ::EntityId frameEntityParentEntityId);
        void UnregisterFrame(AZ::EntityId frameEntityId, AZ::EntityId frameEntityParentEntityId);

        // Includes it's parent
        AZStd::vector<AZ::EntityId> FindFrameParentPath(AZ::EntityId frameEntityId);
        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity);

        void UpdateNamespaces(AZ::EntityId frameEntity, AZStd::string parentsNamespace = "", bool isActive = true);

        AZStd::map<AZ::EntityId, AZStd::set<AZ::EntityId>> m_frameChildren;
        AZStd::map<AZ::EntityId, AZ::EntityId> m_frameParent;
        AZStd::map<AZ::EntityId, AZStd::set<AZ::EntityId>> m_watchedEntities;
        AZStd::map<AZ::EntityId, ROS2FrameSystemTransformHandler> m_watchedEntitiesHandlers;

        AZStd::mutex m_frameSystemMutex;
    };
} // namespace ROS2