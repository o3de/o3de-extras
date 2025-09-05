/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ROS2FrameSystemBus.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <ROS2/Frame/ROS2FrameTrackingInterface.h>
#include <ROS2/ROS2TypeIds.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/map.h>

namespace ROS2
{
    class ROS2FrameGameSystemComponent
        : public AZ::Component
        , protected ROS2FrameSystemInterface::Registrar
        , protected ROS2FrameTrackingRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2FrameGameSystemComponent, ROS2FrameGameSystemComponentTypeId);

        ROS2FrameGameSystemComponent();
        ~ROS2FrameGameSystemComponent();

        static void Reflect(AZ::ReflectContext* context);

        void Activate() override;
        void Deactivate() override;

    protected:
        // ROS2FrameSystemInterface::Registrar
        void RegisterFrame(const AZ::EntityId& frameEntityId) override;
        void UnregisterFrame(const AZ::EntityId& frameEntityId) override;

        // ROS2FrameTrackingRequestBus::Handler
        const AZStd::unordered_set<AZ::EntityId>& GetRegisteredFrameEntityIds() const override;
        bool IsFrameRegistered(const AZ::EntityId& frameEntityId) const override;
        size_t GetRegisteredFrameCount() const override;
        AZ::EntityId GetFrameEntityByNamespacedId(const AZStd::string& namespacedFrameId) const override;
        AZStd::string GetNamespacedFrameId(const AZ::EntityId& frameEntityId) const override;
        AZStd::unordered_set<AZStd::string> GetAllNamespacedFrameIds() const override;
        void DisableEntitySafely(const AZ::EntityId& entityToDisable) override;
        void EnableEntitySafely(const AZ::EntityId& entityToEnable) override;

        AZStd::unordered_set<AZ::EntityId> m_registeredEntities; //!< Set of all registered frame entities.

        AZStd::map<AZ::EntityId, AZStd::string> m_entityIdToFrameId;
        AZStd::map<AZStd::string, AZ::EntityId> m_frameIdToEntityId;
        using EntityIdTransformPair = AZStd::pair<AZ::EntityId, AZ::Transform>;
        AZStd::unordered_map<AZ::EntityId, AZStd::vector<EntityIdTransformPair>> m_cachedChildTransforms;
    };
} // namespace ROS2
