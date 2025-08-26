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

namespace ROS2
{
    class ROS2FrameGameSystemComponent
        : public AZ::Component
        , protected ROS2FrameSystemInterface::Registrar
        , protected ROS2FrameTrackingInterface::Registrar
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

        // ROS2FrameTrackingInterface::Registrar
        const AZStd::unordered_set<AZ::EntityId>& GetRegisteredFrames() const override;
        bool IsFrameRegistered(const AZ::EntityId& frameEntityId) const override;
        size_t GetRegisteredFrameCount() const override;
        AZStd::optional<AZ::EntityId> GetFrameEntityByNamespacedId(const AZStd::string& namespacedFrameId) const override;
        AZStd::optional<AZStd::string> GetNamespacedFrameId(const AZ::EntityId& frameEntityId) const override;
        AZStd::unordered_set<AZStd::string> GetAllNamespacedFrameIds() const override;

        AZStd::unordered_set<AZ::EntityId> m_registeredEntities; //!< Set of all registered frame entities.

        AZStd::map<AZ::EntityId, AZStd::string> m_entityIdToFrameId;
        AZStd::map<AZStd::string, AZ::EntityId> m_frameIdToEntityId;
    };
} // namespace ROS2
