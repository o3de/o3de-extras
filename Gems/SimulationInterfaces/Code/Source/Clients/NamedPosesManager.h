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
#include <AzCore/Script/ScriptTimePoint.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <SimulationInterfaces/NamedPoseManagerRequestBus.h>

namespace SimulationInterfaces
{
    class NamedPoseManager
        : public AZ::Component
        , protected NamedPoseManagerRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(NamedPoseManager);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        NamedPoseManager();
        ~NamedPoseManager();
        void Init() override;
        void Activate() override;
        void Deactivate() override;

    private:
        // NamedPoseManagerRequestBus overrides
        AZ::Outcome<void, FailedResult> RegisterNamedPose(AZ::EntityId& namedPoseEntityId) override;
        AZ::Outcome<void, FailedResult> UnregisterNamedPose(AZ::EntityId& namedPoseEntityId) override;
        AZ::Outcome<NamedPoseList, FailedResult> GetNamedPoses(const TagFilter& tags) override;
        AZ::Outcome<Bounds, FailedResult> GetNamedPoseBounds(const AZStd::string& name) override;

        AZStd::unordered_map<AZStd::string, AZ::EntityId> m_namedPoseToEntityId;
    };

} // namespace SimulationInterfaces
