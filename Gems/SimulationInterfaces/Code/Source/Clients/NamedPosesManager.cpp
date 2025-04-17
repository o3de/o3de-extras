/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Clients/NamedPosesManager.h"

#include "AzCore/Component/EntityId.h"
#include "CommonUtilities.h"
#include "SimulationInterfaces/NamedPoseManagerRequestBus.h"
#include "SimulationInterfaces/Result.h"
#include "SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Entity/GameEntityContextBus.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include <simulation_interfaces/msg/detail/result__struct.hpp>
#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/msg/simulator_features.hpp>
#include <simulation_interfaces/srv/spawn_entity.hpp>

namespace SimulationInterfaces
{

    AZ_COMPONENT_IMPL(NamedPoseManager, "NamedPoseManager", NamedPoseManagerTypeId);

    void NamedPoseManager::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<NamedPoseManager, AZ::Component>()->Version(0);
        }
    }

    void NamedPoseManager::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("NamedPoseManagerService"));
    }

    void NamedPoseManager::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("NamedPoseManagerService"));
    }

    void NamedPoseManager::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    void NamedPoseManager::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("PhysicsService"));
        dependent.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    NamedPoseManager::NamedPoseManager()
    {
        if (NamedPoseManagerRequestBusInterface::Get() == nullptr)
        {
            NamedPoseManagerRequestBusInterface::Register(this);
        }
    }

    NamedPoseManager::~NamedPoseManager()
    {
        if (NamedPoseManagerRequestBusInterface::Get() == this)
        {
            NamedPoseManagerRequestBusInterface::Unregister(this);
        }
    }

    void NamedPoseManager::Init()
    {
    }

    void NamedPoseManager::Activate()
    {
        SimulationFeaturesAggregatorRequestBus::Broadcast(
            &SimulationFeaturesAggregatorRequests::AddSimulationFeatures,
            AZStd::unordered_set<SimulationFeatureType>{
                simulation_interfaces::msg::SimulatorFeatures::NAMED_POSES,
                //  simulation_interfaces::msg::SimulatorFeatures::POSE_BOUNDS
            });

        NamedPoseManagerRequestBus::Handler::BusConnect();
    }

    void NamedPoseManager::Deactivate()
    {
        NamedPoseManagerRequestBus::Handler::BusDisconnect();
    }

    AZ::Outcome<void, FailedResult> NamedPoseManager::CreateNamedPose(NamedPose namedPose)
    {
        AZ::Entity* entityForNamedPose = nullptr;
        AzFramework::EntityContextId entityContextId;
        AzFramework::GameEntityContextRequestBus::BroadcastResult(
            entityContextId, &AzFramework::GameEntityContextRequestBus::Events::GetGameEntityContextId);
        AzFramework::EntityContextRequestBus::EventResult(
            entityForNamedPose, entityContextId, &AzFramework::EntityContextRequestBus::Events::CreateEntity, namedPose.m_name.c_str());
        if (!entityForNamedPose)
        {
            return AZ::Failure(
                FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Failed to create named pose entity"));
        }
        // add transform component

        auto transformComponent = entityForNamedPose->CreateComponent<AzFramework::TransformComponent>();
        if (!transformComponent)
        {
            return AZ::Failure(
                FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Failed to create transform component"));
        }
        // assign valid pose
        transformComponent->SetWorldTM(namedPose.m_pose);
        // add tag component
        auto tagComponent = entityForNamedPose->CreateComponent(AZ::Uuid("{0F16A377-EAA0-47D2-8472-9EAAA680B169}")); // Tag component uuid
        if (!tagComponent)
        {
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Failed to create tag component"));
        }
        AZ::EntityId namedPoseEntityId = entityForNamedPose->GetId();
        entityForNamedPose->Init();
        entityForNamedPose->Activate();

        // add tags to entity
        for (auto& tag : namedPose.m_tags)
        {
            LmbrCentral::TagComponentRequestBus::Event(
                namedPoseEntityId, &LmbrCentral::TagComponentRequestBus::Events::AddTag, LmbrCentral::Tag(tag));
        }

        return AZ::Success();
    }

    AZ::Outcome<NamedPoseSet, FailedResult> NamedPoseManager::GetNamedPoses(const TagFilter& tags)
    {
        auto filteredEntities = Utils::FilterEntitiesByTag(m_namedPoseToEntityId, tags);
        NamedPoseSet namedPosesSet;
        for (auto& [name, entityId] : filteredEntities)
        {
            AZ::Transform worldTM;
            AZ::TransformBus::EventResult(worldTM, entityId, &AZ::TransformBus::Events::GetWorldTM);

            namedPosesSet.emplace_back(name, "", m_humanReadableTags.at(entityId), worldTM);
        }
        return AZ::Success(namedPosesSet);
    }

    AZ::Outcome<Bounds, FailedResult> NamedPoseManager::GetNamedPoseBounds(const AZStd::string& name)
    {
        return AZ::Failure(FailedResult(
            simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED, "Getting named poses bound isn't currently supported"));
    }
} // namespace SimulationInterfaces
