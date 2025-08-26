/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "Clients/NamedPosesManager.h"

#include "CommonUtilities.h"
#include "Components/NamedPoseComponent.h"
#include "SimulationInterfaces/Bounds.h"
#include "SimulationInterfaces/NamedPoseManagerRequestBus.h"
#include "SimulationInterfaces/Result.h"
#include "SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Entity/GameEntityContextBus.h>

#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/SimulatedBodies/StaticRigidBody.h>
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
                simulation_interfaces::msg::SimulatorFeatures::POSE_BOUNDS,
            });

        NamedPoseManagerRequestBus::Handler::BusConnect();
    }

    void NamedPoseManager::Deactivate()
    {
        NamedPoseManagerRequestBus::Handler::BusDisconnect();
    }

    AZ::Outcome<void, FailedResult> NamedPoseManager::RegisterNamedPose(AZ::EntityId& namedPoseEntityId)
    {
        // check if given entity has named pose component
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationBus::Events::FindEntity, namedPoseEntityId);
        if (!entity)
        {
            return AZ::Failure(
                FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Failed to gather entity with given id"));
        }
        auto namedPoseComponent = entity->FindComponent<NamedPoseComponent>();
        if (!namedPoseComponent)
        {
            return AZ::Failure(FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                "Entity with given Id doesn't have NamedPoseComponent, use CreateNamedPose method to create new pose"));
        }

        NamedPose configuration;
        NamedPoseComponentRequestBus::EventResult(configuration, namedPoseEntityId, &NamedPoseComponentRequests::GetConfiguration);
        if (m_namedPoseToEntityId.contains(configuration.m_name))
        {
            return AZ::Failure(FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                "Named pose with given name already exists in the record. Change name of your entity with NamedPoseComponent"));
        }
        m_namedPoseToEntityId[configuration.m_name] = namedPoseEntityId;
        return AZ::Success();
    }

    AZ::Outcome<void, FailedResult> NamedPoseManager::UnregisterNamedPose(AZ::EntityId& namedPoseEntityId)
    {
        if (!namedPoseEntityId.IsValid())
        {
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Given ID is invalid"));
        }
        AZStd::string name;
        AZ::ComponentApplicationBus::BroadcastResult(name, &AZ::ComponentApplicationRequests::GetEntityName, namedPoseEntityId);
        if (m_namedPoseToEntityId.contains(name))
        {
            m_namedPoseToEntityId.erase(name);
            return AZ::Success();
        }
        return AZ::Failure(
            FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Named pose with given id is not registered"));
    }

    AZ::Outcome<NamedPoseList, FailedResult> NamedPoseManager::GetNamedPoses(const TagFilter& tags)
    {
        auto filteredEntities = Utils::FilterEntitiesByTag(m_namedPoseToEntityId, tags);
        NamedPoseList namedPoseList;
        namedPoseList.reserve(filteredEntities.size());
        for (auto& [name, entityId] : filteredEntities)
        {
            NamedPose configuration;
            NamedPoseComponentRequestBus::EventResult(configuration, entityId, &NamedPoseComponentRequests::GetConfiguration);
            namedPoseList.push_back(configuration);
        }

        return AZ::Success(namedPoseList);
    }

    AZ::Outcome<Bounds, FailedResult> NamedPoseManager::GetNamedPoseBounds(const AZStd::string& name)
    {
        if (!m_namedPoseToEntityId.contains(name))
        {
            return AZ::Failure(
                FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "Named pose with given name is not registered"));
        }
        AZ::EntityId namedPoseEntityId = m_namedPoseToEntityId.at(name);
        if (!namedPoseEntityId.IsValid())
        {
            return AZ::Failure(FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                AZStd::string::format("Named pose with given name %s has invalid entity ID", name.c_str())));
        }
        // get simulated body
        auto simulatedBody = Utils::GetSimulatedBody(namedPoseEntityId);
        if (!simulatedBody.IsSuccess())
        {
            Bounds emptyBounds;
            emptyBounds.m_boundsType = simulation_interfaces::msg::Bounds::TYPE_EMPTY;
            return emptyBounds;
        }
        // only static rigid body makes sense to be named pose bounds.
        auto staticRigidBody = azdynamic_cast<AzPhysics::StaticRigidBody*>(simulatedBody.GetValue());

        if (!staticRigidBody)
        {
            return AZ::Failure(FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                "Named Pose in simulation interfaces supports bounds defined by the static rigid body"));
        }

        const AZ::u32 shapesCount = staticRigidBody->GetShapeCount();
        if (shapesCount == 0)
        {
            Bounds emptyBounds;
            emptyBounds.m_boundsType = simulation_interfaces::msg::Bounds::TYPE_EMPTY;
            return emptyBounds;
        }

        if (shapesCount > 1)
        {
            return AZ::Failure(FailedResult(
                simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                "Named Pose in simulation interfaces doesn't support multiple shapes"));
        }

        AZStd::shared_ptr<Physics::Shape> shape = staticRigidBody->GetShape(0);
        auto boundsOutput = Utils::ConvertPhysicalShapeToBounds(shape, namedPoseEntityId);
        if (boundsOutput.IsSuccess())
        {
            return AZ::Success(boundsOutput.GetValue());
        }

        return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, boundsOutput.GetError()));
    }
} // namespace SimulationInterfaces
