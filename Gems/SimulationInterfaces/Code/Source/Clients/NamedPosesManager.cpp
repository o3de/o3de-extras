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
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/Shape.h>
#include <AzFramework/Physics/ShapeConfiguration.h>
#include <AzFramework/Physics/SimulatedBodies/StaticRigidBody.h>
#include <LmbrCentral/Scripting/TagComponentBus.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include <simulation_interfaces/msg/result.hpp>
#include <simulation_interfaces/msg/simulator_features.hpp>
#include <simulation_interfaces/srv/spawn_entity.hpp>

namespace SimulationInterfaces
{
    namespace
    {
        AZ::Outcome<AzPhysics::SimulatedBody*, AZStd::string> GetSimulatedBody(AZ::EntityId entityId)
        {
            AzPhysics::SimulatedBody* parentSimulatedBody = nullptr;
            AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
                parentSimulatedBody, entityId, &AzPhysics::SimulatedBodyComponentRequests::GetSimulatedBody);
            if (parentSimulatedBody == nullptr)
            {
                auto msg = AZStd::string::format("Entity's simulated body doesn't exist");
                return AZ::Failure(msg);
            }
            if (parentSimulatedBody->m_bodyHandle == AzPhysics::InvalidSimulatedBodyHandle)
            {
                auto msg = AZStd::string::format("Entity is not valid simulated body");
                return AZ::Failure(msg);
            }
            return AZ::Success(parentSimulatedBody);
        }
    } // namespace

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

    AZ::Outcome<void, FailedResult> NamedPoseManager::RegisterNamedPose(AZ::EntityId namedPoseEntityId)
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

    AZ::Outcome<void, FailedResult> NamedPoseManager::UnregisterNamedPose(AZ::EntityId namedPoseEntityId)
    {
        if (!namedPoseEntityId.IsValid())
        {
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, "given ID is invalid"));
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
        auto simulatedBody = GetSimulatedBody(namedPoseEntityId);
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
        auto config = shape->GetShapeConfiguration();
        auto shapeType = config->GetShapeType();

        // get final collider transform including entity TM and offsets
        AZStd::pair<AZ::Vector3, AZ::Quaternion> colliderOffsets = shape->GetLocalPose();
        AZ::Transform offsetTransform = AZ::Transform::CreateFromQuaternionAndTranslation(colliderOffsets.second, colliderOffsets.first);
        AZ::Transform entityTransform;
        AZ::TransformBus::EventResult(entityTransform, namedPoseEntityId, &AZ::TransformBus::Events::GetWorldTM);
        AZ::Transform colliderAbsoluteTransform = entityTransform * offsetTransform;

        Bounds bounds;
        switch (shapeType)
        {
        case Physics::ShapeType::Box:
            {
                bounds.m_boundsType = simulation_interfaces::msg::Bounds::TYPE_BOX;

                auto boxConfig = azdynamic_cast<Physics::BoxShapeConfiguration*>(config);
                bounds.m_points.emplace_back(colliderAbsoluteTransform.GetTranslation() + (boxConfig->m_dimensions / 2)); // upper Right
                bounds.m_points.emplace_back(colliderAbsoluteTransform.GetTranslation() - (boxConfig->m_dimensions / 2)); // bottom left
                return bounds;
            }
        case Physics::ShapeType::Sphere:
            {
                bounds.m_boundsType = simulation_interfaces::msg::Bounds::TYPE_SPHERE;
                auto sphereConfig = azdynamic_cast<Physics::SphereShapeConfiguration*>(config);
                bounds.m_points.emplace_back(colliderAbsoluteTransform.GetTranslation()); // sphere center
                bounds.m_points.emplace_back(sphereConfig->m_radius, 0.f, 0.f); // radius and two ignored fields
                return bounds;
            }
        // this type of collider is currently unsupported by the PhysX engine, but this implementation uses provided abstractions and is
        // independent  from selected physic engine.
        case Physics::ShapeType::ConvexHull:
            {
                bounds.m_boundsType = simulation_interfaces::msg::Bounds::TYPE_CONVEX_HULL;
                AZStd::vector<AZ::Vector3> vertices;
                AZStd::vector<AZ::u32> indices;
                shape->GetGeometry(vertices, indices);
                for (auto& vertex : vertices)
                {
                    bounds.m_points.emplace_back(colliderAbsoluteTransform.TransformPoint(vertex));
                }
                return bounds;
            }
        default:
            {
                return AZ::Failure(FailedResult(
                    simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED,
                    AZStd::string::format(
                        "Passed shape type with id %d is not supported by simulation interfaces", static_cast<AZ::u8>(shapeType))));
            }
        }
    }
} // namespace SimulationInterfaces
