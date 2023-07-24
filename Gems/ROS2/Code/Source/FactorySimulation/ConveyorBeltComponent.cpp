/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "ConveyorBeltComponent.h"
#include <AtomLyIntegration/CommonFeatures/Material/MaterialComponentBus.h>
#include <AtomLyIntegration/CommonFeatures/Mesh/MeshComponentBus.h>
#include <AzCore/Asset/AssetSerializer.h>
#include <AzCore/Math/Matrix3x3.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzFramework/Physics/Common/PhysicsSimulatedBody.h>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>
#include <AzFramework/Physics/Material/PhysicsMaterialManager.h>
#include <AzFramework/Physics/Material/PhysicsMaterialSlots.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <LmbrCentral/Shape/SplineComponentBus.h>
#include <Source/RigidBodyComponent.h>
namespace ROS2
{
    static AZ::Data::AssetId GetDefaultPhysicsMaterialAssetId()
    {
        // Used for Edit Context.
        // When the physics material asset property doesn't have an asset assigned it
        // will show "(default)" to indicate that the default material will be used.
        if (auto* materialManager = AZ::Interface<Physics::MaterialManager>::Get())
        {
            if (AZStd::shared_ptr<Physics::Material> defaultMaterial = materialManager->GetDefaultMaterial())
            {
                return defaultMaterial->GetMaterialAsset().GetId();
            }
        }
        return {};
    }

    void ConveyorBeltComponent::Reflect(AZ::ReflectContext* context)
    {
        ConveyorBeltComponentConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ConveyorBeltComponent>()->Version(1)->Field("Configuration", &ConveyorBeltComponent::m_configuration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ConveyorBeltComponent>("Conveyor Belt Component", "Conveyor Belt Component.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ConveyorBeltComponent::m_configuration,
                        "Configuration",
                        "Configuration of the conveyor belt");
            }
        }
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<ConveyorBeltRequestBus>("ConveyorBeltKinematic", "ConveyorBeltRequestBus")
                ->Attribute(AZ::Edit::Attributes::Category, "ROS2/FactorySimulation/ConveyorBelt")
                ->Event("StartBelt", &ConveyorBeltRequestBus::Events::StartBelt)
                ->Event("StopBelt", &ConveyorBeltRequestBus::Events::StopBelt)
                ->Event("IsBeltStopped", &ConveyorBeltRequestBus::Events::IsBeltStopped);
        }
    }

    void ConveyorBeltComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("SplineService"));
    }

    void ConveyorBeltComponent::Activate()
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem, "No physics system");
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene interface");
        AzPhysics::SceneHandle defaultSceneHandle = sceneInterface->GetSceneHandle(AzPhysics::DefaultPhysicsSceneName);
        AZ_Assert(defaultSceneHandle != AzPhysics::InvalidSceneHandle, "Invalid default physics scene handle");
        m_sceneHandle = defaultSceneHandle;

        m_splineTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(m_splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);

        AZ::ConstSplinePtr splinePtr{ nullptr };
        LmbrCentral::SplineComponentRequestBus::EventResult(splinePtr, m_entity->GetId(), &LmbrCentral::SplineComponentRequests::GetSpline);
        AZ_Assert(splinePtr, "Unable to get spline for entity id (%s)", m_entity->GetId().ToString().c_str());

        if (splinePtr)
        {
            m_splineConsPtr = splinePtr;
            m_sceneFinishSimHandler = AzPhysics::SceneEvents::OnSceneSimulationFinishHandler(
                [this]([[maybe_unused]] AzPhysics::SceneHandle sceneHandle, float fixedDeltaTime)
                {
                    SpawnSegments(fixedDeltaTime);
                    MoveSegmentsPhysically(fixedDeltaTime);
                    DespawnSegments();
                },
                aznumeric_cast<int32_t>(AzPhysics::SceneEvents::PhysicsStartFinishSimulationPriority::Components));
            sceneInterface->RegisterSceneSimulationFinishHandler(defaultSceneHandle, m_sceneFinishSimHandler);
            const auto& [startPoint, endPoint] = GetStartAndEndPointOfBelt(splinePtr);
            m_startPoint = startPoint;
            m_endPoint = endPoint;
            m_splineLength = GetSplineLength(splinePtr);

            // initial segment population
            AZ_Assert(m_splineLength != 0.0f, "m_splineLength must be non-zero");
            const float normalizedDistanceStep = SegmentSeparation * m_configuration.m_segmentSize / m_splineLength;
            for (float normalizedIndex = 0.f; normalizedIndex < 1.f; normalizedIndex += normalizedDistanceStep)
            {
                m_conveyorSegments.push_back(CreateSegment(splinePtr, normalizedIndex));
            }
            AZ_Printf("ConveyorBeltComponent", "Initial Number of segments: %d", m_conveyorSegments.size());
            AZ::TickBus::Handler::BusConnect();
        }
        AZ::EntityBus::Handler::BusConnect(m_configuration.m_conveyorEntityId);
        ConveyorBeltRequestBus::Handler::BusConnect(m_configuration.m_conveyorEntityId);
    }

    void ConveyorBeltComponent::Deactivate()
    {
        if (m_sceneFinishSimHandler.IsConnected())
        {
            m_sceneFinishSimHandler.Disconnect();
        }
        ConveyorBeltRequestBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
    }

    AZ::Vector3 ConveyorBeltComponent::GetLocationOfSegment(const AzPhysics::SimulatedBodyHandle handle)
    {
        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface, "No scene interface");
        auto* body = azdynamic_cast<AzPhysics::RigidBody*>(sceneInterface->GetSimulatedBodyFromHandle(m_sceneHandle, handle));
        AZ_Assert(body, "No valid body found");
        if (body)
        {
            AZ::Vector3 beginOfSegments = body->GetPosition();
            return beginOfSegments;
        }
        return AZ::Vector3::CreateZero();
    }
    void ConveyorBeltComponent::OnEntityActivated(const AZ::EntityId& entityId)
    {
        if (m_configuration.m_conveyorEntityId.IsValid() && entityId == m_configuration.m_conveyorEntityId)
        {
            AZ::Render::MaterialComponentRequestBus::EventResult(
                m_graphhicalMaterialId,
                m_configuration.m_conveyorEntityId,
                &AZ::Render::MaterialComponentRequestBus::Events::FindMaterialAssignmentId,
                -1,
                m_configuration.m_graphicalMaterialSlot);

            AZStd::string foundMaterialName;
            AZ::Render::MaterialComponentRequestBus::EventResult(
                foundMaterialName,
                m_configuration.m_conveyorEntityId,
                &AZ::Render::MaterialComponentRequestBus::Events::GetMaterialLabel,
                m_graphhicalMaterialId);

            AZ_Warning(
                "ConveyorBeltComponent",
                m_configuration.m_graphicalMaterialSlot == foundMaterialName,
                "Material slot \"%s\" not found on entity %s, found material slot \"%s\" instead.",
                m_configuration.m_graphicalMaterialSlot.c_str(),
                m_configuration.m_conveyorEntityId.ToString().c_str(),
                foundMaterialName.c_str());
            AZ_Assert(m_graphhicalMaterialId.IsSlotIdOnly(), "m_graphhicalMaterialId should be a slot id only");
        }
    }

    AZStd::pair<float, AzPhysics::SimulatedBodyHandle> ConveyorBeltComponent::CreateSegment(
        AZ::ConstSplinePtr splinePtr, float normalizedLocation)
    {
        AzPhysics::SystemInterface* physicsSystem = AZ::Interface<AzPhysics::SystemInterface>::Get();
        AZ_Assert(physicsSystem != nullptr, "Unable to get Physics System");

        auto colliderConfiguration = AZStd::make_shared<Physics::ColliderConfiguration>();
        colliderConfiguration->m_isInSceneQueries = false;
        colliderConfiguration->m_materialSlots.SetMaterialAsset(0, m_configuration.m_materialAsset);
        colliderConfiguration->m_rotation = AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisX(), AZ::DegToRad(90.0f));
        auto shapeConfiguration =
            AZStd::make_shared<Physics::CapsuleShapeConfiguration>(m_configuration.m_beltWidth, m_configuration.m_segmentSize / 2.0f);
        const auto transform = GetTransformFromSpline(m_splineConsPtr, normalizedLocation);
        AzPhysics::RigidBodyConfiguration conveyorSegmentRigidBodyConfig;
        conveyorSegmentRigidBodyConfig.m_kinematic = true;
        conveyorSegmentRigidBodyConfig.m_position = transform.GetTranslation();
        conveyorSegmentRigidBodyConfig.m_orientation = transform.GetRotation();
        conveyorSegmentRigidBodyConfig.m_colliderAndShapeData = AzPhysics::ShapeColliderPair(colliderConfiguration, shapeConfiguration);
        conveyorSegmentRigidBodyConfig.m_computeCenterOfMass = true;
        conveyorSegmentRigidBodyConfig.m_computeInertiaTensor = true;
        conveyorSegmentRigidBodyConfig.m_startSimulationEnabled = true;
        conveyorSegmentRigidBodyConfig.m_computeMass = false;
        conveyorSegmentRigidBodyConfig.m_mass = 1.0f;
        conveyorSegmentRigidBodyConfig.m_entityId = GetEntityId();
        conveyorSegmentRigidBodyConfig.m_debugName = "ConveyorBeltSegment";
        AzPhysics::SimulatedBodyHandle handle = physicsSystem->GetScene(m_sceneHandle)->AddSimulatedBody(&conveyorSegmentRigidBodyConfig);
        AZ_Assert(handle == AzPhysics::InvalidSimulatedBodyHandle, "Body created with invalid handle");
        return AZStd::make_pair(normalizedLocation, handle);
    }

    void ConveyorBeltComponent::OnTick(float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        MoveSegmentsGraphically(deltaTime);
    }

    void ConveyorBeltComponent::StartBelt()
    {
        m_beltStopped = false;
    }

    void ConveyorBeltComponent::StopBelt()
    {
        m_beltStopped = true;
    }

    bool ConveyorBeltComponent::IsBeltStopped()
    {
        return m_beltStopped;
    }

    AZStd::pair<AZ::Vector3, AZ::Vector3> ConveyorBeltComponent::GetStartAndEndPointOfBelt(AZ::ConstSplinePtr splinePtr)
    {
        AZ::Transform splineTransform = AZ::Transform::Identity();
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);
        const AZ::SplineAddress addressBegin = splinePtr->GetAddressByFraction(0.f);
        const AZ::SplineAddress addressEnd = splinePtr->GetAddressByFraction(1.f);
        const AZ::Vector3 posBegin = splinePtr->GetPosition(addressBegin);
        const AZ::Vector3 posEnd = splinePtr->GetPosition(addressEnd);
        return { m_splineTransform.TransformPoint(posBegin), m_splineTransform.TransformPoint(posEnd) };
    }

    AZ::Transform ConveyorBeltComponent::GetTransformFromSpline(AZ::ConstSplinePtr splinePtr, float distanceNormalized)
    {
        AZ_Assert(splinePtr, "Spline pointer is null");
        AZ::SplineAddress address = splinePtr->GetAddressByFraction(distanceNormalized);
        const AZ::Vector3& p = splinePtr->GetPosition(address);

        // construct the rotation matrix from three orthogonal vectors.
        const AZ::Vector3& v1 = splinePtr->GetTangent(address);
        const AZ::Vector3& v2 = splinePtr->GetNormal(address);
        const AZ::Vector3 v3 = v1.Cross(v2);

        AZ::Matrix3x3 rotationMatrix = AZ::Matrix3x3::CreateFromColumns(v1, v2, v3);
        AZ_Assert(rotationMatrix.IsOrthogonal(0.001f), "Rotation matrix is not orthogonal");
        rotationMatrix.Orthogonalize();
        AZ::Transform transform = AZ::Transform::CreateFromMatrix3x3AndTranslation(
            rotationMatrix, p - AZ::Vector3::CreateAxisZ(m_configuration.m_segmentSize / 2.f));
        return m_splineTransform * transform;
    }

    float ConveyorBeltComponent::GetSplineLength(AZ::ConstSplinePtr splinePtr)
    {
        AZ_Assert(splinePtr, "Spline pointer is null");
        AZ::Transform splineTransform = AZ::Transform::Identity();
        AZ::TransformBus::EventResult(splineTransform, m_entity->GetId(), &AZ::TransformBus::Events::GetWorldTM);
        const AZ::SplineAddress addressEnd = splinePtr->GetAddressByFraction(1.f);
        return splinePtr->GetLength(addressEnd);
    }

    void ConveyorBeltComponent::MoveSegmentsGraphically(float deltaTime)
    {
        if (m_beltStopped)
        { // Do not move texture when stopped
            return;
        }

        if (!m_configuration.m_conveyorEntityId.IsValid())
        {
            return;
        }

        // Animate texture

        m_textureOffset += deltaTime * m_configuration.m_speed * m_configuration.m_textureScale;

        AZ::Render::MaterialComponentRequestBus::Event(
            m_configuration.m_conveyorEntityId,
            &AZ::Render::MaterialComponentRequestBus::Events::SetPropertyValueT<float>,
            m_graphhicalMaterialId,
            "uv.offsetU",
            m_textureOffset);
    }

    void ConveyorBeltComponent::DespawnSegments()
    {
        bool wasSegmentRemoved = false;
        for (auto& [pos, handle] : m_conveyorSegments)
        {
            if (pos > 1.0f)
            {
                AZ::Interface<AzPhysics::SceneInterface>::Get()->RemoveSimulatedBody(m_sceneHandle, handle);
                handle = AzPhysics::InvalidSimulatedBodyHandle;
                wasSegmentRemoved = true;
            }
        }
        if (wasSegmentRemoved)
        {
            const auto isInvalidHandle = [](const auto& pair)
            {
                return pair.second == AzPhysics::InvalidSimulatedBodyHandle;
            };
            // clear object handle from cache
            m_conveyorSegments.erase(
                AZStd::remove_if(m_conveyorSegments.begin(), m_conveyorSegments.end(), isInvalidHandle), m_conveyorSegments.end());
        }
    }

    void ConveyorBeltComponent::MoveSegmentsPhysically(float fixedDeltaTime)
    {
        if (m_beltStopped)
        { // Do not move segments when stopped
            return;
        }

        AzPhysics::SceneInterface* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
        AZ_Assert(sceneInterface != nullptr, "Unable to get Scene Interface");
        // update positions of the segments
        for (auto& [pos, handle] : m_conveyorSegments)
        {
            auto* body = azdynamic_cast<AzPhysics::RigidBody*>(sceneInterface->GetSimulatedBodyFromHandle(m_sceneHandle, handle));
            if (body)
            {
                pos += m_configuration.m_speed * fixedDeltaTime / m_splineLength;
                auto transform = GetTransformFromSpline(m_splineConsPtr, pos);
                transform.SetTranslation(transform.GetTranslation());
                body->SetKinematicTarget(transform);
            }
        }
    }

    void ConveyorBeltComponent::SpawnSegments(float deltaTime)
    {
        m_deltaTimeFromLastSpawn += deltaTime;
        if (m_conveyorSegments.empty())
        {
            m_conveyorSegments.push_back(CreateSegment(m_splineConsPtr, 0.f));
            return;
        }
        if (m_deltaTimeFromLastSpawn > SegmentSeparation * m_configuration.m_segmentSize / m_configuration.m_speed)
        {
            m_deltaTimeFromLastSpawn = 0.f;
            m_conveyorSegments.push_back(CreateSegment(m_splineConsPtr, 0.f));
        }
    }

} // namespace ROS2
