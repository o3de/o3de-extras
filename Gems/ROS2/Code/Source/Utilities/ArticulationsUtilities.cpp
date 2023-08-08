/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzFramework/Physics/PhysicsScene.h>
#include "ArticulationsUtilities.h"
#include "Source/ArticulationLinkComponent.h"

namespace ROS2::Utils
{
    AZ::EntityId GetRootOfArticulation(AZ::EntityId entityId)
    {
        AZ::EntityId parentEntityId{ AZ::EntityId::InvalidEntityId };
        AZ::Entity* parentEntity = nullptr;
        AZ::Entity* thisEntity = nullptr;

        AZ::TransformBus::EventResult(parentEntityId, entityId, &AZ::TransformBus::Events::GetParentId);
        AZ::ComponentApplicationBus::BroadcastResult(parentEntity, &AZ::ComponentApplicationRequests::FindEntity, parentEntityId);
        AZ::ComponentApplicationBus::BroadcastResult(thisEntity, &AZ::ComponentApplicationRequests::FindEntity, entityId);

        constexpr AZ::Uuid EditorArticulationLinkComponentUuid{ "{7D23169B-3214-4A32-ABFC-FCCE6E31F2CF}" };

        if (parentEntity == nullptr)
        {
            return AZ::EntityId(AZ::EntityId::InvalidEntityId);
        }

        if (thisEntity->FindComponent<PhysX::ArticulationLinkComponent>())
        {
            // Get articulation link component, if not found for parent, return current entity - game only
            PhysX::ArticulationLinkComponent* component = parentEntity->FindComponent<PhysX::ArticulationLinkComponent>();
            if (component == nullptr)
            {
                return entityId;
            }
        }
        else if (thisEntity->FindComponent(EditorArticulationLinkComponentUuid)) // EditorArticulationLinkComponent
        {
            // Get articulation link component, if not found for parent, return current entity - Editor only
            AZ::Component* component = parentEntity->FindComponent(EditorArticulationLinkComponentUuid);
            if (component == nullptr)
            {
                return entityId;
            }
        }
        else
        {
            return AZ::EntityId(AZ::EntityId::InvalidEntityId);
        }
        return GetRootOfArticulation(parentEntity->GetId());
    }

    AZStd::unordered_map<AZ::EntityId, AzPhysics::SimulatedBodyHandle> GetSimulatedBodyHandles(AzPhysics::SceneHandle sceneHandle, AZ::EntityId entityId)
    {
        AZ::EntityId rootArticulationEntity = GetRootOfArticulation(entityId);
        AZ::Entity* rootEntity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(rootEntity, &AZ::ComponentApplicationRequests::FindEntity, rootArticulationEntity);

        PhysX::ArticulationLinkComponent* component = rootEntity->FindComponent<PhysX::ArticulationLinkComponent>();
        AZStd::vector<AzPhysics::SimulatedBodyHandle> articulationHandles = component->GetSimulatedBodyHandles();

        AZStd::unordered_map<AZ::EntityId, AzPhysics::SimulatedBodyHandle> result;
        for (auto& articulationHandle : articulationHandles)
        {
            auto* sceneInterface = AZ::Interface<AzPhysics::SceneInterface>::Get();
            auto* body = sceneInterface->GetSimulatedBodyFromHandle(sceneHandle, articulationHandle);
            result.insert({body->GetEntityId(), articulationHandle});
        }

        return result;
    }
} // namespace ROS2::Utils
