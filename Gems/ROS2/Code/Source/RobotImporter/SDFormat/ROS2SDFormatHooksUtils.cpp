/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SDFormatHooksUtils.h"
#include <ROS2/Communication/TopicConfiguration.h>
#include <VehicleDynamics/WheelControllerComponent.h>

#include <sdf/Joint.hh>

namespace ROS2::SDFormat
{
    void HooksUtils::AddTopicConfiguration(
        SensorConfiguration& sensorConfig, const AZStd::string& topic, const AZStd::string& messageType, const AZStd::string& configName)
    {
        TopicConfiguration config;
        config.m_topic = topic;
        config.m_type = messageType;
        sensorConfig.m_publishersConfigurations.insert(AZStd::make_pair(configName, config));
    }

    AZ::EntityId HooksUtils::GetJointEntityId(
        const std::string& jointName, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
    {
        const auto jointPtr = sdfModel.JointByName(jointName);
        if (jointPtr != nullptr)
        {
            const auto linkName(jointPtr->ChildName().c_str());
            const auto linkPtr = sdfModel.LinkByName(linkName);
            if (linkPtr != nullptr && createdEntities.contains(linkPtr))
            {
                const auto& entityResult = createdEntities.at(linkPtr);
                return entityResult.IsSuccess() ? entityResult.GetValue() : AZ::EntityId();
            }
        }

        return AZ::EntityId();
    }

    void HooksUtils::EnableMotor(const AZ::EntityId& entityId)
    {
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
        if (entity != nullptr)
        {
            PhysX::EditorHingeJointComponent* jointComponent = entity->FindComponent<PhysX::EditorHingeJointComponent>();
            if (jointComponent != nullptr)
            {
                jointComponent->Activate();
                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(entityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetBoolValue,
                    PhysX::JointsComponentModeCommon::ParameterNames::EnableMotor,
                    true);
                jointComponent->Deactivate();
                return;
            }

            PhysX::EditorArticulationLinkComponent* articulationComponent = entity->FindComponent<PhysX::EditorArticulationLinkComponent>();
            if (articulationComponent != nullptr)
            {
                const auto wheelComponentPtr = HooksUtils::CreateComponent<VehicleDynamics::WheelControllerComponent>(*entity);
                AZ_Warning("HooksUtils", wheelComponentPtr != nullptr, "Cannot create WheelControllerComponent in articulation link.");
                return;
            }
        }

        AZ_Warning("HooksUtils", false, "Cannot switch on motor in wheel joint. Joint does not exist.");
    }
} // namespace ROS2::SDFormat
