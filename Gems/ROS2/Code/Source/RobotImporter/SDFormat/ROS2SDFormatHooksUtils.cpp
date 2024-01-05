/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2SDFormatHooksUtils.h"
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <RobotImporter/Utils/TypeConversions.h>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>
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
        if (entity == nullptr)
        {
            AZ_Warning("HooksUtils", false, "Cannot switch on motor in wheel joint. Entity was not created successfully.");
            return;
        }

        // Enable motor in hinge joint (only if articulations are not enabled)
        PhysX::EditorHingeJointComponent* jointComponent = entity->FindComponent<PhysX::EditorHingeJointComponent>();
        if (jointComponent != nullptr)
        {
            entity->Activate();
            if (entity->GetState() == AZ::Entity::State::Active)
            {
                PhysX::EditorJointRequestBus::Event(
                    AZ::EntityComponentIdPair(entityId, jointComponent->GetId()),
                    &PhysX::EditorJointRequests::SetBoolValue,
                    PhysX::JointsComponentModeCommon::ParameterNames::EnableMotor,
                    true);
                entity->Deactivate();
            }
        }
    }

    void HooksUtils::SetSensorEntityTransform(AZ::Entity& entity, const sdf::Sensor& sdfSensor)
    {
        const auto sensorSemanticPose = sdfSensor.SemanticPose();
        AZ::Transform tf = Utils::GetLocalTransformURDF(sensorSemanticPose);
        auto* transformInterface = entity.FindComponent<AzToolsFramework::Components::TransformComponent>();
        if (transformInterface)
        {
            AZ_Trace(
                "CreatePrefabFromUrdfOrSdf",
                "Setting transform %s to [%f %f %f] [%f %f %f %f]\n",
                sdfSensor.Name().c_str(),
                tf.GetTranslation().GetX(),
                tf.GetTranslation().GetY(),
                tf.GetTranslation().GetZ(),
                tf.GetRotation().GetX(),
                tf.GetRotation().GetY(),
                tf.GetRotation().GetZ(),
                tf.GetRotation().GetW());
            transformInterface->SetLocalTM(tf);
        }
        else
        {
            AZ_Trace(
                "CreatePrefabFromUrdfOrSdf", "Setting transform failed: %s does not have transform interface\n", sdfSensor.Name().c_str());
        }
    }
} // namespace ROS2::SDFormat
