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
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
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

    namespace HooksUtils::PluginParser
    {
        AZStd::string LastOnPath(AZStd::string path)
        {
            if (path.empty())
            {
                AZ_Warning("PluginParser", false, "Encountered empty parameter value while parsing URDF/SDF plugin.");
                return "";
            }
            if (path.contains('/'))
            {
                int startPos = path.find_last_of('/') + 1;
                path = path.substr(startPos, path.size() - startPos);
            }
            return path;
        }

        // Inserts name (key) and value (val) of given parameter to map.
        void ParseRegularContent(const sdf::Element& content, HooksUtils::PluginParams& remappings)
        {
            std::string contentName = content.GetName();
            AZStd::string key(contentName.c_str(), contentName.size());
            std::string contentValue = content.GetValue()->GetAsString();
            AZStd::string val(contentValue.c_str(), contentValue.size());
            remappings[key] = val;
        }

        // Parses parameters present in <ros> element, inserting them to map.
        void ParseRos2Content(const sdf::Element& rosContent, HooksUtils::PluginParams& remappings)
        {
            if (rosContent.GetName() != "remapping" && rosContent.GetName() != "argument")
            {
                // parameter other than <remapping> or <argument> can be handled as regular parameter
                ParseRegularContent(rosContent, remappings);
                return;
            }
            std::string contentValue = rosContent.GetValue()->GetAsString();

            if (contentValue.find_last_of('=') == -1 || contentValue.find_last_of(':') == -1)
            {
                AZ_Warning("PluginParser", false, "Encountered invalid remapping while parsing URDF/SDF plugin.");
                return;
            }

            // get new name of the topic
            int startVal = std::max(contentValue.find_last_of('/'), contentValue.find_last_of('=')) + 1;
            if (startVal >= contentValue.size())
            {
                AZ_Warning("PluginParser", false, "Encountered invalid (empty) remapping while parsing URDF/SDF plugin.");
                return;
            }
            std::string newTopic = contentValue.substr(startVal, contentValue.size() - startVal);

            // get previous name of the topic
            contentValue = contentValue.substr(0, contentValue.find_first_of(':'));
            int startKey = contentValue.find_last_of('/') + 1;
            std::string prevTopic = contentValue.substr(startKey, contentValue.size() - startKey);

            // insert data into the map - previous topic name as key and new topic name as val
            AZStd::string key(prevTopic.c_str(), prevTopic.size());
            AZStd::string val(newTopic.c_str(), newTopic.size());
            remappings[key] = val;
        }
    } // namespace HooksUtils::PluginParser

    void HooksUtils::ConfigureFrame(ROS2FrameEditorComponent& frameComponent, const HooksUtils::PluginParams& pluginParams)
    {
        if (pluginParams.contains("robotNamespace"))
        {
            frameComponent.UpdateNamespaceConfiguration(
                pluginParams.at("robotNamespace"), NamespaceConfiguration::NamespaceStrategy::Custom);
        }
        else if (pluginParams.contains("namespace"))
        {
            frameComponent.UpdateNamespaceConfiguration(
                PluginParser::LastOnPath(pluginParams.at("namespace")), NamespaceConfiguration::NamespaceStrategy::Custom);
        }
        if (pluginParams.contains("frameName"))
        {
            frameComponent.SetFrameID(pluginParams.at("frameName"));
        }
        else if (pluginParams.contains("frame_name"))
        {
            frameComponent.SetFrameID(pluginParams.at("frame_name"));
        }
    }

    void HooksUtils::ConfigureFrame(AZ::Component& frameComponent, const HooksUtils::PluginParams& pluginParams)
    {
        ConfigureFrame(*dynamic_cast<ROS2FrameEditorComponent*>(&frameComponent), pluginParams);
    }

    void HooksUtils::ConfigureFrame(AZ::Component* frameComponent, const HooksUtils::PluginParams& pluginParams)
    {
        if (frameComponent)
        {
            ConfigureFrame(*dynamic_cast<ROS2FrameEditorComponent*>(frameComponent), pluginParams);
        }
    }

    HooksUtils::PluginParams HooksUtils::GetPluginParams(const sdf::Plugin& plugin)
    {
        HooksUtils::PluginParams remappings;

        for (const auto& content : plugin.Contents())
        {
            std::string contentName = content->GetName();
            if (contentName == "ros")
            {
                // when <ros> tag is present, parse it's elements and insert them into the map
                auto rosContent = content->GetFirstElement();
                while (rosContent != nullptr)
                {
                    PluginParser::ParseRos2Content(*rosContent, remappings);
                    rosContent = rosContent->GetNextElement();
                }
            }
            else
            {
                PluginParser::ParseRegularContent(*content, remappings);
            }
        }

        return remappings;
    }

} // namespace ROS2::SDFormat
