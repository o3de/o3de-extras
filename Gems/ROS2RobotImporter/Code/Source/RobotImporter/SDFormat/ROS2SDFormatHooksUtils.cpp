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
#include <ROS2/Utilities/ROS2Names.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <RobotImporter/Utils/TypeConversions.h>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

#include <sdf/Element.hh>
#include <sdf/Joint.hh>

namespace ROS2RobotImporter::SDFormat
{
    void HooksUtils::AddTopicConfiguration(
        ROS2::SensorConfiguration& sensorConfig,
        const AZStd::string& topic,
        const AZStd::string& messageType,
        const AZStd::string& configName)
    {
        ROS2::TopicConfiguration config;
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
        // Inserts name (key) and value (val) of given parameter to map.
        void ParseRegularContent(const sdf::Element& content, HooksUtils::PluginParams& remappings)
        {
            const AZStd::string contentName = content.GetName().c_str();
            const sdf::ParamPtr contentValuePtr = content.GetValue();
            if (!contentValuePtr || contentName.empty())
            {
                AZ_Warning("PluginParser", false, "Encountered invalid (empty) remapping while parsing URDF/SDF plugin.");
                return;
            }

            const AZStd::string contentValue = contentValuePtr->GetAsString().c_str();
            if (!contentValue.empty())
            {
                remappings[contentName] = contentValue;
            }
        }

        // Parses parameters present in ros element, inserting them to the map.
        void ParseRos2Remapping(const sdf::Element& rosContent, HooksUtils::PluginParams& remappings)
        {
            if (rosContent.GetName() != "remapping" && rosContent.GetName() != "argument")
            {
                // parameter other than remapping or argument can be handled as regular parameter
                ParseRegularContent(rosContent, remappings);
                return;
            }

            const sdf::ParamPtr contentValuePtr = rosContent.GetValue();
            if (!contentValuePtr)
            {
                AZ_Warning("PluginParser", false, "ROS 2 content in parsing URDF/SDF plugin data not available.");
                return;
            }

            AZStd::string contentValue = contentValuePtr->GetAsString().c_str();

            if (contentValue.find_last_of('=') == AZStd::string::npos || contentValue.find_last_of(':') == AZStd::string::npos)
            {
                AZ_Warning("PluginParser", false, "Encountered invalid remapping while parsing URDF/SDF plugin.");
                return;
            }

            // get new name of the topic
            int startVal = contentValue.find_last_of('=');
            if (contentValue.find_last_of('/') != AZStd::string::npos && contentValue.find_last_of('/') > startVal)
            {
                startVal = contentValue.find_last_of("/");
            }
            startVal += 1;

            if (startVal >= contentValue.size())
            {
                AZ_Warning("PluginParser", false, "Encountered invalid (empty) remapping while parsing URDF/SDF plugin.");
                return;
            }
            AZStd::string newTopic = contentValue.substr(startVal, contentValue.size() - startVal);

            // get previous name of the topic
            contentValue = contentValue.substr(0, contentValue.find_first_of(':'));

            int startKey = contentValue.find_last_of('/') != std::string::npos ? contentValue.find_last_of('/') + 1 : 0;
            if (startKey >= contentValue.size())
            {
                AZ_Warning("PluginParser", false, "Encountered invalid (empty) remapping while parsing URDF/SDF plugin.");
                return;
            }
            AZStd::string prevTopic = contentValue.substr(startKey, contentValue.size() - startKey);

            if (ROS2::ROS2Names::ValidateTopic(newTopic).IsSuccess() && ROS2::ROS2Names::ValidateTopic(prevTopic).IsSuccess())
            {
                remappings[prevTopic] = newTopic;
            }
            else
            {
                AZ_Warning("PluginParser", false, "Encountered invalid topic name while parsing URDF/SDF plugin.");
            }
        }
    } // namespace HooksUtils::PluginParser

    ROS2::ROS2FrameConfiguration HooksUtils::GetFrameConfiguration(const HooksUtils::PluginParams& pluginParams)
    {
        ROS2::ROS2FrameConfiguration frameConfiguration;

        const static AZStd::vector<AZStd::string> namespaceRemapNames = { "robotNamespace", "namespace" };
        const AZStd::string remappedNamespace = HooksUtils::ValueOfAny(pluginParams, namespaceRemapNames);
        if (!ROS2::ROS2Names::ValidateNamespace(remappedNamespace).IsSuccess())
        {
            AZ_Warning("PluginParser", false, "Encountered invalid namespace name while parsing URDF/SDF plugin.");
        }
        else if (!remappedNamespace.empty())
        {
            frameConfiguration.m_namespaceConfiguration.SetNamespace(
                remappedNamespace, ROS2::NamespaceConfiguration::NamespaceStrategy::Custom);
        }

        if (pluginParams.contains("frameName"))
        {
            frameConfiguration.m_frameName = pluginParams.at("frameName");
        }
        else if (pluginParams.contains("frame_name"))
        {
            frameConfiguration.m_frameName = pluginParams.at("frame_name");
        }

        return frameConfiguration;
    }

    HooksUtils::PluginParams HooksUtils::GetPluginParams(const sdf::Plugins& plugins)
    {
        HooksUtils::PluginParams remapping;
        if (plugins.empty())
        {
            return remapping;
        }

        const auto plugin = plugins[0];

        for (const auto& content : plugin.Contents())
        {
            std::string contentName = content->GetName();
            if (contentName == "ros")
            {
                // when ros tag is present, parse it's elements and insert them into the map
                auto rosContent = content->GetFirstElement();
                while (rosContent != nullptr)
                {
                    PluginParser::ParseRos2Remapping(*rosContent, remapping);
                    rosContent = rosContent->GetNextElement();
                }
            }
            else
            {
                PluginParser::ParseRegularContent(*content, remapping);
            }
        }

        return remapping;
    }

    AZStd::string HooksUtils::ValueOfAny(
        const HooksUtils::PluginParams& pluginParams, const AZStd::vector<AZStd::string>& paramNames, const AZStd::string& defaultVal)
    {
        for (const auto& paramName : paramNames)
        {
            if (pluginParams.contains(paramName))
            {
                const AZStd::string paramValue = pluginParams.at(paramName);
                return AZ::IO::PathView(paramValue).Filename().String();
            }
        }
        return defaultVal;
    }

    AZStd::string HooksUtils::GetTopicName(const PluginParams& pluginParams, sdf::ElementPtr element, const AZStd::string& defaultVal)
    {
        if (element->HasElement("topic"))
        {
            return element->Get<std::string>("topic").c_str();
        }

        const static AZStd::vector<AZStd::string> remapParamNames = { "topicName", "out" };
        return ValueOfAny(pluginParams, remapParamNames, defaultVal);
    }

    float HooksUtils::GetFrequency(const PluginParams& pluginParams, const float defaultVal)
    {
        const static AZStd::vector<AZStd::string> frequencyParamNames = { "updateRate", "update_rate" };
        return AZStd::stof(ValueOfAny(pluginParams, frequencyParamNames, AZStd::to_string(defaultVal)));
    }

} // namespace ROS2RobotImporter::SDFormat
