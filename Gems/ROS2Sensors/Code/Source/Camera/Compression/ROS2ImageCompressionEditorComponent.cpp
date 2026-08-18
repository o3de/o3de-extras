/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2ImageCompressionEditorComponent.h"
#include "Camera/ROS2CameraSensorEditorComponent.h"
#include "ROS2ImageCompressionComponent.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2Sensors
{
    ROS2ImageCompressionEditorComponent::ROS2ImageCompressionEditorComponent(const ImageCompressionConfiguration& configuration)
        : m_configuration(configuration)
    {
    }

    void ROS2ImageCompressionEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ImageCompressionEditorComponent, AzToolsFramework::Components::EditorComponentBase>()
                ->Version(0)
                ->Field("configuration", &ROS2ImageCompressionEditorComponent::m_configuration);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<ROS2ImageCompressionEditorComponent>("Image Compression Component", "Publishes a compressed image")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/ROS2CameraSensor.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/ROS2CameraSensor.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2ImageCompressionEditorComponent::m_configuration,
                        "Compression configuration",
                        "Camera channel to mirror and the codec settings used for it")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    void ROS2ImageCompressionEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2CameraSensor"));
        required.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2ImageCompressionEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        const auto editorCameraComponent = this->GetEntity()->FindComponent<ROS2CameraSensorEditorComponent>();
        AZ_Assert(editorCameraComponent, "Entity has no ROS2CameraSensorEditorComponent");
        const AZStd::map<AZStd::string, ROS2::TopicConfiguration>& topicConfig =
            editorCameraComponent->GetTopicConfiguration().m_publishersConfigurations;

        AZ_Assert(topicConfig.contains(ROS2Sensors::CameraConstants::ColorImageConfig), "No color topic");
        AZ_Assert(topicConfig.contains(ROS2Sensors::CameraConstants::DepthImageConfig), "No depth topic");

        ROS2::TopicConfiguration orgTopicConfig;
        if (m_configuration.m_channel == ImageCompressionConfiguration::Channel::Color)
        {
            orgTopicConfig = topicConfig.find(ROS2Sensors::CameraConstants::ColorImageConfig)->second;
        }
        else
        {
            orgTopicConfig = topicConfig.find(ROS2Sensors::CameraConstants::DepthImageConfig)->second;
        }
        gameEntity->CreateComponent<ROS2ImageCompressionComponent>(m_configuration.m_channel, orgTopicConfig, m_configuration.m_settings);
    }
} // namespace ROS2Sensors
