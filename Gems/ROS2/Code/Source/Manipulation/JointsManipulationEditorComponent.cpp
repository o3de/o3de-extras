/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "JointsManipulationEditorComponent.h"
#include "JointsManipulationComponent.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Manipulation/Controllers/JointsPositionControllerRequests.h>
#include <ROS2/Manipulation/JointInfo.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <Source/ArticulationLinkComponent.h>
#include <Source/HingeJointComponent.h>
#include <AzCore/Serialization/Json/RegistrationContext.h>

namespace ROS2
{
    AZ::JsonSerializationResult::Result JointsManipulationEditorComponentConfigSerializer::Load(
        void* outputValue,
        [[maybe_unused]] const AZ::Uuid& outputValueTypeId,
        const rapidjson::Value& inputValue,
        AZ::JsonDeserializerContext& context)
    {
        rapidjson::Value::ConstMemberIterator itr = inputValue.FindMember("Initial positions");
        if (itr != inputValue.MemberEnd() && !itr->value.IsArray())
        {
            AZ_Error(
                "JointsManipulationEditorComponent",
                false,
                "An old version of the JointsManipulationEditorComponent is being loaded. Manual conversion is required. The conversion script is "
                "located in: "
                "o3de-extras/Gems/ROS2/Code/Source/Manipulation/Conversions/JointsPositionsConversion.py");

            return context.Report(AZ::JsonSerializationResult::Tasks::ReadField, AZ::JsonSerializationResult::Outcomes::Catastrophic,
                "Old version of JointsManipulationEditorComponent Json was detected.");
        }

        return AZ::BaseJsonSerializer::Load(outputValue, outputValueTypeId, inputValue, context);
    }

    AZ_CLASS_ALLOCATOR_IMPL(JointsManipulationEditorComponentConfigSerializer, AZ::SystemAllocator);

    JointsManipulationEditorComponent::JointsManipulationEditorComponent()
    {
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_type = "sensor_msgs::msg::JointState";
        m_jointStatePublisherConfiguration.m_topicConfiguration.m_topic = "joint_states";
        m_jointStatePublisherConfiguration.m_frequency = 25.0f;

        m_jointPositionsSubscriberConfiguration.m_subscribe = false;
        m_jointPositionsSubscriberConfiguration.m_topicConfiguration.m_type = "std_msgs::msg::Float64MultiArray";
        m_jointPositionsSubscriberConfiguration.m_topicConfiguration.m_topic = "/position_controller/commands";
    }

    void JointsManipulationEditorComponent::BuildGameEntity(AZ::Entity* gameEntity)
    {
        gameEntity->CreateComponent<JointsManipulationComponent>(
            m_jointStatePublisherConfiguration, m_jointPositionsSubscriberConfiguration, m_initialPositions);
    }

    void JointsManipulationEditorComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Frame"));
        required.push_back(AZ_CRC_CE("JointsControllerService"));
    }

    void JointsManipulationEditorComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationEditorComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("JointsManipulationService"));
    }

    void JointsManipulationEditorComponent::Reflect(AZ::ReflectContext* context)
    {
        JointNamePositionPair::Reflect(context);

        if (auto jsonContext = azrtti_cast<AZ::JsonRegistrationContext*>(context))
        {
            jsonContext->Serializer<JointsManipulationEditorComponentConfigSerializer>()->HandlesType<JointsManipulationEditorComponent>();
        }

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<JointsManipulationEditorComponent, AZ::Component>()
                ->Version(1)
                ->Field("JointStatePublisherConfiguration", &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration)
                ->Field("JointPositionsSubscriberConfiguration", &JointsManipulationEditorComponent::m_jointPositionsSubscriberConfiguration)
                ->Field("Initial positions", &JointsManipulationEditorComponent::m_initialPositions);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<JointsManipulationEditorComponent>("JointsManipulationEditorComponent", "Component for manipulation of joints")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/JointsManipulationEditorComponent.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/JointsManipulationEditorComponent.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_jointStatePublisherConfiguration,
                        "Joint State Publisher",
                        "Configuration of Joint State Publisher")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_jointPositionsSubscriberConfiguration,
                        "Joint Positions Subscriber",
                        "Configuration of Joint Positions Subscriber")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &JointsManipulationEditorComponent::m_initialPositions,
                        "Joints order and initial positions",
                        "Initial positions of all the joints and order for their control via subscriber");
            }
        }
    }
} // namespace ROS2
