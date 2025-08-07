/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <signal.h>

#include "ROS2SystemComponent.h"
#include <ROS2/Communication/PublisherConfiguration.h>
#include <ROS2/Communication/QoS.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/ROS2TypeIds.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2/Utilities/ROS2Conversions.h>

#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/Time/ITime.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/sort.h>
#include <AzCore/std/string/string_view.h>
#include <AzFramework/API/ApplicationAPI.h>

namespace ROS2
{

    AZ_COMPONENT_IMPL(ROS2SystemComponent, "ROS2SystemComponent", ROS2SystemComponentTypeId);

    void ROS2SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        // Reflect structs not strictly owned by any single component
        QoS::Reflect(context);
        TopicConfiguration::Reflect(context);
        PublisherConfiguration::Reflect(context);
        SensorConfiguration::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SystemComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SystemComponent>(
                      "ROS 2 System Component",
                      "This component is responsible for creating ROS 2 node and executor, provides ROS 2 interfaces "
                      "and publishes transforms.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("System"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<TFInterfaceBus>("TFInterfaceBus")
                ->Attribute(AZ::Script::Attributes::Category, "ROS2")
                ->Event("GetLatestTransform", &TFInterfaceRequests::GetLatestTransform);
        }
    }

    void ROS2SystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void ROS2SystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void ROS2SystemComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("AssetDatabaseService", 0x3abf5601));
        required.push_back(AZ_CRC_CE("ROS2ClockService"));
    }

    void ROS2SystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2SystemComponent::ROS2SystemComponent()
    {
        if (ROS2Interface::Get() == nullptr)
        {
            ROS2Interface::Register(this);
        }
        if (TFInterface::Get() == nullptr)
        {
            TFInterface::Register(this);
        }
    }

    ROS2SystemComponent::~ROS2SystemComponent()
    {
        if (TFInterface::Get() == this)
        {
            TFInterface::Unregister(this);
        }
        if (ROS2Interface::Get() == this)
        {
            ROS2Interface::Unregister(this);
        }
        rclcpp::shutdown();
    }

    void ROS2SystemComponent::Init()
    {
        rclcpp::init(0, 0);

        // handle signals, e.g. via `Ctrl+C` hotkey or `kill` command
        auto handler = [](int sig)
        {
            rclcpp::shutdown(); // shutdown rclcpp
            std::raise(sig); // shutdown o3de
        };
        signal(SIGINT, handler);
        signal(SIGTERM, handler);
    }

    void ROS2SystemComponent::Activate()
    {
        m_ros2Node = std::make_shared<rclcpp::Node>("o3de_ros2_node");
        m_executor = AZStd::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        m_executor->add_node(m_ros2Node);

        m_staticTFBroadcaster = AZStd::make_unique<tf2_ros::StaticTransformBroadcaster>(m_ros2Node);
        m_dynamicTFBroadcaster = AZStd::make_unique<tf2_ros::TransformBroadcaster>(m_ros2Node);

        // setup tf2 buffer and listener
        m_tfBuffer = AZStd::make_shared<tf2_ros::Buffer>(m_ros2Node->get_clock());
        m_tfListener = AZStd::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);

        AZ::TickBus::Handler::BusConnect();
        TFInterfaceBus::Handler::BusConnect();
        m_nodeChangedEvent.Signal(m_ros2Node);
    }

    void ROS2SystemComponent::Deactivate()
    {
        TFInterfaceBus::Handler::BusDisconnect();
        AZ::TickBus::Handler::BusDisconnect();
        m_dynamicTFBroadcaster.reset();
        m_staticTFBroadcaster.reset();
        if (m_executor)
        {
            if (m_ros2Node)
            {
                m_executor->remove_node(m_ros2Node);
            }
            m_executor.reset();
        }
        m_ros2Node.reset();
        m_nodeChangedEvent.Signal(m_ros2Node);
    }

    std::shared_ptr<rclcpp::Node> ROS2SystemComponent::GetNode() const
    {
        return m_ros2Node;
    }

    void ROS2SystemComponent::ConnectOnNodeChanged(NodeChangedEvent::Handler& handler)
    {
        handler.Connect(m_nodeChangedEvent);
    }

    void ROS2SystemComponent::BroadcastTransform(const geometry_msgs::msg::TransformStamped& t, bool isDynamic)
    {
        if (isDynamic)
        {
            m_frameTransforms.push_back(t);
        }
        else
        {
            m_staticTFBroadcaster->sendTransform(t);
        }
    }

    void ROS2SystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        if (rclcpp::ok())
        {
            m_dynamicTFBroadcaster->sendTransform(m_frameTransforms);
            m_frameTransforms.clear();

            m_executor->spin_some();
        }
    }

    AZ::Outcome<AZ::Transform, AZStd::string> ROS2SystemComponent::GetTransform(
        const AZStd::string& source, const AZStd::string& target, const builtin_interfaces::msg::Time& time)
    {
        AZ_Error("ROS2SimulationInterfacesSystemComponent", m_tfBuffer, "This component was not activated, tf is not available" );
        if (!m_tfBuffer)
        {
            return AZ::Failure("Component was not activated, TFInterface is not available.");
        }

        AZ_Assert(m_tfBuffer, "ROS2 TF buffer is not initialized.");
        try
        {
            const auto transform = m_tfBuffer->lookupTransform(source.c_str(), target.c_str(), time);
            const auto q = ROS2::ROS2Conversions::FromROS2Quaternion(transform.transform.rotation);
            const auto t = ROS2::ROS2Conversions::FromROS2Vector3(transform.transform.translation);
            return AZ::Transform::CreateFromQuaternionAndTranslation(q, t);

        } catch (const tf2::TransformException& ex)
        {
            return AZ::Failure(
                AZStd::string::format("Failed to get transform from %s to %s: %s", source.c_str(), target.c_str(), ex.what()));
        }
    }

    AZ::Transform ROS2SystemComponent::GetLatestTransform(const AZStd::string& source, const AZStd::string& target)
    {
        const auto result = GetTransform(source, target, builtin_interfaces::msg::Time());
        if (result.IsSuccess())
        {
            return AZ::Transform(result.GetValue());
        }
        AZ_Warning(
            "ROS2SystemComponent",
            false,
            "Failed to get latest transform from %s to %s: %s",
            source.c_str(),
            target.c_str(),
            result.GetError().c_str());
        return AZ::Transform::CreateIdentity();
    }

} // namespace ROS2
