/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <ROS2/Clock/PhysicallyStableClock.h>
#include <ROS2/Communication/QoS.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Utilities/Controllers/PidConfiguration.h>
#include <ROS2SystemComponent.h>
#include <VehicleDynamics/VehicleModelComponent.h>

#include <Atom/RPI.Public/Pass/PassSystemInterface.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/Time/ITime.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/string/string_view.h>

namespace ROS2
{
    constexpr AZStd::string_view EnablePhysicsSteadyClockConfigurationKey = "/O3DE/ROS2/SteadyClock";

    void ROS2SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        // Reflect structs not strictly owned by any single component
        QoS::Reflect(context);
        TopicConfiguration::Reflect(context);
        VehicleDynamics::VehicleModelComponent::Reflect(context);
        ROS2::Controllers::PidConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2SystemComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2SystemComponent>("ROS2 System Component", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("System"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
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

    void ROS2SystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("AssetDatabaseService", 0x3abf5601));
        required.push_back(AZ_CRC("RPISystem", 0xf2add773));
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
        return;
    }

    ROS2SystemComponent::~ROS2SystemComponent()
    {
        if (ROS2Interface::Get() == this)
        {
            ROS2Interface::Unregister(this);
        }
        rclcpp::shutdown();
    }

    void ROS2SystemComponent::Init()
    {
        rclcpp::init(0, 0);
    }

    void ROS2SystemComponent::InitClock()
    {
        bool useSteadyTime = false;
        auto* registry = AZ::SettingsRegistry::Get();
        AZ_Assert(registry, "No Registry available");
        if (registry)
        {
            registry->Get(useSteadyTime, EnablePhysicsSteadyClockConfigurationKey);
            if (useSteadyTime)
            {
                AZ_Printf("ROS2SystemComponent", "Enabling Physical steady clock");
                m_simulationClock = AZStd::make_unique<PhysicallyStableClock>();
                return;
            }
        }
        AZ_Printf("ROS2SystemComponent", "Enabling realtime clock");
        m_simulationClock = AZStd::make_unique<SimulationClock>();
    }

    void ROS2SystemComponent::Activate()
    {
        InitClock();
        m_simulationClock->Activate();
        m_ros2Node = std::make_shared<rclcpp::Node>("o3de_ros2_node");
        m_executor = AZStd::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        m_executor->add_node(m_ros2Node);

        m_staticTFBroadcaster = AZStd::make_unique<tf2_ros::StaticTransformBroadcaster>(m_ros2Node);
        m_dynamicTFBroadcaster = AZStd::make_unique<tf2_ros::TransformBroadcaster>(m_ros2Node);

        auto* passSystem = AZ::RPI::PassSystemInterface::Get();
        AZ_Assert(passSystem, "Cannot get the pass system.");

        m_loadTemplatesHandler = AZ::RPI::PassSystemInterface::OnReadyLoadTemplatesEvent::Handler(
            [this]()
            {
                this->LoadPassTemplateMappings();
            });
        passSystem->ConnectEvent(m_loadTemplatesHandler);

        ROS2RequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2SystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ROS2RequestBus::Handler::BusDisconnect();
        m_simulationClock->Deactivate();
        m_loadTemplatesHandler.Disconnect();
        m_dynamicTFBroadcaster.reset();
        m_staticTFBroadcaster.reset();
        m_executor->remove_node(m_ros2Node);
        m_executor.reset();
        m_simulationClock.reset();
        m_ros2Node.reset();
    }

    builtin_interfaces::msg::Time ROS2SystemComponent::GetROSTimestamp() const
    {
        return m_simulationClock->GetROSTimestamp();
    }

    std::shared_ptr<rclcpp::Node> ROS2SystemComponent::GetNode() const
    {
        return m_ros2Node;
    }

    const SimulationClock& ROS2SystemComponent::GetSimulationClock() const
    {
        return *m_simulationClock;
    }

    void ROS2SystemComponent::BroadcastTransform(const geometry_msgs::msg::TransformStamped& t, bool isDynamic) const
    {
        if (isDynamic)
        {
            m_dynamicTFBroadcaster->sendTransform(t);
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
            m_simulationClock->Tick();
            m_executor->spin_some();
        }
    }

    void ROS2SystemComponent::LoadPassTemplateMappings()
    {
        AZ_Printf("ROS2CameraSensorComponent", "LoadPassTemplateMappings\n");
        auto* passSystem = AZ::RPI::PassSystemInterface::Get();
        AZ_Assert(passSystem, "PassSystemInterface is null");

        const char* passTemplatesFile = "Passes/ROSPassTemplates.azasset";
        [[maybe_unused]] bool isOk = passSystem->LoadPassTemplateMappings(passTemplatesFile);
        AZ_Assert(isOk, "LoadPassTemplateMappings return false ");
    }

} // namespace ROS2
