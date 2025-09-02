/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <signal.h>

#include "ROS2SystemComponent.h"
#include <Lidar/LidarCore.h>
#include <ROS2/Clock/ROS2TimeSource.h>
#include <ROS2/Clock/RealTimeSource.h>
#include <ROS2/Clock/SimulationTimeSource.h>
#include <ROS2/Communication/PublisherConfiguration.h>
#include <ROS2/Communication/QoS.h>
#include <ROS2/Communication/TopicConfiguration.h>
#include <ROS2/Sensor/SensorConfiguration.h>
#include <ROS2/Utilities/Controllers/PidConfiguration.h>
#include <VehicleDynamics/VehicleModelComponent.h>

#include <Atom/RPI.Public/Pass/PassSystemInterface.h>

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <AzCore/Time/ITime.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/sort.h>
#include <AzCore/std/string/string_view.h>
#include <AzFramework/API/ApplicationAPI.h>
#include <ROS2/Sensor/SensorConfigurationRequestBus.h>
#include <ROS2/Utilities/ROS2Conversions.h>

namespace ROS2
{
    constexpr AZStd::string_view ClockTypeConfigurationKey = "/O3DE/ROS2/ClockType";
    constexpr AZStd::string_view PublishClockConfigurationKey = "/O3DE/ROS2/PublishClock";
    constexpr size_t FramesNumberForStats = 60;

    void ROS2SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        // Reflect structs not strictly owned by any single component
        QoS::Reflect(context);
        TopicConfiguration::Reflect(context);
        PublisherConfiguration::Reflect(context);
        LidarCore::Reflect(context);
        SensorConfiguration::Reflect(context);
        VehicleDynamics::VehicleModelComponent::Reflect(context);
        ROS2::Controllers::PidConfiguration::Reflect(context);

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

    void ROS2SystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("AssetDatabaseService", 0x3abf5601));
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

    void ROS2SystemComponent::InitClock()
    {
        AZStd::unordered_map<AZStd::string, AZStd::function<AZStd::unique_ptr<ITimeSource>()>> clocksMap;
        clocksMap["realtime"] = []()
        {
            AZ_Info("ROS2SystemComponent", "Enabling realtime clock.");
            return AZStd::make_unique<RealTimeSource>();
        };
        clocksMap["simulation"] = []()
        {
            AZ_Info("ROS2SystemComponent", "Enabling simulation clock.");
            return AZStd::make_unique<SimulationTimeSource>();
        };
        clocksMap["ros2"] = []()
        {
            AZ_Info("ROS2SystemComponent", "Enabling ros 2 clock.");
            return AZStd::make_unique<ROS2TimeSource>();
        };
        AZStd::string clockType{ "" };
        bool publishClock{ true };
        auto* registry = AZ::SettingsRegistry::Get();
        AZ_Assert(registry, "No Registry available");
        if (registry)
        {
            registry->Get(publishClock, PublishClockConfigurationKey);
            registry->Get(clockType, ClockTypeConfigurationKey);
            if (clocksMap.contains(clockType))
            {
                m_simulationClock = AZStd::make_unique<ROS2Clock>(clocksMap[clockType](), publishClock);
                return;
            }
        }
        AZ_Info(
            "ROS2SystemComponent", "Cannot read registry or the clock type '%s' is unknown, enabling simulation clock.", clockType.c_str());
        m_simulationClock = AZStd::make_unique<ROS2Clock>(clocksMap["simulation"](), publishClock);
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

        ROS2RequestBus::Handler::BusConnect();

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
        ROS2RequestBus::Handler::BusDisconnect();
        if (m_simulationClock)
        {
            m_simulationClock->Deactivate();
        }
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
        m_simulationClock.reset();
        m_ros2Node.reset();
        m_nodeChangedEvent.Signal(m_ros2Node);
    }

    builtin_interfaces::msg::Time ROS2SystemComponent::GetROSTimestamp() const
    {
        return m_simulationClock->GetROSTimestamp();
    }

    std::shared_ptr<rclcpp::Node> ROS2SystemComponent::GetNode() const
    {
        return m_ros2Node;
    }

    void ROS2SystemComponent::ConnectOnNodeChanged(NodeChangedEvent::Handler& handler)
    {
        handler.Connect(m_nodeChangedEvent);
    }

    const ROS2Clock& ROS2SystemComponent::GetSimulationClock() const
    {
        return *m_simulationClock;
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

            m_simulationClock->Tick();
            m_executor->spin_some();
        }
        // Calculate simulation loop time
        const auto simulationTimestamp = m_simulationClock->GetROSTimestamp();
        const float deltaSimTime = ROS2Conversions::GetTimeDifference(m_lastSimulationTime, simulationTimestamp);

        // Filter out the outliers
        m_simulationLoopTimes.push_back(deltaSimTime);
        if (m_simulationLoopTimes.size() > FramesNumberForStats)
        {
            m_simulationLoopTimes.pop_front();
        }
        AZStd::vector<float> frameTimeSorted{ m_simulationLoopTimes.begin(), m_simulationLoopTimes.end() };
        AZStd::sort(frameTimeSorted.begin(), frameTimeSorted.end());
        m_simulationLoopTimeMedian = frameTimeSorted[frameTimeSorted.size() / 2];

        m_lastSimulationTime = simulationTimestamp;
    }

    float ROS2SystemComponent::GetExpectedSimulationLoopTime() const
    {
        return m_simulationLoopTimeMedian;
    }

    AZ::Outcome<AZ::Transform, AZStd::string> ROS2SystemComponent::GetTransform(
        const AZStd::string& source, const AZStd::string& target, const builtin_interfaces::msg::Time& time)
    {
        AZ_Error("ROS2SystemComponent", m_tfBuffer, "This component was not activated, tf is not available");
        if (!m_tfBuffer)
        {
            return AZ::Failure("Component was not activated, TFInterface is not available.");
        }

        AZ_Assert(m_tfBuffer, "ROS2 TF buffer is not initialized.");
        try
        {
            const auto transform = m_tfBuffer->lookupTransform(source.c_str(), target.c_str(), time);
            const auto q = ROS2Conversions::FromROS2Quaternion(transform.transform.rotation);
            const auto t = ROS2Conversions::FromROS2Vector3(transform.transform.translation);
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
