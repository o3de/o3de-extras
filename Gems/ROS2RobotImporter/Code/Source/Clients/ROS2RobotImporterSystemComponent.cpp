
#include "ROS2RobotImporterSystemComponent.h"

#include <ROS2RobotImporter/ROS2RobotImporterTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2RobotImporter
{
    AZ_COMPONENT_IMPL(ROS2RobotImporterSystemComponent, "ROS2RobotImporterSystemComponent", ROS2RobotImporterSystemComponentTypeId);

    void ROS2RobotImporterSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2RobotImporterSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ROS2RobotImporterSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2RobotImporterService"));
    }

    void ROS2RobotImporterSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2RobotImporterService"));
    }

    void ROS2RobotImporterSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2RobotImporterSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2RobotImporterSystemComponent::ROS2RobotImporterSystemComponent()
    {
        if (ROS2RobotImporterInterface::Get() == nullptr)
        {
            ROS2RobotImporterInterface::Register(this);
        }
    }

    ROS2RobotImporterSystemComponent::~ROS2RobotImporterSystemComponent()
    {
        if (ROS2RobotImporterInterface::Get() == this)
        {
            ROS2RobotImporterInterface::Unregister(this);
        }
    }

    void ROS2RobotImporterSystemComponent::Init()
    {
    }

    void ROS2RobotImporterSystemComponent::Activate()
    {
        ROS2RobotImporterRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2RobotImporterSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ROS2RobotImporterRequestBus::Handler::BusDisconnect();
    }

    void ROS2RobotImporterSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace ROS2RobotImporter
