
#include "ROS2SensorsSystemComponent.h"

#include <ROS2Sensors/ROS2SensorsTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2Sensors
{
    AZ_COMPONENT_IMPL(ROS2SensorsSystemComponent, "ROS2SensorsSystemComponent",
        ROS2SensorsSystemComponentTypeId);

    void ROS2SensorsSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2SensorsSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void ROS2SensorsSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2SensorsService"));
    }

    void ROS2SensorsSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2SensorsService"));
    }

    void ROS2SensorsSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2SensorsSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2SensorsSystemComponent::ROS2SensorsSystemComponent()
    {
        if (ROS2SensorsInterface::Get() == nullptr)
        {
            ROS2SensorsInterface::Register(this);
        }
    }

    ROS2SensorsSystemComponent::~ROS2SensorsSystemComponent()
    {
        if (ROS2SensorsInterface::Get() == this)
        {
            ROS2SensorsInterface::Unregister(this);
        }
    }

    void ROS2SensorsSystemComponent::Init()
    {
    }

    void ROS2SensorsSystemComponent::Activate()
    {
        ROS2SensorsRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2SensorsSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ROS2SensorsRequestBus::Handler::BusDisconnect();
    }

    void ROS2SensorsSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace ROS2Sensors
