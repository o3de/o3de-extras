
#include "ROS2ControllersSystemComponent.h"

#include <ROS2Controllers/ROS2ControllersTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2Controllers
{
    AZ_COMPONENT_IMPL(ROS2ControllersSystemComponent, "ROS2ControllersSystemComponent",
        ROS2ControllersSystemComponentTypeId);

    void ROS2ControllersSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ROS2ControllersSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void ROS2ControllersSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2ControllersService"));
    }

    void ROS2ControllersSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2ControllersService"));
    }

    void ROS2ControllersSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ROS2ControllersSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ROS2ControllersSystemComponent::ROS2ControllersSystemComponent()
    {
        if (ROS2ControllersInterface::Get() == nullptr)
        {
            ROS2ControllersInterface::Register(this);
        }
    }

    ROS2ControllersSystemComponent::~ROS2ControllersSystemComponent()
    {
        if (ROS2ControllersInterface::Get() == this)
        {
            ROS2ControllersInterface::Unregister(this);
        }
    }

    void ROS2ControllersSystemComponent::Init()
    {
    }

    void ROS2ControllersSystemComponent::Activate()
    {
        ROS2ControllersRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ROS2ControllersSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ROS2ControllersRequestBus::Handler::BusDisconnect();
    }

    void ROS2ControllersSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace ROS2Controllers
