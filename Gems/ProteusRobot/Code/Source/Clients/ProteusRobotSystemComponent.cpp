
#include "ProteusRobotSystemComponent.h"

#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ProteusRobot
{
    void ProteusRobotSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ProteusRobotSystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ProteusRobotSystemComponent>("ProteusRobot", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
        }
    }

    void ProteusRobotSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ProteusRobotService"));
    }

    void ProteusRobotSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ProteusRobotService"));
    }

    void ProteusRobotSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ProteusRobotSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ProteusRobotSystemComponent::ProteusRobotSystemComponent()
    {
        if (ProteusRobotInterface::Get() == nullptr)
        {
            ProteusRobotInterface::Register(this);
        }
    }

    ProteusRobotSystemComponent::~ProteusRobotSystemComponent()
    {
        if (ProteusRobotInterface::Get() == this)
        {
            ProteusRobotInterface::Unregister(this);
        }
    }

    void ProteusRobotSystemComponent::Init()
    {
    }

    void ProteusRobotSystemComponent::Activate()
    {
        ProteusRobotRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void ProteusRobotSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        ProteusRobotRequestBus::Handler::BusDisconnect();
    }

    void ProteusRobotSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace ProteusRobot
