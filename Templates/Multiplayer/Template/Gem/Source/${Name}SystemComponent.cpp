
#include <AzCore/Serialization/SerializeContext.h>

#include "${SanitizedCppName}SystemComponent.h"

#include <${SanitizedCppName}/${SanitizedCppName}TypeIds.h>

#include <Source/AutoGen/AutoComponentTypes.h>
#include <Source/Weapons/WeaponTypes.h>


namespace ${SanitizedCppName}
{
    AZ_COMPONENT_IMPL(${SanitizedCppName}SystemComponent, "${SanitizedCppName}SystemComponent",
        ${SanitizedCppName}SystemComponentTypeId);

    void ${SanitizedCppName}SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        ReflectWeaponEnums(context);
        GatherParams::Reflect(context);
        HitEffect::Reflect(context);
        HitEntity::Reflect(context);
        HitEvent::Reflect(context);
        WeaponParams::Reflect(context);
        GameEffect::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<${SanitizedCppName}SystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void ${SanitizedCppName}SystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("${SanitizedCppName}Service"));
    }

    void ${SanitizedCppName}SystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("${SanitizedCppName}Service"));
    }

    void ${SanitizedCppName}SystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("NetworkingService"));

        // Ensure Multiplayer gem is a requirement so that MultiplayerSystemComponent calls RegisterMultiplayerComponents before this component activates.
        // It's important for multiplayer components to be registered in a consistent order so that the server and client
        //   assign the same component-id for each component.
        required.push_back(AZ_CRC_CE("MultiplayerService"));
    }

    void ${SanitizedCppName}SystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ${SanitizedCppName}SystemComponent::${SanitizedCppName}SystemComponent()
    {
        if (${SanitizedCppName}Interface::Get() == nullptr)
        {
            ${SanitizedCppName}Interface::Register(this);
        }
    }

    ${SanitizedCppName}SystemComponent::~${SanitizedCppName}SystemComponent()
    {
        if (${SanitizedCppName}Interface::Get() == this)
        {
            ${SanitizedCppName}Interface::Unregister(this);
        }
    }

    void ${SanitizedCppName}SystemComponent::Init()
    {
    }

    void ${SanitizedCppName}SystemComponent::Activate()
    {
        ${SanitizedCppName}RequestBus::Handler::BusConnect();
        RegisterMultiplayerComponents();
    }

    void ${SanitizedCppName}SystemComponent::Deactivate()
    {
        ${SanitizedCppName}RequestBus::Handler::BusDisconnect();
    }
}
