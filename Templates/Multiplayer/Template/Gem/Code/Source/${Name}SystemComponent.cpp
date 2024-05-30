/*
 * Copyright (c) Contributors to the Open 3D Engine Project. For complete copyright and license terms please see the LICENSE at the root of this distribution.
 * 
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "${SanitizedCppName}SystemComponent.h"

#include <AzCore/Console/ILogger.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

#include <Source/AutoGen/AutoComponentTypes.h>
#include <Source/Weapons/WeaponTypes.h>
#include <Source/Components/NetworkAiComponent.h>

namespace ${SanitizedCppName}
{
    using namespace AzNetworking;

    void ${SanitizedCppName}SystemComponent::Reflect(AZ::ReflectContext* context)
    {
        ReflectWeaponEnums(context);
        ClientEffect::Reflect(context);
        GatherParams::Reflect(context);
        HitEffect::Reflect(context);
        WeaponParams::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<${SanitizedCppName}SystemComponent, AZ::Component>()
                ->Version(0)
                ;

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<${SanitizedCppName}SystemComponent>("${SanitizedCppName}", "[Description of functionality provided by this System Component]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ;
            }
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

    void ${SanitizedCppName}SystemComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("NetworkingService"));
        required.push_back(AZ_CRC_CE("MultiplayerService"));
    }

    void ${SanitizedCppName}SystemComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        AZ_UNUSED(dependent);
    }

    void ${SanitizedCppName}SystemComponent::Init()
    {
        ;
    }

    void ${SanitizedCppName}SystemComponent::Activate()
    {
        AZ::TickBus::Handler::BusConnect();

        //! Register our gems multiplayer components to assign NetComponentIds
        RegisterMultiplayerComponents();
    }

    void ${SanitizedCppName}SystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
    }

    void ${SanitizedCppName}SystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        ;
    }

    int ${SanitizedCppName}SystemComponent::GetTickOrder()
    {
        // Tick immediately after the multiplayer system component
        return AZ::TICK_PLACEMENT + 2;
    }
}

