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
#include <Source/Spawners/RoundRobinSpawner.h>

#include <Multiplayer/IMultiplayer.h>
#include <Multiplayer/Components/NetBindComponent.h>
#include <Multiplayer/ConnectionData/IConnectionData.h>
#include <Multiplayer/ReplicationWindows/IReplicationWindow.h>

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

        AZ::Interface<Multiplayer::IMultiplayerSpawner>::Register(this);
        m_playerSpawner = AZStd::make_unique<RoundRobinSpawner>();
        AZ::Interface<${SanitizedCppName}::IPlayerSpawner>::Register(m_playerSpawner.get());
    }

    void ${SanitizedCppName}SystemComponent::Deactivate()
    {
        AZ::Interface<${SanitizedCppName}::IPlayerSpawner>::Unregister(m_playerSpawner.get());
        AZ::Interface<Multiplayer::IMultiplayerSpawner>::Unregister(this);
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

    Multiplayer::NetworkEntityHandle ${SanitizedCppName}SystemComponent::OnPlayerJoin(
        [[maybe_unused]] uint64_t userId, [[maybe_unused]] const Multiplayer::MultiplayerAgentDatum& agentDatum)
    {
        AZStd::pair<Multiplayer::PrefabEntityId, AZ::Transform> entityParams = AZ::Interface<IPlayerSpawner>::Get()->GetNextPlayerSpawn();

        Multiplayer::INetworkEntityManager::EntityList entityList =
            AZ::Interface<Multiplayer::IMultiplayer>::Get()->GetNetworkEntityManager()->CreateEntitiesImmediate(
            entityParams.first, Multiplayer::NetEntityRole::Authority, entityParams.second, Multiplayer::AutoActivate::DoNotActivate);

        for (Multiplayer::NetworkEntityHandle subEntity : entityList)
        {
            subEntity.Activate();
        }

        Multiplayer::NetworkEntityHandle controlledEntity;
        if (!entityList.empty())
        {
            controlledEntity = entityList[0];
        }
        else
        {
            AZLOG_WARN("Attempt to spawn prefab %s failed. Check that prefab is network enabled.",
                entityParams.first.m_prefabName.GetCStr());
        }

        return controlledEntity;
    }

    void ${SanitizedCppName}SystemComponent::OnPlayerLeave(
        Multiplayer::ConstNetworkEntityHandle entityHandle, [[maybe_unused]] const Multiplayer::ReplicationSet& replicationSet, [[maybe_unused]] AzNetworking::DisconnectReason reason)
    {
        AZ::Interface<Multiplayer::IMultiplayer>::Get()->GetNetworkEntityManager()->MarkForRemoval(entityHandle);
    }
}

