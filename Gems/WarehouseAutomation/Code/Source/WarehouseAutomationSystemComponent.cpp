/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include "WarehouseAutomationSystemComponent.h"

#include <WarehouseAutomation/WarehouseAutomationTypeIds.h>

#include <AzCore/Serialization/SerializeContext.h>

namespace WarehouseAutomation
{
    AZ_COMPONENT_IMPL(WarehouseAutomationSystemComponent, "WarehouseAutomationSystemComponent",
        WarehouseAutomationSystemComponentTypeId);

    void WarehouseAutomationSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<WarehouseAutomationSystemComponent, AZ::Component>()
                ->Version(0)
                ;
        }
    }

    void WarehouseAutomationSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("WarehouseAutomationService"));
    }

    void WarehouseAutomationSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("WarehouseAutomationService"));
    }

    void WarehouseAutomationSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void WarehouseAutomationSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    WarehouseAutomationSystemComponent::WarehouseAutomationSystemComponent()
    {
        if (WarehouseAutomationInterface::Get() == nullptr)
        {
            WarehouseAutomationInterface::Register(this);
        }
    }

    WarehouseAutomationSystemComponent::~WarehouseAutomationSystemComponent()
    {
        if (WarehouseAutomationInterface::Get() == this)
        {
            WarehouseAutomationInterface::Unregister(this);
        }
    }

    void WarehouseAutomationSystemComponent::Init()
    {
    }

    void WarehouseAutomationSystemComponent::Activate()
    {
        WarehouseAutomationRequestBus::Handler::BusConnect();
        AZ::TickBus::Handler::BusConnect();
    }

    void WarehouseAutomationSystemComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        WarehouseAutomationRequestBus::Handler::BusDisconnect();
    }

    void WarehouseAutomationSystemComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
    }

} // namespace WarehouseAutomation
