/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzFramework/API/ApplicationAPI.h>

#include "Services/DeleteEntityServiceHandler.h"
#include "Services/GetEntitiesServiceHandler.h"
#include "Services/GetEntitiesStatesServiceHandler.h"
#include "Services/GetEntityStateServiceHandler.h"
#include "Services/GetSpawnablesServiceHandler.h"
#include "Services/SetEntityStateServiceHandler.h"
#include "Services/SpawnEntityServiceHandler.h"
#include "Utils/ServicesConfig.h"

namespace SimulationInterfacesROS2
{
    class SimulationInterfacesROS2SystemComponent
        : public AZ::Component
        , public AzFramework::LevelSystemLifecycleNotificationBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(SimulationInterfacesROS2SystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        SimulationInterfacesROS2SystemComponent() = default;
        ~SimulationInterfacesROS2SystemComponent() = default;

    protected:
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;

    private:
        // LevelSystemLifecycleNotificationBus::Handler overrides
        void OnLoadingComplete([[maybe_unused]] const char* levelName) override;
        void OnUnloadComplete([[maybe_unused]] const char* levelName) override;

        AZStd::optional<DeleteEntityServiceHandler> m_deleteEntityServiceHandler;
        AZStd::optional<GetEntitiesServiceHandler> m_getEntitiesServiceHandler;
        AZStd::optional<GetEntitiesStatesServiceHandler> m_getEntitiesStatesServiceHandler;
        AZStd::optional<GetEntityStateServiceHandler> m_getEntityStateServiceHandler;
        AZStd::optional<GetSpawnablesServiceHandler> m_getSpawnablesServiceHandler;
        AZStd::optional<SetEntityStateServiceHandler> m_setEntityStateServiceHandler;
        AZStd::optional<SpawnEntityServiceHandler> m_spawnEntityServiceHandler;
    };

} // namespace SimulationInterfacesROS2
