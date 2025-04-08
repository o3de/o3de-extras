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
#include "Services/GetSimulationFeaturesServiceHandler.h"
#include "Services/GetSpawnablesServiceHandler.h"
#include "Services/ROS2ServiceBase.h"
#include "Services/SetEntityStateServiceHandler.h"
#include "Services/SpawnEntityServiceHandler.h"
#include "Services/ResetSimulationServiceHandler.h"
#include "Services/SetSimulationStateServiceHandler.h"
#include "SimulationInterfacesROS2/SimulationInterfacesROS2RequestBus.h"
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/string/string.h>

namespace SimulationInterfacesROS2
{
    class SimulationInterfacesROS2SystemComponent
        : public AZ::Component
        , public SimulationInterfacesROS2RequestBus::Handler
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

        // SimulationInterfacesROS2RequestBus override
        AZStd::unordered_set<AZ::u8> GetSimulationFeatures() override;

    private:
        AZStd::unordered_map<AZStd::string, AZStd::shared_ptr<IROS2HandlerBase>> m_availableRos2Interface;
    };

} // namespace SimulationInterfacesROS2
