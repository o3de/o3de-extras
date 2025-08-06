/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include "SimulationInterfaces/LevelManagerRequestBus.h"
#include <AzCore/Component/Component.h>

namespace SimulationInterfaces
{
    class LevelManager
        : public AZ::Component
        , protected LevelManagerRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(LevelManager);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        LevelManager();
        ~LevelManager();

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

    private:
        // LevelManagerRequestBus interface implementation
        AZ::Outcome<WorldResourcesList, FailedResult> GetAvailableWorlds(const GetWorldsRequest& request) override;
        AZ::Outcome<WorldResource, FailedResult> GetCurrentWorld() override;
        AZ::Outcome<WorldResource, FailedResult> LoadWorld(const LoadWorldRequest& request) override;
        AZ::Outcome<void, FailedResult> UnloadWorld() override;
        void ReloadLevel() override;

        AZ::Outcome<AZStd::vector<AZStd::string>, FailedResult> GetAllAvailableLevels();
        bool m_isAppEditor = false;
    };
} // namespace SimulationInterfaces
