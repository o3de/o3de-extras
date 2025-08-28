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
#include <SimulationInterfaces/LevelManagerRequestBus.h>

namespace SimulationInterfaces
{
    class LevelManager
        : public AZ::Component
        , protected LevelManagerRequestBus::Handler
        , AzFramework::LevelSystemLifecycleNotificationBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(LevelManager);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        LevelManager() = default;
        ~LevelManager() = default;

    protected:
        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;

    private:
        // LevelManagerRequestBus interface implementation
        AZ::Outcome<WorldResourcesList, FailedResult> GetAvailableWorlds(const GetWorldsRequest& request) override;
        AZ::Outcome<WorldResource, FailedResult> GetCurrentWorld() override;
        AZ::Outcome<WorldResource, FailedResult> LoadWorld(const LoadWorldRequest& request) override;
        AZ::Outcome<void, FailedResult> UnloadWorld() override;
        void ReloadLevel() override;

        // LevelSystemLifecycleNotificationBus implementation
        void OnLoadingStart(const char* levelName) override;
        void OnLoadingComplete(const char* levelName) override;
        void OnUnloadComplete(const char* levelName) override;

        // indicates whether action related to levels was triggered by simulation interfaces or not
        // if it was triggered by imgui etc, it should trigger fallback
        bool m_actionRequestedFromSimInterfaces = false;

        AZ::Outcome<AZStd::vector<AZStd::string>, FailedResult> GetAllAvailableLevels();
        bool m_isAppEditor = false;
    };
} // namespace SimulationInterfaces
