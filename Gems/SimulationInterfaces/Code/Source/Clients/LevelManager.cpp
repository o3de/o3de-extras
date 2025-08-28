/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LevelManager.h"
#include "SimulationInterfaces/SimulationMangerRequestBus.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/base.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/conversions.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/API/ApplicationAPI.h>
#include <SimulationInterfaces/LevelManagerRequestBus.h>
#include <SimulationInterfaces/Resource.h>
#include <SimulationInterfaces/Result.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>
#include <SimulationInterfaces/WorldResource.h>
#include <simulation_interfaces/msg/detail/simulation_state__struct.hpp>
#include <simulation_interfaces/srv/get_available_worlds.hpp>
#include <simulation_interfaces/srv/get_current_world.hpp>
#include <simulation_interfaces/srv/load_world.hpp>
#include <simulation_interfaces/srv/unload_world.hpp>

// ordering important
// clang-format off
#include <CryCommon/ILevelSystem.h>
#include <CryCommon/ISystem.h>
// clang-format on

namespace SimulationInterfaces
{
    namespace
    {
        ILevelSystem* GetLevelSystem()
        {
            ISystem* iSystem = GetISystem();
            return (iSystem != nullptr) ? iSystem->GetILevelSystem() : nullptr;
        }

        AZStd::string GetLevelNameFromAssetPath(const AZStd::string& assetPath)
        {
            AZ::IO::PathView levelPath{ assetPath };
            return levelPath.Stem().Native();
        }
    } // namespace

    AZ_COMPONENT_IMPL(LevelManager, "LevelManager", LevelManagerTypeId);

    void LevelManager::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LevelManager, AZ::Component>()->Version(0);
        }
    }

    void LevelManager::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("LevelManagerService"));
    }

    void LevelManager::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("LevelManagerService"));
    }

    void LevelManager::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("AssetCatalogService"));
        required.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    void LevelManager::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    void LevelManager::Activate()
    {
        LevelManagerRequestBus::Handler::BusConnect();
        AZ::ApplicationTypeQuery appType;
        AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationBus::Events::QueryApplicationType, appType);
        m_isAppEditor = (appType.IsValid() && appType.IsEditor());

        SimulationFeaturesAggregatorRequestBus::Broadcast(
            &SimulationFeaturesAggregatorRequests::AddSimulationFeatures,
            AZStd::unordered_set<SimulationFeatureType>{ simulation_interfaces::msg::SimulatorFeatures::AVAILABLE_WORLDS });

        // level loading and unloading is supported only in GameLauncher, remove features from Editor application
        if (!m_isAppEditor)
        {
            SimulationFeaturesAggregatorRequestBus::Broadcast(
                &SimulationFeaturesAggregatorRequests::AddSimulationFeatures,
                AZStd::unordered_set<SimulationFeatureType>{ simulation_interfaces::msg::SimulatorFeatures::WORLD_INFO_GETTING,
                                                             simulation_interfaces::msg::SimulatorFeatures::WORLD_LOADING,
                                                             simulation_interfaces::msg::SimulatorFeatures::WORLD_UNLOADING });
        }
    }
    void LevelManager::Deactivate()
    {
        LevelManagerRequestBus::Handler::BusDisconnect();
    }

    AZ::Outcome<WorldResourcesList, FailedResult> LevelManager::GetAvailableWorlds(const GetWorldsRequest& request)
    {
        WorldResourcesList availableWorlds;
        // request validation
        if (!request.additionalSources.empty())
        {
            constexpr const char* errorMsg = "Additional Sources are not implemented yet";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED, errorMsg));
        }

        if (!request.filter.m_tags.empty())
        {
            constexpr const char* errorMsg = "Tags filter is not implemented yet";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED, errorMsg));
        }

        if (!request.offlineOnly)
        {
            constexpr const char* errorMsg = "Online search is not implemented yet, only pure offline search is supported";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED, errorMsg));
        }

        const auto allLevels = GetAllAvailableLevels();
        if (!allLevels.IsSuccess())
        {
            return AZ::Failure(allLevels.GetError());
        }
        for (const auto& levelPath : allLevels.GetValue())
        {
            availableWorlds.emplace_back(
                GetLevelNameFromAssetPath(levelPath), Resource{ levelPath, "" }, "", AZStd::vector<AZStd::string>{});
        }
        return AZ::Success(availableWorlds);
    }

    AZ::Outcome<WorldResource, FailedResult> LevelManager::GetCurrentWorld()
    {
        if (m_isAppEditor)
        {
            constexpr const char* errorMsg = "GetCurrentWorld is not supported in Editor";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED, errorMsg));
        }

        WorldResource currentWorld;
        auto* levelInterface = AzFramework::LevelSystemLifecycleInterface::Get();
        if (levelInterface == nullptr)
        {
            constexpr const char* errorMsg = "Failed to get level interface";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::srv::GetAvailableWorlds::Response::DEFAULT_SOURCES_FAILED, errorMsg));
        }

        if (!levelInterface->IsLevelLoaded())
        {
            constexpr const char* errorMsg = "No level loaded";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::srv::GetCurrentWorld::Response::NO_WORLD_LOADED, errorMsg));
        }

        const auto* levelPath = levelInterface->GetCurrentLevelName();
        if (levelPath == nullptr)
        {
            constexpr const char* errorMsg = "Failed to get current level path";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, errorMsg));
        }
        AZStd::string levelPathStr{ levelPath };
        AZStd::to_lower(levelPathStr.begin(), levelPathStr.end());
        auto levelName = GetLevelNameFromAssetPath(levelPathStr);
        currentWorld.m_worldResource.m_uri = levelPathStr;
        currentWorld.m_name = levelName;
        return AZ::Success(currentWorld);
    }

    AZ::Outcome<WorldResource, FailedResult> LevelManager::LoadWorld(const LoadWorldRequest& request)
    {
        if (m_isAppEditor)
        {
            constexpr const char* errorMsg = "LoadWorld is not supported in Editor";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED, errorMsg));
        }

        WorldResource loadedWorld;

        if (request.levelResource.m_resourceString.empty() && request.levelResource.m_uri.empty())
        {
            constexpr const char* errorMsg = "uri and resource string in levelResource cannot be both empty";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::srv::LoadWorld::Response::NO_RESOURCE, errorMsg));
        }
        if (!request.levelResource.m_resourceString.empty() && !request.levelResource.m_uri.empty())
        {
            constexpr const char* errorMsg = "uri and resource string in levelResource cannot be both non-empty";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, errorMsg));
        }
        if (!request.levelResource.m_resourceString.empty())
        {
            constexpr const char* errorMsg = "Loading world from resource string is not implemented yet";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED, errorMsg));
        }

        const auto levelNamesResult = GetAllAvailableLevels();
        if (!levelNamesResult.IsSuccess())
        {
            return AZ::Failure(levelNamesResult.GetError());
        }
        const auto& levelsPaths = levelNamesResult.GetValue();

        if (AZStd::find(levelsPaths.begin(), levelsPaths.end(), request.levelResource.m_uri) == levelsPaths.end())
        {
            const AZStd::string errorMsg = AZStd::string::format("Requested world/level %s not found", request.levelResource.m_uri.c_str());
            AZ_Warning("SimulationInterfaces", false, errorMsg.c_str());
            return AZ::Failure(FailedResult(simulation_interfaces::srv::LoadWorld::Response::MISSING_ASSETS, errorMsg));
        }

        ILevelSystem* levelSystem = GetLevelSystem();
        if (levelSystem == nullptr)
        {
            constexpr const char* errorMsg = "Failed to start, level System not available";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, errorMsg));
        }

        // unload level if needed
        if (GetCurrentWorld().IsSuccess())
        {
            levelSystem->UnloadLevel();
        }
        // notify state machine
        SimulationManagerRequestBus::Broadcast(
            &SimulationManagerRequests::SetSimulationState, simulation_interfaces::msg::SimulationState::STATE_LOADING_WORLD);

        if (!levelSystem->LoadLevel(request.levelResource.m_uri.c_str()))
        {
            constexpr const char* errorMsg = "Failed to load world";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            SimulationManagerRequestBus::Broadcast(
                &SimulationManagerRequests::SetSimulationState, simulation_interfaces::msg::SimulationState::STATE_NO_WORLD);
            return AZ::Failure(FailedResult(simulation_interfaces::srv::LoadWorld::Response::RESOURCE_PARSE_ERROR, errorMsg));
        }
        // notify state machine
        SimulationManagerRequestBus::Broadcast(
            &SimulationManagerRequests::SetSimulationState, simulation_interfaces::msg::SimulationState::STATE_STOPPED);
        // fill up response
        loadedWorld.m_worldResource = request.levelResource;
        return AZ::Success(loadedWorld);
    }

    AZ::Outcome<void, FailedResult> LevelManager::UnloadWorld()
    {
        if (m_isAppEditor)
        {
            constexpr const char* errorMsg = "UnloadWorld is not supported in Editor";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_FEATURE_UNSUPPORTED, errorMsg));
        }
        const auto currentLevel = GetCurrentWorld();
        if (!currentLevel.IsSuccess())
        {
            if (currentLevel.GetError().m_errorCode == simulation_interfaces::srv::GetCurrentWorld::Response::NO_WORLD_LOADED)
            {
                constexpr const char* errorMsg = "No level loaded";
                AZ_Warning("SimulationInterfaces", false, errorMsg);
                return AZ::Failure(FailedResult(simulation_interfaces::srv::UnloadWorld::Response::NO_WORLD_LOADED, errorMsg));
            }
        }
        ILevelSystem* levelSystem = GetLevelSystem();
        if (!levelSystem)
        {
            constexpr const char* errorMsg = "Level system is not available";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, errorMsg));
        }

        levelSystem->UnloadLevel();
        SimulationManagerRequestBus::Broadcast(
            &SimulationManagerRequests::SetSimulationState, simulation_interfaces::msg::SimulationState::STATE_NO_WORLD);
        return AZ::Success();
    }

    void LevelManager::ReloadLevel()
    {
        if (m_isAppEditor)
        {
            [[maybe_unused]] constexpr const char* errorMsg = "ReloadWorld is not supported in Editor";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return;
        }
        auto levelGathering = GetCurrentWorld();
        if (!levelGathering.IsSuccess())
        {
            AZ_Error("LevelManager", false, "Error occurred during level gathering: %s", levelGathering.GetError().m_errorString.c_str());
            return;
        }
        UnloadWorld();
        LoadWorldRequest request;
        request.levelResource = levelGathering.GetValue().m_worldResource;
        LoadWorld(request);
    }

    AZ::Outcome<AZStd::vector<AZStd::string>, FailedResult> LevelManager::GetAllAvailableLevels()
    {
        ILevelSystem* levelSystem = GetLevelSystem();
        if (!levelSystem)
        {
            constexpr const char* errorMsg = "Level system is not available";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::srv::GetAvailableWorlds::Response::DEFAULT_SOURCES_FAILED, errorMsg));
        }
        // Run through all the assets in the asset catalog and gather up the list of level assets

        AZ::Data::AssetType levelAssetType = levelSystem->GetLevelAssetType();
        AZStd::vector<AZStd::string> levelNames;
        auto enumerateCB = [levelAssetType, &levelNames](const AZ::Data::AssetId id, const AZ::Data::AssetInfo& assetInfo)
        {
            if (assetInfo.m_assetType != levelAssetType)
            {
                return;
            }
            AZStd::string assetPath;
            AZ::Data::AssetCatalogRequestBus::BroadcastResult(assetPath, &AZ::Data::AssetCatalogRequestBus::Events::GetAssetPathById, id);
            AZStd::string lowerCaseLevelPath = assetPath;
            AZStd::to_lower(lowerCaseLevelPath.begin(), lowerCaseLevelPath.end());
            if (!lowerCaseLevelPath.starts_with("levels"))
            {
                return;
            }
            levelNames.push_back(assetPath);
        };

        AZ::Data::AssetCatalogRequestBus::Broadcast(
            &AZ::Data::AssetCatalogRequestBus::Events::EnumerateAssets, nullptr, enumerateCB, nullptr);

        return AZ::Success(levelNames);
    }
} // namespace SimulationInterfaces
