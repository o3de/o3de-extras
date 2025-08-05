
#include "LevelManager.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/base.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/API/ApplicationAPI.h>
#include <SimulationInterfaces/LevelManagerRequestBus.h>
#include <SimulationInterfaces/Resource.h>
#include <SimulationInterfaces/Result.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>
#include <SimulationInterfaces/WorldResource.h>
#include <simulation_interfaces/srv/detail/get_current_world__struct.hpp>
#include <simulation_interfaces/srv/detail/load_world__struct.hpp>
#include <simulation_interfaces/srv/detail/unload_world__struct.hpp>
#include <simulation_interfaces/srv/get_available_worlds.hpp>

// ordering important
// begin
#include <CryCommon/ILevelSystem.h>
#include <CryCommon/ISystem.h>

// end

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

    LevelManager::LevelManager()
    {
        if (LevelManagerRequestBusInterface::Get() == nullptr)
        {
            LevelManagerRequestBusInterface::Register(this);
        }
    }

    LevelManager::~LevelManager()
    {
        if (LevelManagerRequestBusInterface::Get() == this)
        {
            LevelManagerRequestBusInterface::Unregister(this);
        }
    }

    void LevelManager::Activate()
    {
        LevelManagerRequestBus::Handler::BusConnect();
        SimulationFeaturesAggregatorRequestBus::Broadcast(
            &SimulationFeaturesAggregatorRequests::AddSimulationFeatures,
            AZStd::unordered_set<SimulationFeatureType>{ simulation_interfaces::msg::SimulatorFeatures::AVAILABLE_WORLDS,
                                                         simulation_interfaces::msg::SimulatorFeatures::WORLD_INFO_GETTING,
                                                         simulation_interfaces::msg::SimulatorFeatures::WORLD_LOADING,
                                                         simulation_interfaces::msg::SimulatorFeatures::WORLD_UNLOADING });
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
            constexpr const char* errorMsg = "Online search is not implemented yet";
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
        auto levelName = GetLevelNameFromAssetPath(levelPath);
        currentWorld.m_worldResource.m_uri = levelPath;
        currentWorld.m_name = levelName;
        return AZ::Success(currentWorld);
    }

    AZ::Outcome<WorldResource, FailedResult> LevelManager::LoadWorld(const LoadWorldRequest& request)
    {
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
            constexpr const char* errorMsg = "Requested world/level not found";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::srv::LoadWorld::Response::MISSING_ASSETS, errorMsg));
        }

        ILevelSystem* levelSystem = GetLevelSystem();
        if (levelSystem == nullptr)
        {
            constexpr const char* errorMsg = "Failed to start, level System not available";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::msg::Result::RESULT_OPERATION_FAILED, errorMsg));
        }

        levelSystem->UnloadLevel();

        if (!levelSystem->LoadLevel(request.levelResource.m_uri.c_str()))
        {
            constexpr const char* errorMsg = "Failed to load world";
            AZ_Warning("SimulationInterfaces", false, errorMsg);
            return AZ::Failure(FailedResult(simulation_interfaces::srv::LoadWorld::Response::RESOURCE_PARSE_ERROR, errorMsg));
        }
        // fill up response
        loadedWorld.m_worldResource = request.levelResource;
        return AZ::Success(loadedWorld);
    }
    AZ::Outcome<void, FailedResult> LevelManager::UnloadWorld()
    {
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
        return AZ::Success();
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
            if (!assetPath.starts_with("levels"))
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