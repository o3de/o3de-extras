
#include "LevelManager.h"
#include "AzCore/Outcome/Outcome.h"
#include "SimulationInterfaces/LevelManagerRequestBus.h"
#include "SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h"
#include "SimulationInterfaces/SimulationInterfacesTypeIds.h"
#include "SimulationInterfaces/WorldResource.h"
#include <AzCore/Serialization/SerializeContext.h>

namespace SimulationInterfaces
{
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

        return AZ::Success(availableWorlds);
    }
    AZ::Outcome<WorldResource, FailedResult> LevelManager::GetCurrentWorld()
    {
        WorldResource currentWorld;
        return AZ::Success(currentWorld);
    }
    AZ::Outcome<WorldResource, FailedResult> LevelManager::LoadWorld(const LoadWorldRequest& request)
    {
        WorldResource loadedWorld;
        return AZ::Success(loadedWorld);
    }
    AZ::Outcome<void, FailedResult> LevelManager::UnloadWorld()
    {
        return AZ::Success();
    }
} // namespace SimulationInterfaces