#pragma once
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
namespace UnitTest
{
    using namespace SimulationInterfaces;
    class SimulationEntityManagerMockedHandler : public SimulationInterfaces::SimulationEntityManagerRequestBus::Handler
    {
    public:
        SimulationEntityManagerMockedHandler()
        {
            SimulationInterfaces::SimulationEntityManagerRequestBus::Handler::BusConnect();
        }
        ~SimulationEntityManagerMockedHandler()
        {
            SimulationInterfaces::SimulationEntityManagerRequestBus::Handler::BusDisconnect();
        }

        MOCK_METHOD1(GetEntities, AZ::Outcome<EntityNameList, FailedResult>(const EntityFilters& filter));
        MOCK_METHOD1(GetEntityState, AZ::Outcome<EntityState, FailedResult>(const AZStd::string& name));
        MOCK_METHOD1(GetEntitiesStates, AZ::Outcome<MultipleEntitiesStates, FailedResult>(const EntityFilters& filter));
        MOCK_METHOD2(SetEntityState, AZ::Outcome<void, FailedResult>(const AZStd::string& name, const EntityState& state));
        MOCK_METHOD2(DeleteEntity, void(const AZStd::string& name, DeletionCompletedCb completedCb));
        MOCK_METHOD0(GetSpawnables, AZ::Outcome<SpawnableList, FailedResult>());
        MOCK_METHOD5(
            SpawnEntity,
            void(const AZStd::string& name, const AZStd::string& uri, const AZ::Transform& initialPose, const bool allowRename, SpawnCompletedCb completedCb));

    };
}
