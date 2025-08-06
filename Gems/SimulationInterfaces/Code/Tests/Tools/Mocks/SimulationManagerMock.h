#pragma once

#include <SimulationInterfaces/SimulationMangerRequestBus.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
namespace UnitTest
{
    using namespace SimulationInterfaces;
    class SimulationManagerMockedHandler : public SimulationInterfaces::SimulationManagerRequestBus::Handler
    {
    public:
        SimulationManagerMockedHandler()
        {
            SimulationInterfaces::SimulationManagerRequestBus::Handler::BusConnect();
        }
        ~SimulationManagerMockedHandler()
        {
            SimulationInterfaces::SimulationManagerRequestBus::Handler::BusDisconnect();
        }

        MOCK_METHOD1(RestartSimulation, void(ReloadLevelCallback));
        MOCK_METHOD1(SetSimulationPaused, void(bool));
        MOCK_METHOD1(StepSimulation, void(AZ::u64));
        MOCK_METHOD(bool, IsSimulationPaused, (), (const));
        MOCK_METHOD0(CancelStepSimulation, void());
        MOCK_METHOD(bool, IsSimulationStepsActive, (), (const));
        MOCK_METHOD(SimulationState, GetSimulationState, (), (const));
        MOCK_METHOD1(SetSimulationState, AZ::Outcome<void, FailedResult>(SimulationState));
    };
} // namespace UnitTest
