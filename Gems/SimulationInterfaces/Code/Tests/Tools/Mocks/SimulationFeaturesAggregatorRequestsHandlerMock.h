#pragma once
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
#include <gmock/gmock.h>
#include <gtest/gtest.h>
namespace UnitTest
{
    using namespace SimulationInterfaces;
    class SimulationFeaturesAggregatorRequestsMockedHandler : public SimulationInterfaces::SimulationFeaturesAggregatorRequestBus::Handler
    {
    public:
        SimulationFeaturesAggregatorRequestsMockedHandler()
        {
            SimulationInterfaces::SimulationFeaturesAggregatorRequestBus::Handler::BusConnect();
        }
        ~SimulationFeaturesAggregatorRequestsMockedHandler()
        {
            SimulationInterfaces::SimulationFeaturesAggregatorRequestBus::Handler::BusDisconnect();
        }

        MOCK_METHOD(void, AddSimulationFeatures, (const AZStd::unordered_set<SimulationFeatureType>& features), (override));
        MOCK_METHOD(const AZStd::unordered_set<SimulationFeatureType>, GetSimulationFeatures, (), (const, override));
        MOCK_METHOD(bool, HasFeature, (SimulationFeatureType feature), (const, override));
    };
} // namespace UnitTest
