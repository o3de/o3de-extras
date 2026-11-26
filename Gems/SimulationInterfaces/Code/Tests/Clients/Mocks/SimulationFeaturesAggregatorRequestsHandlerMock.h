#pragma once
#include <AzTest/AzTest.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>
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
        MOCK_METHOD(AZStd::unordered_set<SimulationFeatureType>, GetSimulationFeatures, (), (override));
        MOCK_METHOD(bool, HasFeature, (SimulationFeatureType feature), (override));
    };
} // namespace UnitTest
