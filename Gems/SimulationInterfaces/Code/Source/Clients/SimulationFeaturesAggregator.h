/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Script/ScriptTimePoint.h>
#include <AzFramework/Entity/EntityContextBus.h>
#include <AzFramework/Physics/PhysicsScene.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>
#include <SimulationInterfaces/SimulationFeaturesAggregatorRequestBus.h>

namespace SimulationInterfaces
{
    class SimulationFeaturesAggregator
        : public AZ::Component
        , protected SimulationFeaturesAggregatorRequestBus::Handler
    {
    public:
        AZ_COMPONENT_DECL(SimulationFeaturesAggregator);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        SimulationFeaturesAggregator();
        ~SimulationFeaturesAggregator();

        // AZ::Component
        void Activate() override;
        void Deactivate() override;

    private:
        // SimulationFeaturesAggregatorRequestBus overrides
        void AddSimulationFeatures(const AZStd::unordered_set<SimulationFeatureType>& features) override;
        AZStd::unordered_set<SimulationFeatureType> GetSimulationFeatures() override;
        bool HasFeature(SimulationFeatureType feature) override;

        AZStd::unordered_set<SimulationFeatureType> m_registeredFeatures;
    };
} // namespace SimulationInterfaces
