/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SimulationFeaturesAggregator.h"

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{

    AZ_COMPONENT_IMPL(SimulationFeaturesAggregator, "SimulationFeaturesAggregator", SimulationFeaturesAggregatorTypeId);

    void SimulationFeaturesAggregator::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SimulationFeaturesAggregator, AZ::Component>()->Version(0);
        }
    }

    void SimulationFeaturesAggregator::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    void SimulationFeaturesAggregator::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SimulationFeaturesAggregator"));
    }

    void SimulationFeaturesAggregator::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void SimulationFeaturesAggregator::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    SimulationFeaturesAggregator::SimulationFeaturesAggregator()
    {
        if (SimulationFeaturesAggregatorRequestBusInterface::Get() == nullptr)
        {
            SimulationFeaturesAggregatorRequestBusInterface::Register(this);
        }
    }

    SimulationFeaturesAggregator::~SimulationFeaturesAggregator()
    {
        if (SimulationFeaturesAggregatorRequestBusInterface::Get() == this)
        {
            SimulationFeaturesAggregatorRequestBusInterface::Unregister(this);
        }
    }

    void SimulationFeaturesAggregator::Activate()
    {
        SimulationFeaturesAggregatorRequestBus::Handler::BusConnect();
    }

    void SimulationFeaturesAggregator::Deactivate()
    {
        SimulationFeaturesAggregatorRequestBus::Handler::BusDisconnect();
    }

    void SimulationFeaturesAggregator::AddSimulationFeatures(const AZStd::unordered_set<SimulationFeatureType>& features)
    {
        m_registeredFeatures.insert(features.begin(), features.end());
    }

    AZStd::unordered_set<SimulationFeatureType> SimulationFeaturesAggregator::GetSimulationFeatures()
    {
        return m_registeredFeatures;
    }

    bool SimulationFeaturesAggregator::HasFeature(SimulationFeatureType feature)
    {
        return m_registeredFeatures.contains(feature);
    }

} // namespace SimulationInterfaces
