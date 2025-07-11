/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/API/ToolsApplicationAPI.h>

#include <Clients/SimulationFeaturesAggregator.h>

namespace SimulationInterfaces
{
    /// System component for SimulationInterfaces editor
    class SimulationFeaturesAggregatorEditor : public SimulationFeaturesAggregator
    {
        using BaseSystemComponent = SimulationFeaturesAggregator;

    public:
        AZ_COMPONENT_DECL(SimulationFeaturesAggregatorEditor);

        static void Reflect(AZ::ReflectContext* context);

        SimulationFeaturesAggregatorEditor();
        ~SimulationFeaturesAggregatorEditor();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // AZ::Component
        void Init() override;
        void Activate() override;
        void Deactivate() override;
    };
} // namespace SimulationInterfaces
