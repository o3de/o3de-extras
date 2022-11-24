/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>

#include <ROS2-Gem-Demo/ROS2-Gem-DemoBus.h>

namespace RobotVacuumSample
{
    class RobotVacuumSampleSystemComponent
        : public AZ::Component
        , protected RobotVacuumSampleRequestBus::Handler
    {
    public:
        AZ_COMPONENT(RobotVacuumSampleSystemComponent, "{c5c1e616-e0bd-4b49-afd0-053756e0f455}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        RobotVacuumSampleSystemComponent();
        ~RobotVacuumSampleSystemComponent();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // RobotVacuumSampleRequestBus interface implementation

        ////////////////////////////////////////////////////////////////////////

        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////
    };
}
