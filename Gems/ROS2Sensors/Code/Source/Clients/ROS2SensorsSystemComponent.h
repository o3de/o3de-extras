/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>

namespace ROS2Sensors
{
    class ROS2SensorsSystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT_DECL(ROS2SensorsSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ROS2SensorsSystemComponent() = default;
        ~ROS2SensorsSystemComponent() = default;

    protected:
        ////////////////////////////////////////////////////////////////////////
        // AZ::Component interface implementation
        void Init() override{};
        void Activate() override{};
        void Deactivate() override{};
        ////////////////////////////////////////////////////////////////////////
    };

} // namespace ROS2Sensors
