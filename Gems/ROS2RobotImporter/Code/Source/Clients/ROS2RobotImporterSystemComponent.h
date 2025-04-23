/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>

namespace ROS2
{
    //! A Component for importing robot definition from standard formats such as URDF.
    //! Sometimes, user decisions will be involved in the process.
    class ROS2RobotImporterSystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(ROS2RobotImporterSystemComponent, "{f2566021-450a-4eae-896f-b268492a58eb}");
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        ROS2RobotImporterSystemComponent() = default;
        virtual ~ROS2RobotImporterSystemComponent() = default;

    protected:
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Init() override{};
        void Activate() override{};
        void Deactivate() override{};
        //////////////////////////////////////////////////////////////////////////
    };
} // namespace ROS2
