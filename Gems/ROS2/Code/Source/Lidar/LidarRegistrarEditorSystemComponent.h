/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzToolsFramework/Entity/EditorEntityContextBus.h>
#include <Lidar/LidarRegistrarSystemComponent.h>

namespace ROS2
{
    class LidarRegistrarEditorSystemComponent : public LidarRegistrarSystemComponent
    {
    public:
        AZ_COMPONENT(LidarRegistrarEditorSystemComponent, "{7f11b599-5ace-4498-a9a4-ad280c92bacc}", LidarRegistrarSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        LidarRegistrarEditorSystemComponent() = default;
        ~LidarRegistrarEditorSystemComponent() = default;

    private:
        void Activate() override;
        void Deactivate() override;
    };
} // namespace ROS2
