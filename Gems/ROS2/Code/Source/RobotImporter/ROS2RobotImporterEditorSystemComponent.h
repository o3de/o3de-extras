/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "RobotImporter/ROS2RobotImporterSystemComponent.h"
#include <AzToolsFramework/Entity/EditorEntityContextBus.h>

namespace ROS2
{
    //! Editor component for RobotImporter widget
    class ROS2RobotImporterEditorSystemComponent
        : public ROS2RobotImporterSystemComponent
        , private AzToolsFramework::EditorEvents::Bus::Handler
    {
    public:
        AZ_COMPONENT(ROS2RobotImporterEditorSystemComponent, "{1cc069d0-72f9-411e-a94b-9159979e5a0c}", ROS2RobotImporterSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        ROS2RobotImporterEditorSystemComponent() = default;
        ~ROS2RobotImporterEditorSystemComponent() = default;

    private:
        void Activate() override;
        void Deactivate() override;
        void NotifyRegisterViews() override;
    };
} // namespace ROS2
