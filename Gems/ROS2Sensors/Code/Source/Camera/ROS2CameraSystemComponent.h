/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/RPI.Public/Pass/PassSystemInterface.h>
#include <AzCore/Component/Component.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2
{
    //! System Component for Camera simulation in ROS 2.
    class ROS2SystemCameraComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(ROS2SystemCameraComponent, ROS2Sensors::ROS2SystemCameraComponentTypeId, AZ::Component);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);

        void InitPassTemplateMappingsHandler();

    protected:
        ////////////////////////////////////////////////////////////////////////
        // AZ::Component override
        void Activate() override;
        void Deactivate() override;
        ////////////////////////////////////////////////////////////////////////

    private:
        //! Load the pass templates of the ROS 2 Camera.
        void LoadPassTemplateMappings();
        AZ::RPI::PassSystemInterface::OnReadyLoadTemplatesEvent::Handler m_loadTemplatesHandler;
    };
} // namespace ROS2
