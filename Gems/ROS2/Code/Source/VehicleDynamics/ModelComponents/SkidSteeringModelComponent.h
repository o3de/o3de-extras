/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <VehicleDynamics/VehicleModelComponent.h>
#include <VehicleDynamics/DriveModels/SkidSteeringDriveModel.h>
#include <AzCore/Component/Component.h>

namespace ROS2::VehicleDynamics
{
    class SkidSteeringModelComponent
        : public VehicleModelComponent
    {
    public:
        AZ_COMPONENT(SkidSteeringModelComponent, "{57950C15-F7CF-422B-A452-E4487118F53E}", VehicleModelComponent);
        SkidSteeringModelComponent() = default;

        static void Reflect(AZ::ReflectContext* context);

        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        VehicleDynamics::SkidSteeringDriveModel m_driveModel;
    protected:
        //////////////////////////////////////////////////////////////////////////
        // VehicleModelComponent overrides
        VehicleDynamics::DriveModel * GetDriveModel() override;
        //////////////////////////////////////////////////////////////////////////

    };
} // namespace ROS2::VehicleDynamics
