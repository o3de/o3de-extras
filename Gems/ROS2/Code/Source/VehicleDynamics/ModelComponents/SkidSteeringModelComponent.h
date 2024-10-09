/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <VehicleDynamics/DriveModels/SkidSteeringDriveModel.h>
#include <VehicleDynamics/VehicleModelComponent.h>

namespace ROS2::VehicleDynamics
{
    class SkidSteeringModelComponent : public VehicleModelComponent
    {
    public:
        AZ_COMPONENT(SkidSteeringModelComponent, "{57950C15-F7CF-422B-A452-E4487118F53E}", VehicleModelComponent);
        SkidSteeringModelComponent() = default;
        SkidSteeringModelComponent(const VehicleConfiguration& vehicleConfiguration, const SkidSteeringDriveModel& driveModel);

        static void Reflect(AZ::ReflectContext* context);

        // Component overrides
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        // Component overrides
        void Activate() override;

    private:
        SkidSteeringDriveModel m_driveModel;

    protected:
        // VehicleModelComponent overrides
        DriveModel* GetDriveModel() override;
    };
} // namespace ROS2::VehicleDynamics
