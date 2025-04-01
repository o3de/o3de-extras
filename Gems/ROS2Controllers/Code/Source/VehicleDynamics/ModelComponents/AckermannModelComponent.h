/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <VehicleDynamics/DriveModel.h>
#include <VehicleDynamics/DriveModels/AckermannDriveModel.h>
#include <VehicleDynamics/VehicleModelComponent.h>

namespace ROS2::VehicleDynamics
{
    class AckermannVehicleModelComponent : public VehicleModelComponent
    {
    public:
        AZ_COMPONENT(AckermannVehicleModelComponent, "{7618dbb9-fad4-44b3-8587-0d4f97336d3c}", VehicleModelComponent);
        AckermannVehicleModelComponent() = default;
        AckermannVehicleModelComponent(const VehicleConfiguration& vehicleConfiguration, const AckermannDriveModel& driveModel);

        static void Reflect(AZ::ReflectContext* context);

        // Component overrides
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        void Activate() override;

    private:
        AckermannDriveModel m_driveModel;

    protected:
        // VehicleModelComponent overrides
        DriveModel* GetDriveModel() override;
    };
} // namespace ROS2::VehicleDynamics
