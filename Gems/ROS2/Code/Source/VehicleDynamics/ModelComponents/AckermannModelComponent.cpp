/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AckermannModelComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    void AckermannVehicleModelComponent::Reflect(AZ::ReflectContext* context)
    {
        AckermannDriveModel::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<AckermannVehicleModelComponent, VehicleModelComponent>()->Version(3)->Field(
                "DriveModel", &AckermannVehicleModelComponent::m_driveModel);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<AckermannVehicleModelComponent>("Ackermann Vehicle Model", "Ackermann vehicle model component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/AckermannVehicleModel.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/AckermannVehicleModel.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &AckermannVehicleModelComponent::m_driveModel,
                        "Drive model",
                        "Settings of the selected drive model");
            }
        }
    }

    AckermannVehicleModelComponent::AckermannVehicleModelComponent(
        const VehicleConfiguration& vehicleConfiguration, const AckermannDriveModel& driveModel)
        : m_driveModel(driveModel)
    {
        m_vehicleConfiguration = vehicleConfiguration;
    }

    void AckermannVehicleModelComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("AckermannModelService"));
    }

    void AckermannVehicleModelComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("AckermannModelService"));
    }

    VehicleDynamics::DriveModel* AckermannVehicleModelComponent::GetDriveModel()
    {
        return &m_driveModel;
    };

    void AckermannVehicleModelComponent::Activate()
    {
        VehicleModelComponent::Activate();
        m_driveModel.Activate(m_vehicleConfiguration);
    }

} // namespace ROS2::VehicleDynamics
