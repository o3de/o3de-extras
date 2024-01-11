/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "SkidSteeringModelComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2::VehicleDynamics
{
    void SkidSteeringModelComponent::Reflect(AZ::ReflectContext* context)
    {
        SkidSteeringDriveModel::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<SkidSteeringModelComponent, VehicleModelComponent>()->Version(3)->Field(
                "DriveModel", &SkidSteeringModelComponent::m_driveModel);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<SkidSteeringModelComponent>("Skid Steering Vehicle Model", "Skid steering vehicle model component")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/SkidSteeringVehicleModel.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Editor/Icons/Components/Viewport/SkidSteeringVehicleModel.svg")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &SkidSteeringModelComponent::m_driveModel,
                        "Drive model",
                        "Settings of the selected drive model");
            }
        }
    }

    SkidSteeringModelComponent::SkidSteeringModelComponent(
        const VehicleConfiguration& vehicleConfiguration, const SkidSteeringDriveModel& driveModel)
        : m_driveModel(driveModel)
    {
        m_vehicleConfiguration = vehicleConfiguration;
    }

    void SkidSteeringModelComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SkidSteeringModelService"));
    }

    void SkidSteeringModelComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SkidSteeringModelService"));
    }

    VehicleDynamics::DriveModel* SkidSteeringModelComponent::GetDriveModel()
    {
        return &m_driveModel;
    };

    void SkidSteeringModelComponent::Activate()
    {
        VehicleModelComponent::Activate();
        m_driveModel.Activate(m_vehicleConfiguration);
    }
} // namespace ROS2::VehicleDynamics
