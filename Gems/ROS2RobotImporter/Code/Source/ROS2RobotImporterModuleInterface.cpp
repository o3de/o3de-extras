/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2RobotImporterModuleInterface.h"
#include <AzCore/Memory/Memory.h>

#include <ROS2RobotImporter/ROS2RobotImporterTypeIds.h>

#include <Clients/ROS2RobotImporterSystemComponent.h>

namespace ROS2RobotImporter
{
    AZ_TYPE_INFO_WITH_NAME_IMPL(
        ROS2RobotImporterModuleInterface, "ROS2RobotImporterModuleInterface", ROS2RobotImporterModuleInterfaceTypeId);
    AZ_RTTI_NO_TYPE_INFO_IMPL(ROS2RobotImporterModuleInterface, AZ::Module);
    AZ_CLASS_ALLOCATOR_IMPL(ROS2RobotImporterModuleInterface, AZ::SystemAllocator);

    ROS2RobotImporterModuleInterface::ROS2RobotImporterModuleInterface()
    {
        m_descriptors.insert(
            m_descriptors.end(),
            {
                ROS2RobotImporterSystemComponent::CreateDescriptor(),
            });
    }

    AZ::ComponentTypeList ROS2RobotImporterModuleInterface::GetRequiredSystemComponents() const
    {
        return AZ::ComponentTypeList{
            azrtti_typeid<ROS2RobotImporterSystemComponent>(),
        };
    }
} // namespace ROS2RobotImporter
