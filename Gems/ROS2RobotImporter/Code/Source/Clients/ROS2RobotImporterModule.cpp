/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2RobotImporterSystemComponent.h"
#include <ROS2RobotImporter/ROS2RobotImporterTypeIds.h>
#include <ROS2RobotImporterModuleInterface.h>

namespace ROS2RobotImporter
{
    class ROS2RobotImporterModule : public ROS2RobotImporterModuleInterface
    {
    public:
        AZ_RTTI(ROS2RobotImporterModule, ROS2RobotImporterModuleTypeId, ROS2RobotImporterModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2RobotImporterModule, AZ::SystemAllocator);
    };
} // namespace ROS2RobotImporter

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), ROS2RobotImporter::ROS2RobotImporterModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_ROS2RobotImporter, ROS2RobotImporter::ROS2RobotImporterModule)
#endif
