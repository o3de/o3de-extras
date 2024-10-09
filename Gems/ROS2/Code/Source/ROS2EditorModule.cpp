/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/RTTI/RTTIMacros.h>
#include <Camera/ROS2CameraSensorEditorComponent.h>
#include <Camera/ROS2EditorCameraSystemComponent.h>
#include <Frame/ROS2FrameSystemComponent.h>
#include <Georeference/GeoreferenceLevelEditorComponent.h>
#include <Lidar/LidarRegistrarEditorSystemComponent.h>
#include <Manipulation/JointsManipulationEditorComponent.h>
#include <Manipulation/JointsPositionsEditorComponent.h>
#include <QtCore/qglobal.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2ModuleInterface.h>
#include <RobotImporter/ROS2RobotImporterEditorSystemComponent.h>
#include <SdfAssetBuilder/SdfAssetBuilderSystemComponent.h>
#include <Spawner/ROS2SpawnPointEditorComponent.h>
#include <Spawner/ROS2SpawnerEditorComponent.h>
#include <SystemComponents/ROS2EditorSystemComponent.h>

void InitROS2Resources()
{
    // Registration of Qt (ROS2.qrc) resources
    Q_INIT_RESOURCE(ROS2);
}

namespace ROS2
{
    class ROS2EditorModule : public ROS2ModuleInterface
    {
    public:
        AZ_RTTI(ROS2EditorModule, "{3DDFC98F-D1CC-4658-BAF8-2CC34A9D39F3}", ROS2ModuleInterface);
        AZ_CLASS_ALLOCATOR(ROS2EditorModule, AZ::SystemAllocator);

        ROS2EditorModule()
        {
            InitROS2Resources();

            m_descriptors.insert(
                m_descriptors.end(),
                { ROS2EditorSystemComponent::CreateDescriptor(),
                  ROS2EditorCameraSystemComponent::CreateDescriptor(),
                  LidarRegistrarEditorSystemComponent::CreateDescriptor(),
                  ROS2RobotImporterEditorSystemComponent::CreateDescriptor(),
                  ROS2CameraSensorEditorComponent::CreateDescriptor(),
                  ROS2SpawnerEditorComponent::CreateDescriptor(),
                  ROS2SpawnPointEditorComponent::CreateDescriptor(),
                  SdfAssetBuilderSystemComponent::CreateDescriptor(),
                  JointsManipulationEditorComponent::CreateDescriptor(),
                  JointsPositionsEditorComponent::CreateDescriptor(),
                  GeoReferenceLevelEditorComponent::CreateDescriptor(),
                  ROS2FrameSystemComponent::CreateDescriptor(),
                  ROS2FrameEditorComponent::CreateDescriptor() });
        }

        AZ::ComponentTypeList GetRequiredSystemComponents() const override
        {
            return AZ::ComponentTypeList{
                azrtti_typeid<ROS2EditorSystemComponent>(),           azrtti_typeid<ROS2EditorCameraSystemComponent>(),
                azrtti_typeid<LidarRegistrarEditorSystemComponent>(), azrtti_typeid<ROS2RobotImporterEditorSystemComponent>(),
                azrtti_typeid<SdfAssetBuilderSystemComponent>(),      azrtti_typeid<ROS2FrameSystemComponent>(),
            };
        }
    };
} // namespace ROS2

AZ_DECLARE_MODULE_CLASS(Gem_ROS2, ROS2::ROS2EditorModule)
