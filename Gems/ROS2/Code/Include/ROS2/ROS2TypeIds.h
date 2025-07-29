/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

namespace ROS2
{
    // System Component TypeIds
    inline constexpr const char* ROS2SystemComponentTypeId = "{CB28D486-AFA4-4A9F-A237-AC5EB42E1C87}";
    inline constexpr const char* ROS2EditorSystemComponentTypeId = "{349883C1-C66E-45F5-90D7-884565FCFA7E}";

    // Clock
    inline constexpr const char* ROS2ClockRequestsTypeId = "{6DB40AA6-C8B7-40D5-A3D2-BABF9CC3864F}";
    inline constexpr const char* ROS2ClockSystemComponentTypeId = "{C5E8C74A-7A1F-43BF-BB05-D8FDA6B7D732}";
    inline constexpr const char* ROS2EditorClockSystemComponentTypeId = "{BF6C01D0-1E0E-41CD-A024-10A8DD503AB2}";

    // Module derived classes TypeIds
    inline constexpr const char* ROS2ModuleInterfaceTypeId = "{8B5567CB-1DE9-49AF-9CD4-9750D4ABCD6B}";
    inline constexpr const char* ROS2ModuleTypeId = "{3DDFC98F-D1CC-4658-BAF8-2CC34A9D39F3}";
    // The Editor Module by default is mutually exclusive with the Client Module
    // so they use the Same TypeId
    inline constexpr const char* ROS2EditorModuleTypeId = ROS2ModuleTypeId;

    // Interface TypeIds
    inline constexpr const char* ROS2RequestsTypeId = "{A9BDBFF6-E644-430D-8096-CDB53C88E8FC}";
    inline constexpr const char* ROS2EditorRequestsTypeId = "{E8A645D7-B0A4-4D6A-827B-64A44211BDB4}";

    // Communication Interface TypeIds
    inline constexpr const char* PublisherConfigurationTypeId = "{E50D1926-0322-4832-BD72-C48F854131C8}";
    inline constexpr const char* QoSTypeId = "{46692EA4-EA4C-495E-AD3C-426EAB8954D3}";
    inline constexpr const char* TopicConfigurationTypeId = "{7535D58F-5284-4657-A799-1F69D3F5AA42}";

    // Frame Interface TypeIds
    inline constexpr const char* NamespaceConfigurationTypeId = "{5E5BC6EA-DD01-480E-A4D1-6857CF70FDC8}";
    inline constexpr const char* ROS2FrameComponentTypeId = "{AC74CBC1-A5DC-4014-85D7-0E7934F352BD}";
    inline constexpr const char* ROS2FrameConfigurationTypeId = "{04882F01-5451-4EFA-B4F8-CD57E4B6CADF}";
    inline constexpr const char* ROS2FrameEditorComponentTypeId = "{F76D6F29-73C3-40B2-BCC2-47FC824C25DF}";

    // Sensors Interface TypeIds
    inline constexpr const char* SensorConfigurationTypeId = "{4755363D-0B5A-42D7-BBEF-152D87BA10D7}";
    inline constexpr const char* SensorConfigurationRequestTypeId = "{01904EAB-FA33-7487-B634-E3F8361EB5FB}";
    inline constexpr const char* ROS2SensorComponentBaseTypeId = "{2DF9A652-DF5D-43B1-932F-B6A838E36E97}";
    inline constexpr const char* EventSourceAdapterTypeId = "{DC8BB5F7-8E0E-42A1-BD82-5FCD9D31B9DD}";
    inline constexpr const char* TickBasedSourceTypeId = "{AD3CC041-5F7C-45E8-AA2D-5D8A1D4CC466}";
    inline constexpr const char* PhysicsBasedSourceTypeId = "{48BB21A8-F14E-4869-95DC-28EEA279Cf53}";

    // Spawner Bus TypeIds
    inline constexpr const char* SpawnerRequestsTypeId = "{3C42A3A1-1B8E-4800-9473-E4441315D7C8}";
    inline constexpr const char* SpawnerNotificationTypeId = "{0D22B024-48C7-457C-BFFB-D292045B68EC}";
    inline constexpr const char* SpawnerBusHandlerTypeId = "{9EB89664-0BE5-4E89-8E17-01B21073EBB8}";
} // namespace ROS2
