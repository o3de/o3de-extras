/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Math/Color.h>
#include <ROS2Sensors/Lidar/ClassSegmentationBus.h>
#include <ROS2Sensors/Lidar/SegmentationClassConfiguration.h>
#include <ROS2Sensors/ROS2SensorsTypeIds.h>

namespace ROS2
{
    class ClassSegmentationConfigurationComponent
        : public AZ::Component
        , ClassSegmentationRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ClassSegmentationConfigurationComponent, ROS2Sensors::ClassSegmentationConfigurationComponentTypeId, AZ::Component);

        ClassSegmentationConfigurationComponent() = default;
        ~ClassSegmentationConfigurationComponent() override = default;

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        // ClassSegmentationRequestBus overrides
        AZ::Color GetClassColor(uint8_t classId) const;
        AZStd::optional<uint8_t> GetClassIdForTag(LmbrCentral::Tag tag) const;
        const SegmentationClassConfigList& GetClassConfigList() const;

        // AZ::Component overrides
        void Activate() override;
        void Deactivate() override;

    private:
        AZ::Outcome<void, AZStd::string> ValidateSegmentationClasses(void* newValue, const AZ::TypeId& valueType) const;
        void ConstructSegmentationClassMaps();
        AZ::Crc32 OnSegmentationClassesChanged();

        SegmentationClassConfigList m_segmentationClasses;
        AZStd::unordered_map<LmbrCentral::Tag, uint8_t> m_tagToClassId;
        AZStd::unordered_map<uint8_t, AZ::Color> m_classIdToColor;
    };
} // namespace ROS2
