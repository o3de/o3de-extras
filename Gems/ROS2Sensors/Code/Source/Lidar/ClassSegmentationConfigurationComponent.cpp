/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Lidar/ClassSegmentationConfigurationComponent.h>

namespace ROS2
{
    void ClassSegmentationConfigurationComponent::Reflect(AZ::ReflectContext* context)
    {
        SegmentationClassConfiguration::Reflect(context);
        if (auto* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<ClassSegmentationConfigurationComponent, AZ::Component>()->Version(0)->Field(
                "Segmentation Classes", &ClassSegmentationConfigurationComponent::m_segmentationClasses);

            if (auto* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<ClassSegmentationConfigurationComponent>("Class Segmentation Configuration", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZStd::vector<AZ::Crc32>({ AZ_CRC_CE("Level") }))
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ClassSegmentationConfigurationComponent::m_segmentationClasses,
                        "Segmentation classes",
                        "Segmentation classes and their colors.")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->Attribute(
                        AZ::Edit::Attributes::ChangeValidate, &ClassSegmentationConfigurationComponent::ValidateSegmentationClasses)
                    ->Attribute(
                        AZ::Edit::Attributes::ChangeNotify, &ClassSegmentationConfigurationComponent::OnSegmentationClassesChanged);
            }
        }
    }

    void ClassSegmentationConfigurationComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ClassSegmentationConfig"));
    }

    void ClassSegmentationConfigurationComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ClassSegmentationConfig"));
    }

    AZ::Color ClassSegmentationConfigurationComponent::GetClassColor(uint8_t classId) const
    {
        auto it = m_classIdToColor.find(classId);
        if (it == m_classIdToColor.end())
        {
            return AZ::Colors::White;
        }

        return it->second;
    }

    AZStd::optional<uint8_t> ClassSegmentationConfigurationComponent::GetClassIdForTag(LmbrCentral::Tag tag) const
    {
        auto it = m_tagToClassId.find(tag);
        if (it == m_tagToClassId.end())
        {
            return AZStd::nullopt;
        }

        return it->second;
    }

    const SegmentationClassConfigList& ClassSegmentationConfigurationComponent::GetClassConfigList() const
    {
        return m_segmentationClasses;
    }

    void ClassSegmentationConfigurationComponent::Activate()
    {
        ConstructSegmentationClassMaps();

        if (!ClassSegmentationInterface::Get())
        {
            ClassSegmentationInterface::Register(this);
        }
    }

    void ClassSegmentationConfigurationComponent::Deactivate()
    {
        if (ClassSegmentationInterface::Get() == this)
        {
            ClassSegmentationInterface::Unregister(this);
        }

        m_classIdToColor.clear();
        m_tagToClassId.clear();
    }

    AZ::Outcome<void, AZStd::string> ClassSegmentationConfigurationComponent::ValidateSegmentationClasses(
        void* newValue, const AZ::TypeId& valueType) const
    {
        AZ_Assert(azrtti_typeid<SegmentationClassConfigList>() == valueType, "Unexpected value type");
        if (azrtti_typeid<SegmentationClassConfigList>() != valueType)
        {
            return AZ::Failure(AZStd::string("Unexpectedly received an invalid type as segmentation classes!"));
        }

        const auto& segmentationClasses = *azrtti_cast<SegmentationClassConfigList*>(newValue);

        bool unknownPresent{ false }, groundPresent{ false };
        for (const auto& segmentationClass : segmentationClasses)
        {
            if (segmentationClass.m_classId == SegmentationClassConfiguration::UnknownClass.m_classId &&
                segmentationClass.m_className == SegmentationClassConfiguration::UnknownClass.m_className)
            {
                unknownPresent = true;
            }

            if (segmentationClass.m_classId == SegmentationClassConfiguration::GroundClass.m_classId &&
                segmentationClass.m_className == SegmentationClassConfiguration::GroundClass.m_className)
            {
                groundPresent = true;
            }
        }

        if (!unknownPresent || !groundPresent)
        {
            return AZ::Failure(
                AZStd::string::format("Segmentation class with name %s must exist.", (!unknownPresent ? "Unknown" : "Ground")));
        }

        return AZ::Success();
    }

    void ClassSegmentationConfigurationComponent::ConstructSegmentationClassMaps()
    {
        m_classIdToColor.reserve(m_segmentationClasses.size());
        m_tagToClassId.reserve(m_segmentationClasses.size());

        for (const auto& segmentationClass : m_segmentationClasses)
        {
            m_classIdToColor.insert(AZStd::make_pair(segmentationClass.m_classId, segmentationClass.m_classColor));
            m_tagToClassId.insert(AZStd::make_pair(LmbrCentral::Tag(segmentationClass.m_className), segmentationClass.m_classId));
        }
    }

    AZ::Crc32 ClassSegmentationConfigurationComponent::OnSegmentationClassesChanged()
    {
        bool unknownPresent{ false }, groundPresent{ false };
        for (auto& segmentationClass : m_segmentationClasses)
        {
            if (segmentationClass.m_classId == SegmentationClassConfiguration::UnknownClass.m_classId)
            {
                unknownPresent = true;
            }

            if (segmentationClass.m_classId == SegmentationClassConfiguration::GroundClass.m_classId)
            {
                groundPresent = true;
            }
        }

        if (unknownPresent && groundPresent)
        {
            return AZ::Edit::PropertyRefreshLevels::None;
        }

        if (!unknownPresent)
        {
            m_segmentationClasses.push_back(SegmentationClassConfiguration::UnknownClass);
        }

        if (!groundPresent)
        {
            m_segmentationClasses.push_back(SegmentationClassConfiguration::GroundClass);
        }

        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }
} // namespace ROS2
