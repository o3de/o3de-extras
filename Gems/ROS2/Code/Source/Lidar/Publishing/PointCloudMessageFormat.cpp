/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/RTTI/RTTIMacros.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/limits.h>
#include <Lidar/Publishing/PointCloudMessageFormat.h>

namespace ROS2
{
    bool IsPadding(FieldFlags fieldFlag)
    {
        return fieldFlag == FieldFlags::Padding8 || fieldFlag == FieldFlags::Padding16 || fieldFlag == FieldFlags::Padding32;
    }

    size_t GetFieldByteSize(FieldFlags fieldFlag)
    {
        switch (fieldFlag)
        {
            // clang-format off
        case FieldFlags::PositionXYZF32:            return 12U;
        case FieldFlags::SegmentationData96:        return 8U;
        case FieldFlags::IntensityF32:
        case FieldFlags::TU32:
        case FieldFlags::RangeU32:
        case FieldFlags::Padding32:                 return 4U;
        case FieldFlags::ReflectivityU16:
        case FieldFlags::RingU16:
        case FieldFlags::AmbientU16:
        case FieldFlags::Padding16:                 return 2U;
        case FieldFlags::Padding8:                  return 1U;
        default:                                    return AZStd::numeric_limits<size_t>::max();
            // clang-format on
        }
    }

    size_t GetActualFieldCount(FieldFlags fieldFlag)
    {
        switch (fieldFlag)
        {
            // clang-format off
        case FieldFlags::PositionXYZF32:        return 3;
        case FieldFlags::SegmentationData96:    return 2;
        default:                                return 1;
            // clang-format on
        }
    }

    RaycastResultFlags GetFieldProvider(FieldFlags fieldFlag)
    {
        switch (fieldFlag)
        {
            // clang-format off
        case FieldFlags::PositionXYZF32:            return RaycastResultFlags::Point;
        case FieldFlags::IntensityF32:              return RaycastResultFlags::Intensity;
        case FieldFlags::RangeU32:                  return RaycastResultFlags::Range;
        case FieldFlags::SegmentationData96:        return RaycastResultFlags::SegmentationData;
        default:                                    return RaycastResultFlags::None;
            // clang-format on
        }
    }

    AZStd::optional<AZStd::string> GetDefaultFieldName(FieldFlags fieldFlag)
    {
        switch (fieldFlag)
        {
            // clang-format off
        case FieldFlags::PositionXYZF32:            return "x, y, z";
        case FieldFlags::IntensityF32:              return "intensity";
        case FieldFlags::TU32:                      return "t";
        case FieldFlags::ReflectivityU16:           return "reflectivity";
        case FieldFlags::RingU16:                   return "ring";
        case FieldFlags::AmbientU16:                return "ambient";
        case FieldFlags::RangeU32:                  return "range";
        case FieldFlags::SegmentationData96:        return "entity_id, class_id";
        case FieldFlags::Padding8:
        case FieldFlags::Padding16:
        case FieldFlags::Padding32:
        default:                                    return AZStd::nullopt;
            // clang-format on
        }
    }

    AZStd::optional<AZStd::string> GetDatatypeString(FieldFlags fieldFlag)
    {
        switch (fieldFlag)
        {
            // clang-format off
        case FieldFlags::PositionXYZF32:            return "3 x F32";
        case FieldFlags::IntensityF32:              return "F32";
        case FieldFlags::TU32:                      return "U32";
        case FieldFlags::ReflectivityU16:
        case FieldFlags::RingU16:
        case FieldFlags::AmbientU16:                return "U16";
        case FieldFlags::RangeU32:                  return "U32";
        case FieldFlags::SegmentationData96:        return "I32 + U32 (rgba) + U8 + 24-bit padding";
        case FieldFlags::Padding8:
        case FieldFlags::Padding16:
        case FieldFlags::Padding32:
        default:                                    return AZStd::nullopt;
            // clang-format on
        }
    }

    FieldFormat::FieldFormat(FieldFlags flag)
        : m_fieldFlag(flag)
    {
        if (const auto name = GetDefaultFieldName(flag); name.has_value())
        {
            m_name = name.value();
        }

        if (const auto dataType = GetDatatypeString(flag); dataType.has_value())
        {
            m_dataType = dataType.value();
        }
    }

    void FieldFormat::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<FieldFormat>()
                ->Version(0)
                ->Field("fieldName", &FieldFormat::m_name)
                ->Field("datatype", &FieldFormat::m_dataType)
                ->Field("fieldOffset", &FieldFormat::m_fieldOffset)
                ->Field("fieldType", &FieldFormat::m_fieldFlag);

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<FieldFormat>("Lidar Segmentation Class Configuration", "Lidar Segmentation Class configuration")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FieldFormat::m_name,
                        "Field Name",
                        "A coma seperated list of ROS2 PC2 message field names.")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &FieldFormat::IsRegularField)
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &FieldFormat::ValidateFieldName)
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &FieldFormat::m_fieldFlag,
                        "Field Type",
                        "Type of the field. Each field may represent multiple actual ROS2 PC2 message fields (e.g. Position field "
                        "represents three F32 PC2 message fields).")
                    ->EnumAttribute(FieldFlags::PositionXYZF32, "Position (x,y,z)")
                    ->EnumAttribute(FieldFlags::IntensityF32, "Intensity")
                    ->EnumAttribute(FieldFlags::TU32, "T")
                    ->EnumAttribute(FieldFlags::ReflectivityU16, "Reflectivity")
                    ->EnumAttribute(FieldFlags::RingU16, "Ring")
                    ->EnumAttribute(FieldFlags::AmbientU16, "Ambient")
                    ->EnumAttribute(FieldFlags::RangeU32, "Range")
                    ->EnumAttribute(FieldFlags::SegmentationData96, "Segmentation Entity ID and class ID")
                    ->EnumAttribute(FieldFlags::Padding8, "Padding (1 byte)")
                    ->EnumAttribute(FieldFlags::Padding16, "Padding (2 bytes)")
                    ->EnumAttribute(FieldFlags::Padding32, "Padding (4 bytes)")
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &FieldFormat::OnTypeChanged)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default, &FieldFormat::m_fieldOffset, "Offset (bytes)", "Offset of the field in bytes.")
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, false)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &FieldFormat::IsRegularField)
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FieldFormat::m_dataType,
                        "Field Type",
                        "Description of the types comprising this field.")
                    ->Attribute(AZ::Edit::Attributes::ContainerCanBeModified, false)
                    ->Attribute(AZ::Edit::Attributes::Visibility, &FieldFormat::IsRegularField);
            }
        }
    }

    bool FieldFormat::IsRegularField() const
    {
        return m_fieldFlag != FieldFlags::Padding8 && m_fieldFlag != FieldFlags::Padding16 && m_fieldFlag != FieldFlags::Padding32;
    }

    AZ::Crc32 FieldFormat::OnTypeChanged()
    {
        auto fieldName = GetDefaultFieldName(m_fieldFlag);
        if (fieldName.has_value())
        {
            m_name = fieldName.value();
        }

        auto dataType = GetDatatypeString(m_fieldFlag);
        m_dataType = dataType.value_or("");

        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    bool NameUtils::IsNamesStringValid(const AZStd::string& namesString, size_t count)
    {
        AZStd::smatch match;
        auto searchStart = namesString.cbegin();
        for (size_t i = 0; i < count; ++i)
        {
            if (searchStart == namesString.cend())
            {
                return false;
            }

            if (!AZStd::regex_search(searchStart, namesString.cend(), match, SubNameRegex))
            {
                return false;
            }

            searchStart = match.suffix().first;
        }

        return true;
    }

    AZ::Outcome<void, AZStd::string> FieldFormat::ValidateFieldName(void* newValue, const AZ::Uuid& valueType) const
    {
        static const auto FieldNameRegex = AZStd::regex(R"(^\s*([^\s,]+)(?:\s*,\s*([^\s,]+))*\s*$)");

        if (azrtti_typeid<AZStd::string>() != valueType)
        {
            return AZ::Failure("Unexpectedly received a non-string type for a field name.");
        }

        if (!IsRegularField())
        {
            return AZ::Success();
        }

        const AZStd::string fieldName = *static_cast<AZStd::string*>(newValue);
        if (fieldName.empty() || !AZStd::regex_match(fieldName, FieldNameRegex) ||
            !NameUtils::IsNamesStringValid(fieldName, GetActualFieldCount(m_fieldFlag)))
        {
            return AZ::Failure("Invalid field name. Configured field names should consist of PC2 field names separated by comas if the "
                               "configured field represents multiple PC2 fields.");
        }

        return AZ::Success();
    }

    RaycastResultFlags GetNecessaryProviders(const Pc2MessageFormat& messageFormat)
    {
        auto providerFlags = RaycastResultFlags::None;
        for (const FieldFormat& fieldFormat : messageFormat)
        {
            providerFlags |= GetFieldProvider(fieldFormat.m_fieldFlag);
        }

        return providerFlags;
    }
} // namespace ROS2