/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Math/Sfmt.h>
#include <AzCore/std/string/string.h>
#include <Lidar/Publishing/PointCloudMessageBuilder.h>
#include <sensor_msgs/msg/point_field.h>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace ROS2
{
    enum Pc2FieldType : uint8_t
    {
        // clang-format off
        I8  = 1,
        U8  = 2,
        I16 = 3,
        U16 = 4,
        I32 = 5,
        U32 = 6,
        F32 = 7,
        F64 = 8,
        // clang-format on
    };

    static AZStd::optional<uint8_t> GetFieldType(FieldFlags flag)
    {
        switch (flag)
        {
            // clang-format off
        case FieldFlags::IntensityF32:              return F32;
        case FieldFlags::TU32:                      return U32;
        case FieldFlags::ReflectivityU16:
        case FieldFlags::RingU16:
        case FieldFlags::AmbientU16:                return U16;
        case FieldFlags::RangeU32:                  return U32;
        default:
            return AZStd::nullopt;
        }
        // clang-format on
    }

    size_t Pc2MessageWrapper::GetPointCount() const
    {
        return m_message.height * m_message.width;
    }

    Pc2MessageBuilder::Pc2MessageBuilder(const Pc2MessageFormat& messageFormat)
    {
        m_messageWrapper.m_fieldNames.clear();
        m_messageWrapper.m_fieldNames.reserve(messageFormat.size() + 4U); // Position takes up 3, segmentation 3 fields
        m_messageWrapper.m_fieldFlags.resize(messageFormat.size());
        m_messageWrapper.m_fieldOffsets.resize(messageFormat.size());
        for (size_t i = 0; i < messageFormat.size(); ++i)
        {
            auto& fieldFormat = messageFormat[i];
            m_messageWrapper.m_fieldFlags[i] = messageFormat[i].m_fieldFlag;
            m_messageWrapper.m_fieldOffsets[i] = messageFormat[i].m_fieldOffset;
            AddFieldFormatFields(fieldFormat);
        }
    }

    Pc2MessageWrapper Pc2MessageBuilder::Get(const AZStd::string& frameId, builtin_interfaces::msg::Time timeStamp, size_t count)
    {
        auto& message = m_messageWrapper.m_message;
        message.header.frame_id = frameId.data();
        message.header.stamp = timeStamp;

        message.point_step = m_offset;
        message.width = count;
        message.height = 1;

        message.row_step = message.width * message.point_step;
        message.data.resize(message.row_step);

        return m_messageWrapper;
    }

    void Pc2MessageBuilder::AddFieldFormatFields(const FieldFormat& fieldFormat)
    {
        if (fieldFormat.m_fieldFlag == FieldFlags::PositionXYZF32)
        {
            const auto fieldNames = NameUtils::ExtractFieldNameStrings<3>(fieldFormat.m_name);
            for (auto& name : fieldNames)
            {
                AddField(name, F32);
            }
            return;
        }

        if (fieldFormat.m_fieldFlag == FieldFlags::SegmentationData96)
        {
            const auto fieldNames = NameUtils::ExtractFieldNameStrings<2>(fieldFormat.m_name);
            AddField(fieldNames[0], Pc2FieldType::I32);
            AddField("rgba", Pc2FieldType::U32);
            AddField(fieldNames[1], Pc2FieldType::U8);
            m_offset += 3; // 3 bytes of padding to match the structure (4 byte alignment).
            return;
        }

        if (IsPadding(fieldFormat.m_fieldFlag))
        {
            m_offset += GetFieldByteSize(fieldFormat.m_fieldFlag);
            return;
        }

        if (const auto type = GetFieldType(fieldFormat.m_fieldFlag); type.has_value())
        {
            AddField(fieldFormat.m_name, type.value());
        }
        else
        {
            AZ_Warning(
                __func__, false, "Tried to add an invalid field of type %u. Publishing may not work as expected.", fieldFormat.m_fieldFlag);
        }
    }

    void Pc2MessageBuilder::AddField(const AZStd::string& name, uint8_t dataType)
    {
        sensor_msgs::msg::PointField point_field;
        m_messageWrapper.m_fieldNames.push_back(name);
        point_field.name = m_messageWrapper.m_fieldNames.back().c_str();
        point_field.count = 1U;
        point_field.datatype = dataType;
        point_field.offset = m_offset;
        m_messageWrapper.m_message.fields.push_back(point_field);

        m_offset += point_field.count * sizeOfPointField(dataType);
    }

} // namespace ROS2
