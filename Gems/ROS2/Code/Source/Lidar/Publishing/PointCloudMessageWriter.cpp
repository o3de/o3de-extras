/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <Lidar/Publishing/PointCloudMessageWriter.h>

namespace ROS2
{
    PointCloudMessageWriter::PointCloudMessageWriter(const Pc2MessageFormat& format)
        : m_isSet(format.size())
        , m_messageBuilder(format)
    {
        m_message = m_messageBuilder.Get();
    }

    void PointCloudMessageWriter::Reset(
        const AZStd::string& frameId, builtin_interfaces::msg::Time timeStamp, size_t width, size_t height, bool isDense)
    {
        m_message.Update(frameId, timeStamp, width, height, isDense);
    }

    void PointCloudMessageWriter::WriteResults(const RaycastResults& results, bool skipNonHits)
    {
        for (size_t i = 0; i < m_message.m_fieldFlags.size(); ++i)
        {
            const auto fielFlag = m_message.m_fieldFlags[i];
            if (IsPadding(fielFlag))
            {
                continue;
            }

            bool wasWrittenTo = false;
            switch (GetFieldProvider(m_message.m_fieldFlags.at(i)))
            {
            case RaycastResultFlags::Point:
                wasWrittenTo = WriteResultIfPresent<RaycastResultFlags::Point>(results, fielFlag, i, skipNonHits);
                break;
            case RaycastResultFlags::Range:
                wasWrittenTo = WriteResultIfPresent<RaycastResultFlags::Range>(results, fielFlag, i, skipNonHits);
                break;
            case RaycastResultFlags::Intensity:
                wasWrittenTo = WriteResultIfPresent<RaycastResultFlags::Intensity>(results, fielFlag, i, skipNonHits);
                break;
            case RaycastResultFlags::SegmentationData:
                wasWrittenTo = WriteResultIfPresent<RaycastResultFlags::SegmentationData>(results, fielFlag, i, skipNonHits);
                break;
            case RaycastResultFlags::Ring:
                wasWrittenTo = WriteResultIfPresent<RaycastResultFlags::Ring>(results, fielFlag, i, skipNonHits);
            default:
                break;
            }

            if (!wasWrittenTo)
            {
                FillWithDefaultValues(m_message.m_fieldFlags.at(i), i);
            }
        }
    }

    const Pc2Message& PointCloudMessageWriter::GetMessage() const
    {
        return m_message.m_message;
    }

    void PointCloudMessageWriter::FillWithDefaultValues(FieldFlags fieldFlag, size_t fieldIndex)
    {
        switch (fieldFlag)
        {
        case FieldFlags::PositionXYZF32:
            FillWithDefaultValues<FieldFlags::PositionXYZF32>(fieldIndex);
            break;
        case FieldFlags::IntensityF32:
            FillWithDefaultValues<FieldFlags::IntensityF32>(fieldIndex);
            break;
        case FieldFlags::TU32:
            FillWithDefaultValues<FieldFlags::TU32>(fieldIndex);
            break;
        case FieldFlags::ReflectivityU16:
            FillWithDefaultValues<FieldFlags::ReflectivityU16>(fieldIndex);
            break;
        case FieldFlags::RingU8:
            FillWithDefaultValues<FieldFlags::RingU8>(fieldIndex);
            break;
        case FieldFlags::RingU16:
            FillWithDefaultValues<FieldFlags::RingU16>(fieldIndex);
            break;
        case FieldFlags::AmbientU16:
            FillWithDefaultValues<FieldFlags::AmbientU16>(fieldIndex);
            break;
        case FieldFlags::RangeU32:
            FillWithDefaultValues<FieldFlags::RangeU32>(fieldIndex);
            break;
        case FieldFlags::SegmentationData96:
            FillWithDefaultValues<FieldFlags::SegmentationData96>(fieldIndex);
            break;
        default:
            break;
        }
    }
} // namespace ROS2