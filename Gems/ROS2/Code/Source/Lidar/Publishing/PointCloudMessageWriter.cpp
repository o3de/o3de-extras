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
    }

    void PointCloudMessageWriter::Reset(const AZStd::string& frameId, builtin_interfaces::msg::Time timeStamp, size_t count)
    {
        m_message = m_messageBuilder.Get(frameId, timeStamp, count);
    }

    void PointCloudMessageWriter::WriteResults(const RaycastResults& results)
    {
        for (size_t i = 0; i < m_message.m_fieldFlags.size(); ++i)
        {
            switch (GetFieldProvider(m_message.m_fieldFlags.at(i)))
            {
            case RaycastResultFlags::Point:
                WriteResultIfPresent<RaycastResultFlags::Point>(results, m_message.m_fieldFlags[i], i);
                break;
            case RaycastResultFlags::Range:
                WriteResultIfPresent<RaycastResultFlags::Range>(results, m_message.m_fieldFlags[i], i);
                break;
            case RaycastResultFlags::Intensity:
                WriteResultIfPresent<RaycastResultFlags::Intensity>(results, m_message.m_fieldFlags[i], i);
                break;
            case RaycastResultFlags::SegmentationData:
                WriteResultIfPresent<RaycastResultFlags::SegmentationData>(results, m_message.m_fieldFlags[i], i);
                break;
            default:
                // TODO
                break;
            }
        }
    }

    const Pc2Message& PointCloudMessageWriter::GetMessage()
    {
        return m_message.m_message;
    }
} // namespace ROS2