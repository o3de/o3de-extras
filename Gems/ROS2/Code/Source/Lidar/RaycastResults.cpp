/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <ROS2/Lidar/RaycastResults.h>

namespace ROS2
{
    RaycastResults::RaycastResults(RaycastResultFlags flags, size_t count)
        : m_count{ count }
    {
        EnsureFlagSatisfied<RaycastResultFlags::Point>(flags, count);
        EnsureFlagSatisfied<RaycastResultFlags::Range>(flags, count);
        EnsureFlagSatisfied<RaycastResultFlags::Intensity>(flags, count);
    }

    RaycastResults::RaycastResults(RaycastResults&& other)
        : m_count{ other.m_count }
        , m_points{ AZStd::move(other.m_points) }
        , m_ranges{ AZStd::move(other.m_ranges) }
        , m_intensities{ AZStd::move(other.m_intensities) }
    {
        other.m_count = 0U;
    }

    void RaycastResults::Clear()
    {
        m_count = 0U;
        ClearFieldIfPresent<RaycastResultFlags::Point>();
        ClearFieldIfPresent<RaycastResultFlags::Range>();
        ClearFieldIfPresent<RaycastResultFlags::Intensity>();
    }

    void RaycastResults::Resize(size_t count)
    {
        m_count = count;
        ResizeFieldIfPresent<RaycastResultFlags::Point>(count);
        ResizeFieldIfPresent<RaycastResultFlags::Range>(count);
        ResizeFieldIfPresent<RaycastResultFlags::Intensity>(count);
    }

    RaycastResults& RaycastResults::operator=(RaycastResults&& other)
    {
        if (this == &other)
        {
            return *this;
        }

        m_count = other.m_count;
        other.m_count = 0U;

        m_points = AZStd::move(other.m_points);
        m_ranges = AZStd::move(other.m_ranges);
        m_intensities = AZStd::move(other.m_intensities);

        return *this;
    }

    bool RaycastResults::IsEmpty() const
    {
        return GetCount() == 0U;
    }

    size_t RaycastResults::GetCount() const
    {
        return m_count;
    }
} // namespace ROS2
