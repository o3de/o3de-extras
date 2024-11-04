/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>
#include <Lidar/Publishing/PointCloudMessageFormat.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace ROS2
{
    using Pc2Message = sensor_msgs::msg::PointCloud2;

    struct Pc2MessageWrapper
    {
        [[nodiscard]] size_t GetPointCount() const;

        Pc2Message m_message;
        AZStd::vector<FieldFlags> m_fieldFlags; // Easier iterator generation.
        AZStd::vector<AZStd::string> m_fieldNames; // Stores copies of field names to avoid bad c-string access.
        // Since some Fields represent multiple actual message fields (e.g. position) and padding do not represent actual fields we store
        // offsets of the abstract fields separately.
        AZStd::vector<size_t> m_fieldOffsets;
    };

    class Pc2MessageBuilder
    {
    public:
        explicit Pc2MessageBuilder(const Pc2MessageFormat& messageFormat);
        Pc2MessageWrapper Get(
            const AZStd::string& frameId, builtin_interfaces::msg::Time timeStamp, size_t width, size_t height, bool isDense);

    private:
        // static AZStd::array<AZStd::string, 3> GetPositionFieldNames(const AZStd::string& name);
        void AddFieldFormatFields(const FieldFormat& fieldFormat);
        void AddField(const AZStd::string& name, uint8_t dataType);
        size_t m_offset = 0U;
        Pc2MessageWrapper m_messageWrapper;
    };
} // namespace ROS2
