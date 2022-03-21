#pragma once
#include <AzCore/Math/Vector3.h>

// TODO - switch to interface
namespace ROS2
{
    class LidarRaycaster
    {
    public:
        LidarRaycaster();
        void PerformRaycast(const AZ::Vector3 &start, const AZStd::vector<AZ::Vector3> &directions,
                       float distance, AZStd::vector<AZ::Vector3> &results);
    };
}  // namespace ROS2