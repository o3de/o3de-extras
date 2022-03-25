#pragma once
#include <AzCore/Math/Vector3.h>

// TODO - switch to interface
namespace ROS2
{
    class LidarRaycaster
    {
    public:
        // TODO - different starting points for rays, distance from reference point, noise models, rotating mirror sim, other
        // TODO - customized settings. Encapsulate in lidar definition and pass in constructor, update transform.
        AZStd::vector<AZ::Vector3> PerformRaycast(const AZ::Vector3 &start, const AZStd::vector<AZ::Vector3> &directions, float distance);
    };
}  // namespace ROS2
