#pragma once

#include <map>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Utils/Utils.h>
#include <AzCore/Math/MathUtils.h>

namespace ROS2
{
    class LidarTemplate
    {
    public:
        enum LidarModel
        {
            SickMRS6000
        };

        LidarModel m_model;
        AZStd::string m_name;
        float m_minHAngle;
        float m_maxHAngle;
        float m_minVAngle;
        float m_maxVAngle;
        int m_layers;
        int m_numberOfIncrements;
        float m_maxRange;
    };

    class LidarTemplateUtils
    {
    public:
        static LidarTemplate GetTemplate(LidarTemplate::LidarModel model)
        {
            static std::map<LidarTemplate::LidarModel, LidarTemplate> templates;

            if (templates.empty())
            {
                LidarTemplate sickMRS6000 =
                {
                    .m_model = LidarTemplate::SickMRS6000,
                    .m_name = "SickMRS6000",
                    .m_minHAngle = -120.0f,
                    .m_maxHAngle = 120.0f,
                    .m_minVAngle = 35.0f, //Test
                    .m_maxVAngle = -35.0f, //Test
                    .m_layers = 24,
                    .m_numberOfIncrements = 924,
                    .m_maxRange = 100.0f
                };
                templates[LidarTemplate::SickMRS6000] = sickMRS6000;
            }

            auto it = templates.find(model);
            if (it == templates.end())
            {
                return LidarTemplate(); // TODO - handle it
            }

            return it->second;
        }

        static size_t TotalPointCount(const LidarTemplate &t)
        {
            return t.m_layers * t.m_numberOfIncrements;
        }

        // TODO - lidars in reality do not have uniform distributions - populating needs to be defined per model
        static AZStd::vector<AZ::Vector3> PopulateRayDirections(LidarTemplate::LidarModel model)
        {
            auto lidarTemplate = GetTemplate(model);

            const float minVertAngle = AZ::DegToRad(lidarTemplate.m_minVAngle);
            const float maxVertAngle = AZ::DegToRad(lidarTemplate.m_maxVAngle);
            const float minHorAngle = AZ::DegToRad(lidarTemplate.m_minHAngle);
            const float maxHorAngle = AZ::DegToRad(lidarTemplate.m_maxHAngle);

            const float verticalStep = (maxVertAngle - minVertAngle)
                / static_cast<float>(lidarTemplate.m_layers);
            const float horizontalStep = (maxHorAngle - minHorAngle)
                / static_cast<float>(lidarTemplate.m_numberOfIncrements);

            AZStd::vector<AZ::Vector3> directions;

            for (int incr = 0; incr < lidarTemplate.m_numberOfIncrements; incr++)
            {
                for (int layer = 0; layer < lidarTemplate.m_layers; layer++)
                {
                    // roll is equal to 0, so it's skipped in the calculations
                    const float pitch = minVertAngle + layer * verticalStep;
                    const float yaw = minHorAngle + incr * horizontalStep;

                    const float x = AZ::Cos(yaw) * AZ::Cos(pitch);
                    const float y = AZ::Sin(yaw) * AZ::Cos(pitch);
                    const float z = AZ::Sin(pitch);

                    directions.push_back(AZ::Vector3(x, y, z));
                }
            }

            return directions;
        }
    };
}

