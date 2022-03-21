#pragma once

#include <map>
#include <AzCore/Math/Vector3.h>
#include <AzCore/Math/Quaternion.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Utils/Utils.h>

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
        static void PopulateRayDirections(LidarTemplate::LidarModel model, AZStd::vector<AZ::Vector3> &directions)
        {
            directions.clear();
            auto lidarTemplate = GetTemplate(model);
            float vertIncrement = (lidarTemplate.m_maxVAngle - lidarTemplate.m_minVAngle) / (float)(lidarTemplate.m_layers);
            float azimuthIncrAngle = (lidarTemplate.m_maxHAngle - lidarTemplate.m_minHAngle) / lidarTemplate.m_numberOfIncrements;

            for (int incr = 0; incr < lidarTemplate.m_numberOfIncrements; incr++)
            {
                for (int layer = 0; layer < lidarTemplate.m_layers; layer++)
                {
                    float angle = lidarTemplate.m_minVAngle + (float)layer * vertIncrement;
                    float azimuth = lidarTemplate.m_minHAngle + incr * azimuthIncrAngle;
                    AZ::Vector3 angles(0, angle, azimuth);
                    AZ::Vector3 normalizedForward = AZ::Vector3::CreateAxisX(1.0f);
                    auto quat = AZ::Quaternion::CreateFromEulerAnglesDegrees(angles);
                    directions.push_back(quat.TransformVector(normalizedForward));
                }
            }
        }
    };
}

