/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/UnitTest/TestTypes.h>
#include <AzTest/AzTest.h>

#include <Georeference/GNSSFormatConversions.h>

namespace UnitTest
{

    class GNSSTest : public LeakDetectionFixture
    {
    };

    constexpr double OneMillimiter = 0.001; // 1 mm in meters
    constexpr double OneMillimeterInDegreesOnEquator = 360.0 / (40000.0 * 1000.0 / OneMillimiter); // 40000 km is the equator length

    TEST_F(GNSSTest, WGS84ToECEF)
    {
        using namespace ROS2::WGS;
        const AZStd::vector<AZStd::pair<WGS84Coordinate, Vector3d>> inputGoldSet = {
            { { 10.0, 20.0, 300.0 }, { 5903307.167667380, 2148628.092761247, 1100300.642188661 } },
            { { -70.0, 170.0, 500.0 }, { -2154856.524084172, 379959.3447517005, -5971509.853428957 } },
            { { 50.0, -120.0, -100.0 }, { -2053899.906222906, -3557458.991239029, 4862712.433262121 } },
        };
        for (const auto& [input, goldResult] : inputGoldSet)
        {
            const auto result = ROS2::Utils::GeodeticConversions::WGS84ToECEF(input);
            EXPECT_NEAR(result.m_x, goldResult.m_x, OneMillimiter);
            EXPECT_NEAR(result.m_y, goldResult.m_y, OneMillimiter);
            EXPECT_NEAR(result.m_z, goldResult.m_z, OneMillimiter);
        }
    }

    TEST_F(GNSSTest, ECEFToENU)
    {
        using namespace ROS2::WGS;
        const AZStd::vector<AZStd::tuple<Vector3d, WGS84Coordinate, Vector3d>> inputGoldSet = {
            { { -2053900.0, -3557459.0, 4862712.0 }, { 50.0, -120.0, -100.0 }, { -0.076833, -0.3202, -0.2969 } },
            { { 5903307.167667380, 2148628.092761247, 1100300.642188661 },
              { 11.0, 21.0, 400.0 },
              { -109638.9539891188, -110428.2398398574, -2004.501240225796 } },
            { { -2154856.524084172, 379959.3447517005, -5971509.853428957 },
              { -72.0, 169.0, 1000.0 },
              { 38187.58712786288, 222803.8182465429, -4497.428919329745 } },
        };
        for (const auto& [input, refWGS84, goldResult] : inputGoldSet)
        {
            const auto result = ROS2::Utils::GeodeticConversions::ECEFToENU(refWGS84, input);
            EXPECT_NEAR(result.m_x, goldResult.m_x, OneMillimiter);
            EXPECT_NEAR(result.m_y, goldResult.m_y, OneMillimiter);
            EXPECT_NEAR(result.m_z, goldResult.m_z, OneMillimiter);
        }
    }

    TEST_F(GNSSTest, ENUToECEF)
    {
        using namespace ROS2::WGS;
        const AZStd::vector<AZStd::tuple<Vector3d, WGS84Coordinate, Vector3d>> inputGoldSet = {
            { { -0.076833, -0.3202, -0.2969 }, { 50.0, -120.0, -100.0 }, { -2053900.0, -3557459.0, 4862712.0 } },
            { { -109638.9539891188, -110428.2398398574, -2004.501240225796 },
              { 11.0, 21.0, 400.0 },
              { 5903307.167667380, 2148628.092761247, 1100300.642188661 } },
            { { 38187.58712786288, 222803.8182465429, -4497.428919329745 },
              { -72.0, 169.0, 1000.0 },
              { -2154856.524084172, 379959.3447517005, -5971509.853428957 } },
        };
        for (const auto& [input, refWGS84, goldResult] : inputGoldSet)
        {
            const auto result = ROS2::Utils::GeodeticConversions::ENUToECEF(refWGS84, input);
            EXPECT_NEAR(result.m_x, goldResult.m_x, OneMillimiter);
            EXPECT_NEAR(result.m_y, goldResult.m_y, OneMillimiter);
            EXPECT_NEAR(result.m_z, goldResult.m_z, OneMillimiter);
        }
    }

    TEST_F(GNSSTest, ECEFToWSG84)
    {
        using namespace ROS2::WGS;
        const AZStd::vector<AZStd::pair<Vector3d, WGS84Coordinate>> inputGoldSet = {
            { { 5903307.167667380, 2148628.092761247, 1100300.642188661 }, { 10.0, 20.0, 300.0 } },
            { { -2154856.524084172, 379959.3447517005, -5971509.853428957 }, { -70.0, 170.0, 500.0 } },
            { { -2053899.906222906, -3557458.991239029, 4862712.433262121 }, { 50.0, -120.0, -100.0 } },
        };
        for (const auto& [input, goldResult] : inputGoldSet)
        {
            const auto result = ROS2::Utils::GeodeticConversions::ECEFToWGS84(input);
            EXPECT_NEAR(result.m_longitude, goldResult.m_longitude, OneMillimeterInDegreesOnEquator);
            EXPECT_NEAR(result.m_latitude, goldResult.m_latitude, OneMillimeterInDegreesOnEquator);
            EXPECT_NEAR(result.m_altitude, goldResult.m_altitude, OneMillimiter);
        }
    }
} // namespace UnitTest
