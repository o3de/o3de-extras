#include <AzTest/AzTest.h>
#include <AzCore/UnitTest/TestTypes.h>

#include "GNSS/GNSSFormatConversions.h"

namespace UnitTest {

    class GNSSTest : public AllocatorsTestFixture {
    };

    TEST_F(GNSSTest, WGS84ToECEF) {
        const std::vector<std::pair<AZ::Vector3, AZ::Vector3>> input_gold_set = {
                { {10.0f, 20.0f, 300.0f}, {5903307.167667380f, 2148628.092761247f, 1100300.642188661f} },
                { {-70.0f, 170.0f, 500.0f}, {-2154856.524084172f, 379959.3447517005f, -5971509.853428957f} },
                { {50.0f, -120.0f, -100.0f}, {-2053899.906222906f, -3557458.991239029f, 4862712.433262121f} },
        };
        for (const auto& [input, goldResult] : input_gold_set) {
            AZ::Vector3 result = ROS2::GNSS::WGS84ToECEF(input);
            EXPECT_NEAR(result.GetX(), goldResult.GetX(), 1.0f);
            EXPECT_NEAR(result.GetY(), goldResult.GetY(), 1.0f);
            EXPECT_NEAR(result.GetZ(), goldResult.GetZ(), 1.0f);
        }
    }

    TEST_F(GNSSTest, ECEFToENU) {
        const std::vector<std::tuple<AZ::Vector3, AZ::Vector3, AZ::Vector3>> input_gold_set = {
                { { -2053900.0f, -3557459.0f, 4862712.0f},
                  {50.0f, -120.0f, -100.0f},
                  {-0.076833f, -0.3202f, -0.2969f} },
                { {5903307.167667380f, 2148628.092761247f, 1100300.642188661f},
                  {11.0f, 21.0f, 400.0f},
                  {-109638.9539891188f, -110428.2398398574f, -2004.501240225796f} },
                { {-2154856.524084172f, 379959.3447517005f, -5971509.853428957f},
                  {-72.0f, 169.0f, 1000.0f},
                  {38187.58712786288f, 222803.8182465429f, -4497.428919329745f} },
        };
        for (const auto& [input, refWGS84, goldResult] : input_gold_set) {
            AZ::Vector3 result = ROS2::GNSS::ECEFToENU(refWGS84, input);
            EXPECT_NEAR(result.GetX(), goldResult.GetX(), 1.0f);
            EXPECT_NEAR(result.GetY(), goldResult.GetY(), 1.0f);
            EXPECT_NEAR(result.GetZ(), goldResult.GetZ(), 1.0f);
        }
    }

    TEST_F(GNSSTest, ENUToECEF) {
        const std::vector<std::tuple<AZ::Vector3, AZ::Vector3, AZ::Vector3>> input_gold_set = {
                { {-0.076833f, -0.3202f, -0.2969f},
                  {50.0f, -120.0f, -100.0f},
                  {-2053900.0f, -3557459.0f, 4862712.0f} },
                { {-109638.9539891188f, -110428.2398398574f, -2004.501240225796f},
                  {11.0f, 21.0f, 400.0f},
                  {5903307.167667380f, 2148628.092761247f, 1100300.642188661f} },
                { {38187.58712786288f, 222803.8182465429f, -4497.428919329745f},
                  {-72.0f, 169.0f, 1000.0f},
                  {-2154856.524084172f, 379959.3447517005f, -5971509.853428957f} },
        };
        for (const auto& [ input, refWGS84, goldResult] : input_gold_set) {
            AZ::Vector3 result = ROS2::GNSS::ENUToECEF(refWGS84, input);
            EXPECT_NEAR(result.GetX(), goldResult.GetX(), 1.0f);
            EXPECT_NEAR(result.GetY(), goldResult.GetY(), 1.0f);
            EXPECT_NEAR(result.GetZ(), goldResult.GetZ(), 1.0f);
        }
    }

    TEST_F(GNSSTest, ECEFToWSG84) {
        const std::vector<std::pair<AZ::Vector3, AZ::Vector3>> input_gold_set = {
                { {5903307.167667380f, 2148628.092761247f, 1100300.642188661f},   {10.0f, 20.0f, 300.0f}   },
                { {-2154856.524084172f, 379959.3447517005f, -5971509.853428957f}, {-70.0f, 170.0f, 500.0f}  },
                { {-2053899.906222906f, -3557458.991239029f, 4862712.433262121f}, {50.0f, -120.0f, -100.0f} },
        };
        for (const auto& [input, goldResult] : input_gold_set) {
            AZ::Vector3 result = ROS2::GNSS::ECEFToWGS84(input);
            EXPECT_NEAR(result.GetX(), goldResult.GetX(), 0.001f);
            EXPECT_NEAR(result.GetY(), goldResult.GetY(), 0.001f);
            EXPECT_NEAR(result.GetZ(), goldResult.GetZ(), 1.0f);
        }
    }
}
