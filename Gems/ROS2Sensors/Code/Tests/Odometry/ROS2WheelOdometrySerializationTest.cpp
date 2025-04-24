#include <Odometry/ROS2WheelOdometry.h>
#include <AzCore/UnitTest/TestTypes.h>
#include <AzTest/AzTest.h>
#include <rapidjson/stringbuffer.h>
#include <rapidjson/writer.h>

namespace UnitTest
{
    class ROS2SensorsTestFixture : public ::testing::Test
    {
    };

    // Store test for the old schema, where there is no configuration class.
    TEST_F(ROS2SensorsTestFixture, Deserialize_WhenOldSchemaProvided_StoresSuccessfully)
    {
        ROS2::ROS2WheelOdometryComponent wheelOdometryComponent;

        rapidjson::Document jsonDocument(rapidjson::kObjectType);
        jsonDocument.Parse(R"({
            "Twist covariance": {
                "Linear covariance": [
                    123.0,
                    456.0,
                    789.0
                ],
                "Angular covariance": [
                    987.0,
                    654.0,
                    321.0
                ]
            },
            "Pose covariance": {
                "Linear covariance": [
                    321.0,
                    654.0,
                    987.0
                ],
                "Angular covariance": [
                    789.0,
                    456.0,
                    123.0
                ]
            }
        })");

        AZ::JsonDeserializerSettings settings;
        auto storeResult = AZ::JsonSerialization::Load(wheelOdometryComponent, jsonDocument, settings);

        EXPECT_EQ(storeResult.GetOutcome(), AZ::JsonSerializationResult::Outcomes::PartialDefaults);

        rapidjson::Document document(rapidjson::kObjectType);
        document.Parse("{}");

        AZ::JsonSerializerSettings serializerSettings;
        serializerSettings.m_keepDefaults = true;

        AZ::JsonSerializationResult::ResultCode resultCode =
            AZ::JsonSerialization::Store(document, document.GetAllocator(), wheelOdometryComponent, serializerSettings);

        EXPECT_EQ(resultCode.GetOutcome(), AZ::JsonSerializationResult::Outcomes::Success);

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        document.Accept(writer);

        const char* expectedOutput =
            R"~({"Id":0,"SensorConfiguration":{"Visualize":true,"Publishing Enabled":true,"Frequency (HZ)":10.0,"Publishers":{"nav_msgs::msg::Odometry":{"Type":"nav_msgs::msg::Odometry","Topic":"odom","QoS":{"Reliability":2,"Durability":2,"Depth":5}}}},"Odometry configuration":{"Pose covariance":{"Linear covariance":[321.0,654.0,987.0],"Angular covariance":[789.0,456.0,123.0]},"Twist covariance":{"Linear covariance":[123.0,456.0,789.0],"Angular covariance":[987.0,654.0,321.0]}}})~";
        EXPECT_STREQ(buffer.GetString(), expectedOutput);
    }

    // Store test for the new schema, where there is a configuration class.
    TEST_F(ROS2SensorsTestFixture, Deserialize_WhenNewSchemaProvided_StoresSuccessfully)
    {
        ROS2::ROS2WheelOdometryComponent wheelOdometryComponent;

        rapidjson::Document jsonDocument(rapidjson::kObjectType);
        jsonDocument.Parse(R"({
            "Odometry configuration": {
            "Twist covariance": {
                "Linear covariance": [
                123.0,
                456.0,
                789.0
                ],
                "Angular covariance": [
                987.0,
                654.0,
                321.0
                ]
            },
            "Pose covariance": {
                "Linear covariance": [
                321.0,
                654.0,
                987.0
                ],
                "Angular covariance": [
                789.0,
                456.0,
                123.0
                ]
            }
            }
        })");

        AZ::JsonDeserializerSettings settings;
        auto storeResult = AZ::JsonSerialization::Load(wheelOdometryComponent, jsonDocument, settings);

        EXPECT_EQ(storeResult.GetOutcome(), AZ::JsonSerializationResult::Outcomes::PartialDefaults);

        rapidjson::Document document(rapidjson::kObjectType);
        document.Parse("{}");

        AZ::JsonSerializerSettings serializerSettings;
        serializerSettings.m_keepDefaults = true;

        AZ::JsonSerializationResult::ResultCode resultCode =
            AZ::JsonSerialization::Store(document, document.GetAllocator(), wheelOdometryComponent, serializerSettings);

        EXPECT_EQ(resultCode.GetOutcome(), AZ::JsonSerializationResult::Outcomes::Success);

        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        document.Accept(writer);

        const char* expectedOutput =
            R"~({"Id":0,"SensorConfiguration":{"Visualize":true,"Publishing Enabled":true,"Frequency (HZ)":10.0,"Publishers":{"nav_msgs::msg::Odometry":{"Type":"nav_msgs::msg::Odometry","Topic":"odom","QoS":{"Reliability":2,"Durability":2,"Depth":5}}}},"Odometry configuration":{"Pose covariance":{"Linear covariance":[321.0,654.0,987.0],"Angular covariance":[789.0,456.0,123.0]},"Twist covariance":{"Linear covariance":[123.0,456.0,789.0],"Angular covariance":[987.0,654.0,321.0]}}})~";
        EXPECT_STREQ(buffer.GetString(), expectedOutput);
    }
} // namespace UnitTest
