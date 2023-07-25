/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/UnitTest/TestTypes.h>
#include <AzCore/std/string/string.h>
#include <AzTest/AzTest.h>
#include <RobotImporter/SDFormat/Parser.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>

#include "SdfModel.h"
#include <sdf/Box.hh>
#include <sdf/sdf.hh>

namespace UnitTest
{

    class SdfModelTest : public LeakDetectionFixture
    {
    };

    TEST_F(SdfModelTest, CheckModelCorrectnessOneLink)
    {
        const auto xmlStr = SdfModel::GetSdfWithOneLink();
        const auto sdfRoot = ROS2::SDFormat::Parser::Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();

        EXPECT_EQ(sdfModel->Name(), "test_one_link");

        EXPECT_EQ(sdfModel->LinkCount(), 1U);
        const auto* link1 = sdfModel->LinkByName("link1");
        ASSERT_TRUE(link1);

        const auto& inertial = link1->Inertial();
        EXPECT_EQ(inertial.MassMatrix().Mass(), 1.0);
        EXPECT_EQ(inertial.MassMatrix().Ixx(), 1.0);
        EXPECT_EQ(inertial.MassMatrix().Ixy(), 0.0);
        EXPECT_EQ(inertial.MassMatrix().Ixz(), 0.0);
        EXPECT_EQ(inertial.MassMatrix().Iyy(), 1.0);
        EXPECT_EQ(inertial.MassMatrix().Iyz(), 0.0);
        EXPECT_EQ(inertial.MassMatrix().Izz(), 1.0);

        EXPECT_EQ(link1->VisualCount(), 1U);
        const auto visualGeometry = link1->VisualByIndex(0U)->Geom();
        ASSERT_TRUE(visualGeometry);
        EXPECT_EQ(visualGeometry->Type(), sdf::GeometryType::BOX);

        const auto material = link1->VisualByIndex(0U)->Material();
        ASSERT_TRUE(material);
        EXPECT_NEAR(material->Ambient().R(), 0.0, 1e-5);
        EXPECT_NEAR(material->Ambient().G(), 0.3, 1e-5);
        EXPECT_NEAR(material->Ambient().B(), 0.6, 1e-5);
        EXPECT_NEAR(material->Ambient().A(), 1.0, 1e-5);

        const auto* visualBox = visualGeometry->BoxShape();
        ASSERT_TRUE(visualBox);
        EXPECT_EQ(visualBox->Size().X(), 1.0);
        EXPECT_EQ(visualBox->Size().Y(), 2.0);
        EXPECT_EQ(visualBox->Size().Z(), 3.0);

        EXPECT_EQ(link1->CollisionCount(), 1U);
        const auto collisionGeometry = link1->CollisionByIndex(0U)->Geom();
        ASSERT_TRUE(collisionGeometry);
        EXPECT_EQ(collisionGeometry->Type(), sdf::GeometryType::BOX);

        const auto* collisionBox = collisionGeometry->BoxShape();
        ASSERT_TRUE(collisionBox);
        EXPECT_EQ(collisionBox->Size().X(), 1.0);
        EXPECT_EQ(collisionBox->Size().Y(), 2.0);
        EXPECT_EQ(collisionBox->Size().Z(), 3.0);

        EXPECT_EQ(link1->SensorCount(), 1U);
        const auto* sensor = link1->SensorByIndex(0U);
        ASSERT_TRUE(sensor);
        EXPECT_EQ(sensor->Type(), sdf::SensorType::CAMERA);
        EXPECT_EQ(sensor->UpdateRate(), 10);
        auto* cameraSensor = sensor->CameraSensor();
        ASSERT_TRUE(cameraSensor);
        EXPECT_EQ(cameraSensor->ImageWidth(), 640);
        EXPECT_EQ(cameraSensor->ImageHeight(), 480);
        EXPECT_NEAR(cameraSensor->HorizontalFov().Radian(), 2.0, 1e-5);
        EXPECT_NEAR(cameraSensor->NearClip(), 0.01, 1e-5);
        EXPECT_NEAR(cameraSensor->FarClip(), 1000, 1e-5);
    }

    TEST_F(SdfModelTest, CheckModelCorrectnessTwoLinks)
    {
        const auto xmlStr = SdfModel::GetSdfWithTwoLinksAndJoint();
        const auto sdfRoot = ROS2::SDFormat::Parser::Parse(xmlStr);
        const auto* sdfModel = sdfRoot->Model();

        EXPECT_EQ(sdfModel->Name(), "test_two_links");

        EXPECT_EQ(sdfModel->LinkCount(), 2U);
        const auto* link2 = sdfModel->LinkByName("link2");
        ASSERT_TRUE(link2);

        EXPECT_EQ(link2->VisualCount(), 1U);
        const auto visualGeometry = link2->VisualByIndex(0U)->Geom();
        ASSERT_TRUE(visualGeometry);
        EXPECT_EQ(visualGeometry->Type(), sdf::GeometryType::BOX);

        const auto* visualBox = visualGeometry->BoxShape();
        ASSERT_TRUE(visualBox);
        EXPECT_EQ(visualBox->Size().X(), 1.0);
        EXPECT_EQ(visualBox->Size().Y(), 2.0);
        EXPECT_EQ(visualBox->Size().Z(), 3.0);

        EXPECT_EQ(link2->CollisionCount(), 1U);
        const auto collisionGeometry = link2->CollisionByIndex(0U)->Geom();
        ASSERT_TRUE(collisionGeometry);
        EXPECT_EQ(collisionGeometry->Type(), sdf::GeometryType::BOX);

        const auto* collisionBox = collisionGeometry->BoxShape();
        ASSERT_TRUE(collisionBox);
        EXPECT_EQ(collisionBox->Size().X(), 1.0);
        EXPECT_EQ(collisionBox->Size().Y(), 2.0);
        EXPECT_EQ(collisionBox->Size().Z(), 3.0);

        EXPECT_EQ(link2->SensorCount(), 1U);
        const auto* sensor = link2->SensorByIndex(0U);
        ASSERT_TRUE(sensor);
        EXPECT_EQ(sensor->Type(), sdf::SensorType::LIDAR);
        EXPECT_EQ(sensor->UpdateRate(), 20);
        auto* lidarSensor = sensor->LidarSensor();
        ASSERT_TRUE(lidarSensor);
        EXPECT_EQ(lidarSensor->HorizontalScanSamples(), 640);
        EXPECT_NEAR(lidarSensor->HorizontalScanResolution(), 1.0, 1e-5);
        EXPECT_NEAR(lidarSensor->HorizontalScanMinAngle().Radian(), -2.0, 1e-5);
        EXPECT_NEAR(lidarSensor->HorizontalScanMaxAngle().Radian(), 2.5, 1e-5);
        EXPECT_NEAR(lidarSensor->RangeResolution(), 0.01, 1e-5);
        EXPECT_NEAR(lidarSensor->RangeMin(), 0.02, 1e-5);
        EXPECT_NEAR(lidarSensor->RangeMax(), 10.0, 1e-5);
        EXPECT_EQ(sensor->Plugins().size(), 1U);
        EXPECT_EQ(sensor->Plugins().at(0).Name(), "laser_plug");
        EXPECT_EQ(sensor->Plugins().at(0).Filename(), "librayplugin.so");

        EXPECT_EQ(sdfModel->JointCount(), 1U);
        const auto* joint = sdfModel->JointByName("joint");
        ASSERT_TRUE(joint);
        EXPECT_EQ(joint->Type(), sdf::JointType::REVOLUTE);
        EXPECT_EQ(joint->ParentName(), "link1");
        EXPECT_EQ(joint->ChildName(), "link2");
        EXPECT_NEAR(joint->RawPose().X(), 0.0, 1e-5);
        EXPECT_NEAR(joint->RawPose().Y(), 0.5, 1e-5);
        EXPECT_NEAR(joint->RawPose().Z(), 1.0, 1e-5);
        EXPECT_NEAR(joint->Axis()->Xyz().X(), 0.0, 1e-5);
        EXPECT_NEAR(joint->Axis()->Xyz().Y(), 0.0, 1e-5);
        EXPECT_NEAR(joint->Axis()->Xyz().Z(), 1.0, 1e-5);

        EXPECT_EQ(sdfModel->Plugins().size(), 1U);
        EXPECT_EQ(sdfModel->Plugins().at(0).Name(), "joint_state");
        EXPECT_EQ(sdfModel->Plugins().at(0).Filename(), "libgazebo_ros_joint_state_publisher.so");
    }
} // namespace UnitTest
