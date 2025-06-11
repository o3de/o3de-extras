/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/MathUtils.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <ROS2Controllers/Controllers/PidConfiguration.h>
#include <ROS2Controllers/ROS2ControllersEditorBus.h>
#include <ROS2Controllers/RobotControl/ControlConfiguration.h>
#include <ROS2Controllers/VehicleDynamics/AxleConfiguration.h>
#include <ROS2Controllers/VehicleDynamics/VehicleConfiguration.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>

#include <sdf/Joint.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>

namespace ROS2RobotImporter::SDFormat
{
    namespace SkidSteeringParser
    {
        ROS2Controllers::VehicleDynamics::VehicleConfiguration CreateVehicleConfiguration(
            const sdf::ElementPtr element, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
        {
            ROS2Controllers::VehicleDynamics::VehicleConfiguration configuration;
            auto addAxle = [&sdfModel, &createdEntities, &configuration](
                               const std::string& jointNameLeft,
                               const std::string& jointNameRight,
                               const float wheelDiameter,
                               AZStd::string tag) -> void
            {
                const auto entityIdLeft = HooksUtils::GetJointEntityId(jointNameLeft, sdfModel, createdEntities);
                const auto entityIdRight = HooksUtils::GetJointEntityId(jointNameRight, sdfModel, createdEntities);
                if (entityIdLeft.IsValid() && entityIdRight.IsValid())
                {
                    AZ::Entity* entityLeft = nullptr;
                    AZ::Entity* entityRight = nullptr;
                    AZ::ComponentApplicationBus::BroadcastResult(entityLeft, &AZ::ComponentApplicationRequests::FindEntity, entityIdLeft);
                    AZ::ComponentApplicationBus::BroadcastResult(entityRight, &AZ::ComponentApplicationRequests::FindEntity, entityIdRight);
                    auto interface = ROS2Controllers::ROS2ControllersEditorInterface::Get();
                    if (entityLeft && entityRight && interface)
                    {
                        interface->CreateWheelControllerComponent(*entityLeft, AZ::EntityId(), 0.0f);
                        interface->CreateWheelControllerComponent(*entityRight, AZ::EntityId(), 0.0f);
                    }
                    else
                    {
                        AZ_Warning(
                            "ROS2RobotImporter",
                            false,
                            "ROS2ControllersInterface is not available. Cannot create wheel controller components.");
                    }

                    HooksUtils::EnableMotor(entityIdLeft);
                    HooksUtils::EnableMotor(entityIdRight);

                    constexpr bool steering = false; // Skid steering model does not have any steering wheels.
                    constexpr bool drive = true;
                    configuration.m_axles.emplace_back(ROS2Controllers::VehicleDynamics::AxleConfiguration(
                        entityIdLeft, entityIdRight, AZStd::move(tag), wheelDiameter / 2.0f, steering, drive));
                }
                else
                {
                    AZ_Warning(
                        "CreateVehicleConfiguration",
                        entityIdLeft.IsValid(),
                        "Cannot find left joint entity %s while creating the axle %s.",
                        jointNameLeft.c_str(),
                        tag.c_str());
                    AZ_Warning(
                        "CreateVehicleConfiguration",
                        entityIdRight.IsValid(),
                        "Cannot find right joint entity %s while creating the axle %s.",
                        jointNameRight.c_str(),
                        tag.c_str());
                }
            };

            if (element->HasElement("wheelSeparation") && element->HasElement("wheelDiameter"))
            {
                // switch into configuration.m_track https://github.com/o3de/o3de-extras/issues/615
                configuration.m_wheelbase = element->Get<float>("wheelSeparation");

                { // ROS 1 version libgazebo_ros_skid_steer_drive.so
                    if (element->HasElement("leftJoint") && element->HasElement("rightJoint"))
                    {
                        addAxle(
                            element->Get<std::string>("leftJoint"),
                            element->Get<std::string>("rightJoint"),
                            element->Get<float>("wheelDiameter"),
                            "");
                    }
                }

                { // ROS 1 version libgazebo_ros_diff_drive.so
                    if (element->HasElement("leftFrontJoint") && element->HasElement("rightFrontJoint"))
                    {
                        addAxle(
                            element->Get<std::string>("leftFrontJoint"),
                            element->Get<std::string>("rightFrontJoint"),
                            element->Get<float>("wheelDiameter"),
                            "Front");
                    }
                    if (element->HasElement("leftRearJoint") && element->HasElement("rightRearJoint"))
                    {
                        addAxle(
                            element->Get<std::string>("leftRearJoint"),
                            element->Get<std::string>("rightRearJoint"),
                            element->Get<float>("wheelDiameter"),
                            "Rear");
                    }
                }
            }

            if (element->HasElement("wheel_separation") && element->HasElement("wheel_diameter"))
            {
                // ROS 2 version of either libgazebo_ros_skid_steer_drive.so or libgazebo_ros_diff_drive.so
                int dataCount = 0;

                auto wheelSeparation = element->GetElement("wheel_separation");
                auto wheelDiameter = element->GetElement("wheel_diameter");
                auto jointLeft = element->GetElement("left_joint");
                auto jointRight = element->GetElement("right_joint");
                while (jointLeft != nullptr && jointRight != nullptr && wheelSeparation != nullptr && wheelDiameter != nullptr)
                {
                    dataCount++;
                    if (dataCount == 1)
                    {
                        // switch into configuration.m_track https://github.com/o3de/o3de-extras/issues/615
                        configuration.m_wheelbase = wheelSeparation->Get<float>();
                    }
                    else
                    {
                        AZ_Warning(
                            "CreateVehicleConfiguration",
                            AZ::IsClose(configuration.m_wheelbase, wheelSeparation->Get<float>()),
                            "Different wheel separation distances in one model are not supported.");
                    }

                    addAxle(
                        jointLeft->Get<std::string>(),
                        jointRight->Get<std::string>(),
                        wheelDiameter->Get<float>(),
                        AZStd::to_string(dataCount));

                    jointLeft = jointLeft->GetNextElement("left_joint");
                    jointRight = jointRight->GetNextElement("right_joint");
                    wheelSeparation = wheelSeparation->GetNextElement("wheel_separation");
                    wheelDiameter = wheelDiameter->GetNextElement("wheel_diameter");
                }
                [[maybe_unused]] const auto wheelPairs = element->Get<int>("num_wheel_pairs", 1).first;
                AZ_Warning(
                    "CreateVehicleConfiguration",
                    wheelPairs == configuration.m_axles.size(),
                    "VehicleConfiguration parsing might be incorrect: expected %d axles, found %d.",
                    wheelPairs,
                    configuration.m_axles.size());
            }

            AZ_Warning(
                "ROS2SkidSteeringModelHook", !configuration.m_axles.empty(), "VehicleConfiguration parsing error: cannot find any axles.");

            return configuration;
        }
    } // namespace SkidSteeringParser

    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2SkidSteeringModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames =
            AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_skid_steer_drive.so", "libgazebo_ros_diff_drive.so" };
        importerHook.m_supportedPluginParams =
            AZStd::unordered_set<AZStd::string>{ ">wheelSeparation", ">wheelDiameter",    ">leftJoint",       ">rightJoint",
                                                 ">wheelDiameter",   ">leftFrontJoint",   ">rightFrontJoint", ">leftRearJoint",
                                                 ">rightRearJoint",  ">wheel_separation", ">wheel_diameter",  ">num_wheel_pairs",
                                                 ">left_joint",      ">right_joint" };

        importerHook.m_sdfPluginToComponentCallback =
            [](AZ::Entity& entity, const sdf::Plugin& sdfPlugin, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
            -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            // Parse parameters
            const sdf::ElementPtr element = sdfPlugin.Element();
            const auto vehicleConfiguration = SkidSteeringParser::CreateVehicleConfiguration(element, sdfModel, createdEntities);
            ROS2Controllers::ControlConfiguration controlConfiguration;
            controlConfiguration.m_steering = ROS2Controllers::ControlConfiguration::Steering::Twist;

            // libgazebo_ros_diff_drive plugin does not implement all parameters, use O3DE defaults:
            constexpr float linearLimit = 2.0f;
            constexpr float angularLimit = 3.5f;
            constexpr float linearAcceleration = 3.5f;
            constexpr float angularAcceleration = 2.0f;

            // Create required components
            auto* interface = ROS2Controllers::ROS2ControllersEditorInterface::Get();
            AZ_Assert(interface, "ROS2ControllersEditorInterface not available in ROS2SkidSteeringModelPluginHook");
            if (!interface)
            {
                return AZ::Failure(AZStd::string("ROS2ControllersInterface is not available. Cannot create components."));
            }

            HooksUtils::CreateComponent<ROS2::ROS2FrameEditorComponent>(entity);
            interface->CreateROS2RobotControlComponent(entity, controlConfiguration);
            interface->CreateSkidSteeringModelComponent(
                entity, vehicleConfiguration, linearLimit, angularLimit, linearAcceleration, angularAcceleration);

            // Create Skid Steering Control Component
            if (interface->CreateSkidSteeringControlComponent(entity))
            {
                return AZ::Success();
            }
            else
            {
                return AZ::Failure(AZStd::string("Failed to create ROS 2 Skid Steering Control Component"));
            }
        };

        return importerHook;
    }
} // namespace ROS2RobotImporter::SDFormat
