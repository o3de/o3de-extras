/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/MathUtils.h>
#include <ROS2/Frame/ROS2FrameEditorComponent.h>
#include <RobotControl/Controllers/SkidSteeringController/SkidSteeringControlComponent.h>
#include <RobotControl/ROS2RobotControlComponent.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <VehicleDynamics/ModelComponents/SkidSteeringModelComponent.h>
#include <VehicleDynamics/Utilities.h>

#include <sdf/Joint.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>

namespace ROS2::SDFormat
{
    namespace SkidSteeringParser
    {
        VehicleDynamics::VehicleConfiguration CreateVehicleConfiguration(
            const sdf::ElementPtr element, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
        {
            VehicleDynamics::VehicleConfiguration configuration;
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
                    HooksUtils::EnableMotor(entityIdLeft);
                    HooksUtils::CreateComponent<VehicleDynamics::WheelControllerComponent>(entityIdLeft);
                    HooksUtils::EnableMotor(entityIdRight);
                    HooksUtils::CreateComponent<VehicleDynamics::WheelControllerComponent>(entityIdRight);
                    constexpr bool steering = false; // Skid steering model does not have any steering wheels.
                    constexpr bool drive = true;
                    configuration.m_axles.emplace_back(VehicleDynamics::Utilities::Create2WheelAxle(
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
                const auto wheelPairs = element->Get<int>("num_wheel_pairs", 1).first;
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
                        constexpr float epsilon = 0.001f;
                        AZ_Warning(
                            "CreateVehicleConfiguration",
                            AZ::IsClose(configuration.m_wheelbase, wheelSeparation->Get<float>(), epsilon),
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
                AZ_Warning(
                    "CreateVehicleConfiguration",
                    wheelPairs == configuration.m_axles.size(),
                    "VehicleConfiguration parsing might be incorrect: expected %d axles, found %d.",
                    wheelPairs,
                    configuration.m_axles.size());
            }

            AZ_Warning(
                "CreateVehicleConfiguration", !configuration.m_axles.empty(), "VehicleConfiguration parsing error: cannot find any axles.");

            return configuration;
        }

        VehicleDynamics::SkidSteeringModelLimits CreateModelLimits(const sdf::ElementPtr element)
        {
            VehicleDynamics::SkidSteeringModelLimits modelLimits;
            if (element->HasElement("wheelAcceleration"))
            {
                modelLimits.SetAngularAccelerationLimit(element->Get<float>("wheelAcceleration"));
            }
            else if (element->HasElement("max_wheel_acceleration"))
            {
                modelLimits.SetAngularAccelerationLimit(element->Get<float>("max_wheel_acceleration"));
            }
            else
            {
                AZ_Warning("CreateModelLimits", false, "VehicleConfiguration parsing error: cannot determine model limits.");
            }

            return modelLimits;
        }
    } // namespace SkidSteeringParser

    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2SkidSteeringModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames =
            AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_skid_steer_drive.so", "libgazebo_ros_diff_drive.so" };

        importerHook.m_sdfPluginToComponentCallback =
            [](AZ::Entity& entity, const sdf::Plugin& sdfPlugin, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
            -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            // Parse parameters
            const sdf::ElementPtr element = sdfPlugin.Element();
            VehicleDynamics::VehicleConfiguration vehicleConfiguration =
                SkidSteeringParser::CreateVehicleConfiguration(element, sdfModel, createdEntities);
            VehicleDynamics::SkidSteeringModelLimits modelLimits = SkidSteeringParser::CreateModelLimits(element);

            // Create required components
            HooksUtils::CreateComponent<ROS2FrameEditorComponent>(entity);
            HooksUtils::CreateComponent<ROS2RobotControlComponent>(entity);
            HooksUtils::CreateComponent<VehicleDynamics::SkidSteeringModelComponent>(
                entity, vehicleConfiguration, VehicleDynamics::SkidSteeringDriveModel(modelLimits));

            // Create Skid Steering Control Component
            if (HooksUtils::CreateComponent<SkidSteeringControlComponent>(entity))
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
} // namespace ROS2::SDFormat
