/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Math/MathUtils.h>
#include <AzToolsFramework/Entity/EditorEntityHelpers.h>
#include <AzToolsFramework/ToolsComponents/TransformComponent.h>
#include <PhysX/EditorColliderComponentRequestBus.h>
#include <ROS2/ROS2EditorBus.h>
#include <ROS2Controllers/Controllers/PidConfiguration.h>
#include <ROS2Controllers/ROS2ControllersEditorBus.h>
#include <ROS2Controllers/RobotControl/ControlConfiguration.h>
#include <ROS2Controllers/VehicleDynamics/AxleConfiguration.h>
#include <ROS2Controllers/VehicleDynamics/VehicleConfiguration.h>
#include <RobotImporter/SDFormat/ROS2ModelPluginHooks.h>
#include <RobotImporter/SDFormat/ROS2SDFormatHooksUtils.h>
#include <Source/EditorColliderComponent.h>
#include <Source/EditorHingeJointComponent.h>

#include <gz/math/Vector2.hh>
#include <gz/math/Vector3.hh>
#include <sdf/Joint.hh>
#include <sdf/Link.hh>
#include <sdf/Model.hh>
#include <sdf/Plugin.hh>

namespace ROS2RobotImporter::SDFormat
{
    namespace AckermannParser
    {
        static constexpr float epsilon = 0.001f;

        namespace Wheels
        {
            static constexpr unsigned int FrontLeft = 0;
            static constexpr unsigned int FrontRight = 1;
            static constexpr unsigned int RearLeft = 2;
            static constexpr unsigned int RearRight = 3;
            static constexpr unsigned int SteeringLeft = 4;
            static constexpr unsigned int SteeringRight = 5;
            static constexpr unsigned int Total = 6;
        } // namespace Wheels

        AZ::Outcome<AZ::Vector3, AZStd::string> GetPosition(const AZ::Entity* entity)
        {
            if (entity)
            {
                if (auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>())
                {
                    return AZ::Success(transformInterface->GetWorldTM().GetTranslation());
                }
            }

            return AZ::Failure(AZStd::string("Failed to obtain position parameter for wheel entity."));
        }

        float GetTrack(const AZ::Entity* entityLeft, const AZ::Entity* entityRight)
        {
            const auto positionLeft = GetPosition(entityLeft);
            const auto positionRight = GetPosition(entityRight);
            if (positionLeft.IsSuccess() && positionRight.IsSuccess())
            {
                return positionLeft.GetValue().GetDistance(positionRight.GetValue());
            }

            return 0.0f;
        }

        bool IsSteeringWheel(const std::string& jointNameSteering, const std::string& jointNameWheel, const sdf::Model& sdfModel)
        {
            const auto jointPtrSteering = sdfModel.JointByName(jointNameSteering);
            const auto jointPtrWheel = sdfModel.JointByName(jointNameWheel);
            if (jointPtrSteering != nullptr && jointPtrWheel != nullptr)
            {
                return (jointPtrSteering->ChildName() == jointPtrWheel->ParentName());
            }

            return false;
        }

        float GetWheelRadius(AZ::Entity* entity)
        {
            if (entity != nullptr)
            {
                PhysX::EditorColliderComponent* colliderComponent = entity->FindComponent<PhysX::EditorColliderComponent>();
                if (colliderComponent != nullptr)
                {
                    float radius = 0.0f;
                    AZ_Assert(entity->GetState() != AZ::Entity::State::Active, "Entity is active");
                    entity->Activate();
                    if (entity->GetState() == AZ::Entity::State::Active)
                    {
                        PhysX::EditorPrimitiveColliderComponentRequestBus::EventResult(
                            radius,
                            AZ::EntityComponentIdPair(entity->GetId(), colliderComponent->GetId()),
                            &PhysX::EditorPrimitiveColliderComponentRequests::GetCylinderRadius);
                        entity->Deactivate();
                    }
                    return radius;
                }
            }

            return 0.0f;
        }

        float GetWheelRadius(AZ::Entity* entityLeft, AZ::Entity* entityRight)
        {
            const float radiusLeft = GetWheelRadius(entityLeft);
            const float radiusRight = GetWheelRadius(entityRight);

            if (AZ::IsClose(radiusLeft, radiusRight, epsilon))
            {
                return radiusLeft;
            }
            else
            {
                AZ_Warning(
                    "ROS2AckermannModelPluginHook",
                    false,
                    "VehicleConfiguration parsing error: left and right wheel radii (%.4f and %.4f) do not match.",
                    radiusLeft,
                    radiusRight);
                return 0.0f;
            }
        }

        AZ::Vector3 GetWheelTranslation(const AZ::EntityId& entityId)
        {
            AZ::Entity* entity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationRequests::FindEntity, entityId);
            if (entity != nullptr)
            {
                if (auto* transformInterface = entity->FindComponent<AzToolsFramework::Components::TransformComponent>())
                {
                    return transformInterface->GetLocalTM().GetTranslation();
                }
            }

            AZ_Warning(
                "ROS2AckermannModelPluginHook", false, "Cannot find wheel position; track and wheelbase parameters might be incorrect.");
            return AZ::Vector3();
        }

        ROS2Controllers::PidConfiguration GetModelPidConfiguration(const sdf::ElementPtr element)
        {
            auto pid = element->Get<gz::math::Vector3d>("linear_velocity_pid_gain", gz::math::Vector3d::Zero).first;
            auto iRange = element->Get<gz::math::Vector2d>("linear_velocity_i_range", gz::math::Vector2d::Zero).first;

            ROS2Controllers::PidConfiguration config(pid.X(), pid.Y(), pid.Z(), iRange.Y(), iRange.X(), false, 0.0);
            return config;
        }

        ROS2Controllers::VehicleDynamics::VehicleConfiguration GetConfiguration(
            const sdf::ElementPtr element, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
        {
            const static AZStd::array<std::string, Wheels::Total> jointNamesSDFormat{ { "front_left_joint",
                                                                                        "front_right_joint",
                                                                                        "rear_left_joint",
                                                                                        "rear_right_joint",
                                                                                        "left_steering_joint",
                                                                                        "right_steering_joint" } };
            // Stores description of all joints in the SDF and a mapping to O3DE entities
            struct JointMapping
            {
                std::string m_jointName;
                AZ::EntityId m_entityId;
                AZ::Entity* m_entity{};
            };
            AZStd::array<JointMapping, Wheels::Total> jointMapper;

            for (unsigned int i = 0; i < Wheels::Total; ++i)
            {
                const std::string& name = jointNamesSDFormat[i];
                AZ_Warning(
                    "ROS2AckermannModelPluginHook",
                    element->HasElement(name),
                    "VehicleConfiguration parsing error: cannot find joint %s",
                    name.c_str());
                jointMapper[i].m_jointName = element->Get<std::string>(name, name).first;
                jointMapper[i].m_entityId = HooksUtils::GetJointEntityId(jointMapper[i].m_jointName, sdfModel, createdEntities);
                if (jointMapper[i].m_entityId.IsValid())
                {
                    jointMapper[i].m_entity = AzToolsFramework::GetEntityById(jointMapper[i].m_entityId);
                }
            }

            ROS2Controllers::VehicleDynamics::VehicleConfiguration configuration;
            auto controllersInterface = ROS2Controllers::ROS2ControllersEditorInterface::Get();
            if (!controllersInterface)
            {
                AZ_Warning(
                    "ROS2RobotImporter", false, "ROS2ControllersInterface is not available. Cannot create wheel controller components.");
                return configuration;
            }

            bool foundSteeringWheels = false;
            auto addAxle = [&](const unsigned int wheelIdLeft, const unsigned int wheelIdRight, AZStd::string tag) -> void
            {
                const auto& jointLeft = jointMapper[wheelIdLeft];
                const auto& jointRight = jointMapper[wheelIdRight];
                const auto& jointNameSteeringLeft = jointMapper[Wheels::SteeringLeft].m_jointName;
                const auto& jointNameSteeringRight = jointMapper[Wheels::SteeringRight].m_jointName;
                if (jointLeft.m_entityId.IsValid() && jointRight.m_entityId.IsValid())
                {
                    const bool isSteering = IsSteeringWheel(jointNameSteeringLeft, jointLeft.m_jointName, sdfModel) &&
                        IsSteeringWheel(jointNameSteeringRight, jointRight.m_jointName, sdfModel);
                    const bool isDrive = !isSteering;
                    const float wheelRadius = GetWheelRadius(jointLeft.m_entity, jointRight.m_entity);
                    configuration.m_axles.emplace_back(ROS2Controllers::VehicleDynamics::AxleConfiguration(
                        jointLeft.m_entityId, jointRight.m_entityId, AZStd::move(tag), wheelRadius, isSteering, isDrive));

                    const float track = GetTrack(jointLeft.m_entity, jointRight.m_entity);
                    if (track > epsilon)
                    {
                        AZ_Warning(
                            "ROS2AckermannModelPluginHook",
                            (AZ::IsClose(configuration.m_track, 0.0f, epsilon) && AZ::IsClose(configuration.m_track, track, epsilon)),
                            "VehicleConfiguration parsing error: different track per axis not supported.");
                        configuration.m_track = track;
                    }

                    if (isSteering)
                    {
                        const auto entityIdSteeringLeft = HooksUtils::GetJointEntityId(jointNameSteeringLeft, sdfModel, createdEntities);
                        controllersInterface->CreateWheelControllerComponent(*jointLeft.m_entity, entityIdSteeringLeft, 1.0f);
                        HooksUtils::EnableMotor(entityIdSteeringLeft);

                        const auto entityIdSteeringRight = HooksUtils::GetJointEntityId(jointNameSteeringRight, sdfModel, createdEntities);
                        controllersInterface->CreateWheelControllerComponent(*jointRight.m_entity, entityIdSteeringRight, 1.0f);
                        HooksUtils::EnableMotor(entityIdSteeringRight);
                        foundSteeringWheels = true;
                    }
                    else
                    {
                        controllersInterface->CreateWheelControllerComponent(*jointLeft.m_entity, AZ::EntityId(), 0.0f);
                        controllersInterface->CreateWheelControllerComponent(*jointRight.m_entity, AZ::EntityId(), 0.0f);
                    }
                    HooksUtils::EnableMotor(jointLeft.m_entityId);
                    HooksUtils::EnableMotor(jointRight.m_entityId);
                }
                else
                {
                    AZ_Warning(
                        "ROS2AckermannModelPluginHook",
                        false,
                        "Cannot find entity ID for one of the joints: %s or %s",
                        jointLeft.m_jointName.c_str(),
                        jointRight.m_jointName.c_str());
                }
            };

            addAxle(Wheels::FrontLeft, Wheels::FrontRight, "Front");
            addAxle(Wheels::RearLeft, Wheels::RearRight, "Rear");

            AZ_Warning(
                "ROS2AckermannModelPluginHook", foundSteeringWheels, "VehicleConfiguration parsing error: did not create steering joints.");

            const auto positionFrontLeft = GetPosition(jointMapper[Wheels::FrontLeft].m_entity);
            const auto positionFrontRight = GetPosition(jointMapper[Wheels::FrontRight].m_entity);
            const auto positionRearLeft = GetPosition(jointMapper[Wheels::RearLeft].m_entity);
            const auto positionRearRight = GetPosition(jointMapper[Wheels::RearRight].m_entity);
            if (positionFrontLeft.IsSuccess() && positionFrontRight.IsSuccess() && positionRearLeft.IsSuccess() &&
                positionRearRight.IsSuccess())
            {
                const auto positionFrontAxle = (positionFrontLeft.GetValue() + positionFrontRight.GetValue()) / 2.0f;
                const auto positionRearAxle = (positionRearLeft.GetValue() + positionRearRight.GetValue()) / 2.0f;
                configuration.m_wheelbase = positionFrontAxle.GetDistance(positionRearAxle);
            }

            return configuration;
        }
    } // namespace AckermannParser

    ModelPluginImporterHook ROS2ModelPluginHooks::ROS2AckermannModel()
    {
        ModelPluginImporterHook importerHook;
        importerHook.m_pluginNames = AZStd::unordered_set<AZStd::string>{ "libgazebo_ros_ackermann_drive.so" };
        importerHook.m_supportedPluginParams = AZStd::unordered_set<AZStd::string>{
            ">front_left_joint",     ">front_right_joint",        ">rear_left_joint",         ">rear_right_joint", ">left_steering_joint",
            ">right_steering_joint", ">linear_velocity_pid_gain", ">linear_velocity_i_range", ">max_speed",        ">max_steer"
        };

        importerHook.m_sdfPluginToComponentCallback =
            [](AZ::Entity& entity, const sdf::Plugin& sdfPlugin, const sdf::Model& sdfModel, const CreatedEntitiesMap& createdEntities)
            -> ModelPluginImporterHook::ConvertPluginOutcome
        {
            // Parse parameters
            const sdf::ElementPtr element = sdfPlugin.Element();
            const auto vehicleConfiguration = AckermannParser::GetConfiguration(element, sdfModel, createdEntities);
            const auto steeringPid = AckermannParser::GetModelPidConfiguration(element);
            ROS2Controllers::ControlConfiguration controlConfiguration;
            controlConfiguration.m_steering = ROS2Controllers::ControlConfiguration::Steering::Ackermann;

            // Set default limits as in libgazebo_ros_ackermann_drive plugin, see:
            // https://github.com/ros-simulation/gazebo_ros_pkgs/blob/79fd94c6da76781a91499bc0f54b70560b90a9d2/gazebo_plugins/src/gazebo_ros_ackermann_drive.cpp#L306C1-L307C65
            const float speedLimit = element->Get<double>("max_speed", 20.0).first;
            const float steeringLimit = element->Get<double>("max_steer", 0.6).first;
            const float accelLimit = 100.0f; // Acceleration limit is not specified in the plugin, so we use a default value.

            // Create required components
            auto* ros2interface = ROS2::ROS2EditorInterface::Get();
            AZ_Assert(ros2interface, "ROS2EditorInterface not available in ROS2ImuSensorHook");
            auto* controllersInterface = ROS2Controllers::ROS2ControllersEditorInterface::Get();
            AZ_Assert(controllersInterface, "ROS2ControllersEditorInterface not available in ROS2AckermannModelPluginHook");
            if (ros2interface && controllersInterface)
            {
                auto* ros2FrameComponent = ros2interface->CreateROS2FrameEditorComponent(entity, ROS2::ROS2FrameConfiguration());
                controllersInterface->CreateROS2RobotControlComponent(entity, controlConfiguration);
                controllersInterface->CreateAckermannVehicleModelComponent(
                    entity, vehicleConfiguration, speedLimit, steeringLimit, accelLimit, steeringPid);

                // Create Ackermann Control Component
                auto* ackermannComponent = controllersInterface->CreateAckermannControlComponent(entity);
                if (ros2FrameComponent && ackermannComponent)
                {
                    return AZ::Success();
                }
            }

            return AZ::Failure(AZStd::string("Failed to create ROS 2 Ackermann Control Component"));
        };

        return importerHook;
    }
} // namespace ROS2RobotImporter::SDFormat
