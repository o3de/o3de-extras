/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Math/MathScriptHelpers.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <SimulationInterfaces/SimulationEntityManagerRequestBus.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>

namespace SimulationInterfacesCommands
{

    using namespace SimulationInterfaces;
    static void simulationinterfaces_GetEntities(const AZ::ConsoleCommandContainer& arguments)
    {
        AZ::Outcome<EntityNameList, FailedResult> entities;
        SimulationEntityManagerRequestBus::BroadcastResult(entities, &SimulationEntityManagerRequestBus::Events::GetEntities, EntityFilters());
        if (!entities.IsSuccess())
        {
            AZ_Printf("SimulationInterfacesConsole", "Failed to get entities: %s\n", entities.GetError().error_string.c_str());
            return;
        }

        for (const auto& entity : entities.GetValue())
        {
            AZ_Printf("SimulationInterfacesConsole", "      - %s\n", entity.c_str());
        }
    }

    static void simulationinterfaces_Pause(const AZ::ConsoleCommandContainer& arguments)
    {
        SimulationManagerRequestBus::Broadcast(&SimulationManagerRequestBus::Events::SetSimulationPaused, true);
    }

    static void simulationinterfaces_Resume(const AZ::ConsoleCommandContainer& arguments)
    {
        SimulationManagerRequestBus::Broadcast(&SimulationManagerRequestBus::Events::SetSimulationPaused, false);
    }

    static void simulationinterfaces_Step(const AZ::ConsoleCommandContainer& arguments)
    {
        if (arguments.empty())
        {
            AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_Step <number of steps>\n");
            return;
        }
        uint32_t steps = AZStd::stoi(AZStd::string(arguments[0]));

        SimulationManagerRequestBus::Broadcast(&SimulationManagerRequestBus::Events::StepSimulation, steps);
    }

    static void simulationinterfaces_GetEntitiesSphere(const AZ::ConsoleCommandContainer& arguments)
    {
        float sphereShape = 10.f;
        AZ::Vector3 position = AZ::Vector3::CreateZero();
        sphereShape = arguments.empty() ? 10.f : (AZStd::stof(AZStd::string(arguments[0])));
        position.SetX(arguments.size() > 1 ? (AZStd::stof(AZStd::string(arguments[1]))) : 0.f);
        position.SetY(arguments.size() > 2 ? (AZStd::stof(AZStd::string(arguments[2]))) : 0.f);
        position.SetZ(arguments.size() > 3 ? (AZStd::stof(AZStd::string(arguments[3]))) : 0.f);

        AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_GetEntities in radius %f \n", sphereShape);
        AZ_Printf("SimulationInterfacesConsole", "position %f %f %f \n", position.GetX(), position.GetY(), position.GetZ());
        EntityFilters filter;
        filter.m_bounds_shape = AZStd::make_shared<Physics::SphereShapeConfiguration>(sphereShape);

        AZ::Outcome<EntityNameList, FailedResult> entities;
        SimulationEntityManagerRequestBus::BroadcastResult(entities, &SimulationEntityManagerRequestBus::Events::GetEntities, filter);
        if (!entities.IsSuccess())
        {
            AZ_Printf("SimulationInterfacesConsole", "Failed to get entities: %s\n", entities.GetError().error_string.c_str());
            return;
        }

        for (const auto& entity : entities.GetValue())
        {
            AZ_Printf("SimulationInterfacesConsole", "      - %s\n", entity.c_str());
        }
    }

    static void simulationinterfaces_GetEntityState(const AZ::ConsoleCommandContainer& arguments)
    {
        if (arguments.empty())
        {
            AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_GetEntityState requires entity name\n");
            return;
        }
        const AZStd::string entityName = arguments[0];
        AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_GetEntityState %s\n", entityName.c_str());
        AZ::Outcome<EntityState, FailedResult> entityStateResult;
        SimulationEntityManagerRequestBus::BroadcastResult(entityStateResult, &SimulationEntityManagerRequestBus::Events::GetEntityState, entityName);
        if (!entityStateResult.IsSuccess())
        {
            AZ_Printf("SimulationInterfacesConsole", "Failed to get entity state: %s\n", entityStateResult.GetError().error_string.c_str());
            return;
        }
        const auto &entityState = entityStateResult.GetValue();
        AZ_Printf("SimulationInterfacesConsole", "Entity %s\n", entityName.c_str());
        AZ_Printf(
            "SimulationInterfacesConsole",
            "Pose %s\n",
            AZ::Vector3ToString(entityState.m_pose.GetTranslation()).c_str());
        AZ_Printf(
            "SimulationInterfacesConsole",
            "Rotation %s \n",
             AZ::QuaternionToString(entityState.m_pose.GetRotation()).c_str());

        const AZ::Vector3 euler = entityState.m_pose.GetRotation().GetEulerDegrees();
        AZ_Printf("SimulationInterfacesConsole", "Rotation (euler) %s\n", AZ::Vector3ToString(euler).c_str());
        AZ_Printf(
            "SimulationInterfacesConsole",
            "Twist Linear %s\n",
            AZ::Vector3ToString(entityState.m_twist_linear).c_str());
        AZ_Printf(
            "SimulationInterfacesConsole",
            "Twist Angular %s\n",
            AZ::Vector3ToString(entityState.m_twist_angular).c_str());
    }

    static void simulationinterfaces_SetStateXYZ(const AZ::ConsoleCommandContainer& arguments)
    {
        if (arguments.empty())
        {
            AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_GetEntityState requires entity name\n");
            return;
        }
        const AZStd::string entityName = arguments[0];
        AZ::Vector3 position = AZ::Vector3::CreateZero();
        position.SetX(arguments.size() > 1 ? (AZStd::stof(AZStd::string(arguments[1]))) : 0.f);
        position.SetY(arguments.size() > 2 ? (AZStd::stof(AZStd::string(arguments[2]))) : 0.f);
        position.SetZ(arguments.size() > 3 ? (AZStd::stof(AZStd::string(arguments[3]))) : 0.f);
        EntityState entityState{};
        entityState.m_pose = AZ::Transform::CreateIdentity();
        entityState.m_pose.SetTranslation(position);
        AZ::Outcome<void, FailedResult> result;
        SimulationEntityManagerRequestBus::BroadcastResult(
            result, &SimulationEntityManagerRequestBus::Events::SetEntityState, entityName, entityState);

        if (!result.IsSuccess())
        {
            AZ_Printf("SimulationInterfacesConsole", "Failed to set entity state: %s\n", result.GetError().error_string.c_str());
            return;
        }
        AZ_Printf("SimulationInterfacesConsole", "Entity %s state set\n", entityName.c_str());

    }

    static void simulationinterfaces_DeleteEntity(const AZ::ConsoleCommandContainer& arguments)
    {
        if (arguments.empty())
        {
            AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_DeleteEntity requires entity name\n");
            return;
        }
        const AZStd::string entityName = arguments[0];
        DeletionCompletedCb cb = [](const AZ::Outcome<void, FailedResult>& result)
        {
            if (result.IsSuccess())
            {
                AZ_Printf("SimulationInterfacesConsole", "Entity deleted\n");
            }
            else
            {
                AZ_Printf("SimulationInterfacesConsole", "Failed to delete entity: %s\n", result.GetError().error_string.c_str());
            }
        };
        SimulationEntityManagerRequestBus::Broadcast(&SimulationEntityManagerRequestBus::Events::DeleteEntity, entityName, cb);

    }

    static void simulationinterfaces_GetSpawnables(const AZ::ConsoleCommandContainer& arguments)
    {
        AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_GetSpawnables\n");
        AZ::Outcome<SpawnableList, FailedResult> spawnables;
        SimulationEntityManagerRequestBus::BroadcastResult(spawnables, &SimulationEntityManagerRequestBus::Events::GetSpawnables);
        if (!spawnables.IsSuccess())
        {
            AZ_Printf("SimulationInterfacesConsole", "Failed to get spawnables: %s\n", spawnables.GetError().error_string.c_str());
            return;
        }
        for (const auto& spawnable : spawnables.GetValue())
        {
            AZ_Printf("SimulationInterfaces", "      - %s\n", spawnable.m_uri.c_str());
        }
    }

    static void simulationinterfaces_Spawn(const AZ::ConsoleCommandContainer& arguments)
    {
        if (arguments.size() < 2)
        {
            AZ_Printf("SimulationInterfacesConsole", "simulationinterface_Spawn minimal :\n");
            AZ_Printf("SimulationInterfacesConsole", "     simulationinterface_Spawn <name> <uri>\n");
            AZ_Printf("SimulationInterfacesConsole", "simulationinterface_Spawn optional :\n");
            AZ_Printf("SimulationInterfacesConsole", "     simulationinterface_Spawn <name> <uri> <x> <y> <z> \n");
            return;
        }
        AZStd::string name = arguments[0];
        AZStd::string uri = arguments[1];
        AZ::Transform initialPose = AZ::Transform::CreateIdentity();
        if (arguments.size() > 5)
        {
            initialPose.SetTranslation(
                AZ::Vector3(
                    AZStd::stof(AZStd::string(arguments[3])),
                    AZStd::stof(AZStd::string(arguments[4])),
                    AZStd::stof(AZStd::string(arguments[5]))));
        }
        SpawnCompletedCb completedCb = [](const AZ::Outcome<SpawnedEntities, FailedResult>& result)
        {
            if (!result.IsSuccess())
            {
                AZ_Printf("SimulationInterfacesConsole", "Failed to spawn entity: %s\n", result.GetError().error_string.c_str());
                return;
            }
            AZ_Printf("SimulationInterfacesConsole", "Entity spawned and registered : %s\n", result.GetValue().m_name.c_str());

        };
        constexpr bool allowRename = true;
        SimulationEntityManagerRequestBus::Broadcast(&SimulationEntityManagerRequestBus::Events::SpawnEntity, name, uri, initialPose, allowRename, completedCb);
        AZ_Printf("SimulationInterfacesConsole", "simulationinterface_Spawn %s %s\n", name.c_str(), uri.c_str());
    }

    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetEntities, AZ::ConsoleFunctorFlags::DontReplicate, "Get all simulated entities in the scene.");

    AZ_CONSOLEFREEFUNC(simulationinterfaces_Pause, AZ::ConsoleFunctorFlags::DontReplicate, "Pause simulation.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_Resume, AZ::ConsoleFunctorFlags::DontReplicate, "Resume simulation.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_Step, AZ::ConsoleFunctorFlags::DontReplicate, "Step simulation.");


    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetEntitiesSphere, AZ::ConsoleFunctorFlags::DontReplicate, "Get all simulated entities in the radius.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_GetEntityState, AZ::ConsoleFunctorFlags::DontReplicate, "Get state of the entity.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_SetStateXYZ, AZ::ConsoleFunctorFlags::DontReplicate, "Set state of the entity.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_DeleteEntity, AZ::ConsoleFunctorFlags::DontReplicate, "Delete entity.");
    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetSpawnables, AZ::ConsoleFunctorFlags::DontReplicate, "Get all spawnable entities in the scene.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_Spawn, AZ::ConsoleFunctorFlags::DontReplicate, "Spawn entity.");
} // namespace SimulationInterfacesCommands
