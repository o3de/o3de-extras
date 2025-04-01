/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <SimulationInterfaces/SimulationInterfacesBus.h>
#include <SimulationInterfaces/SimulationMangerRequestBus.h>

namespace SimulationInterfacesCommands
{

    using namespace SimulationInterfaces;
    static void simulationinterfaces_GetEntities(const AZ::ConsoleCommandContainer& arguments)
    {
        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, EntityFilter());
        AZ_Printf("SimulationInterfacesConsole", "Number of simulation entities: %d\n", entities.size());
        for (const auto& entity : entities)
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
        EntityFilter filter;
        filter.m_bounds_shape = AZStd::make_shared<Physics::SphereShapeConfiguration>(sphereShape);

        AZStd::vector<AZStd::string> entities;
        SimulationInterfacesRequestBus::BroadcastResult(entities, &SimulationInterfacesRequestBus::Events::GetEntities, filter);
        AZ_Printf("SimulationInterfacesConsole", "Number of simulation entities: %d\n", entities.size());
        for (const auto& entity : entities)
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
        EntityState entityState;
        SimulationInterfacesRequestBus::BroadcastResult(entityState, &SimulationInterfacesRequestBus::Events::GetEntityState, entityName);
        AZ_Printf("SimulationInterfacesConsole", "Entity %s\n", entityName.c_str());
        AZ_Printf(
            "SimulationInterfacesConsole",
            "Pose %f %f %f\n",
            entityState.m_pose.GetTranslation().GetX(),
            entityState.m_pose.GetTranslation().GetY(),
            entityState.m_pose.GetTranslation().GetZ());
        AZ_Printf(
            "SimulationInterfacesConsole",
            "Rotation (quaternion) %f %f %f %f\n",
            entityState.m_pose.GetRotation().GetX(),
            entityState.m_pose.GetRotation().GetY(),
            entityState.m_pose.GetRotation().GetZ(),
            entityState.m_pose.GetRotation().GetW());
        const AZ::Vector3 euler = entityState.m_pose.GetRotation().GetEulerDegrees();
        AZ_Printf("SimulationInterfacesConsole", "Rotation (euler) %f %f %f\n", euler.GetX(), euler.GetY(), euler.GetZ());
        AZ_Printf(
            "SimulationInterfacesConsole",
            "Twist Linear %f %f %f\n",
            entityState.m_twist_linear.GetX(),
            entityState.m_twist_linear.GetY(),
            entityState.m_twist_linear.GetZ());
        AZ_Printf(
            "SimulationInterfacesConsole",
            "Twist Angular %f %f %f\n",
            entityState.m_twist_angular.GetX(),
            entityState.m_twist_angular.GetY(),
            entityState.m_twist_angular.GetZ());
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
        bool isOk = false;
        SimulationInterfacesRequestBus::BroadcastResult(
            isOk, &SimulationInterfacesRequestBus::Events::SetEntityState, entityName, entityState);
        if (isOk)
        {
            AZ_Printf("SimulationInterfacesConsole", "Entity %s state set\n", entityName.c_str());
        }
        else
        {
            AZ_Printf("SimulationInterfacesConsole", "Entity %s state NOT set\n", entityName.c_str());
        }
    }

    static void simulationinterfaces_DeleteEntity(const AZ::ConsoleCommandContainer& arguments)
    {
        if (arguments.empty())
        {
            AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_DeleteEntity requires entity name\n");
            return;
        }
        const AZStd::string entityName = arguments[0];
        AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_DeleteEntity %s\n", entityName.c_str());
        bool isOk = false;
        SimulationInterfacesRequestBus::BroadcastResult(isOk, &SimulationInterfacesRequestBus::Events::DeleteEntity, entityName);
        if (isOk)
        {
            AZ_Printf("SimulationInterfacesConsole", "Entity %s deleted\n", entityName.c_str());
        }
        else
        {
            AZ_Printf("SimulationInterfacesConsole", "Entity %s NOT deleted\n", entityName.c_str());
        }
    }

    static void simulationinterfaces_GetSpawnables(const AZ::ConsoleCommandContainer& arguments)
    {
        AZ_Printf("SimulationInterfacesConsole", "simulationinterfaces_GetSpawnables\n");
        AZStd::vector<Spawnable> spawnables;
        SimulationInterfacesRequestBus::BroadcastResult(spawnables, &SimulationInterfacesRequestBus::Events::GetSpawnables);
        AZ_Printf("SimulationInterfacesConsole", "Number of spawnables: %d\n", spawnables.size());
        for (const auto& spawnable : spawnables)
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
            AZ_Printf("SimulationInterfacesConsole", "     simulationinterface_Spawn <name> <uri> <namespace> <x> <y> <z> \n");
            return;
        }
        AZStd::string name = arguments[0];
        AZStd::string uri = arguments[1];
        AZStd::string entityNamespace = arguments.size() > 2 ? arguments[2] : "";
        AZ::Transform initialPose = AZ::Transform::CreateIdentity();
        if (arguments.size() > 5)
        {
            initialPose.SetTranslation(
                AZ::Vector3(
                    AZStd::stof(AZStd::string(arguments[3])),
                    AZStd::stof(AZStd::string(arguments[4])),
                    AZStd::stof(AZStd::string(arguments[5]))));
        }
        SimulationInterfacesRequests::SpawnCompletedCb completedCb = [](const AZ::Outcome<AZStd::string, AZStd::string>& name)
        {
            if (name.IsSuccess())
            {
                AZ_Printf("SimulationInterfacesConsole", "Entity %s spawned and registered\n", name.GetValue().c_str());
            }
            else
            {
                AZ_Printf("SimulationInterfacesConsole", "Entity NOT spawned. Error : %s\n", name.GetError().c_str());
            }
        };
        SimulationInterfacesRequestBus::Broadcast(&SimulationInterfacesRequestBus::Events::SpawnEntity, name, uri, entityNamespace, initialPose, completedCb);
        AZ_Printf("SimulationInterfacesConsole", "simulationinterface_Spawn %s %s\n", name.c_str(), uri.c_str());
    }

    AZ_CONSOLEFREEFUNC(simulationinterfaces_Pause, AZ::ConsoleFunctorFlags::DontReplicate, "Pause simulation.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_Resume, AZ::ConsoleFunctorFlags::DontReplicate, "Resume simulation.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_Step, AZ::ConsoleFunctorFlags::DontReplicate, "Step simulation.");

    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetEntities, AZ::ConsoleFunctorFlags::DontReplicate, "Get all simulated entities in the scene.");
    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetEntitiesSphere, AZ::ConsoleFunctorFlags::DontReplicate, "Get all simulated entities in the radius.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_GetEntityState, AZ::ConsoleFunctorFlags::DontReplicate, "Get state of the entity.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_SetStateXYZ, AZ::ConsoleFunctorFlags::DontReplicate, "Set state of the entity.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_DeleteEntity, AZ::ConsoleFunctorFlags::DontReplicate, "Delete entity.");
    AZ_CONSOLEFREEFUNC(
        simulationinterfaces_GetSpawnables, AZ::ConsoleFunctorFlags::DontReplicate, "Get all spawnable entities in the scene.");
    AZ_CONSOLEFREEFUNC(simulationinterfaces_Spawn, AZ::ConsoleFunctorFlags::DontReplicate, "Spawn entity.");
} // namespace SimulationInterfacesCommands
