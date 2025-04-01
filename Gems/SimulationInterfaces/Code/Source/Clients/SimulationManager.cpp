/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "SimulationManager.h"

#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

#include "CommonUtilities.h"
#include "ConsoleCommands.icl"
#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetManagerBus.h>
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/regex.h>
#include <AzFramework/Components/TransformComponent.h>
#include <AzFramework/Physics/PhysicsSystem.h>
#include <AzFramework/Physics/RigidBodyBus.h>
#include <AzFramework/Physics/SimulatedBodies/RigidBody.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace SimulationInterfaces
{

   AZ_COMPONENT_IMPL(SimulationManager, "SimulationManager", SimulationManagerTypeId);

   void SimulationManager::Reflect(AZ::ReflectContext* context)
   {
       if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
       {
           serializeContext->Class<SimulationManager, AZ::Component>()->Version(0);
       }
   }

   void SimulationManager::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
   {
       provided.push_back(AZ_CRC_CE("SimulationManagerService"));
   }

   void SimulationManager::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
   {
       incompatible.push_back(AZ_CRC_CE("SimulationManagerService"));
   }

   void SimulationManager::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
   {
       required.push_back(AZ_CRC_CE("PhysicsService"));
   }

   void SimulationManager::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
   {
   }

   SimulationManager::SimulationManager()
   {
//       if (SimulationInterfacesInterface::Get() == nullptr)
//       {
//           SimulationInterfacesInterface::Register(this);
//       }
   }

   SimulationManager::~SimulationManager()
   {
//       if (SimulationInterfacesInterface::Get() == this)
//       {
//           SimulationInterfacesInterface::Unregister(this);
//       }
   }

   void SimulationManager::Init()
   {
   }

   void SimulationManager::Activate()
   {
   }

   void SimulationManager::Deactivate()
   {

   }

} // namespace SimulationInterfaces
