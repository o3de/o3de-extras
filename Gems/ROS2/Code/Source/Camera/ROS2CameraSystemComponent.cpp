/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#include "ROS2CameraSystemComponent.h"
#include <Atom/RPI.Public/Pass/PassSystemInterface.h>

namespace ROS2
{
   void ROS2SystemCameraComponent::Reflect(AZ::ReflectContext* context)
   {
       if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
       {
           serialize->Class<ROS2SystemCameraComponent, AZ::Component>()->Version(0);

           if (AZ::EditContext* ec = serialize->GetEditContext())
           {
               ec->Class<ROS2SystemCameraComponent>("ROS 2 System Camera Component", "This system component is responsible for setting a pass template for simulation of camera sensors.")
                   ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                   ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC_CE("System"))
                   ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                   ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
           }
       }
   }

   void ROS2SystemCameraComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
   {
       provided.push_back(AZ_CRC_CE("ROS2CameraSystemService"));
   }

   void ROS2SystemCameraComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
   {
       incompatible.push_back(AZ_CRC_CE("ROS2CameraSystemService"));
   }

   void ROS2SystemCameraComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
   {
       required.push_back(AZ_CRC_CE("ROS2Service"));
       required.push_back(AZ_CRC_CE("RPISystem"));
   }

   void ROS2SystemCameraComponent::InitPassTemplateMappingsHandler()
   {
       auto* passSystem = AZ::RPI::PassSystemInterface::Get();
       AZ_Assert(passSystem, "Cannot get the pass system.");

       m_loadTemplatesHandler = AZ::RPI::PassSystemInterface::OnReadyLoadTemplatesEvent::Handler(
           [this]()
           {
               this->LoadPassTemplateMappings();
           });
       passSystem->ConnectEvent(m_loadTemplatesHandler);
   }

   void ROS2SystemCameraComponent::Activate()
   {
       AZ::ApplicationTypeQuery appType;
       AZ::ComponentApplicationBus::Broadcast(&AZ::ComponentApplicationBus::Events::QueryApplicationType, appType);
       if (appType.IsGame() || appType.IsEditor())
       {
           InitPassTemplateMappingsHandler();
       }
   }

   void ROS2SystemCameraComponent::Deactivate()
   {
       m_loadTemplatesHandler.Disconnect();
   }

   void ROS2SystemCameraComponent::LoadPassTemplateMappings()
   {
       AZ_Printf("ROS2SystemCameraComponent", "LoadPassTemplateMappings\n");
       auto* passSystem = AZ::RPI::PassSystemInterface::Get();
       AZ_Assert(passSystem, "PassSystemInterface is null");

       const char* passTemplatesFile = "Passes/ROSPassTemplates.azasset";
       [[maybe_unused]] const bool isOk = passSystem->LoadPassTemplateMappings(passTemplatesFile);
       AZ_Assert(isOk, "LoadPassTemplateMappings return false ");
   }
} // namespace ROS2
