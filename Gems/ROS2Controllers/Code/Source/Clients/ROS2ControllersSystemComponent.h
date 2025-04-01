/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <ROS2Controllers/ROS2ControllersBus.h>

namespace ROS2Controllers {
class ROS2ControllersSystemComponent
    : public AZ::Component,
      protected ROS2ControllersRequestBus::Handler,
      public AZ::TickBus::Handler {
public:
  AZ_COMPONENT_DECL(ROS2ControllersSystemComponent);

  static void Reflect(AZ::ReflectContext *context);

  static void
  GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType &provided);
  static void GetIncompatibleServices(
      AZ::ComponentDescriptor::DependencyArrayType &incompatible);
  static void
  GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType &required);
  static void
  GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType &dependent);

  ROS2ControllersSystemComponent();
  ~ROS2ControllersSystemComponent();

protected:
  ////////////////////////////////////////////////////////////////////////
  // ROS2ControllersRequestBus interface implementation

  ////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////
  // AZ::Component interface implementation
  void Init() override;
  void Activate() override;
  void Deactivate() override;
  ////////////////////////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////////////////////
  // AZTickBus interface implementation
  void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
  ////////////////////////////////////////////////////////////////////////
};

} // namespace ROS2Controllers
