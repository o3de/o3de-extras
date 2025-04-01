/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ROS2Controllers/ROS2ControllersTypeIds.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/Interface/Interface.h>

namespace ROS2Controllers {
class ROS2ControllersRequests {
public:
  AZ_RTTI(ROS2ControllersRequests, ROS2ControllersRequestsTypeId);
  virtual ~ROS2ControllersRequests() = default;
  // Put your public methods here
};

class ROS2ControllersBusTraits : public AZ::EBusTraits {
public:
  //////////////////////////////////////////////////////////////////////////
  // EBusTraits overrides
  static constexpr AZ::EBusHandlerPolicy HandlerPolicy =
      AZ::EBusHandlerPolicy::Single;
  static constexpr AZ::EBusAddressPolicy AddressPolicy =
      AZ::EBusAddressPolicy::Single;
  //////////////////////////////////////////////////////////////////////////
};

using ROS2ControllersRequestBus =
    AZ::EBus<ROS2ControllersRequests, ROS2ControllersBusTraits>;
using ROS2ControllersInterface = AZ::Interface<ROS2ControllersRequests>;

} // namespace ROS2Controllers
