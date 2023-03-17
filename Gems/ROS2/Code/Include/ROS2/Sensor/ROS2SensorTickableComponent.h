/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#pragma once

#include "SensorConfiguration.h"
#include "ROS2SensorComponent.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <ROS2/ROS2GemUtilities.h>

namespace ROS2
{
    //! Extends ROS2SensorComponent with mechanism to trigger sensor's measurement with given frequency.
    //! Note that this mechanism utilizes AZ::TickBus::Handler::OnTick method.
   class ROS2SensorTickableComponent
       : public ROS2SensorComponent
   {
   public:
       ROS2SensorTickableComponent() = default;
       virtual ~ROS2SensorTickableComponent() = default;
       AZ_COMPONENT(ROS2SensorTickableComponent, "{ec5e8bd4-2953-4d93-9dc2-744d726517e1}", ROS2SensorComponent);

       //////////////////////////////////////////////////////////////////////////
       // Component overrides
       void Activate() override;
       void Deactivate() override;
       //////////////////////////////////////////////////////////////////////////

       static void Reflect(AZ::ReflectContext* context);
       void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;

   private:
       //! Executes the sensor action (acquire data -> publish) according to frequency.
       //! Override to implement a specific sensor behavior.
       virtual void FrequencyTick(){};

   };
} // namespace ROS2
