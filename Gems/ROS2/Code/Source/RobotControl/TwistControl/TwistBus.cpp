/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/
#include "RobotControl/TwistControl/TwistBus.h"

namespace ROS2
{
    void TwistNotificationHandler::TwistReceived(const AZ::Vector3& v, const AZ::Vector3 &a)
    {
        Call(FN_TwistReceived, v, a);
    }

    void TwistNotificationHandler::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<TwistNotificationBus>("TwistNotificationBus")->
                    Handler<TwistNotificationHandler>()->
                    Event("TwistReceived", &TwistNotificationBus::Events::TwistReceived)
                    ;
        }
    }
}
