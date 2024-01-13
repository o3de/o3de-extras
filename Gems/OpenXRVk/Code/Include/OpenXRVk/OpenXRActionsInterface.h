/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Interface/Interface.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/Math/Transform.h>

#include <Atom/RHI.Reflect/Handle.h>

namespace OpenXRVk
{
    struct PoseWithVelocities
    {
        AZ::Transform m_pose;
        AZ::Vector3 m_linearVelocity;
        AZ::Vector3 m_angularVelocity;
    };

    //! Interface used to query the state of actions and also
    //! to drive the state of haptic feedback actions.
    //! The implementation is encouraged to expose each method
    //! of this interface as global functions in the behavior context.
    class IOpenXRActions
    {
    public:
        AZ_RTTI(IOpenXRActions, "{7B163790-BDBE-4C5B-832E-768CF5CDF585}");
        AZ_DISABLE_COPY_MOVE(IOpenXRActions);

        IOpenXRActions() = default;
        virtual ~IOpenXRActions() = default;

        using ActionHandle = AZ::RHI::Handle<uint16_t, IOpenXRActions>;

        virtual AZStd::vector<AZStd::string> GetAllActionSets() const = 0;
        virtual AZStd::vector<AZStd::string> GetActiveActionSets() const = 0;
        virtual AZStd::vector<AZStd::string> GetInactiveActionSets() const = 0;
        virtual AZ::Outcome<bool, AZStd::string> ChangeActionSetState(const AZStd::string& actionSetName, bool activate) = 0;
        virtual AZ::Outcome<bool, AZStd::string> ChangeActionSetsState(const  AZStd::vector<AZStd::string>& actionSetNames, bool activate) = 0;

        virtual ActionHandle GetActionHandle(const AZStd::string& actionSetName, const AZStd::string& actionName) const = 0;

        virtual AZ::Outcome<bool, AZStd::string> GetActionStateBoolean(ActionHandle actionHandle) const = 0;
        virtual AZ::Outcome<float, AZStd::string> GetActionStateFloat(ActionHandle actionHandle) const = 0;
        virtual AZ::Outcome<AZ::Vector2, AZStd::string> GetActionStateVector2(ActionHandle actionHandle) const = 0;
        virtual AZ::Outcome<AZ::Transform, AZStd::string> GetActionStatePose(ActionHandle actionHandle) const = 0;
        //! Same as above, but also queries location (linear) and orientation (angular) velocities
        virtual AZ::Outcome<PoseWithVelocities, AZStd::string> GetActionStatePoseWithVelocities(ActionHandle actionHandle) const = 0;
        
        virtual AZ::Outcome<bool, AZStd::string> ApplyHapticVibrationAction(ActionHandle actionHandle, uint64_t durationNanos, float frequencyHz, float amplitude) = 0;
        virtual AZ::Outcome<bool, AZStd::string> StopHapticVibrationAction(ActionHandle actionHandle) = 0;
    };

    using OpenXRActionsInterface = AZ::Interface<IOpenXRActions>;

} // namespace OpenXRVk