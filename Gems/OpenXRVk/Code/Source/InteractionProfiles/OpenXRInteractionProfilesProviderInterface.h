/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <openxr/openxr.h>

#include <AzCore/Interface/Interface.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/vector.h>

#include "OpenXRInteractionProfileDescriptor.h"

namespace OpenXRVk
{
    class IOpenXRInteractionProfilesProvider
    {
    public:
        AZ_RTTI(IOpenXRInteractionProfilesProvider, "{25598F43-3634-4B2D-9A87-95E8BFDAB673}");
        AZ_DISABLE_COPY_MOVE(IOpenXRInteractionProfilesProvider);

        IOpenXRInteractionProfilesProvider() = default;
        virtual ~IOpenXRInteractionProfilesProvider() = default;

        virtual const AZStd::vector<AZStd::string>& GetInteractionProfileNames() const = 0;

        virtual const OpenXRInteractionProfileDescriptor * GetInteractionProfileDescriptor(const AZStd::string profileName) const = 0;

        // //////////////////////////////////////////////////////////
        // // The following functions are called during asset creation time. 
        // virtual AZStd::string GetName() const = 0;
        // virtual AZStd::vector<AZStd::string> GetUserPaths() const = 0;
        // virtual AZStd::string GetUserTopPath(const AZStd::string& userPathName) const = 0;
        // virtual AZStd::vector<AZStd::string> GetComponentPaths(const AZStd::string& userPath) const = 0;
        // //
        // //////////////////////////////////////////////////////////
        // 
        // 
        // ////////////////////////////////////////////////////////////
        // // The following functions are called at runtime.
        // //
        // struct ActionPathInfo
        // {
        //     //! Absolute path looks like:
        //     //! /user/hand/left/input/select/click
        //     AZStd::string m_absolutePath;
        //     XrActionType m_actionType = XrActionType::XR_ACTION_TYPE_MAX_ENUM;
        // };
        // 
        // //! @param userPath Typically comes from OpenXRActionPath::m_userPath.
        // //! @param componentPath Typically comes from OpenXRActionPath::m_componentPath.
        // virtual ActionPathInfo GetActionPathInfo(const AZStd::string& userPath, const AZStd::string& componentPath) const = 0;
        // 
        // //! @returns An interaction profile/provider string path that looks like:
        // //! "/interaction_profiles/khr/simple_controller"
        // //! "/interaction_profiles/oculus/touch_controller"
        // virtual AZStd::string GetInteractionProviderPath() const = 0;
        // //
        // /////////////////////////////////////////////////////////////
    };

    using OpenXRInteractionProfilesProviderInterface = AZ::Interface<IOpenXRInteractionProfilesProvider>;
}
