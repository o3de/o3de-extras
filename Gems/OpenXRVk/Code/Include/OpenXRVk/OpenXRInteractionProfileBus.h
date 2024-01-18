/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <openxr/openxr.h>

#include <AzCore/EBus/EBus.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/vector.h>

namespace OpenXRVk
{
    class OpenXRInteractionProfile
        : public AZ::EBusTraits
    {
    public:
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::ById;
        // The bus ID is OpenXRActionPath::m_interactionProfile.
        // Should be the exact same string as returned by GetName()
        typedef AZStd::string BusIdType;

        //////////////////////////////////////////////////////////
        // The following functions are called during asset creation time. 
        virtual AZStd::string GetName() const = 0;
        virtual AZStd::vector<AZStd::string> GetUserPaths() const = 0;
        virtual AZStd::string GetUserTopPath(const AZStd::string& userPathName) const = 0;
        virtual AZStd::vector<AZStd::string> GetComponentPaths(const AZStd::string& userPath) const = 0;
        //
        //////////////////////////////////////////////////////////
        

        ////////////////////////////////////////////////////////////
        // The following functions are called at runtime.
        //
        struct ActionPathInfo
        {
            //! Absolute path looks like:
            //! /user/hand/left/input/select/click
            AZStd::string m_absolutePath;
            XrActionType m_actionType = XrActionType::XR_ACTION_TYPE_MAX_ENUM;
        };

        //! @param userPath Typically comes from OpenXRActionPath::m_userPath.
        //! @param componentPath Typically comes from OpenXRActionPath::m_componentPath.
        virtual ActionPathInfo GetActionPathInfo(const AZStd::string& userPath, const AZStd::string& componentPath) const = 0;

        virtual AZStd::string GetInteractionProviderPath() const = 0;
        //
        /////////////////////////////////////////////////////////////
    };

    using OpenXRInteractionProfileBus = AZ::EBus<OpenXRInteractionProfile>;
}
