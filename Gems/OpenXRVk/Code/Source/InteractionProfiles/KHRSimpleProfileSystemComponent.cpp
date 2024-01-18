/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */


#include <AzCore/Serialization/SerializeContext.h>

#include "KHRSimpleProfileSystemComponent.h"

namespace OpenXRVk
{
    void KHRSimpleProfileSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("InteractionProfileProviderService"));
    }

    void KHRSimpleProfileSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<KHRSimpleProfileSystemComponent, AZ::Component>()
                ->Version(1);
        }
    }

    void KHRSimpleProfileSystemComponent::Activate()
    {
        m_name = { 
            "Khronos Simple", // Khronos Simple Interaction Profile
            "/interaction_profiles/khr/simple_controller"
        };

        m_userPaths = {
            {LeftHand, "/user/hand/left"},
            {RightHand, "/user/hand/right"}
        };

        const AZStd::vector<OpenXRComponentPath> commonPaths = {
                {{"Select Button", "/input/select/click"}, XR_ACTION_TYPE_BOOLEAN_INPUT},
                {{"Menu Button",   "/input/menu/click"},   XR_ACTION_TYPE_BOOLEAN_INPUT},
                {{"Grip Pose",     "/input/grip/pose"},    XR_ACTION_TYPE_POSE_INPUT},
                {{"Aim Pose",      "/input/aim/pose"},     XR_ACTION_TYPE_POSE_INPUT},
                {{"Vibration",     "/output/haptic"},      XR_ACTION_TYPE_VIBRATION_OUTPUT},
        };
        m_componentPaths[LeftHand] = commonPaths;
        m_componentPaths[RightHand] = commonPaths;

        OpenXRInteractionProfileBus::Handler::BusConnect(m_name.m_displayName);
    }

    void KHRSimpleProfileSystemComponent::Deactivate()
    {
        OpenXRInteractionProfileBus::Handler::BusDisconnect();
    }

    ///////////////////////////////////////////////////////////////////
    // OpenXRInteractionProfileBus::Handler overrides
    //! Create OpenXRVk::Instance object
    AZStd::string KHRSimpleProfileSystemComponent::GetName() const
    {
        return m_name.m_displayName;
    }

    AZStd::vector<AZStd::string> KHRSimpleProfileSystemComponent::GetUserPaths() const
    {
        AZStd::vector<AZStd::string> retList;
        retList.reserve(m_userPaths.size());
        for (const auto& pathTuple : m_userPaths)
        {
            retList.push_back(pathTuple.m_displayName);
        }
        return retList;
    }

    AZStd::string KHRSimpleProfileSystemComponent::GetUserTopPath(const AZStd::string& userPathName) const
    {
        for (const auto& openxrPath : m_userPaths)
        {
            if (openxrPath.m_displayName == userPathName)
            {
                return openxrPath.m_xrRelativePath;
            }
        }
        return AZStd::string("");
    }

    AZStd::vector<AZStd::string> KHRSimpleProfileSystemComponent::GetComponentPaths(const AZStd::string& userPath) const
    {
        if (!m_componentPaths.contains(userPath))
        {
            AZ_Error(LogName, false, "Invalid user path [%s].\n", userPath.c_str());
            return {};
        }

        const auto& paths = m_componentPaths.at(userPath);
        AZStd::vector<AZStd::string> retList;
        retList.reserve(paths.size());
        for (const auto& pathTuple : paths)
        {
            retList.push_back(pathTuple.m_displayName);
        }
        return retList;
    }

    OpenXRInteractionProfile::ActionPathInfo KHRSimpleProfileSystemComponent::GetActionPathInfo(const AZStd::string& userPath, const AZStd::string& componentPath) const
    {
        OpenXRInteractionProfile::ActionPathInfo retPathInfo;

        const auto* openxrUserPath = AZStd::find_if(m_userPaths.begin(), m_userPaths.end(),
            [userPath](const OpenXRPath& entry) {
                return entry.m_displayName == userPath;
            });

        if (openxrUserPath == m_userPaths.end())
        {
            AZ_Error(LogName, false, "Invalid user path [%s].\n", userPath.c_str());
            return retPathInfo;
        }

        const auto& componentPaths = m_componentPaths.at(userPath);
        const auto* openxrComponentPath = AZStd::find_if(componentPaths.begin(), componentPaths.end(),
            [componentPath](const OpenXRComponentPath& entry) {
                return entry.m_displayName == componentPath;
            });

        if (openxrComponentPath == componentPaths.end())
        {
            AZ_Error(LogName, false, "Invalid component path [%s] for user path [%s].\n", componentPath.c_str(), userPath.c_str());
            return retPathInfo;
        }

        retPathInfo.m_actionType = openxrComponentPath->m_actionType;
        retPathInfo.m_absolutePath = openxrUserPath->m_xrRelativePath + openxrComponentPath->m_xrRelativePath;

        return retPathInfo;
    }

    AZStd::string KHRSimpleProfileSystemComponent::GetInteractionProviderPath() const
    {
        return m_name.m_xrRelativePath;
    }
    ///////////////////////////////////////////////////////////////////
}