/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */


#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Asset/AssetManagerBus.h>

#include "OpenXRInteractionProfilesProviderSystemComponent.h"

namespace OpenXRVk
{
    void OpenXRInteractionProfilesProviderSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<OpenXRInteractionProfilesProviderSystemComponent, AZ::Component>()
                ->Version(1);
        }
    }

    void OpenXRInteractionProfilesProviderSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("InteractionProfileProviderService"));
    }

    void OpenXRInteractionProfilesProviderSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("InteractionProfileProviderService"));
    }

    void OpenXRInteractionProfilesProviderSystemComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        (void)required;
    }

    void OpenXRInteractionProfilesProviderSystemComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        dependent.push_back(AZ_CRC("AssetCatalogService"));
    }

    void OpenXRInteractionProfilesProviderSystemComponent::Activate()
    {
        // m_name = { 
        //     "Khronos Simple", // Khronos Simple Interaction Profile
        //     "/interaction_profiles/khr/simple_controller"
        // };
        // 
        // m_userPaths = {
        //     {LeftHand, "/user/hand/left"},
        //     {RightHand, "/user/hand/right"}
        // };
        // 
        // const AZStd::vector<OpenXRComponentPath> commonPaths = {
        //         {{"Select Button", "/input/select/click"}, XR_ACTION_TYPE_BOOLEAN_INPUT},
        //         {{"Menu Button",   "/input/menu/click"},   XR_ACTION_TYPE_BOOLEAN_INPUT},
        //         {{"Grip Pose",     "/input/grip/pose"},    XR_ACTION_TYPE_POSE_INPUT},
        //         {{"Aim Pose",      "/input/aim/pose"},     XR_ACTION_TYPE_POSE_INPUT},
        //         {{"Vibration",     "/output/haptic"},      XR_ACTION_TYPE_VIBRATION_OUTPUT},
        // };

        // m_name = {
        //     "Oculus Touch",
        //     "/interaction_profiles/oculus/touch_controller"
        // };
        // 
        // m_userPaths = {
        //     {LeftHand, "/user/hand/left"},
        //     {RightHand, "/user/hand/right"}
        // };
        // 
        // const AZStd::vector<OpenXRComponentPath> commonPaths = {
        //         {{"X Click",   "/input/x/click"},   XR_ACTION_TYPE_BOOLEAN_INPUT},
        //         {{"X Touch",   "/input/x/touch"},   XR_ACTION_TYPE_BOOLEAN_INPUT},
        //         {{"Select Button", "/input/select/click"}, XR_ACTION_TYPE_BOOLEAN_INPUT}, // Just Testing. NOT supported in oculus
        //         {{"Menu Button",   "/input/menu/click"},   XR_ACTION_TYPE_BOOLEAN_INPUT},
        //         {{"Grip Pose",     "/input/grip/pose"},    XR_ACTION_TYPE_POSE_INPUT},
        //         {{"Aim Pose",      "/input/aim/pose"},     XR_ACTION_TYPE_POSE_INPUT},
        //         {{"Vibration",     "/output/haptic"},      XR_ACTION_TYPE_VIBRATION_OUTPUT},
        // };
        // m_componentPaths[LeftHand] = commonPaths;
        // m_componentPaths[RightHand] = commonPaths;

        AzFramework::AssetCatalogEventBus::Handler::BusConnect();
    }

    void OpenXRInteractionProfilesProviderSystemComponent::Deactivate()
    {
        if (AzFramework::AssetCatalogEventBus::Handler::BusIsConnected())
        {
            AzFramework::AssetCatalogEventBus::Handler::BusDisconnect();
        }

        if (AZ::Data::AssetBus::Handler::BusIsConnected())
        {
            AZ::Data::AssetBus::Handler::BusDisconnect();
        }
    }

    AZStd::string OpenXRInteractionProfilesProviderSystemComponent::GetDefaultInteractionProfilesAssetPath()
    {
        // TODO: Make this user configurable with registry property.
        return "system.xrprofiles";
    }

    //////////////////////////////////////////////////////////////////////////
    // AssetCatalogEventBus
    void OpenXRInteractionProfilesProviderSystemComponent::OnCatalogLoaded(const char* /*catalogFile*/)
    {
        // No need to staying connected anymore.
        AzFramework::AssetCatalogEventBus::Handler::BusDisconnect();

        // Load, asychronously, the asset that contains the list of supported
        // OpenXR Interaction Profiles.
        constexpr bool AutoGenerateId = false;
        AZ::Data::AssetId assetId;
        const auto assetPath = GetDefaultInteractionProfilesAssetPath();
        AZ::Data::AssetCatalogRequestBus::BroadcastResult(
            assetId, &AZ::Data::AssetCatalogRequestBus::Events::GetAssetIdByPath,
            assetPath.c_str(), azrtti_typeid<OpenXRInteractionProfilesAsset>(), AutoGenerateId);
        if (!assetId.IsValid())
        {
            AZ_Assert(false, "Failed to find the interaction profiles asset with path [%s]", assetPath.c_str());
            AZ_Error(LogName, false, "Failed to find the interaction profiles asset with path [%s]", assetPath.c_str());
            return;
        }

        AZ::Data::AssetBus::Handler::BusConnect(assetId);

        m_interactionProfilesAsset = AZ::Data::AssetManager::Instance().GetAsset<OpenXRInteractionProfilesAsset>(assetId, AZ::Data::AssetLoadBehavior::QueueLoad);

    }
    //////////////////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////////////////
    // AZ::Data::AssetBus::Handler overrides
    void OpenXRInteractionProfilesProviderSystemComponent::OnAssetReady(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        AZ::Data::AssetBus::Handler::BusDisconnect();
        m_interactionProfilesAsset = asset;
    }

    void OpenXRInteractionProfilesProviderSystemComponent::OnAssetError(AZ::Data::Asset<AZ::Data::AssetData> asset)
    {
        AZ::Data::AssetBus::Handler::BusDisconnect();
        m_interactionProfilesAsset = {};
        AZ_Assert(false, "Failed to load interaction profiles asset");
        AZ_Error(LogName, false, "Failed to load interaction profiles asset");

    }
    //////////////////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////////////
    // OpenXRInteractionProfileBus::Handler overrides
    const AZStd::vector<AZStd::string>& OpenXRInteractionProfilesProviderSystemComponent::GetInteractionProfileNames() const
    {
        static AZStd::vector<AZStd::string> profileNames;

        if (!m_interactionProfilesAsset.IsReady())
        {
            AZ_Error(LogName, false, "Interaction Profiles Asset Doesn't exist or it is not ready yet.");
            return profileNames;
        }

        if (profileNames.empty())
        {
            for (const auto& profileDescriptor : m_interactionProfilesAsset->m_interactionProfileDescriptors)
            {
                profileNames.push_back(profileDescriptor.m_uniqueName);
            }
        }

        return profileNames;
    }

    const OpenXRInteractionProfileDescriptor* OpenXRInteractionProfilesProviderSystemComponent::GetInteractionProfileDescriptor(const AZStd::string profileName) const
    {
        if (!m_interactionProfilesAsset.IsReady())
        {
            AZ_Error(LogName, false, "Interaction Profiles Asset Doesn't exist or it is not ready yet.");
            return nullptr;
        }

        for (const auto& profileDescriptor : m_interactionProfilesAsset->m_interactionProfileDescriptors)
        {
            if (profileName == profileDescriptor.m_uniqueName)
            {
                return &profileDescriptor;
            }
        }

        AZ_Error(LogName, false, "Interaction Profile with name [%s] doesn't exist.", profileName.c_str());
        return nullptr;
    }

    // AZStd::string KHRSimpleProfileSystemComponent::GetName() const
    // {
    //     return m_name.m_displayName;
    // }
    // 
    // AZStd::vector<AZStd::string> KHRSimpleProfileSystemComponent::GetUserPaths() const
    // {
    //     AZStd::vector<AZStd::string> retList;
    //     retList.reserve(m_userPaths.size());
    //     for (const auto& pathTuple : m_userPaths)
    //     {
    //         retList.push_back(pathTuple.m_displayName);
    //     }
    //     return retList;
    // }
    // 
    // AZStd::string KHRSimpleProfileSystemComponent::GetUserTopPath(const AZStd::string& userPathName) const
    // {
    //     for (const auto& openxrPath : m_userPaths)
    //     {
    //         if (openxrPath.m_displayName == userPathName)
    //         {
    //             return openxrPath.m_xrRelativePath;
    //         }
    //     }
    //     return AZStd::string("");
    // }
    // 
    // AZStd::vector<AZStd::string> KHRSimpleProfileSystemComponent::GetComponentPaths(const AZStd::string& userPath) const
    // {
    //     if (!m_componentPaths.contains(userPath))
    //     {
    //         AZ_Error(LogName, false, "Invalid user path [%s].\n", userPath.c_str());
    //         return {};
    //     }
    // 
    //     const auto& paths = m_componentPaths.at(userPath);
    //     AZStd::vector<AZStd::string> retList;
    //     retList.reserve(paths.size());
    //     for (const auto& pathTuple : paths)
    //     {
    //         retList.push_back(pathTuple.m_displayName);
    //     }
    //     return retList;
    // }
    // 
    // OpenXRInteractionProfile::ActionPathInfo KHRSimpleProfileSystemComponent::GetActionPathInfo(const AZStd::string& userPath, const AZStd::string& componentPath) const
    // {
    //     OpenXRInteractionProfile::ActionPathInfo retPathInfo;
    // 
    //     const auto* openxrUserPath = AZStd::find_if(m_userPaths.begin(), m_userPaths.end(),
    //         [userPath](const OpenXRPath& entry) {
    //             return entry.m_displayName == userPath;
    //         });
    // 
    //     if (openxrUserPath == m_userPaths.end())
    //     {
    //         AZ_Error(LogName, false, "Invalid user path [%s].\n", userPath.c_str());
    //         return retPathInfo;
    //     }
    // 
    //     const auto& componentPaths = m_componentPaths.at(userPath);
    //     const auto* openxrComponentPath = AZStd::find_if(componentPaths.begin(), componentPaths.end(),
    //         [componentPath](const OpenXRComponentPath& entry) {
    //             return entry.m_displayName == componentPath;
    //         });
    // 
    //     if (openxrComponentPath == componentPaths.end())
    //     {
    //         AZ_Error(LogName, false, "Invalid component path [%s] for user path [%s].\n", componentPath.c_str(), userPath.c_str());
    //         return retPathInfo;
    //     }
    // 
    //     retPathInfo.m_actionType = openxrComponentPath->m_actionType;
    //     retPathInfo.m_absolutePath = openxrUserPath->m_xrRelativePath + openxrComponentPath->m_xrRelativePath;
    // 
    //     return retPathInfo;
    // }
    // 
    // AZStd::string KHRSimpleProfileSystemComponent::GetInteractionProviderPath() const
    // {
    //     return m_name.m_xrRelativePath;
    // }
    ///////////////////////////////////////////////////////////////////
} //namespace OpenXRVk