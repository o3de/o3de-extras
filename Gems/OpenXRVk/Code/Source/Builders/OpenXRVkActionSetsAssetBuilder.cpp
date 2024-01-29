/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/StringFunc/StringFunc.h>
#include <AssetBuilderSDK/SerializationDependencies.h>

#include <OpenXRVk/OpenXRVkInteractionProfilesAsset.h>
#include <OpenXRVk/OpenXRVkActionSetsAsset.h>

#include "OpenXRVkActionSetsAssetBuilder.h"

#pragma optimize( "", off ) // GALIB

namespace OpenXRVkBuilders
{
    template<class AssetType>
    static AZStd::unique_ptr<AssetType> LoadAssetAsUniquePtr(const AZStd::string& filePath)
    {
        AZ::ObjectStream::FilterDescriptor loadFilter = AZ::ObjectStream::FilterDescriptor(&AZ::Data::AssetFilterNoAssetLoading, AZ::ObjectStream::FILTERFLAG_IGNORE_UNKNOWN_CLASSES);
        auto actionSetsAssetPtr = AZ::Utils::LoadObjectFromFile<AssetType>(filePath, nullptr, loadFilter);
        if (!actionSetsAssetPtr)
        {
            return nullptr;
        }
        return AZStd::unique_ptr<AssetType>(actionSetsAssetPtr);
    }

    // A regular Name is just a utf-8 string, it can contain upper case letters
    // and spaces in between. But it can not be empty, and can not contain leading to trailing spaces.
    static AZ::Outcome<void, AZStd::string> ValidateName(const AZStd::string& name)
    {
        if (name.empty())
        {
            return AZ::Failure("Name should not be empty.");
        }
        //Spaces at the beginning and end of the name are not allowed.
        AZStd::string tmpName(name);
        AZ::StringFunc::TrimWhiteSpace(tmpName, true, true);
        if (tmpName.empty())
        {
            return AZ::Failure("Name is just a bunch of spaces.");
        }
        if (tmpName.size() != name.size())
        {
            return AZ::Failure(
                AZStd::string::format("Trailing or leading spaces are not allowed in a Name [%s].",
                    name.c_str())
            );
        }
        return AZ::Success();
    }


    // Unlike an OpenXR Name, a Localized Name is just a utf-8 string, it can contain upper case letters
    // and spaces in between. But it can not be empty, and can not contain leading to trailing spaces.
    static AZ::Outcome<void, AZStd::string> ValidateOpenXRLocalizedName(const AZStd::string& name)
    {
        auto outcome = ValidateName(name);
        if (!outcome.IsSuccess())
        {
            return AZ::Failure(
                AZStd::string::format("Localized Name [%s] is invalid. Reason:\n%s", name.c_str(), outcome.GetError().c_str())
            );
        }
        return AZ::Success();
    }

    
    void OpenXRActionSetsAssetBuilder::CreateJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        //! First get the extension 
        constexpr bool includeDot = false;
        AZStd::string fileExtension;
        bool result = AZ::StringFunc::Path::GetExtension(request.m_sourceFile.c_str(), fileExtension, includeDot);
        if (result && (fileExtension == OpenXRVk::OpenXRInteractionProfilesAsset::s_assetExtension))
        {
            CreateInteractionProfilesAssetJobs(request, response);
            return;
        }

        if (result && (fileExtension == OpenXRVk::OpenXRActionSetsAsset::s_assetExtension))
        {
            CreateActionSetsAssetJobs(request, response);
            return;
        }

        //! Unknown extension.
        AZ_Error(LogName, false, "Unknown file extension [%s] for this builder. Source file [%s]", fileExtension.c_str(), request.m_sourceFile.c_str());
        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Failed;
    }
    
    
    void OpenXRActionSetsAssetBuilder::ProcessJob(const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const
    {
        //! First get the extension 
        constexpr bool includeDot = false;
        AZStd::string fileExtension;
        bool result = AZ::StringFunc::Path::GetExtension(request.m_sourceFile.c_str(), fileExtension, includeDot);
        if (result && (fileExtension == OpenXRVk::OpenXRInteractionProfilesAsset::s_assetExtension))
        {
            ProcessInteractionProfilesAssetJob(request, response);
            return;
        }
        if (result && (fileExtension == OpenXRVk::OpenXRActionSetsAsset::s_assetExtension))
        {
            ProcessActionSetsAssetJob(request, response);
        }
    }


    /////////////////////////////////////////////////////////////////////////////////
    // OpenXRInteractionProfilesAsset Support Begin
    void OpenXRActionSetsAssetBuilder::CreateInteractionProfilesAssetJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            // Very high priority because this asset is required to initialize the OpenXR runtime
            // and initialize the I/O actions system.
            jobDescriptor.m_priority = 1000;
            jobDescriptor.m_critical = true;
            jobDescriptor.m_jobKey = InteractionProfilesAssetJobKey;
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());
            response.m_createJobOutputs.emplace_back(AZStd::move(jobDescriptor));
        } // for all request.m_enabledPlatforms

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    static AZ::Outcome<void, AZStd::string> ValidateActionTypeString(const AZStd::string& actionTypeStr)
    {
        using CPD = OpenXRVk::OpenXRInteractionComponentPathDescriptor;
        static const AZStd::unordered_set<AZStd::string> ValidActionTypes{
            {CPD::s_TypeBoolStr},
            {CPD::s_TypeFloatStr},
            {CPD::s_TypeVector2Str},
            {CPD::s_TypePoseStr},
            {CPD::s_TypeVibrationStr}
        };

        if (!ValidActionTypes.contains(actionTypeStr))
        {
            static AZStd::string ValidListStr;
            if (ValidListStr.empty())
            {
                ValidListStr += "[ ";
                for (const auto& validActionTypeStr : ValidActionTypes)
                {
                    if (!ValidListStr.empty())
                    {
                        ValidListStr += ", ";
                    }
                    ValidListStr += validActionTypeStr;
                }
                ValidListStr += " ]";
            }
            return AZ::Failure(
                AZStd::string::format("Action Type [%s] is invalid. It can only be one of %s",
                    actionTypeStr.c_str(), ValidListStr.c_str())
            );
        }

        return AZ::Success();
    }

    // An OpenXR path string only contain characters as described here
    // https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#well-formed-path-strings
    static AZ::Outcome<void, AZStd::string> ValidateOpenXRPath(const AZStd::string& path)
    {
        static AZStd::regex s_validCharactersRegEx(R"(^(/[a-z0-9\-_\.]+)+$)", AZStd::regex::ECMAScript);
        if (!AZStd::regex_match(path, s_validCharactersRegEx))
        {
            return AZ::Failure(
                AZStd::string::format("The path [%s] contains an invalid character, or is missing a leading '/' or contains a leading '/'", path.c_str())
            );
        }
        return AZ::Success();
    }

    static AZ::Outcome<void, AZStd::string> ValidateComponentPathDescriptor(const OpenXRVk::OpenXRInteractionComponentPathDescriptor& componentPathDescriptor,
        AZStd::unordered_set<AZStd::string>& uniqueComponentPathNames, AZStd::unordered_set<AZStd::string>& uniqueComponentPathPaths)
    {
        {
            if (uniqueComponentPathNames.contains(componentPathDescriptor.m_name))
            {
                return AZ::Failure(
                    AZStd::string::format("A Component Path with name [%s] already exists.",
                        componentPathDescriptor.m_name.c_str())
                );
            }
            uniqueComponentPathNames.emplace(componentPathDescriptor.m_name);
            auto outcome = ValidateName(componentPathDescriptor.m_name);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Component Name[% s] is invalid.Reason:\n % s", componentPathDescriptor.m_name.c_str(), outcome.GetError().c_str())
                );
            }
        }
        {
            if (uniqueComponentPathPaths.contains(componentPathDescriptor.m_path))
            {
                return AZ::Failure(
                    AZStd::string::format("A Component Path with path [%s] already exists.",
                        componentPathDescriptor.m_path.c_str())
                );
            }
            uniqueComponentPathPaths.emplace(componentPathDescriptor.m_path);
            auto outcome = ValidateOpenXRPath(componentPathDescriptor.m_path);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Component Path path [%s] is invalid. Reason:\n%s", componentPathDescriptor.m_path.c_str(), outcome.GetError().c_str())
                );
            }
        }

        auto outcome = ValidateActionTypeString(componentPathDescriptor.m_actionTypeStr);
        if (!outcome.IsSuccess())
        {
            return AZ::Failure(
                AZStd::string::format("Component Path path [%s] has an invalid action type. Reason:\n%s", componentPathDescriptor.m_name.c_str(), outcome.GetError().c_str())
            );
        }

        return AZ::Success();
    }


    static AZ::Outcome<void, AZStd::string> ValidateUserPathDescriptor(const OpenXRVk::OpenXRInteractionUserPathDescriptor& userPathDescriptor,
        AZStd::unordered_set<AZStd::string>& uniqueUserPathNames, AZStd::unordered_set<AZStd::string>& uniqueUserPathPaths,
        AZStd::unordered_set<AZStd::string>& uniqueComponentPathNames, AZStd::unordered_set<AZStd::string>& uniqueComponentPathPaths)
    {
        {
            if (uniqueUserPathNames.contains(userPathDescriptor.m_name))
            {
                return AZ::Failure(
                    AZStd::string::format("An User Path with name [%s] already exists.",
                        userPathDescriptor.m_name.c_str())
                );
            }
            uniqueUserPathNames.emplace(userPathDescriptor.m_name);
            auto outcome = ValidateName(userPathDescriptor.m_name);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("User Path Name [%s] is invalid. Reason:\n%s", userPathDescriptor.m_name.c_str(), outcome.GetError().c_str())
                );
            }
        }
        {
            if (uniqueUserPathPaths.contains(userPathDescriptor.m_path))
            {
                return AZ::Failure(
                    AZStd::string::format("An User Path with path [%s] already exists.",
                        userPathDescriptor.m_path.c_str())
                );
            }
            uniqueUserPathPaths.emplace(userPathDescriptor.m_path);
            auto outcome = ValidateOpenXRPath(userPathDescriptor.m_path);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("User Path path [%s] is invalid. Reason:\n%s", userPathDescriptor.m_path.c_str(), outcome.GetError().c_str())
                );
            }
        }
        for (const auto& componentPathDescriptor : userPathDescriptor.m_componentPathDescriptors)
        {
            auto outcome = ValidateComponentPathDescriptor(componentPathDescriptor,
                uniqueComponentPathNames, uniqueComponentPathPaths);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Invalid Component Path [%s]. Reason:\n%s", componentPathDescriptor.m_name.c_str(), outcome.GetError().c_str())
                );
            }
        }

        return AZ::Success();
    }


    static AZ::Outcome<void, AZStd::string> ValidateInteractionProfileDescriptor(
        const OpenXRVk::OpenXRInteractionProfileDescriptor& interactionProfileDescriptor,
        AZStd::unordered_set<AZStd::string>& uniqueNames, AZStd::unordered_set<AZStd::string>& uniquePaths)
    {
        {
            if (uniqueNames.contains(interactionProfileDescriptor.m_uniqueName))
            {
                return AZ::Failure(
                    AZStd::string::format("An Interaction Profile with name [%s] already exists.",
                        interactionProfileDescriptor.m_uniqueName.c_str())
                );
            }
            uniqueNames.emplace(interactionProfileDescriptor.m_uniqueName);
            auto outcome = ValidateName(interactionProfileDescriptor.m_uniqueName);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Interaction Profile Unique Name [%s] is invalid. Reason:\n%s", interactionProfileDescriptor.m_uniqueName.c_str(), outcome.GetError().c_str())
                );
            }
        }

        {
            if (uniquePaths.contains(interactionProfileDescriptor.m_path))
            {
                return AZ::Failure(
                    AZStd::string::format("An Interaction Profile with path [%s] already exists.",
                        interactionProfileDescriptor.m_path.c_str())
                );
            }
            uniquePaths.emplace(interactionProfileDescriptor.m_path);
            auto outcome = ValidateOpenXRPath(interactionProfileDescriptor.m_path);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Interaction Profile Path [%s] is invalid. Reason:\n%s", interactionProfileDescriptor.m_path.c_str(), outcome.GetError().c_str())
                );
            }
        }

        AZStd::unordered_set<AZStd::string> uniqueUserPathNames;
        AZStd::unordered_set<AZStd::string> uniqueUserPathPaths;
        AZStd::unordered_set<AZStd::string> uniqueComponentPathNames;
        AZStd::unordered_set<AZStd::string> uniqueComponentPathPaths;
        for (const auto& userPathDescriptor : interactionProfileDescriptor.m_userPathDescriptors)
        {
            auto outcome = ValidateUserPathDescriptor(userPathDescriptor,
                uniqueUserPathNames, uniqueUserPathPaths, uniqueComponentPathNames, uniqueComponentPathPaths);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Invalid User Path [%s]. Reason:\n%s", userPathDescriptor.m_name.c_str(), outcome.GetError().c_str())
                );
            }
        }

        for (const auto& componentPathDescriptor : interactionProfileDescriptor.m_commonComponentPathDescriptors)
        {
            auto outcome = ValidateComponentPathDescriptor(componentPathDescriptor,
                uniqueComponentPathNames, uniqueComponentPathPaths);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Invalid Common Component Path [%s]. Reason:\n%s", componentPathDescriptor.m_name.c_str(), outcome.GetError().c_str())
                );
            }
        }

        return AZ::Success();
    }


    static AZ::Outcome<void, AZStd::string> ValidateInteractionProfilesAsset(
        const OpenXRVk::OpenXRInteractionProfilesAsset& interactionProfilesAsset)
    {
        if (interactionProfilesAsset.m_interactionProfileDescriptors.empty())
        {
            return AZ::Failure("An InteractionProfiles asset requires at least one Interaction Profile");
        }

        AZStd::unordered_set<AZStd::string> uniqueNames;
        AZStd::unordered_set<AZStd::string> uniquePaths;
        uint32_t i = 0;
        for (const auto& interactionProfileDescriptor : interactionProfilesAsset.m_interactionProfileDescriptors)
        {
            auto outcome = ValidateInteractionProfileDescriptor(interactionProfileDescriptor,
                uniqueNames, uniquePaths);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("InteractionProfile[%u] is invalid. Reason:\n%s",
                        i, outcome.GetError().c_str())
                );
            }
            i++;
        }
        return AZ::Success();
    }

    void OpenXRActionSetsAssetBuilder::ProcessInteractionProfilesAssetJob([[maybe_unused]] const AssetBuilderSDK::ProcessJobRequest& request, [[maybe_unused]] AssetBuilderSDK::ProcessJobResponse& response) const
    {
        // Open the file, and make sure there's no redundant data, the OpenXR Paths are well formatted, etc.
       auto interactionProfilesAssetPtr = LoadAssetAsUniquePtr<OpenXRVk::OpenXRInteractionProfilesAsset>(request.m_fullPath);
       if (!interactionProfilesAssetPtr)
       {
           AZ_Error(LogName, false, "Failed to load interaction profile source asset [%s]", request.m_fullPath.c_str());
           response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
           return;
       }

       auto outcome = ValidateInteractionProfilesAsset(*interactionProfilesAssetPtr.get());
       if (!outcome.IsSuccess())
       {
           AZ_Error(LogName, false, "Invalid InteractionProfilesAsset [%s]. Reason:\n%s",
               request.m_fullPath.c_str(), outcome.GetError().c_str());
           response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
           return;
       }

       // We keep exact same asset name and extension.
       AZStd::string assetFileName;
       AZ::StringFunc::Path::GetFullFileName(request.m_fullPath.c_str(), assetFileName);

       // Construct product full path
       AZStd::string assetOutputPath;
       AzFramework::StringFunc::Path::ConstructFull(request.m_tempDirPath.c_str(), assetFileName.c_str(), assetOutputPath, true);

       bool result = AZ::Utils::SaveObjectToFile(assetOutputPath, AZ::DataStream::ST_XML, interactionProfilesAssetPtr.get());
       if (result == false)
       {
           AZ_Error(LogName, false, "Failed to save asset to %s", assetOutputPath.c_str());
           response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
           return;
       }

       AssetBuilderSDK::JobProduct jobProduct;
       if (!AssetBuilderSDK::OutputObject(interactionProfilesAssetPtr.get(), assetOutputPath,
           azrtti_typeid<OpenXRVk::OpenXRInteractionProfilesAsset>(),
           aznumeric_cast<uint32_t>(0), jobProduct))
       {
           AZ_Error(LogName, false, "FIXME this message.");
           response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
           return;
       }
       response.m_outputProducts.emplace_back(AZStd::move(jobProduct));
       response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
    }
    // OpenXRInteractionProfilesAsset Support End
    /////////////////////////////////////////////////////////////////////////////////////


    static AZStd::string GetInteractionProfileAssetSourcePath(const OpenXRVk::OpenXRActionSetsAsset& actionSetsAsset)
    {
        const auto& sourceUuid = actionSetsAsset.m_interactionProfilesAsset.GetId().m_guid;
        bool foundSource = false;
        AZ::Data::AssetInfo sourceAssetInfo;
        AZStd::string sourceWatchFolder;
        AzToolsFramework::AssetSystemRequestBus::BroadcastResult(foundSource, &AzToolsFramework::AssetSystemRequestBus::Events::GetSourceInfoBySourceUUID,
            sourceUuid, sourceAssetInfo, sourceWatchFolder);
        AZStd::string sourcePath;
        if (foundSource)
        {
            constexpr bool caseInsensitive = false;
            constexpr bool normalize = true;
            AZ::StringFunc::Path::Join(sourceWatchFolder.c_str(), sourceAssetInfo.m_relativePath.c_str(), sourcePath, caseInsensitive, normalize);
        }
        return sourcePath;
    }

    void OpenXRActionSetsAssetBuilder::CreateActionSetsAssetJobs(const AssetBuilderSDK::CreateJobsRequest& request, AssetBuilderSDK::CreateJobsResponse& response) const
    {
        // Make sure the InteractionProfiles asset referenced in this ActionSets asset exists. and if so,
        // also declare job dependency.
        auto actionSetsAssetPtr = LoadAssetAsUniquePtr<OpenXRVk::OpenXRActionSetsAsset>(request.m_sourceFile);
        if (!actionSetsAssetPtr)
        {
            AZ_Error(LogName, false, "Failed to load the ActionSets asset at path[%s].", request.m_sourceFile.c_str());
            response.m_result = AssetBuilderSDK::CreateJobsResultCode::Failed;
            return;
        }

        auto interactionProfileSourcePath = GetInteractionProfileAssetSourcePath(*actionSetsAssetPtr.get());
        if (interactionProfileSourcePath.empty())
        {
            AZ_Error(LogName, false, "An ActionSets source asset requires a valid InteractionProfiles source asset.");
            response.m_result = AssetBuilderSDK::CreateJobsResultCode::Failed;
            return;
        }
        
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            if (platformInfo.m_identifier != "pc") //FIXME: GALIB REMOVE ME.
            {
                continue;
            }
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            // Very high priority because this asset is required to initialize the OpenXR runtime
            // and initialize the I/O actions system.
            jobDescriptor.m_priority = 999;
            jobDescriptor.m_critical = true;
            jobDescriptor.m_jobKey = ActionSetsAssetJobKey;
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());

            AssetBuilderSDK::SourceFileDependency sourceFileDependency{};
            sourceFileDependency.m_sourceFileDependencyPath = interactionProfileSourcePath;
            auto jobDependency = AssetBuilderSDK::JobDependency(InteractionProfilesAssetJobKey, platformInfo.m_identifier,
                AssetBuilderSDK::JobDependencyType::Order, sourceFileDependency);
            jobDescriptor.m_jobDependencyList.emplace_back(AZStd::move(jobDependency));

            response.m_createJobOutputs.emplace_back(AZStd::move(jobDescriptor));
        } // for all request.m_enabledPlatforms

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }


    // An OpenXR name string only contain characters which are allowed in a SINGLE LEVEL of a well-formed path string
    // https://registry.khronos.org/OpenXR/specs/1.0/html/xrspec.html#well-formed-path-strings
    static AZ::Outcome<void, AZStd::string> ValidateOpenXRName(const AZStd::string& name)
    {
        static AZStd::regex s_validCharactersRegEx (R"(^[a-z0-9\-_\.]+$)", AZStd::regex::ECMAScript);
        if (!AZStd::regex_match(name, s_validCharactersRegEx))
        {
            return AZ::Failure(
                AZStd::string::format("The name [%s] contains an invalid character", name.c_str())
            );
        }
        return AZ::Success();
    }

    
    static AZ::Outcome<void, AZStd::string> ValidateActionPathDescriptor(const OpenXRVk::OpenXRActionPathDescriptor& actionPathDescriptor,
        const OpenXRVk::OpenXRInteractionProfilesAsset& interactionProfilesAsset,
        AZStd::unordered_set<AZStd::string>& uniqueActionPaths)
    {
        auto concatenatedActionPath = actionPathDescriptor.m_interactionProfileName
            + actionPathDescriptor.m_userPathName
            + actionPathDescriptor.m_componentPathName;
        if (uniqueActionPaths.contains(concatenatedActionPath))
        {
            return AZ::Failure(
                AZStd::string::format("An Action Path with profile[%s], userPath[%s], componentPath[%s] already exists.",
                    actionPathDescriptor.m_interactionProfileName.c_str(),
                    actionPathDescriptor.m_userPathName.c_str(),
                    actionPathDescriptor.m_componentPathName.c_str())
            );
        }
        uniqueActionPaths.emplace(AZStd::move(concatenatedActionPath));

        if (actionPathDescriptor.m_interactionProfileName.empty())
        {
            return AZ::Failure(
                AZStd::string::format("ActionPath Descriptor must have an InteractionProfile name.")
            );
        }
        const auto interactionProfileDescriptorPtr = interactionProfilesAsset.GetInteractionProfileDescriptor(actionPathDescriptor.m_interactionProfileName);
        if (!interactionProfileDescriptorPtr)
        {
            return AZ::Failure(
                AZStd::string::format("Unknown Interaction Profile Descriptor named [%s].",
                    actionPathDescriptor.m_interactionProfileName.c_str())
            );
        }

        if (actionPathDescriptor.m_userPathName.empty())
        {
            return AZ::Failure(
                AZStd::string::format("ActionPath Descriptor must have an UserPath name.")
            );
        }
        const auto userPathDescriptorPtr = interactionProfileDescriptorPtr->GetUserPathDescriptor(actionPathDescriptor.m_userPathName);
        if (!userPathDescriptorPtr)
        {
            return AZ::Failure(
                AZStd::string::format("Unknown UserPath descriptor named [%s].",
                    actionPathDescriptor.m_userPathName.c_str())
            );
        }

        if (actionPathDescriptor.m_componentPathName.empty())
        {
            return AZ::Failure(
                AZStd::string::format("ActionPath Descriptor must have a ComponentPath name.")
            );
        }
        const auto componentPathDescriptorPtr = interactionProfileDescriptorPtr->GetComponentPathDescriptor(*userPathDescriptorPtr, actionPathDescriptor.m_componentPathName);
        if (!componentPathDescriptorPtr)
        {
            return AZ::Failure(
                AZStd::string::format("Unknown ComponentPath descriptor named [%s].",
                    actionPathDescriptor.m_componentPathName.c_str())
            );
        }

        return AZ::Success();
    }


    static const AZStd::string& GetActionTypeStringFromActionPathDescriptor(
        const OpenXRVk::OpenXRInteractionProfilesAsset& interactionProfilesAsset,
        const OpenXRVk::OpenXRActionPathDescriptor& actionPathDescriptor
        )
    {
        static const AZStd::string emptyStr;

        const auto interactionProfileDescriptorPtr = interactionProfilesAsset.GetInteractionProfileDescriptor(actionPathDescriptor.m_interactionProfileName);
        if (!interactionProfileDescriptorPtr)
        {
            return emptyStr;
        }

        const auto userPathDescriptorPtr = interactionProfileDescriptorPtr->GetUserPathDescriptor(actionPathDescriptor.m_userPathName);
        if (!userPathDescriptorPtr)
        {
            return emptyStr;
        }
        const auto componentPathDescriptorPtr = interactionProfileDescriptorPtr->GetComponentPathDescriptor(*userPathDescriptorPtr, actionPathDescriptor.m_componentPathName);
        if (!componentPathDescriptorPtr)
        {
            return emptyStr;
        }
        return componentPathDescriptorPtr->m_actionTypeStr;
    }

    static bool IsActionTypeBoolOrFloat(const AZStd::string& actionTypeStr)
    {
        return (
            (actionTypeStr == OpenXRVk::OpenXRInteractionComponentPathDescriptor::s_TypeBoolStr) ||
            (actionTypeStr == OpenXRVk::OpenXRInteractionComponentPathDescriptor::s_TypeFloatStr)
        );
    }

    static bool AreCompatibleActionTypeStrings(const AZStd::string& lhs, const AZStd::string& rhs)
    {
        if (IsActionTypeBoolOrFloat(lhs) && IsActionTypeBoolOrFloat(rhs))
        {
            return true;
        }
        return (lhs == rhs);
    }


    static AZ::Outcome<void, AZStd::string> ValidateActionDescriptor(
        const OpenXRVk::OpenXRInteractionProfilesAsset& interactionProfilesAsset,
        const OpenXRVk::OpenXRActionDescriptor& actionDescriptor,
        AZStd::unordered_set<AZStd::string>& uniqueActionNames,
        AZStd::unordered_set<AZStd::string>& uniqueActionLocalizedNames)
    {
        {
            if (uniqueActionNames.contains(actionDescriptor.m_name))
            {
                return AZ::Failure(
                    AZStd::string::format("An Action with name [%s] already exists.",
                        actionDescriptor.m_name.c_str())
                );
            }
            uniqueActionNames.emplace(actionDescriptor.m_name);
            auto outcome = ValidateOpenXRName(actionDescriptor.m_name);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Failed to validate Action Descriptor named=[%s].\nReason:\n%s",
                        actionDescriptor.m_name.c_str(), outcome.GetError().c_str())
                );
            }
        }

        {
            if (uniqueActionLocalizedNames.contains(actionDescriptor.m_localizedName))
            {
                return AZ::Failure(
                    AZStd::string::format("An Action with localized name [%s] already exists.",
                        actionDescriptor.m_localizedName.c_str())
                );
            }
            uniqueActionLocalizedNames.emplace(actionDescriptor.m_localizedName);
            auto outcome = ValidateOpenXRLocalizedName(actionDescriptor.m_localizedName);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Failed to validate localized name of Action Descriptor named=[%s]\nReason:\n%s",
                        actionDescriptor.m_name.c_str(), outcome.GetError().c_str())
                );
            }
        }


        if (actionDescriptor.m_actionPathDescriptors.empty())
        {
            return AZ::Failure(
                AZStd::string::format("At least one ActionPath Descriptor is required by Action Descriptor named=[%s].\n",
                    actionDescriptor.m_name.c_str())
            );
        }

        AZStd::unordered_set<AZStd::string> uniqueActionPaths;
        
        // It is very important that all action path descriptors have compatible data types.
        const AZStd::string& firstActionTypeStr = GetActionTypeStringFromActionPathDescriptor(
            interactionProfilesAsset, actionDescriptor.m_actionPathDescriptors[0]);
        uint32_t actionPathIndex = 0;
        for (const auto& actionPathDescriptor : actionDescriptor.m_actionPathDescriptors)
        {
            auto outcome = ValidateActionPathDescriptor(actionPathDescriptor, interactionProfilesAsset, uniqueActionPaths);
            if (!outcome.IsSuccess())
            {
                return AZ::Failure(
                    AZStd::string::format("Failed to validate Action Path Descriptor for Action Descriptor named=[%s].\nReason:\n%s",
                        actionDescriptor.m_name.c_str(), outcome.GetError().c_str())
                );
            }
            const AZStd::string& actionTypeStr = GetActionTypeStringFromActionPathDescriptor(
                interactionProfilesAsset, actionPathDescriptor);
            if (!AreCompatibleActionTypeStrings(firstActionTypeStr, actionTypeStr))
            {
                return AZ::Failure(
                    AZStd::string::format("ActionType=[%s] of ActionPath Descriptor[%u] is NOT compatible with the ActionType=[%s] ActionPath Descriptor[0]",
                        actionTypeStr.c_str(), actionPathIndex, firstActionTypeStr.c_str())
                );
            }
            actionPathIndex++;
        }

        return AZ::Success();
    }


    static AZ::Outcome<void, AZStd::string> ValidateActionSetsAsset(const OpenXRVk::OpenXRActionSetsAsset& actionSetsAsset,
        const OpenXRVk::OpenXRInteractionProfilesAsset& interactionProfilesAsset)
    {
        if (actionSetsAsset.m_actionSetDescriptors.empty())
        {
            return AZ::Failure("At least one ActionSet must be listed in an ActionSets asset");
        }

        AZStd::unordered_set<AZStd::string> uniqueActionSetNames;
        AZStd::unordered_set<AZStd::string> uniqueActionSetLocalizedNames;
        for (const auto& actionSetDescriptor : actionSetsAsset.m_actionSetDescriptors)
        {
            {
                if (uniqueActionSetNames.contains(actionSetDescriptor.m_name))
                {
                    return AZ::Failure(
                        AZStd::string::format("An ActionSet named=[%s] already exists.",
                            actionSetDescriptor.m_name.c_str())
                    );
                }
                uniqueActionSetNames.emplace(actionSetDescriptor.m_name);
                auto outcome = ValidateOpenXRName(actionSetDescriptor.m_name);
                if (!outcome.IsSuccess())
                {
                    return AZ::Failure(
                        AZStd::string::format("Failed to validate ActionSet Descriptor name=[%s]. Reason:\n%s",
                            actionSetDescriptor.m_name.c_str(), outcome.GetError().c_str())
                    );
                }
            }
            
            {
                if (uniqueActionSetLocalizedNames.contains(actionSetDescriptor.m_localizedName))
                {
                    return AZ::Failure(
                        AZStd::string::format("An ActionSet with localized named=[%s] already exists.",
                            actionSetDescriptor.m_localizedName.c_str())
                    );
                }
                uniqueActionSetLocalizedNames.emplace(actionSetDescriptor.m_localizedName);
                auto outcome = ValidateOpenXRLocalizedName(actionSetDescriptor.m_localizedName);
                if (!outcome.IsSuccess())
                {
                    return AZ::Failure(
                        AZStd::string::format("Failed to validate ActionSet Descriptor name=[%s]. Reason:\n%s",
                            actionSetDescriptor.m_name.c_str(), outcome.GetError().c_str())
                    );
                }
            }

            if (actionSetDescriptor.m_actionDescriptors.empty())
            {
                return AZ::Failure(
                    AZStd::string::format("ActionSet [%s] must contain at least one ActionDescriptor.",
                        actionSetDescriptor.m_name.c_str())
                );
            }
            
            AZStd::unordered_set<AZStd::string> uniqueActionNames;
            AZStd::unordered_set<AZStd::string> uniqueActionLocalizedNames;
            for (const auto& actionDescriptor : actionSetDescriptor.m_actionDescriptors)
            {
                auto outcome = ValidateActionDescriptor(interactionProfilesAsset, actionDescriptor,
                    uniqueActionNames, uniqueActionLocalizedNames);
                if (!outcome.IsSuccess())
                {
                    return AZ::Failure(
                        AZStd::string::format("Failed to validate ActionSet Descriptor name=[%s]. Reason:\n%s",
                            actionSetDescriptor.m_name.c_str(), outcome.GetError().c_str())
                    );
                }
            }
        }

        return AZ::Success();
    }

    //! Each action in an actionSet has a "name" and a "localizedName". The "name" can never be empty, but
    //! if "localizedName" is empty we automatically patch it as an identical copy of "name".
    static void FixEmptyLocalizedNames(OpenXRVk::OpenXRActionSetsAsset& actionSetAsset)
    {
        for (auto& actionSetDescriptor : actionSetAsset.m_actionSetDescriptors)
        {
            if (actionSetDescriptor.m_localizedName.empty())
            {
                AZ_Printf(OpenXRActionSetsAssetBuilder::LogName, "ActionSet had empty LocalizedName. Taking new value of [%s]", actionSetDescriptor.m_name.c_str());
                actionSetDescriptor.m_localizedName = actionSetDescriptor.m_name;
            }
            for (auto& actionDescriptor : actionSetDescriptor.m_actionDescriptors)
            {
                if (actionDescriptor.m_localizedName.empty())
                {
                    AZ_Printf(OpenXRActionSetsAssetBuilder::LogName, "Action in ActionSet [%s] had empty LocalizedName. Taking new value of [%s]",
                        actionSetDescriptor.m_name.c_str(), actionDescriptor.m_name.c_str());
                    actionDescriptor.m_localizedName = actionDescriptor.m_name;
                }
            }
        }
    }

    void OpenXRActionSetsAssetBuilder::ProcessActionSetsAssetJob(const AssetBuilderSDK::ProcessJobRequest& request, AssetBuilderSDK::ProcessJobResponse& response) const
    {
        auto actionSetsAssetPtr = LoadAssetAsUniquePtr<OpenXRVk::OpenXRActionSetsAsset>(request.m_fullPath);
        if (!actionSetsAssetPtr)
        {
            AZ_Error(LogName, false, "Failed to Load ActionsSet asset from File %s", request.m_fullPath.c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        FixEmptyLocalizedNames(*actionSetsAssetPtr.get());

        // The Action Sets Asset contains an asset reference to the OpenXRInteractionProfilesAsset that was used
        // to construct the data in it. Because we are running in a builder context, the OpenXRInteractionProfilesAsset
        // is loaded with a null handle, BUT the AssetHint is valid and we'll use the AssetHint to discover
        // the OpenXRInteractionProfilesAsset and load it manually.
        auto interactionProfileSourcePath = GetInteractionProfileAssetSourcePath(*actionSetsAssetPtr.get());
        if (interactionProfileSourcePath.empty())
        {
            AZ_Error(LogName, false, "An ActionSets source asset requires a valid InteractionProfiles source asset.");
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }
        auto interactionProfileAssetPtr = LoadAssetAsUniquePtr<OpenXRVk::OpenXRInteractionProfilesAsset>(interactionProfileSourcePath);
        if (!interactionProfileAssetPtr)
        {
            AZ_Error(LogName, false, "Failed to Load InteractionProfiles asset from File %s", interactionProfileSourcePath.c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        auto outcome = ValidateActionSetsAsset(*actionSetsAssetPtr.get(), *interactionProfileAssetPtr.get());
        if (!outcome.IsSuccess())
        {
            AZ_Error(LogName, false, "Invalid source ActionSets content when using source InteractionProfiles asset file [%s]. Reason:\n%s",
                interactionProfileSourcePath.c_str(), outcome.GetError().c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        // We keep exact same asset name and extension.
        AZStd::string assetFileName;
        AZ::StringFunc::Path::GetFullFileName(request.m_fullPath.c_str(), assetFileName);

        // Construct product full path
        AZStd::string assetOutputPath;
        AzFramework::StringFunc::Path::ConstructFull(request.m_tempDirPath.c_str(), assetFileName.c_str(), assetOutputPath, true);

        bool result = AZ::Utils::SaveObjectToFile(assetOutputPath, AZ::DataStream::ST_XML, actionSetsAssetPtr.get());
        if (result == false)
        {
            AZ_Error(LogName, false, "Failed to save asset to %s", assetOutputPath.c_str());
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }

        // This step is very important, because it declares that this OpenXRActionsSetAsset depends on a OpenXRInteractionProfilesAsset.
        // This will guarantee that when the OpenXRActionsSetAsset is loaded at runtime, the Asset Catalog will report OnAssetReady
        // only after the OpenXRInteractionProfilesAsset is already fully loaded and ready.
        AssetBuilderSDK::JobProduct jobProduct;
        if (!AssetBuilderSDK::OutputObject(actionSetsAssetPtr.get(), assetOutputPath, azrtti_typeid<OpenXRVk::OpenXRActionSetsAsset>(),
            aznumeric_cast<uint32_t>(0), jobProduct))
        {
            AZ_Error(LogName, false, "FIXME this message.");
            response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Failed;
            return;
        }
        response.m_outputProducts.emplace_back(AZStd::move(jobProduct));
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
    }

    
} // namespace OpenXRVkBuilders

#pragma optimize( "", on ) // GALIB
