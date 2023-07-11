/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/Asset/AssetManager.h>
#include <AzCore/Asset/AssetDataStream.h>
#include <AzCore/IO/FileIO.h>
#include <AzCore/IO/IOUtils.h>
#include <AzCore/Serialization/Json/JsonUtils.h>

#include <AzFramework/StringFunc/StringFunc.h>
#include <AzToolsFramework/Prefab/Procedural/ProceduralPrefabAsset.h>

#include <AssetBuilderSDK/SerializationDependencies.h>
#include <SdfAssetBuilder/SdfAssetBuilder.h>

namespace ROS2
{
        namespace
        {
            [[maybe_unused]] constexpr const char* SdfAssetBuilderName = "SdfAssetBuilder";
            constexpr const char* SdfAssetBuilderJobKey = "Sdf Asset Builder";
            constexpr const char* SdfAssetSourceExtensions[] =
            {
                "sdf",
                "urdf",
                "world",
                "xacro"
            };
            const uint32_t NumberOfSourceExtensions = AZ_ARRAY_SIZE(SdfAssetSourceExtensions);
        }


    void SdfAssetBuilder::RegisterBuilder()
    {
        // Register Sdf Asset Builder

        // build source extension patterns
        AZStd::vector<AssetBuilderSDK::AssetBuilderPattern> patterns(NumberOfSourceExtensions);
        AZStd::for_each(patterns.begin(), patterns.end(), [&, index = 0](AssetBuilderSDK::AssetBuilderPattern& pattern) mutable
        {
            pattern = AssetBuilderSDK::AssetBuilderPattern(AZStd::string("*.") + SdfAssetSourceExtensions[index++], AssetBuilderSDK::AssetBuilderPattern::PatternType::Wildcard);
        });

        AssetBuilderSDK::AssetBuilderDesc sdfAssetBuilderDescriptor;
        sdfAssetBuilderDescriptor.m_name = SdfAssetBuilderJobKey;
        sdfAssetBuilderDescriptor.m_version = 1; // bump this to rebuild all sdf files
        sdfAssetBuilderDescriptor.m_patterns.insert(sdfAssetBuilderDescriptor.m_patterns.end(), patterns.begin(), patterns.end());
        sdfAssetBuilderDescriptor.m_busId = azrtti_typeid<SdfAssetBuilder>();
        sdfAssetBuilderDescriptor.m_createJobFunction = AZStd::bind(&SdfAssetBuilder::CreateJobs, this,
            AZStd::placeholders::_1, AZStd::placeholders::_2);
        sdfAssetBuilderDescriptor.m_processJobFunction = AZStd::bind(&SdfAssetBuilder::ProcessJob, this,
            AZStd::placeholders::_1, AZStd::placeholders::_2);

        BusConnect(sdfAssetBuilderDescriptor.m_busId);
        AssetBuilderSDK::AssetBuilderBus::Broadcast(
            &AssetBuilderSDK::AssetBuilderBus::Handler::RegisterBuilderInformation, sdfAssetBuilderDescriptor);
    }

    void SdfAssetBuilder::CreateJobs(
        const AssetBuilderSDK::CreateJobsRequest& request,
        AssetBuilderSDK::CreateJobsResponse& response) const
    {
        for (const AssetBuilderSDK::PlatformInfo& platformInfo : request.m_enabledPlatforms)
        {
            AssetBuilderSDK::JobDescriptor jobDescriptor;
            jobDescriptor.m_critical = false;
            jobDescriptor.m_jobKey = "SDF (Simulation Description Format) Asset";
            jobDescriptor.SetPlatformIdentifier(platformInfo.m_identifier.c_str());

            response.m_createJobOutputs.push_back(jobDescriptor);
        }

        response.m_result = AssetBuilderSDK::CreateJobsResultCode::Success;
    }

    void SdfAssetBuilder::ProcessJob(
        const AssetBuilderSDK::ProcessJobRequest& request,
        AssetBuilderSDK::ProcessJobResponse& response) const
    {
        AZStd::string outputFilename = request.m_sourceFile;
        AzFramework::StringFunc::Path::ReplaceExtension(outputFilename, "procprefab");

        AZStd::string tempAssetOutputPath;
        AzFramework::StringFunc::Path::ConstructFull(request.m_tempDirPath.c_str(), outputFilename.c_str(), tempAssetOutputPath, true);

        const char *testJson = 
        R"({
        "ContainerEntity": {
        "Id": "ContainerEntity",
        "Name": "Test",
        "Components": {
            "EditorDisabledCompositionComponent": {
                "$type": "EditorDisabledCompositionComponent",
                "Id": 14656326877098292841
            },
            "EditorEntityIconComponent": {
                "$type": "EditorEntityIconComponent",
                "Id": 11614206288064806714
            },
            "EditorEntitySortComponent": {
                "$type": "EditorEntitySortComponent",
                "Id": 1126088702235031237,
                "Child Entity Order": [
                    "Entity_[455432209490]"
                ]
            },
            "EditorInspectorComponent": {
                "$type": "EditorInspectorComponent",
                "Id": 13612639098028421457
            },
            "EditorLockComponent": {
                "$type": "EditorLockComponent",
                "Id": 2827882931258296541
            },
            "EditorOnlyEntityComponent": {
                "$type": "EditorOnlyEntityComponent",
                "Id": 12437311853692771656
            },
            "EditorPendingCompositionComponent": {
                "$type": "EditorPendingCompositionComponent",
                "Id": 4022707151152308083
            },
            "EditorPrefabComponent": {
                "$type": "EditorPrefabComponent",
                "Id": 17843864682529387902
            },
            "EditorVisibilityComponent": {
                "$type": "EditorVisibilityComponent",
                "Id": 11388594862630242209
            },
            "TransformComponent": {
                "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
                "Id": 1199612156053455197,
                "Parent Entity": ""
            }
        }
    },
    "Entities": {
        "Entity_[455432209490]": {
            "Id": "Entity_[455432209490]",
            "Name": "Test",
            "Components": {
                "EditorDisabledCompositionComponent": {
                    "$type": "EditorDisabledCompositionComponent",
                    "Id": 3912802081073994307
                },
                "EditorEntityIconComponent": {
                    "$type": "EditorEntityIconComponent",
                    "Id": 15825240119987901455
                },
                "EditorEntitySortComponent": {
                    "$type": "EditorEntitySortComponent",
                    "Id": 11869028326276761445
                },
                "EditorInspectorComponent": {
                    "$type": "EditorInspectorComponent",
                    "Id": 13793518228311740492,
                    "ComponentOrderEntryArray": [
                        {
                            "ComponentId": 2710992383319380785
                        }
                    ]
                },
                "EditorLockComponent": {
                    "$type": "EditorLockComponent",
                    "Id": 6074305383028854732
                },
                "EditorOnlyEntityComponent": {
                    "$type": "EditorOnlyEntityComponent",
                    "Id": 14948070210246835195
                },
                "EditorPendingCompositionComponent": {
                    "$type": "EditorPendingCompositionComponent",
                    "Id": 1646028063715116529
                },
                "EditorVisibilityComponent": {
                    "$type": "EditorVisibilityComponent",
                    "Id": 18223707151398401988
                },
                "TransformComponent": {
                    "$type": "{27F1E1A1-8D9D-4C3B-BD3A-AFB9762449C0} TransformComponent",
                    "Id": 2710992383319380785,
                    "Parent Entity": "ContainerEntity"
                }
            }
        }
    }
})";

        rapidjson::Document doc;
        doc.Parse(testJson);
        AZ::JsonSerializationUtils::WriteJsonFile(doc, tempAssetOutputPath.c_str());

        AssetBuilderSDK::JobProduct sdfJobProduct;
        sdfJobProduct.m_productFileName = tempAssetOutputPath;
        sdfJobProduct.m_productSubID = 0;
        sdfJobProduct.m_productAssetType = azrtti_typeid<AZ::Prefab::ProceduralPrefabAsset>();
        sdfJobProduct.m_dependenciesHandled = true;

        response.m_outputProducts.push_back(AZStd::move(sdfJobProduct));
        response.m_resultCode = AssetBuilderSDK::ProcessJobResult_Success;
    }
} // ROS2
