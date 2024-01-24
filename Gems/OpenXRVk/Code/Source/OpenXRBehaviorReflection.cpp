/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "OpenXRBehaviorReflection.h"

// #include <AzCore/Script/ScriptContext.h>

#include <OpenXRVk/OpenXRVisualizedSpacesInterface.h>

namespace OpenXRVk
{
    class OpenXRVisualizedSpaces
    {
    public:
        AZ_TYPE_INFO(OpenXRVisualizedSpaces, "{A060D5C5-1514-421B-9AAA-1E276BA2E33E}");
        AZ_CLASS_ALLOCATOR(OpenXRVisualizedSpaces, AZ::SystemAllocator);

        static constexpr char LogName[] = "OpenXRVisualizedSpaces";

        OpenXRVisualizedSpaces() = default;
        ~OpenXRVisualizedSpaces() = default;

        static AZStd::vector<AZStd::string> GetVisualizedSpaceNames()
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__);
                return {};
            }
            return iface->GetVisualizedSpaceNames();
        }

        static AZ::Outcome<bool, AZStd::string> AddVisualizedSpace(IOpenXRVisualizedSpaces::ReferenceSpaceId referenceSpaceType,
            const AZStd::string& spaceName, const AZ::Transform& poseInReferenceSpace)
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__)
                );
            }
            return iface->AddVisualizedSpace(referenceSpaceType, spaceName, poseInReferenceSpace);
        }

        static AZ::Outcome<bool, AZStd::string> RemoveVisualizedSpace(const AZStd::string& spaceName)
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__)
                );
            }
            return iface->RemoveVisualizedSpace(spaceName);
        }

        static AZ::Outcome<AZ::Transform, AZStd::string> GetVisualizedSpacePose(const AZStd::string& spaceName, const AZStd::string& baseSpaceName)
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__)
                );
            }
            return iface->GetVisualizedSpacePose(spaceName, baseSpaceName);
        }

        static AZ::Outcome<bool, AZStd::string> SetBaseSpaceForViewSpacePose(const AZStd::string& spaceName)
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__)
                );
            }
            return iface->SetBaseSpaceForViewSpacePose(spaceName);
        }

        static const AZStd::string& GetBaseSpaceForViewSpacePose()
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__);
                static const AZStd::string emptyStr;
                return emptyStr;
            }
            return iface->GetBaseSpaceForViewSpacePose();
        }

        static const AZ::Transform& GetViewSpacePose()
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__);
                return AZ::Transform::Identity();
            }
            return iface->GetViewSpacePose();
        }

        static uint32_t GetViewCount()
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__);
                return 0;
            }
            return iface->GetViewCount();
        }

        static const AZ::Transform& GetViewPose(uint32_t eyeIndex)
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__);
                return AZ::Transform::Identity();
            }
            return iface->GetViewPose(eyeIndex);
        }

        //TODO: Serialize AZ::RPI::FovData
        static const AZ::RPI::FovData& GetViewFovData(uint32_t eyeIndex)
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__);
                static const AZ::RPI::FovData fovData{};
                return fovData;
            }
            return iface->GetViewFovData(eyeIndex);
        }

        static const AZStd::vector<AZ::Transform>& GetViewPoses()
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__);
                static AZStd::vector<AZ::Transform> EmptyList;
                return EmptyList;
            }
            return iface->GetViewPoses();
        }

        static void ForceViewPosesCacheUpdate()
        {
            const auto iface = OpenXRVisualizedSpacesInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRVisualizedSpacesInterface is not available.", __FUNCTION__);
                return;
            }
            iface->ForceViewPosesCacheUpdate();
        }

    };

    void OpenXRBehaviorReflect(AZ::BehaviorContext& context)
    {
        context.Class<OpenXRVisualizedSpaces>("OpenXRVisualizedSpaces")
            ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
            ->Attribute(AZ::Script::Attributes::Module, "openxr")
            ->Constant("ReferenceSpaceIdView", BehaviorConstant(IOpenXRVisualizedSpaces::ReferenceSpaceIdView))
            ->Constant("ReferenceSpaceIdLocal", BehaviorConstant(IOpenXRVisualizedSpaces::ReferenceSpaceIdLocal))
            ->Constant("ReferenceSpaceIdStage", BehaviorConstant(IOpenXRVisualizedSpaces::ReferenceSpaceIdStage))
            ->Constant("ReferenceSpaceNameView", BehaviorConstant(AZStd::string(IOpenXRVisualizedSpaces::ReferenceSpaceNameView)))
            ->Constant("ReferenceSpaceNameLocal", BehaviorConstant(AZStd::string(IOpenXRVisualizedSpaces::ReferenceSpaceNameLocal)))
            ->Constant("ReferenceSpaceNameStage", BehaviorConstant(AZStd::string(IOpenXRVisualizedSpaces::ReferenceSpaceNameStage)))
            ->Constant("LeftEyeViewId", BehaviorConstant(IOpenXRVisualizedSpaces::LeftEyeView))
            ->Constant("RightEyeViewId", BehaviorConstant(IOpenXRVisualizedSpaces::RightEyeView))
            ->Method("GetVisualizedSpaceNames", &OpenXRVisualizedSpaces::GetVisualizedSpaceNames)
            ->Method("AddVisualizedSpace", &OpenXRVisualizedSpaces::AddVisualizedSpace)
            ->Method("RemoveVisualizedSpace", &OpenXRVisualizedSpaces::RemoveVisualizedSpace)
            ->Method("GetVisualizedSpacePose", &OpenXRVisualizedSpaces::GetVisualizedSpacePose)
            ->Method("SetBaseSpaceForViewSpacePose", &OpenXRVisualizedSpaces::SetBaseSpaceForViewSpacePose)
            ->Method("GetBaseSpaceForViewSpacePose", &OpenXRVisualizedSpaces::GetBaseSpaceForViewSpacePose)
            ->Method("GetViewSpacePose", &OpenXRVisualizedSpaces::GetViewSpacePose)
            ->Method("GetViewCount", &OpenXRVisualizedSpaces::GetViewCount)
            ->Method("GetViewPose", &OpenXRVisualizedSpaces::GetViewPose)
            //TODO: Serialize AZ::RPI::FovData
            //->Method("GetViewFovData", &OpenXRVisualizedSpaces::GetViewFovData)
            ->Method("GetViewPoses", &OpenXRVisualizedSpaces::GetViewPoses)
            ->Method("ForceViewPosesCacheUpdate", &OpenXRVisualizedSpaces::ForceViewPosesCacheUpdate)
             ;
    }

}
