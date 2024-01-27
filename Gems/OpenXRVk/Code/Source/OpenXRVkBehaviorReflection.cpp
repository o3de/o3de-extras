/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRVkVisualizedSpacesInterface.h>
#include <OpenXRVk/OpenXRVkActionsInterface.h>

#include "OpenXRVkBehaviorReflection.h"

namespace OpenXRVk
{
    void PoseWithVelocities::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<PoseWithVelocities>()
                ->Version(1)
                ->Field("Pose", &PoseWithVelocities::m_pose)
                ->Field("LinearVelocity", &PoseWithVelocities::m_linearVelocity)
                ->Field("AngularVelocitu", &PoseWithVelocities::m_angularVelocity)
                ;
        }

        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->Class<PoseWithVelocities>()
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Module, "openxr")
                ->Property("pose", BehaviorValueGetter(&PoseWithVelocities::m_pose), nullptr)
                ->Property("linearVelocity", BehaviorValueGetter(&PoseWithVelocities::m_linearVelocity), nullptr)
                ->Property("angularVelocity", BehaviorValueGetter(&PoseWithVelocities::m_angularVelocity), nullptr)
                ;
        }
    }

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

    }; // class OpenXRVisualizedSpaces

    class OpenXRActions
    {
    public:
        AZ_TYPE_INFO(OpenXRActions, "{290DF9D5-4042-4843-BF23-D49F9FAD6D90}");
        AZ_CLASS_ALLOCATOR(OpenXRActions, AZ::SystemAllocator);

        static constexpr char LogName[] = "OpenXRActions";

        OpenXRActions() = default;
        ~OpenXRActions() = default;

        static AZStd::vector<AZStd::string> GetAllActionSets()
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRActionsInterface is not available.", __FUNCTION__);
                return {};
            }
            return iface->GetAllActionSets();
        }

        static AZ::Outcome<bool, AZStd::string> GetActionSetState(const AZStd::string& actionSetName)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRActionsInterface is not available.", __FUNCTION__);
                return {};
            }
            return iface->GetActionSetState(actionSetName);
        }

        static AZ::Outcome<bool, AZStd::string> SetActionSetState(const AZStd::string& actionSetName, bool activate)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRActionsInterface is not available.", __FUNCTION__);
                return {};
            }
            return iface->SetActionSetState(actionSetName, activate);
        }

        static AZ::Outcome<IOpenXRActions::ActionHandle, AZStd::string> GetActionHandle(const AZStd::string& actionSetName, const AZStd::string& actionName)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRActionsInterface is not available.", __FUNCTION__)
                );
            }
            return iface->GetActionHandle(actionSetName, actionName);
        }

        static AZ::Outcome<bool, AZStd::string> GetActionStateBoolean(IOpenXRActions::ActionHandle actionHandle)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRActionsInterface is not available.", __FUNCTION__)
                );
            }
            return iface->GetActionStateBoolean(actionHandle);
        }

        static AZ::Outcome<float, AZStd::string> GetActionStateFloat(IOpenXRActions::ActionHandle actionHandle)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRActionsInterface is not available.", __FUNCTION__)
                );
            }
            return iface->GetActionStateFloat(actionHandle);
        }

        static AZ::Outcome<AZ::Vector2, AZStd::string> GetActionStateVector2(IOpenXRActions::ActionHandle actionHandle)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRActionsInterface is not available.", __FUNCTION__)
                );
            }
            return iface->GetActionStateVector2(actionHandle);
        }

        static AZ::Outcome<bool, AZStd::string> SetBaseVisualizedSpaceForPoseActions(const AZStd::string& visualizedSpaceName)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRActionsInterface is not available.", __FUNCTION__)
                );
            }
            return iface->SetBaseVisualizedSpaceForPoseActions(visualizedSpaceName);
        }

        static const AZStd::string& GetBaseVisualizedSpaceForPoseActions()
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                AZ_Error(LogName, false, "%s: OpenXRActionsInterface is not available.", __FUNCTION__);
                static const AZStd::string emptyStr;
                return emptyStr;
            }
            return iface->GetBaseVisualizedSpaceForPoseActions();
        }

        static AZ::Outcome<AZ::Transform, AZStd::string> GetActionStatePose(IOpenXRActions::ActionHandle actionHandle)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRActionsInterface is not available.", __FUNCTION__)
                );
            }
            return iface->GetActionStatePose(actionHandle);
        }

        static AZ::Outcome<PoseWithVelocities, AZStd::string> GetActionStatePoseWithVelocities(IOpenXRActions::ActionHandle actionHandle)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRActionsInterface is not available.", __FUNCTION__)
                );
            }
            return iface->GetActionStatePoseWithVelocities(actionHandle);
        }

        static AZ::Outcome<bool, AZStd::string> ApplyHapticVibrationAction(IOpenXRActions::ActionHandle actionHandle, uint64_t durationNanos, float frequencyHz, float amplitude)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRActionsInterface is not available.", __FUNCTION__)
                );
            }
            return iface->ApplyHapticVibrationAction(actionHandle, durationNanos, frequencyHz, amplitude);
        }

        static AZ::Outcome<bool, AZStd::string> StopHapticVibrationAction(IOpenXRActions::ActionHandle actionHandle)
        {
            const auto iface = OpenXRActionsInterface::Get();
            if (!iface)
            {
                return AZ::Failure(
                    AZStd::string::format("%s: OpenXRActionsInterface is not available.", __FUNCTION__)
                );
            }
            return iface->StopHapticVibrationAction(actionHandle);
        }

    }; // class OpenXRActions

    void OpenXRBehaviorReflect(AZ::BehaviorContext& context)
    {
        IOpenXRActions::ActionHandle::Reflect(&context);
        PoseWithVelocities::Reflect(&context);

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

        context.Class<OpenXRActions>("OpenXRActions")
            ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
            ->Attribute(AZ::Script::Attributes::Module, "openxr")
            ->Method("GetAllActionSets", &OpenXRActions::GetAllActionSets)
            ->Method("GetActionSetState", &OpenXRActions::GetActionSetState)
            ->Method("SetActionSetState", &OpenXRActions::SetActionSetState)
            ->Method("GetActionHandle", &OpenXRActions::GetActionHandle)
            ->Method("GetActionStateBoolean", &OpenXRActions::GetActionStateBoolean)
            ->Method("GetActionStateFloat", &OpenXRActions::GetActionStateFloat)
            ->Method("GetActionStateVector2", &OpenXRActions::GetActionStateVector2)
            ->Method("SetBaseVisualizedSpaceForPoseActions", &OpenXRActions::SetBaseVisualizedSpaceForPoseActions)
            ->Method("GetBaseVisualizedSpaceForPoseActions", &OpenXRActions::GetBaseVisualizedSpaceForPoseActions)
            ->Method("GetActionStatePose", &OpenXRActions::GetActionStatePose)
            ->Method("GetActionStatePoseWithVelocities", &OpenXRActions::GetActionStatePoseWithVelocities)
            ->Method("ApplyHapticVibrationAction", &OpenXRActions::ApplyHapticVibrationAction)
            ->Method("StopHapticVibrationAction", &OpenXRActions::StopHapticVibrationAction)
            ;
    }

}
