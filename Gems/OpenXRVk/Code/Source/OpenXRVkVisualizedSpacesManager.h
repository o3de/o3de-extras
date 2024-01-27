/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <openxr/openxr.h>

#include <OpenXRVk/OpenXRVkVisualizedSpacesInterface.h>

namespace OpenXRVk
{
    class VisualizedSpacesManager final :
        public OpenXRVisualizedSpacesInterface::Registrar
    {
    public:
        AZ_CLASS_ALLOCATOR(VisualizedSpacesManager, AZ::SystemAllocator);
        AZ_RTTI(VisualizedSpacesManager, "{4BC4D0C0-02D4-4676-8352-8BC51306AF02}", IOpenXRVisualizedSpaces)

        static constexpr char LogName[] = "OpenXRVkVisualizedSpacesManager";

        //! Initialize various actions and actionSets according to the
        //! "openxr.xractions" action bindings asset.
        bool Init(XrInstance xrInstance, XrSession xrSession, XrViewConfigurationType xrViewConfigurationType, uint32_t numEyeViews);

        //! Called by the Session each tick.
        bool SyncViews(XrTime predictedDisplayTime);

        // Spaces are reset every time the proximity sensor turns off, or the user wears the headset
        // when the proximity sensor is ON.
        void ResetSpaces();

        const AZStd::vector<XrView>& GetXrViews() const;

        XrSpace GetViewSpaceXrSpace() const;

        /////////////////////////////////////////////////
        /// OpenXRVisualizedSpacesInterface overrides
        AZStd::vector<AZStd::string> GetVisualizedSpaceNames() const override;
        AZ::Outcome<bool, AZStd::string> AddVisualizedSpace(ReferenceSpaceId referenceSpaceType,
            const AZStd::string& spaceName, const AZ::Transform& poseInReferenceSpace) override;
        AZ::Outcome<bool, AZStd::string> RemoveVisualizedSpace(const AZStd::string& spaceName) override;

        const void * GetVisualizedSpaceNativeHandle(const AZStd::string& spaceName) const override;

        AZ::Outcome<AZ::Transform, AZStd::string> GetVisualizedSpacePose(const AZStd::string& spaceName,
            const AZStd::string& baseSpaceName) const override;

        AZ::Outcome<bool, AZStd::string> SetBaseSpaceForViewSpacePose(const AZStd::string& spaceName) override;
        const AZStd::string& GetBaseSpaceForViewSpacePose() const override;
        const AZ::Transform& GetViewSpacePose() const override;

        uint32_t GetViewCount() const override;
        const AZ::Transform& GetViewPose(uint32_t eyeIndex) const override;
        const AZ::RPI::FovData& GetViewFovData(uint32_t eyeIndex) const override;
        const AZStd::vector<AZ::Transform>& GetViewPoses() const override;

        void ForceViewPosesCacheUpdate() override;
        /// OpenXRVisualizedSpacesInterface overrides
        /////////////////////////////////////////////////

    private:

        XrInstance m_xrInstance = XR_NULL_HANDLE;
        XrSession m_xrSession = XR_NULL_HANDLE;
        XrViewConfigurationType m_xrViewConfigurationType;

        // Updated each time the Session calls UpdateViewSpacePoseAndEyeViewPoses().
        XrTime m_predictedDisplaytime;

        struct VisualizedSpace
        {
            AZStd::string m_name;
            //! We shave this transform in case we have to reset the spaces.
            AZ::Transform m_offsetPose;
            // Runtime data
            XrSpace m_xrSpace;
        };

        // At the bare minimum this map will always contain three spaces that can not be removed:
        // "View", "Local" and "Stage".
        AZStd::unordered_map<AZStd::string, VisualizedSpace> m_spaces;

        // We cache here the base space what will be used to "locate" the View Space pose.
        const VisualizedSpace* m_baseSpaceForViewSpace;
        const VisualizedSpace* m_viewSpace; // Cached for convenience. This pointer is set once during initialization and never changes.

        AZ::Transform m_viewSpacePose;
        //! The following poses are always relative to @m_viewSpacePose.
        AZStd::vector<AZ::Transform> m_eyeViewPoses;
        AZStd::vector<AZ::RPI::FovData> m_eyeViewFovDatas;
        AZStd::vector<XrView> m_xrViews;

        XrSpace CreateXrSpace(XrReferenceSpaceType referenceSpaceType, const AZ::Transform& relativePose);
    };
}