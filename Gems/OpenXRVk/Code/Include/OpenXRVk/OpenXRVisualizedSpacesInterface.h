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
#include <Atom/RPI.Public/XR/XRRenderingInterface.h>

namespace OpenXRVk
{
    //! REMARK: This class must be instantiated BEFORE OpenXRActionsInterface.
    class IOpenXRVisualizedSpaces
    {
    public:
        AZ_RTTI(IOpenXRVisualizedSpaces, "{244D24BE-DD6F-430A-8F99-1D24AC1665B6}");
        AZ_DISABLE_COPY_MOVE(IOpenXRVisualizedSpaces);

        IOpenXRVisualizedSpaces() = default;
        virtual ~IOpenXRVisualizedSpaces() = default;

        // Identifiers for all reference spaces that all OpenXR implementations
        // must support. These are the default system spaces. Most OpenXR implementations
        // support additional reference spaces, and you can try to use their id to attempt
        // to create an space of them at runtime. This explains why this is an ordinary integral
        // instead of an enum.
        using ReferenceSpaceId = uint32_t;
        static constexpr ReferenceSpaceId ReferenceSpaceIdView = 1;
        static constexpr ReferenceSpaceId ReferenceSpaceIdLocal = 2;
        static constexpr ReferenceSpaceId ReferenceSpaceIdStage = 3;
        // With these names you can refer to the default system spaces.
        static constexpr char ReferenceSpaceViewName[] = "View"; // Typically represents the user's head centroid.
        static constexpr char ReferenceSpaceLocalName[] = "Local"; 
        static constexpr char ReferenceSpaceStageName[] = "Stage";

        virtual AZStd::vector<AZStd::string> GetVisualizedSpaceNames() const = 0;
        virtual AZ::Outcome<bool, AZStd::string> AddVisualizedSpace(ReferenceSpaceId referenceSpaceType, const AZStd::string& spaceName, const AZ::Transform& poseInReferenceSpace) = 0;
        virtual AZ::Outcome<bool, AZStd::string> RemoveVisualizedSpace(const AZStd::string& spaceName) = 0;

        //! REMARK: Maybe it's a good idea to move this into a private interface for the OpenXRVk Gem,
        //! and privately use the native handle (XrSpace)
        virtual const void * GetVisualizedSpaceNativeHandle(const AZStd::string& spaceName) const = 0;

        // Returns the Pose of @spaceName relative to @baseSpaceName.
        virtual AZ::Outcome<AZ::Transform, AZStd::string> GetVisualizedSpacePose(const AZStd::string& spaceName, const AZStd::string& baseSpaceName) const = 0;

        // By default the "View" space is the only space that will be automatically
        // located on each frame. The "View" space represents the User's head centroid.
        // We need a base space to use a reference when locating the user's head, and
        // the default base space is the "Local" space. But you can change that with this API.
        // Internally the Pose for each Eye View will be calculated relative to the View Space centroid
        // and it will be reported via the notification bus: AZ::RPI::XRSpaceNotificationBus 
        virtual AZ::Outcome<bool, AZStd::string> SetBaseSpaceForViewSpacePose(const AZStd::string& spaceName) = 0;
        virtual const AZStd::string& GetBaseSpaceForViewSpacePose() const = 0;
        // The following poses are the same as reported by AZ::RPI::XRSpaceNotificationBus
        //! Head/View pose relative to BaseSpaceForHeadCentroid.
        virtual const AZ::Transform& GetViewSpacePose() const = 0;

        static constexpr uint32_t LeftEyeView = 0;
        static constexpr uint32_t RightEyeView = 1;

        // For an AR application running in a Phone (Mono) the count will be 1.
        // For an AR/VR application running a typical headset (Stereo) the count will be 2.
        // Some headsets like the Varjo support Quad Views and the eye counts will be 4. 
        virtual uint32_t GetViewCount() const = 0;

        // Pose of a view(aka eye) relative to the View Space (aka Head) Pose.
        // For AR applications running on a Phone (aka Mono view configuration) the Eye View Pose
        // is centered exactly where the View Space Centroid is located, so calling this function wouldn't
        // make much sense because it'll return an Identity transform.
        virtual const AZ::Transform& GetViewPose(uint32_t eyeIndex) const = 0;

        virtual const AZ::RPI::FovData& GetViewFovData(uint32_t eyeIndex) const = 0;

        virtual const AZStd::vector<AZ::Transform>& GetViewPoses() const = 0;

        //! Forces updating the cached pose and projection data for all Views.
        //! Each frame, all view (aka eye) poses and projections are updated automatically, making this function optional to use.
        //! By default the caller simply gets the per-frame cached version, which should be fine for most applications.
        //! This API was added because according to OpenXR xrLocateViews spec:
        //! "Repeatedly calling xrLocateViews with the same time may not necessarily return the same result.
        //!  Instead the prediction gets increasingly accurate as the function is called closer to the
        //!  given time for which a prediction is made"
        virtual void ForceViewPosesCacheUpdate() = 0;
    };

    using OpenXRVisualizedSpacesInterface = AZ::Interface<IOpenXRVisualizedSpaces>;

} // namespace OpenXRVk