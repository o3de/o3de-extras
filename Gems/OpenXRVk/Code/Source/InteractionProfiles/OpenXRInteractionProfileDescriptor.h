/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <openxr/openxr.h>

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/vector.h>

namespace OpenXRVk
{
    class OpenXRInteractionComponentPathDescriptor final
    {
    public:
        AZ_RTTI(OpenXRInteractionComponentPathDescriptor, "{E2038854-929D-484F-A34E-1C7390EE2CCB}");
        virtual ~OpenXRInteractionComponentPathDescriptor() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        static constexpr AZStd::string_view s_TypeBoolStr{ "XrBool" };
        static constexpr AZStd::string_view s_TypeFloatStr = "XrFloat";
        static constexpr AZStd::string_view s_TypeVector2Str = "XrVector2";
        static constexpr AZStd::string_view s_TypePoseStr = "XrPose";
        static constexpr AZStd::string_view s_TypeVibrationStr = "XrVibration";

        //! Helper method
        static XrActionType GetXrActionType(AZStd::string_view actionTypeStr);
        XrActionType GetXrActionType() const;

        //! A user friendly name.
        AZStd::string m_name;
        //! For OpenXR a Component Path string would look like:
        //! "/input/x/click", or "/input/trigger/value", etc
        AZStd::string m_path;
        //! Whether this is a boolean, float, vector2 or pose.
        //! The user will be presented with a combo box to avoid
        //! chances for error.
        AZStd::string m_actionTypeStr;

    private:
        AZStd::string GetEditorText();

    };

    class OpenXRInteractionUserPathDescriptor final
    {
    public:
        AZ_RTTI(OpenXRInteractionUserPathDescriptor, "{F3913A15-41FC-4EC9-A381-296C0AB6D6C6}");
        virtual ~OpenXRInteractionUserPathDescriptor() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        //! A user friendly name.
        AZStd::string m_name;
        //! For OpenXR a User Path string would look like:
        //! "/user/hand/left", or "/user/hand/right", etc
        AZStd::string m_path;
        //! Component Paths that are only supported under this user path.
        //! This list can be empty.
        AZStd::vector<OpenXRInteractionComponentPathDescriptor> m_componentPathDescriptors;

    private:
        AZStd::string GetEditorText();

    };

    class OpenXRInteractionProfileDescriptor final
    {
    public:
        AZ_RTTI(OpenXRInteractionProfileDescriptor, "{BC73B4BC-4F15-4B1E-AEA9-B133FBB5AD16}");
        virtual ~OpenXRInteractionProfileDescriptor() = default;

        static void Reflect(AZ::ReflectContext* reflection);

        static constexpr char LogName[] = "OpenXRInteractionProfileDescriptor";

        //! Returns success only if the data makes sense, has proper casing, etc.
        AZ::Outcome<void, AZStd::string> Validate() const;

        const OpenXRInteractionUserPathDescriptor* GetUserPathDescriptor(const AZStd::string& userPathName) const;

        //! Unique name across all OpenXRInteractionProfileDescriptor.
        //! It serves also as user friendly display name, and because
        //! it is unique it can be used in a dictionary.
        AZStd::string m_uniqueName;
        AZStd::string m_path;

        AZStd::vector<OpenXRInteractionUserPathDescriptor> m_userPathDescriptors;
        // ComponentsPaths that are supported by all User Paths listed in @m_userPathDescriptors
        AZStd::vector<OpenXRInteractionComponentPathDescriptor> m_commonComponentPathDescriptors;

    private:
        AZStd::string GetEditorText();
    };

}// namespace OpenXRVk
