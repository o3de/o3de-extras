/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "OpenXRInteractionProfileDescriptor.h"

namespace OpenXRVk
{
    ///////////////////////////////////////////////////////////
    /// OpenXRInteractionComponentPathDescriptor
    XrActionType OpenXRInteractionComponentPathDescriptor::GetXrActionType(AZStd::string_view actionTypeStr)
    {
        if (actionTypeStr == s_TypeBoolStr)
        {
            return XR_ACTION_TYPE_BOOLEAN_INPUT;
        }
        else if (actionTypeStr == s_TypeFloatStr)
        {
            return XR_ACTION_TYPE_FLOAT_INPUT;
        }
        else if (actionTypeStr == s_TypeVector2Str)
        {
            return XR_ACTION_TYPE_VECTOR2F_INPUT;
        }
        else if (actionTypeStr == s_TypePoseStr)
        {
            return XR_ACTION_TYPE_POSE_INPUT;
        }
        else if (actionTypeStr == s_TypePoseStr)
        {
            return XR_ACTION_TYPE_VIBRATION_OUTPUT;
        }
        return XR_ACTION_TYPE_MAX_ENUM;
    }

    XrActionType OpenXRInteractionComponentPathDescriptor::GetXrActionType() const
    {
        return GetXrActionType(m_actionTypeStr);
    }

    static AZStd::vector<AZStd::string> GetEditorXrActionTypeNames()
    {
        static AZStd::vector<AZStd::string> s_actionTypeNames = {
                {OpenXRInteractionComponentPathDescriptor::s_TypeBoolStr   },
                {OpenXRInteractionComponentPathDescriptor::s_TypeFloatStr  },
                {OpenXRInteractionComponentPathDescriptor::s_TypeVector2Str},
                {OpenXRInteractionComponentPathDescriptor::s_TypePoseStr   },
                {OpenXRInteractionComponentPathDescriptor::s_TypeVibrationStr   },
        };
        return s_actionTypeNames;
    }

    void OpenXRInteractionComponentPathDescriptor::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRInteractionComponentPathDescriptor>()
                ->Version(1)
                ->Field("Name", &OpenXRInteractionComponentPathDescriptor::m_name)
                ->Field("Path", &OpenXRInteractionComponentPathDescriptor::m_path)
                ->Field("ActionType", &OpenXRInteractionComponentPathDescriptor::m_actionTypeStr)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRInteractionComponentPathDescriptor>("Component Path", "An OpenXR Component Path that is supported by an OpenXR User Path")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionComponentPathDescriptor::m_name, "Name", "User friendly name.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionComponentPathDescriptor::m_path, "Path", "An OpenXR Path string that starts with '/' BUT is relative to a User Path.")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &OpenXRInteractionComponentPathDescriptor::m_actionTypeStr, "Action Type", "Data type of this action.")
                        ->Attribute(AZ::Edit::Attributes::StringList, &GetEditorXrActionTypeNames)
                    ;
            }
        }
    }
    /// OpenXRInteractionComponentPathDescriptor
    ///////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////
    /// OpenXRInteractionUserPathDescriptor
    void OpenXRInteractionUserPathDescriptor::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRInteractionUserPathDescriptor>()
                ->Version(1)
                ->Field("Name", &OpenXRInteractionUserPathDescriptor::m_name)
                ->Field("Path", &OpenXRInteractionUserPathDescriptor::m_path)
                ->Field("ComponentPaths", &OpenXRInteractionUserPathDescriptor::m_componentPathDescriptors)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRInteractionUserPathDescriptor>("User Path", "Represents a User Path supported by an Interaction Profile")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionUserPathDescriptor::m_name, "Name", "User friendly name.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionUserPathDescriptor::m_path, "Path", "An OpenXR Path string that starts with '/'.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionUserPathDescriptor::m_componentPathDescriptors, "Component Paths", "List of component paths supported by this User Path")
                    ;
            }
        }
    }
    /// OpenXRInteractionUserPathDescriptor
    ///////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////
    /// OpenXRInteractionProfileDescriptor
    void OpenXRInteractionProfileDescriptor::Reflect(AZ::ReflectContext* context)
    {
        OpenXRInteractionComponentPathDescriptor::Reflect(context);
        OpenXRInteractionUserPathDescriptor::Reflect(context);

        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRInteractionProfileDescriptor>()
                ->Version(1)
                ->Field("UniqueName", &OpenXRInteractionProfileDescriptor::m_uniqueName)
                ->Field("Path", &OpenXRInteractionProfileDescriptor::m_path)
                ->Field("UserPathDescriptors", &OpenXRInteractionProfileDescriptor::m_userPathDescriptors)
                ->Field("CommonComponentPathDescriptors", &OpenXRInteractionProfileDescriptor::m_commonComponentPathDescriptors)
                ;
  
            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRInteractionProfileDescriptor>(
                    "Interaction Profile", "Defines an OpenXR Interaction Profile Supported by O3DE.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionProfileDescriptor::m_uniqueName, "Unique Name", "Unique name across all interaction profiles")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionProfileDescriptor::m_path, "Path", "OpenXR Canonical Path for this interation profile.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionProfileDescriptor::m_userPathDescriptors, "User Paths", "List of user paths")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRInteractionProfileDescriptor::m_commonComponentPathDescriptors, "Common Component Paths", "List of component paths supported by all User Paths")
                    ;
            }
        }
    }
    /// OpenXRInteractionProfileDescriptor
    ///////////////////////////////////////////////////////////

} // namespace OpenXRVk
