/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <OpenXRVk/OpenXRInteractionProviderBus.h>
#include "OpenXRActionsBindingAsset.h"

namespace OpenXRVk
{
    ///////////////////////////////////////////////////////////
    /// OpenXRActionPath
    void OpenXRActionPath::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRActionPath>()
                ->Version(1)
                ->Field("Profile", &OpenXRActionPath::m_interactionProfile)
                ->Field("UserPath", &OpenXRActionPath::m_userPath)
                ->Field("ComponentPath", &OpenXRActionPath::m_componentPath)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRActionPath>("OpenXRActionPath", "A specific OpenXR I/O action path.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::NameLabelOverride, &OpenXRActionPath::GetEditorText)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &OpenXRActionPath::m_interactionProfile, "Interaction Profile", "The Interaction I/O profile")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &OpenXRActionPath::OnInteractionProfileSelected)
                        ->Attribute(AZ::Edit::Attributes::StringList, &OpenXRActionPath::GetInteractionProfiles)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &OpenXRActionPath::m_userPath, "User Path", "Root path identifier")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &OpenXRActionPath::OnUserPathSelected)
                        ->Attribute(AZ::Edit::Attributes::StringList, &OpenXRActionPath::GetUserPaths)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &OpenXRActionPath::m_componentPath, "I/O Component Path", "I/O Component Path identifier")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &OpenXRActionPath::OnComponentPathSelected)
                        ->Attribute(AZ::Edit::Attributes::StringList, &OpenXRActionPath::GetComponentPaths)
                    ;
            }
        }
    }

    AZStd::string OpenXRActionPath::GetEditorText() const
    {
        return AZStd::string::format("%s %s %s", m_interactionProfile.c_str(), m_userPath.c_str(), m_componentPath.c_str());
    }

    AZ::Crc32 OpenXRActionPath::OnInteractionProfileSelected()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZStd::vector<AZStd::string> OpenXRActionPath::GetInteractionProfiles() const
    {
        AZStd::vector<AZStd::string> retList;

        OpenXRInteractionProviderBus::EnumerateHandlers([&retList](OpenXRInteractionProvider* handler) -> bool {
            retList.push_back(handler->GetName());
            return true;
        });

        return retList;
    }

    AZ::Crc32 OpenXRActionPath::OnUserPathSelected()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZStd::vector<AZStd::string> OpenXRActionPath::GetUserPaths() const
    {
        AZStd::vector<AZStd::string> retList;

        OpenXRInteractionProviderBus::EventResult(retList, m_interactionProfile, &OpenXRInteractionProvider::GetUserPaths);

        return retList;
    }

    AZ::Crc32 OpenXRActionPath::OnComponentPathSelected()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZStd::vector<AZStd::string> OpenXRActionPath::GetComponentPaths() const
    {
        AZStd::vector<AZStd::string> retList;

        OpenXRInteractionProviderBus::EventResult(retList, m_interactionProfile, &OpenXRInteractionProvider::GetComponentPaths, m_userPath);

        return retList;
    }
    /// OpenXRActionPath
    ///////////////////////////////////////////////////////////
    

    ///////////////////////////////////////////////////////////
    /// OpenXRAction
    void OpenXRAction::Reflect(AZ::ReflectContext* context)
    {
        OpenXRActionPath::Reflect(context);

        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRAction>()
                ->Version(1)
                ->Field("Name", &OpenXRAction::m_name)
                ->Field("LocalizedName", &OpenXRAction::m_localizedName)
                ->Field("ActionPaths", &OpenXRAction::m_actionPaths)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRAction>("OpenXRAction", "An action bound to one or more OpenXR I/O Paths.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::NameLabelOverride, &OpenXRAction::GetEditorText)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRAction::m_name, "Name", "Runtime identifier for this action.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ_CRC("RefreshAttributesAndValues"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRAction::m_localizedName, "Localized Name", "User friendly display name.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ_CRC("RefreshAttributesAndValues"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRAction::m_actionPaths, "Action Paths", "List of action paths bound to this action")
                    ;
            }
        }
    }

    AZStd::string OpenXRAction::GetEditorText() const
    {
        if (!m_localizedName.empty())
        {
            return m_localizedName;
        }
        return m_name.empty() ? "<Unspecified Action>" : m_name;
    }
    /// OpenXRAction
    ///////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////
    /// OpenXRActionSet
    void OpenXRActionSet::Reflect(AZ::ReflectContext* context)
    {
        OpenXRAction::Reflect(context);

        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRActionSet>()
                ->Version(1)
                ->Field("Name", &OpenXRActionSet::m_name)
                ->Field("LocalizedName", &OpenXRActionSet::m_localizedName)
                ->Field("Priority", &OpenXRActionSet::m_priority)
                ->Field("Actions", &OpenXRActionSet::m_actions)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRActionSet>("OpenXRActionSet", "A group of OpenXR Actions that can be selectively activated/deactivated at runtime.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::NameLabelOverride, &OpenXRActionSet::GetEditorText)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionSet::m_name, "Name", "Runtime identifier for this action set.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ_CRC("RefreshAttributesAndValues"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionSet::m_localizedName, "Localized Name", "Action set display name.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ_CRC("RefreshAttributesAndValues"))
                    ->DataElement(AZ::Edit::UIHandlers::SpinBox, &OpenXRActionSet::m_priority, "Priority", "The higher this value the higher the priority.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionSet::m_actions, "Actions", "List of actions for this action set.")
                    ;
            }
        }
    }

    AZStd::string OpenXRActionSet::GetEditorText() const
    {
        if (!m_localizedName.empty())
        {
            return m_localizedName;
        }
        return m_name.empty() ? "<Unspecified Action Set>" : m_name;
    }
    /// OpenXRActionSet
    ///////////////////////////////////////////////////////////


    ///////////////////////////////////////////////////////////
    /// OpenXRActionBindingsAsset
    void OpenXRActionBindingsAsset::Reflect(AZ::ReflectContext* context)
    {
        OpenXRActionSet::Reflect(context);

        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRActionBindingsAsset, AZ::Data::AssetData>()
                ->Version(1)
                ->Attribute(AZ::Edit::Attributes::EnableForAssetEditor, true)
                ->Field("ActionSets", &OpenXRActionBindingsAsset::m_actionSets)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRActionBindingsAsset>(
                    s_assetTypeName, "Defines the OpenXR Actions an application cares about.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionBindingsAsset::m_actionSets, "Action Sets", "List of action sets.")
                    ;
            }
        }
    }
    /// OpenXRActionBindingsAsset
    ///////////////////////////////////////////////////////////

} // namespace OpenXRVk
