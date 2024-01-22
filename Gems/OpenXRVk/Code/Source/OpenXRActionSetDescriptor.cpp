/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "InteractionProfiles/OpenXRInteractionProfilesProviderInterface.h"
#include "OpenXRActionSetDescriptor.h"

namespace OpenXRVk
{
    ///////////////////////////////////////////////////////////
    /// OpenXRActionPathDescriptor
    void OpenXRActionPathDescriptor::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRActionPathDescriptor>()
                ->Version(1)
                ->Field("InteractionProfile", &OpenXRActionPathDescriptor::m_interactionProfileName)
                ->Field("UserPath", &OpenXRActionPathDescriptor::m_userPathName)
                ->Field("ComponentPath", &OpenXRActionPathDescriptor::m_componentPathName)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRActionPathDescriptor>("OpenXRActionPathDescriptor", "A specific OpenXR I/O action path.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::NameLabelOverride, &OpenXRActionPathDescriptor::GetEditorText)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &OpenXRActionPathDescriptor::m_interactionProfileName, "Interaction Profile", "The name of the Interaction Profile.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &OpenXRActionPathDescriptor::OnInteractionProfileSelected)
                        ->Attribute(AZ::Edit::Attributes::StringList, &OpenXRActionPathDescriptor::GetInteractionProfiles)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &OpenXRActionPathDescriptor::m_userPathName, "User Path", "Name of the User Path.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &OpenXRActionPathDescriptor::OnUserPathSelected)
                        ->Attribute(AZ::Edit::Attributes::StringList, &OpenXRActionPathDescriptor::GetUserPaths)
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &OpenXRActionPathDescriptor::m_componentPathName, "I/O Component Path", "The name of I/O Component Path.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &OpenXRActionPathDescriptor::OnComponentPathSelected)
                        ->Attribute(AZ::Edit::Attributes::StringList, &OpenXRActionPathDescriptor::GetComponentPaths)
                    ;
            }
        }
    }

    AZStd::string OpenXRActionPathDescriptor::GetEditorText() const
    {
        return AZStd::string::format("%s %s %s", m_interactionProfileName.c_str(), m_userPathName.c_str(), m_componentPathName.c_str());
    }

    AZ::Crc32 OpenXRActionPathDescriptor::OnInteractionProfileSelected()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZStd::vector<AZStd::string> OpenXRActionPathDescriptor::GetInteractionProfiles() const
    {
        auto interactionProviderIface = OpenXRInteractionProfilesProviderInterface::Get();
        if (!interactionProviderIface)
        {
            return {};
        }
        return interactionProviderIface->GetInteractionProfileNames();
    }

    AZ::Crc32 OpenXRActionPathDescriptor::OnUserPathSelected()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZStd::vector<AZStd::string> OpenXRActionPathDescriptor::GetUserPaths() const
    {
        AZStd::vector<AZStd::string> retList;
        auto interactionProviderIface = OpenXRInteractionProfilesProviderInterface::Get();
        if (!interactionProviderIface)
        {
            return retList;
        }

        const auto * profileDescriptor = interactionProviderIface->GetInteractionProfileDescriptor(m_interactionProfileName);
        if (!profileDescriptor)
        {
            return retList;
        }
        for (const auto& userPathDescriptor : profileDescriptor->m_userPathDescriptors)
        {
            retList.push_back(userPathDescriptor.m_name);
        }
        return retList;
    }

    AZ::Crc32 OpenXRActionPathDescriptor::OnComponentPathSelected()
    {
        return AZ::Edit::PropertyRefreshLevels::AttributesAndValues;
    }

    AZStd::vector<AZStd::string> OpenXRActionPathDescriptor::GetComponentPaths() const
    {
        AZStd::vector<AZStd::string> retList;
        auto interactionProviderIface = OpenXRInteractionProfilesProviderInterface::Get();
        if (!interactionProviderIface)
        {
            return retList;
        }

        const auto* profileDescriptor = interactionProviderIface->GetInteractionProfileDescriptor(m_interactionProfileName);
        if (!profileDescriptor)
        {
            return retList;
        }
        const auto* userPathDescriptor = profileDescriptor->GetUserPathDescriptor(m_userPathName);
        if (!userPathDescriptor)
        {
            return retList;
        }

        for (const auto& componentPath : userPathDescriptor->m_componentPathDescriptors)
        {
            retList.push_back(componentPath.m_name);
        }

        for (const auto& componentPath : profileDescriptor->m_commonComponentPathDescriptors)
        {
            retList.push_back(componentPath.m_name);
        }

        return retList;
    }
    /// OpenXRActionPathDescriptor
    ///////////////////////////////////////////////////////////
    

    ///////////////////////////////////////////////////////////
    /// OpenXRActionDescriptor
    void OpenXRActionDescriptor::Reflect(AZ::ReflectContext* context)
    {
        OpenXRActionPathDescriptor::Reflect(context);

        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRActionDescriptor>()
                ->Version(1)
                ->Field("Name", &OpenXRActionDescriptor::m_name)
                ->Field("LocalizedName", &OpenXRActionDescriptor::m_localizedName)
                ->Field("ActionPathDescriptors", &OpenXRActionDescriptor::m_actionPathDescriptors)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRActionDescriptor>("OpenXRActionDescriptor", "An action bound to one or more OpenXR Action Paths.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::NameLabelOverride, &OpenXRActionDescriptor::GetEditorText)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionDescriptor::m_name, "Name", "Runtime identifier for this action.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ_CRC("RefreshAttributesAndValues"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionDescriptor::m_localizedName, "Localized Name", "User friendly display name.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ_CRC("RefreshAttributesAndValues"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionDescriptor::m_actionPathDescriptors, "Action Paths", "List of action paths bound to this action")
                    ;
            }
        }
    }

    AZStd::string OpenXRActionDescriptor::GetEditorText() const
    {
        if (!m_localizedName.empty())
        {
            return m_localizedName;
        }
        return m_name.empty() ? "<Unknown Action>" : m_name;
    }
    /// OpenXRActionDescriptor
    ///////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////////////
    /// OpenXRActionSetDescriptor
    void OpenXRActionSetDescriptor::Reflect(AZ::ReflectContext* context)
    {
        OpenXRActionDescriptor::Reflect(context);

        AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context);
        if (serialize)
        {
            serialize->Class<OpenXRActionSetDescriptor>()
                ->Version(1)
                ->Field("Name", &OpenXRActionSetDescriptor::m_name)
                ->Field("LocalizedName", &OpenXRActionSetDescriptor::m_localizedName)
                ->Field("Priority", &OpenXRActionSetDescriptor::m_priority)
                ->Field("ActionDescriptors", &OpenXRActionSetDescriptor::m_actionDescriptors)
                ;

            AZ::EditContext* edit = serialize->GetEditContext();
            if (edit)
            {
                edit->Class<OpenXRActionSetDescriptor>("OpenXRActionSetDescriptor", "A group of OpenXR Actions that can be selectively activated/deactivated at runtime.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->Attribute(AZ::Edit::Attributes::NameLabelOverride, &OpenXRActionSetDescriptor::GetEditorText)
                        ->Attribute(AZ::Edit::Attributes::AutoExpand, true)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionSetDescriptor::m_name, "Name", "Runtime identifier for this action set.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ_CRC("RefreshAttributesAndValues"))
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionSetDescriptor::m_localizedName, "Localized Name", "Action set display name.")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, AZ_CRC("RefreshAttributesAndValues"))
                    ->DataElement(AZ::Edit::UIHandlers::SpinBox, &OpenXRActionSetDescriptor::m_priority, "Priority", "The higher this value the higher the priority.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &OpenXRActionSetDescriptor::m_actionDescriptors, "Actions", "List of actions for this action set.")
                    ;
            }
        }
    }

    AZStd::string OpenXRActionSetDescriptor::GetEditorText() const
    {
        if (!m_localizedName.empty())
        {
            return m_localizedName;
        }
        return m_name.empty() ? "<Unknown Action Set>" : m_name;
    }
    /// OpenXRActionSetDescriptor
    ///////////////////////////////////////////////////////////

} // namespace OpenXRVk
