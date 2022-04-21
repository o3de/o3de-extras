/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include "Frame/NamespaceConfiguration.h"
#include "Utilities/ROS2Names.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>

namespace ROS2
{
    void NamespaceConfiguration::PopulateNamespace(bool isRoot, const AZStd::string &entityName)
    {
        m_isRoot = isRoot;
        m_entityName = entityName;
        OnNamespaceStrategySelected();
    }

    void NamespaceConfiguration::UpdateNamespace()
    {
        auto nss = m_namespaceStrategy;
        if (NamespaceStrategy::Custom == nss)
        {   // Leave the name as it is
            return;
        }

        if (NamespaceStrategy::Empty == nss || (!m_isRoot && NamespaceStrategy::Default == nss))
        {
            m_namespace = "";
        }
        else if (NamespaceStrategy::FromEntityName == nss || (m_isRoot && NamespaceStrategy::Default == nss))
        {
            m_namespace = ROS2Names::RosifyName(m_entityName);
        }
    }

    AZ::Crc32 NamespaceConfiguration::OnNamespaceStrategySelected()
    {
        UpdateNamespace();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZStd::string NamespaceConfiguration::GetNamespace(const AZStd::string& parentNamespace) const
    {
        if (parentNamespace.empty())
        {
            return m_namespace;
        }

        if (m_namespace.empty())
        {
            return parentNamespace;
        }

        return ROS2Names::GetNamespacedName(parentNamespace, m_namespace);
    }

    bool NamespaceConfiguration::IsNamespaceCustom()
    {
        return m_namespaceStrategy == NamespaceConfiguration::NamespaceStrategy::Custom;
    }

    void NamespaceConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<NamespaceConfiguration>()
                ->Version(1)
                ->Field("Namespace Strategy", &NamespaceConfiguration::m_namespaceStrategy)
                ->Field("Namespace", &NamespaceConfiguration::m_namespace)
                ;

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<NamespaceConfiguration>("Namespace Configuration", "Handles ROS2 namespaces")
                    ->DataElement(AZ::Edit::UIHandlers::ComboBox, &NamespaceConfiguration::m_namespaceStrategy,
                            "Namespace strategy", "Determines how namespace for frames and topics is created in hierarchy")
                        ->Attribute(AZ::Edit::Attributes::ChangeNotify, &NamespaceConfiguration::OnNamespaceStrategySelected)
                        ->EnumAttribute(NamespaceConfiguration::NamespaceStrategy::Default, "Default")
                        ->EnumAttribute(NamespaceConfiguration::NamespaceStrategy::Empty, "Empty")
                        ->EnumAttribute(NamespaceConfiguration::NamespaceStrategy::FromEntityName, "Generate from entity name")
                        ->EnumAttribute(NamespaceConfiguration::NamespaceStrategy::Custom, "Custom")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NamespaceConfiguration::m_namespace, "Namespace", "Namespace")
                        ->Attribute(AZ::Edit::Attributes::Visibility, &NamespaceConfiguration::IsNamespaceCustom)
                        // TODO - hide for now, but desired Editor component behavior would be to show a read only value
                        ;
            }
        }
    }
}  // namespace ROS2

