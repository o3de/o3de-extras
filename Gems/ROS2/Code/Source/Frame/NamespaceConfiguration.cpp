/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void NamespaceConfiguration::PopulateNamespace(bool isRoot, const AZStd::string& entityName)
    {
        m_isRoot = isRoot;
        m_entityName = entityName;
        OnNamespaceStrategySelected();
    }

    void NamespaceConfiguration::UpdateNamespace()
    {
        switch (m_namespaceStrategy)
        {
        case NamespaceStrategy::Empty:
            m_namespace = "";
            break;
        case NamespaceStrategy::Default:
            m_namespace = m_isRoot ? ROS2Names::RosifyName(m_entityName) : "";
            break;
        case NamespaceStrategy::FromEntityName:
            m_namespace = ROS2Names::RosifyName(m_entityName);
            break;
        case NamespaceStrategy::Custom:
            // Leave the namespace as is
            break;
        default:
            AZ_Assert(false, "Unhandled namespace strategy");
            break;
        }
    }

    AZ::Crc32 NamespaceConfiguration::OnNamespaceStrategySelected()
    {
        UpdateNamespace();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZStd::string NamespaceConfiguration::GetNamespace() const
    {
        if (m_parentNamespace.empty())
        {
            return m_namespace;
        }

        if (m_namespace.empty())
        {
            return m_parentNamespace;
        }

        return ROS2Names::GetNamespacedName(m_parentNamespace, m_namespace);
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

    void NamespaceConfiguration::SetNamespace(const AZStd::string& ros2Namespace, NamespaceStrategy strategy)
    {
        m_namespace = ros2Namespace;
        m_namespaceStrategy = strategy;
        UpdateNamespace();
    }

    void NamespaceConfiguration::SetParentNamespace(const AZStd::string& parentNamespace)
    {
        m_parentNamespace = parentNamespace;
        UpdateNamespace();
    }

    bool NamespaceConfiguration::IsNamespaceCustom() const
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
                ->Field("Namespace", &NamespaceConfiguration::m_namespace);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<NamespaceConfiguration>("Namespace Configuration", "Handles ROS2 namespaces")
                    ->DataElement(
                        AZ::Edit::UIHandlers::ComboBox,
                        &NamespaceConfiguration::m_namespaceStrategy,
                        "Namespace strategy",
                        "Determines how namespace for frames and topics is created from the hierarchy")
                    ->EnumAttribute(NamespaceConfiguration::NamespaceStrategy::Default, "Default")
                    ->EnumAttribute(NamespaceConfiguration::NamespaceStrategy::Empty, "Empty")
                    ->EnumAttribute(NamespaceConfiguration::NamespaceStrategy::FromEntityName, "Generate from entity name")
                    ->EnumAttribute(NamespaceConfiguration::NamespaceStrategy::Custom, "Custom")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NamespaceConfiguration::m_namespace, "Namespace", "Namespace")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &NamespaceConfiguration::IsNamespaceCustom)
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &ROS2Names::ValidateNamespaceField);
            }
        }
    }
} // namespace ROS2
