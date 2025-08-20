/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Outcome/Outcome.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/ROS2NamesBus.h>

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
            m_namespace.clear();
            break;

        case NamespaceStrategy::Default:
            if (m_isRoot)
            {
                ROS2NamesRequestBus::BroadcastResult(m_namespace, &ROS2NamesRequests::RosifyName, m_entityName);
            }
            else
            {
                m_namespace.clear();
            }
            break;

        case NamespaceStrategy::FromEntityName:
            ROS2NamesRequestBus::BroadcastResult(m_namespace, &ROS2NamesRequests::RosifyName, m_entityName);
            break;

        case NamespaceStrategy::Custom:
            m_namespace = m_customNamespace;
            break;

        default:
            AZ_Assert(false, "Unhandled namespace strategy: %d", static_cast<int>(m_namespaceStrategy));
            m_namespace.clear();
            break;
        }
    }

    AZ::Crc32 NamespaceConfiguration::OnNamespaceStrategySelected()
    {
        UpdateNamespace();
        return AZ::Edit::PropertyRefreshLevels::EntireTree;
    }

    AZ::Outcome<void, AZStd::string> NamespaceConfiguration::ValidateNamespaceField(void* newValue, const AZ::Uuid& valueType)
    {
        AZ::Outcome<void, AZStd::string> outcome = AZ::Failure<AZStd::string>("ROS2NameRequests EBus is not available");
        ROS2NamesRequestBus::BroadcastResult(outcome, &ROS2NamesRequests::ValidateNamespaceField, newValue, valueType);
        return outcome;
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

        AZStd::string namespacedName;
        ROS2NamesRequestBus::BroadcastResult(namespacedName, &ROS2NamesRequests::GetNamespacedName, m_parentNamespace, m_namespace);
        return namespacedName;
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

        AZStd::string namespacedName;
        ROS2NamesRequestBus::BroadcastResult(namespacedName, &ROS2NamesRequests::GetNamespacedName, parentNamespace, m_namespace);
        return namespacedName;
    }

    void NamespaceConfiguration::SetNamespace(const AZStd::string& ros2Namespace, NamespaceStrategy strategy)
    {
        m_namespaceStrategy = strategy;

        if (strategy == NamespaceStrategy::Custom)
        {
            m_customNamespace = ros2Namespace;
        }

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

    void NamespaceConfiguration::Init()
    {
        // Ensure namespace is properly initialized
        OnNamespaceStrategySelected();
    }

    void NamespaceConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<NamespaceConfiguration>()
                ->Version(1)
                ->Field("Namespace Strategy", &NamespaceConfiguration::m_namespaceStrategy)
                ->Field("Namespace", &NamespaceConfiguration::m_customNamespace);

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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &NamespaceConfiguration::m_customNamespace, "Namespace", "Namespace")
                    ->Attribute(AZ::Edit::Attributes::Visibility, &NamespaceConfiguration::IsNamespaceCustom)
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &NamespaceConfiguration::ValidateNamespaceField)
                    ->Attribute(AZ::Edit::Attributes::ChangeNotify, &NamespaceConfiguration::UpdateNamespace);
            }
        }
    }
} // namespace ROS2
