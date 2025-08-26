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
#include <ROS2/ROS2NamesBus.h>

namespace ROS2
{

    AZ::Outcome<void, AZStd::string> NamespaceConfiguration::ValidateNamespaceField(void* newValue, const AZ::Uuid& valueType)
    {
        AZ::Outcome<void, AZStd::string> outcome;
        ROS2NamesRequestBus::BroadcastResult(outcome, &ROS2NamesRequests::ValidateNamespaceField, newValue, valueType);
        return outcome;
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
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &NamespaceConfiguration::ValidateNamespaceField);
            }
        }
    }
} // namespace ROS2
