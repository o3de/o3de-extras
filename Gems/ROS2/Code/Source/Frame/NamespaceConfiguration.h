/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    /// Configuration for handling of namespaces. Namespaces are useful for various ROS 2 components.
    struct NamespaceConfiguration
    {
    public:
        AZ_TYPE_INFO(NamespaceConfiguration, "{5E5BC6EA-DD01-480E-A4D1-6857CF70FDC8}");
        static void Reflect(AZ::ReflectContext* context);

        enum NamespaceStrategy
        {
            Default, // FromEntityName for top-level frames, Empty otherwise
            Empty,
            FromEntityName,
            Custom   // Non-empty but unrelated to entity name
        };
        
        void PopulateNamespace(bool isRoot, AZStd::string entityName);
        AZStd::string GetNamespace(const AZStd::string& parentNamespace) const;

    private:
        AZStd::string m_namespace;
        NamespaceStrategy m_namespaceStrategy = NamespaceStrategy::Default;
        bool m_isRoot;
        AZStd::string m_entityName;

        bool IsNamespaceCustom() const;
        void UpdateNamespace();
        AZ::Crc32 OnNamespaceStrategySelected();
    };
}  // namespace ROS2

