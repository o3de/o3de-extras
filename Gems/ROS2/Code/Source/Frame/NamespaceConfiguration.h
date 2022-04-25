/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
*/

#pragma once

#include <AzCore/std/string/string.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    /// Configuration for handling of namespaces
    struct NamespaceConfiguration
    {
    public:
        AZ_RTTI(NamespaceConfiguration, "{5E5BC6EA-DD01-480E-A4D1-6857CF70FDC8}");
        NamespaceConfiguration() = default;
        virtual ~NamespaceConfiguration() = default;

        enum NamespaceStrategy
        {
            Default, // FromEntityName for top-level frames, Empty otherwise
            Empty,
            FromEntityName,
            Custom   // Non-empty but unrelated to entity name
        };

        void PopulateNamespace(bool isRoot, AZStd::string entityName);
        AZStd::string GetNamespace(const AZStd::string& parentNamespace) const;

        static void Reflect(AZ::ReflectContext* context);

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

