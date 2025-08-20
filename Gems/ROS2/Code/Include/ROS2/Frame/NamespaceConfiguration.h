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
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    //! Configuration for handling ROS2 namespaces.
    //!
    //! Namespaces are essential for organizing ROS2 components, especially in multi-robot
    //! simulations. This structure encapsulates namespace management, composition,
    //! and context-dependent default values.
    //!
    //! @note This structure is managed through ROS2FrameComponent and provides
    //!       flexible namespace strategies for different use cases.
    struct NamespaceConfiguration
    {
    public:
        AZ_TYPE_INFO(NamespaceConfiguration, NamespaceConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        //! Strategy for determining namespace values.
        //! @note For multi-robot simulations, top-level ROS2FrameComponents typically
        //!       represent robots and their namespaces are often derived from entity names.
        //!       This is why the default behavior for top-level frames is FromEntityName.
        enum class NamespaceStrategy
        {
            Default, //!< FromEntityName for top-level frames, Empty for others
            Empty, //!< Empty namespace
            FromEntityName, //!< Generate from entity name using ROS2 naming conventions
            Custom //!< User-defined namespace
        };

        //! Initialize namespace based on entity context.
        //! @param isRoot Whether this namespace belongs to a top-level entity
        //! @param entityName Raw entity name (will be converted to ROS2 naming conventions)
        void PopulateNamespace(bool isRoot, const AZStd::string& entityName);

        //! Get the complete namespace including parent namespaces.
        //! @return Full namespace path from the namespace hierarchy
        AZStd::string GetNamespace() const;

        //! Get the namespace with explicit parent namespace.
        //! @param parentNamespace The parent namespace to use
        //! @return Combined namespace including the provided parent
        AZStd::string GetNamespace(const AZStd::string& parentNamespace) const;

        //! Update namespace and strategy simultaneously.
        //! @param ros2Namespace The desired namespace value
        //! @param strategy The namespace strategy to apply
        void SetNamespace(const AZStd::string& ros2Namespace, NamespaceStrategy strategy);

        //! Set the parent namespace.
        //! @param parentNamespace The namespace from parent frame
        void SetParentNamespace(const AZStd::string& parentNamespace);

        //! Initialize the namespace configuration.
        //! @note This should be called in component Init() methods
        void Init();

    private:
        AZStd::string m_customNamespace; //!< User-defined namespace
        AZStd::string m_namespace; //!< Current effective namespace
        AZStd::string m_parentNamespace; //!< Parent namespace from hierarchy
        NamespaceStrategy m_namespaceStrategy = NamespaceStrategy::Default;
        bool m_isRoot = false; //!< Whether this is a root-level frame
        AZStd::string m_entityName; //!< Raw entity name

        //! Check if using custom namespace strategy.
        bool IsNamespaceCustom() const;

        //! Update namespace based on current strategy and context.
        void UpdateNamespace();

        //! UI callback for strategy selection.
        AZ::Crc32 OnNamespaceStrategySelected();

        //! Validation callback for namespace input.
        AZ::Outcome<void, AZStd::string> ValidateNamespaceField(void* newValue, const AZ::Uuid& valueType);
    };
} // namespace ROS2
