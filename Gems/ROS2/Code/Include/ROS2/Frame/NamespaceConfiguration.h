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
    //! Configuration for handling of namespaces.
    //! Namespaces are useful for various ROS2 components. This structure encapsulates the namespace itself,
    //! composing namespaces and context-dependent default values.
    //! @note This structure is handled through ROS2FrameComponent.
    struct NamespaceConfiguration
    {
    public:
        AZ_TYPE_INFO(NamespaceConfiguration, "{5E5BC6EA-DD01-480E-A4D1-6857CF70FDC8}");
        static void Reflect(AZ::ReflectContext* context);

        //! A choice of methods to handle namespaces.
        //! @note Top level ROS2FrameComponent will likely be associated with an interesting object (robot). For multi-robot
        //! simulations, namespaces are often derived from the robot name itself. For this reason, the default behavior
        //! for top level ROS2FrameComponent is to generate the namespace from entity name.
        enum class NamespaceStrategy
        {
            Default, //!< FromEntityName for top-level frames, Empty otherwise.
            Empty, //!< An empty, blank namespace
            FromEntityName, //!< Generate from Entity name, but substitute disallowed characters through RosifyName.
            Custom //!< Non-empty and based on user-provided value.
        };

        //! Set namespace based on context.
        //! @param isRoot Whether or not the namespace belongs to top-level entity in the entity hierarchy.
        //! @param entityName Raw (not ros-ified) name of the entity to which the namespace belongs.
        void PopulateNamespace(bool isRoot, const AZStd::string& entityName);

        //! Get the namespace of the frame, based on the parent namespace from the ROS2FrameSystemComponent.
        //! @return namespace of the frame.
        AZStd::string GetNamespace() const;

        //! Get the namespace of the frame, based on the provided namespace.
        //! @param parentNamespace namespace of the parent frame.
        //! @return namespace of the frame.
        AZStd::string GetNamespace(const AZStd::string& parentNamespace) const;

        //! Update namespace and strategy.
        //! @param ros2Namespace Desired namespace.
        //! @param strategy Namespace strategy.
        void SetNamespace(const AZStd::string& ros2Namespace, NamespaceStrategy strategy);

        //! Update the parents namespace.
        //! @param parentNamespace parent namespace.
        void SetParentNamespace(const AZStd::string& parentNamespace);

    private:
        AZStd::string m_namespace;
        AZStd::string m_parentNamespace;
        NamespaceStrategy m_namespaceStrategy = NamespaceStrategy::Default;
        bool m_isRoot;
        AZStd::string m_entityName;

        //! Determine if namespace is using the Custom namespace strategy
        bool IsNamespaceCustom() const;

        //! Update the namespace based on the current attributes
        void UpdateNamespace();

        AZ::Crc32 OnNamespaceStrategySelected();
    };
} // namespace ROS2
