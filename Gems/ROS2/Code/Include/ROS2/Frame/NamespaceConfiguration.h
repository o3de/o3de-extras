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
    //! Configuration for handling of namespaces.
    //! Namespaces are useful for various ROS2 components. This structure encapsulates the namespace itself,
    //! composing namespaces and context-dependent default values.
    //! @note This structure is handled through ROS2FrameComponent.
    struct NamespaceConfiguration
    {
        AZ_TYPE_INFO(NamespaceConfiguration, NamespaceConfigurationTypeId);
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

        AZStd::string m_customNamespace = ""; //!< Custom namespace that can be set by the user
        NamespaceStrategy m_namespaceStrategy = NamespaceStrategy::Default;
    private:
        //! Determine if namespace is using the Custom namespace strategy
        bool IsNamespaceCustom() const;

        //! Helpers methods for UI
        AZ::Outcome<void, AZStd::string> ValidateNamespaceField(void* newValue, const AZ::Uuid& valueType);
    };
} // namespace ROS2
