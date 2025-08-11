/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/EBus/EBus.h>
#include <AzCore/EBus/Policies.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! Utility bus for handling ROS 2 naming rules.
    class ROS2NamesRequests : public AZ::EBusTraits
    {
    public:
        // EBusTraits settings
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
        static const AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;

        //! Joins namespace and the given name.
        //! @param ns Namespace to join.
        //! @param name Name to join.
        //! @return Namespaced name in the format "namespace/name".
        virtual AZStd::string GetNamespacedName(const AZStd::string& ns, const AZStd::string& name) = 0;

        //! Converts input to a ROS 2-acceptable name for topics and namespaces.
        //! Any characters not fitting ROS 2 naming specification are replaced with underscores.
        //! @param input Input string to convert.
        //! @return Converted string.
        virtual AZStd::string RosifyName(const AZStd::string& input) = 0;

        //! Validate namespace adherence to ROS 2 specification. Delegates validation to ROS 2 layers.
        //! @param ros2Namespace Namespace to validate.
        //! @return AZ::Success if namespace is valid, AZ::Failure with error message otherwise
        virtual AZ::Outcome<void, AZStd::string> ValidateNamespace(const AZStd::string& ros2Namespace) = 0;

        //! Validate namespace field. Fits ChangeValidate for Editor fields.
        //! @param newValue Pointer to the new value to validate.
        //! @param valueType Type of the value to validate.
        //! @return AZ::Success if namespace field is valid, AZ::Failure with error message otherwise
        virtual AZ::Outcome<void, AZStd::string> ValidateNamespaceField(void* newValue, const AZ::Uuid& valueType) = 0;

        //! Validate topic adherence to ROS 2 specification.
        //! @param topic Topic to validate.
        //! @return AZ::Success if topic is valid, AZ::Failure with error message otherwise
        virtual AZ::Outcome<void, AZStd::string> ValidateTopic(const AZStd::string& topic) = 0;

        //! Validate topic field. Fits ChangeValidate for Editor fields.
        //! @param newValue Pointer to the new value to validate.
        //! @param valueType Type of the value to validate.
        //! @return AZ::Success if topic field is valid, AZ::Failure with error message
        virtual AZ::Outcome<void, AZStd::string> ValidateTopicField(void* newValue, const AZ::Uuid& valueType) = 0;
    };

    using ROS2NamesRequestBus = AZ::EBus<ROS2NamesRequests>;
} // namespace ROS2
