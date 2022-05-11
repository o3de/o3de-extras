/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/string/string.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/RTTI/TypeInfo.h>

namespace ROS2
{
    class ROS2Names
    {
    public:
        static AZStd::string GetNamespacedName(const AZStd::string& ns, const AZStd::string& name);
        static AZStd::string RosifyName(const AZStd::string& input);

        // Validate namespace. One signature is straightforward, the other fits ChangeValidate for Editor fields.
        static AZ::Outcome<void, AZStd::string> ValidateNamespace(const AZStd::string& ros2Namespace);
        static AZ::Outcome<void, AZStd::string> ValidateNamespaceField(void* newValue, const AZ::Uuid& valueType);

        static AZ::Outcome<void, AZStd::string> ValidateTopic(const AZStd::string& topic);
        static AZ::Outcome<void, AZStd::string> ValidateTopicField(void* newValue, const AZ::Uuid& valueType);
    };
}  // namespace ROS2