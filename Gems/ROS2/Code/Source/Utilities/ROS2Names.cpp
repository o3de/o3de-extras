/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/std/string/regex.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <rcl/validate_topic_name.h>
#include <rmw/validate_namespace.h>

namespace ROS2
{
    AZStd::string ROS2Names::GetNamespacedName(const AZStd::string& ns, const AZStd::string& name)
    {
        if (ns.empty())
        {
            return name;
        }

        return AZStd::string::format("%s/%s", ns.c_str(), name.c_str());
        ;
    }

    AZStd::string ROS2Names::RosifyName(const AZStd::string& input)
    {
        AZStd::string rosified = input;
        if (input.empty())
        {
            return rosified;
        }

        const char underscore = '_';
        if (input[0] == underscore)
        {
            AZ_Warning(
                "RosifyName",
                false,
                "'%s' name starts with an underscore, which makes topic/namespace/parameter hidden by default. Is this intended?",
                input.c_str());
        }

        const AZStd::string stringToReplaceViolations(1, underscore);
        const AZStd::regex ros2Disallowedlist("[^0-9|a-z|A-Z|_]");
        rosified = AZStd::regex_replace(rosified, ros2Disallowedlist, stringToReplaceViolations);

        if (AZStd::isdigit(rosified[0]) || (input[0] != underscore && rosified[0] == underscore))
        { // Prepend "o3de_" if it would otherwise start with a number (which would violate ros2 name requirements)
            // Also, starting with '_' is not desired unless explicit. Topics/namespaces/parameters starting with "_" are hidden by default.
            const AZStd::string prependToNumberStart = "o3de_";
            rosified = prependToNumberStart + rosified;
        }

        if (input != rosified)
        {
            AZ_TracePrintf(
                "RosifyName",
                "Name '%s' has been changed to '%s' to conform with ros2 naming restrictions\n",
                input.c_str(),
                rosified.c_str());
        }
        return rosified;
    }

    AZ::Outcome<void, AZStd::string> ROS2Names::ValidateNamespace(const AZStd::string& ros2Namespace)
    {
        auto ros2GlobalizedNamespace = ros2Namespace;
        const char namespacePrefix = '/';
        if (!ros2Namespace.starts_with(namespacePrefix))
        { // Prepend "/" if not included, this is done automatically by rclcpp so "/"-less namespaces are ok.
            ros2GlobalizedNamespace = namespacePrefix + ros2Namespace;
        }

        int validationResult = 0;
        auto ret = rmw_validate_namespace(ros2GlobalizedNamespace.c_str(), &validationResult, NULL);
        if (ret != RMW_RET_OK)
        {
            AZ_Error("ValidateNamespace", false, "Call to rmw validation for namespace failed");
            return AZ::Failure(AZStd::string("Unable to validate namespace due to rmw error"));
        }

        if (validationResult != RMW_NAMESPACE_VALID)
        {
            return AZ::Failure(AZStd::string(rmw_namespace_validation_result_string(validationResult)));
        }

        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> ROS2Names::ValidateNamespaceField(void* newValue, const AZ::Uuid& valueType)
    {
        if (azrtti_typeid<AZStd::string>() != valueType)
        {
            return AZ::Failure(AZStd::string("Unexpected field type: the only valid input is a character string"));
        }

        const AZStd::string& ros2Namespace(*reinterpret_cast<const AZStd::string*>(newValue));
        return ValidateNamespace(ros2Namespace);
    }

    AZ::Outcome<void, AZStd::string> ROS2Names::ValidateTopic(const AZStd::string& topic)
    {
        int validationResult = 0;
        [[maybe_unused]] size_t invalidIndex;
        if (rcl_validate_topic_name(topic.c_str(), &validationResult, &invalidIndex) != RCL_RET_OK)
        {
            AZ_Error("ValidateTopic", false, "Call to rcl validation for topic failed");
            return AZ::Failure(AZStd::string("Unable to validate topic due to rcl error"));
        }

        if (RCL_TOPIC_NAME_VALID != validationResult)
        {
            return AZ::Failure(AZStd::string(rcl_topic_name_validation_result_string(validationResult)));
        }

        return AZ::Success();
    }

    AZ::Outcome<void, AZStd::string> ROS2Names::ValidateTopicField(void* newValue, const AZ::Uuid& valueType)
    {
        if (azrtti_typeid<AZStd::string>() != valueType)
        {
            return AZ::Failure(AZStd::string("Unexpected field type: the only valid input is a character string"));
        }

        const AZStd::string& topic(*reinterpret_cast<const AZStd::string*>(newValue));
        return ValidateTopic(topic);
    }
} // namespace ROS2
