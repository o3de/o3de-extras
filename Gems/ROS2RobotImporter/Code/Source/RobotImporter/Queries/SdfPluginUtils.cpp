/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <RobotImporter/Queries/SdfPluginUtils.h>

#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/function/function_template.h>

namespace ROS2RobotImporter::SDFormat
{
    AZStd::string GetPluginFilename(const sdf::Plugin& plugin)
    {
        const AZ::IO::Path path{ plugin.Filename().c_str() };
        return path.Filename().String();
    }

    AZStd::vector<AZStd::string> GetUnsupportedParams(
        const sdf::ElementPtr& rootElement,
        const AZStd::unordered_set<AZStd::string>& supportedParams,
        const AZStd::unordered_set<AZStd::string>& pluginNames,
        const AZStd::unordered_set<AZStd::string>& supportedPluginParams)
    {
        AZStd::vector<AZStd::string> unsupportedParams;

        AZStd::function<void(
            const sdf::ElementPtr& elementPtr,
            const std::string& info,
            const std::string& prefix,
            const AZStd::unordered_set<AZStd::string>& supportedSet)>
            elementVisitor = [&](const sdf::ElementPtr& elementPtr,
                                 const std::string& info,
                                 const std::string& prefix,
                                 const AZStd::unordered_set<AZStd::string>& supportedSet) -> void
        {
            auto childPtr = elementPtr->GetFirstElement();

            AZStd::string prefixAz(prefix.c_str(), prefix.size());
            if (!childPtr && !prefixAz.empty() && !supportedSet.contains(prefixAz))
            {
                prefixAz.insert(0, AZStd::string(info.c_str(), info.size()));
                unsupportedParams.push_back(prefixAz);
            }

            while (childPtr)
            {
                if (childPtr->GetName() == "plugin")
                {
                    break;
                }

                std::string currentName = prefix;
                currentName.append(">");
                currentName.append(childPtr->GetName());

                elementVisitor(childPtr, info, currentName, supportedSet);
                childPtr = childPtr->GetNextElement();
            }
        };

        elementVisitor(rootElement, "", "", supportedParams);

        auto pluginPtr = rootElement->GetFirstElement();
        while (pluginPtr)
        {
            if (pluginPtr->GetName() == "plugin")
            {
                if (pluginPtr->HasAttribute("filename"))
                {
                    std::string fileName = pluginPtr->GetAttribute("filename")->GetAsString();
                    std::string info = "plugin \"" + fileName + "\"";

                    AZStd::string fileNameAz(fileName.c_str(), fileName.size());
                    if (!pluginNames.contains(fileNameAz))
                    {
                        unsupportedParams.push_back(AZStd::string(info.c_str(), info.size()));
                    }
                    else
                    {
                        info.append(": ");
                        elementVisitor(pluginPtr, info, "", supportedPluginParams);
                    }
                }
            }
            pluginPtr = pluginPtr->GetNextElement();
        }

        return unsupportedParams;
    }

    bool IsPluginSupported(const sdf::Plugin& plugin, const AZStd::unordered_set<AZStd::string>& supportedPlugins)
    {
        return supportedPlugins.contains(GetPluginFilename(plugin));
    }
} // namespace ROS2RobotImporter::SDFormat
