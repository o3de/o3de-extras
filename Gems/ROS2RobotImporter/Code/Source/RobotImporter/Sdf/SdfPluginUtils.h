/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

#include <sdf/sdf.hh>

namespace ROS2RobotImporter::Utils::SDFormat
{
    //! Retrieve plugin's filename. The filepath is converted into the filename if necessary.
    //! @param plugin plugin in the parsed SDFormat data
    //! @returns filename (including extension) without path
    AZStd::string GetPluginFilename(const sdf::Plugin& plugin);

    //! Retrieve all parameters that were defined for an element in XML data that are not supported in O3DE.
    //! Allows to store the list of unsupported parameters in metadata and logs. It is typically used with sensors and plugins.
    //! @param rootElement pointer to a root Element in parsed XML data that will be a subject to heuristics
    //! @param supportedParams set of predefined parameters that are supported
    //! @param supportedPlugins set of predefined plugins that are supported
    //! @param supportedPluginParams set of supported plugin params
    //! @returns list of unsupported parameters defined for given element
    AZStd::vector<AZStd::string> GetUnsupportedParams(
        const sdf::ElementPtr& rootElement,
        const AZStd::unordered_set<AZStd::string>& supportedParams,
        const AZStd::unordered_set<AZStd::string>& supportedPlugins = AZStd::unordered_set<AZStd::string>(),
        const AZStd::unordered_set<AZStd::string>& supportedPluginParams = AZStd::unordered_set<AZStd::string>());

    //! Check if plugin is supported by using it's filename. The filepath is converted into the filename if necessary.
    //! @param plugin plugin in the parsed SDFormat data
    //! @param supportedPlugins set of predefined plugins that are supported
    //! @returns true if plugin is supported
    bool IsPluginSupported(const sdf::Plugin& plugin, const AZStd::unordered_set<AZStd::string>& supportedPlugins);
} // namespace ROS2RobotImporter::Utils::SDFormat
