/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/XML/rapidxml.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    namespace Utils
    {
        //! Modifies in memory URDF to add missing inertia to links, circle navigate SDF error 19
        //! @param urdf - the in memory URDF to modify
        //! @returns a list of links that were modified
        AZStd::vector<AZStd::string> AddMissingInertiaToLink(AZ::rapidxml::xml_node<>* urdf);

        //! Modifies names of links and joints to be unique, circle navigate SDF error 2
        //! @param urdf - the in memory URDF to modify
        //! @returns a list of links that were modified
        AZStd::vector<AZStd::string> ChangeDuplications(AZ::rapidxml::xml_node<>* urdf);

        //! Modifies in memory URDF to add missing elements
        //! @param urdf - the in memory URDF to modify
        //! @returns a modified URDF and a list of XML element that were modified
        AZStd::pair<std::string, AZStd::vector<AZStd::string>> ModifyURDFInMemory(const std::string& data);

    } // namespace Utils
} // namespace ROS2
