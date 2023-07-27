/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/string/string.h>

#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Types.hh>

namespace ROS2::SDFormat
{
    //! Class for parsing SDFormat data.
    namespace Parser
    {
        //! Parse string with SDFormat data and generate model. If the file is a URDF
        //! file it is converted to SDF first.
        //! @param xmlString a string that contains SDFormat data (XML format).
        //! @param parsingLog is a string to which parsing errors will be appended.
        //! @return model represented as a tree of parsed links.
        AZStd::shared_ptr<sdf::Root> Parse(const AZStd::string& xmlString, AZStd::string& parsingLog);

        //! Parse string with SDFormat data and generate model. If the file is a URDF
        //! file it is converted to SDF first.
        //! @param xmlString a string that contains SDFormat data (XML format).
        //! @return model represented as a tree of parsed links.
        AZStd::shared_ptr<sdf::Root> Parse(const AZStd::string& xmlString);

        //! Parse file with SDFormat data and generate model. If the file is a URDF
        //! file it is converted to SDF first.
        //! @param filePath is a path to file with SDFormat data that will be loaded and parsed.
        //! @param parsingLog is a string to which parsing errors will be appended.
        //! @return model represented as a tree of parsed links.
        AZStd::shared_ptr<sdf::Root> ParseFromFile(const AZStd::string& filePath, AZStd::string& parsingLog);

        //! Parse file with SDFormat data and generate model. If the file is a URDF
        //! file it is converted to SDF first.
        //! @param filePath is a path to file with SDFormat data that will be loaded and parsed.
        //! @return model represented as a tree of parsed links.
        AZStd::shared_ptr<sdf::Root> ParseFromFile(const AZStd::string& filePath);
    }; // namespace Parser
} // namespace ROS2::SDFormat
