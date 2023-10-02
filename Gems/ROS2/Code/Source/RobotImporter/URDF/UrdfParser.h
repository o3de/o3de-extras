/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/utility/expected.h>
#include <RobotImporter/FixURDF/URDFModifications.h>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>
#include <sdf/Collision.hh>
#include <sdf/Geometry.hh>
#include <sdf/Joint.hh>
#include <sdf/JointAxis.hh>
#include <sdf/Link.hh>
#include <sdf/Material.hh>
#include <sdf/Mesh.hh>
#include <sdf/Model.hh>
#include <sdf/Root.hh>
#include <sdf/Visual.hh>

namespace ROS2::UrdfParser
{
    //! Functions for parsing URDF/SDF data.

    //! Stores the result of parsing URDF/SDF data
    //! The operator bool returns true if the sdf::Errors vector is empty
    struct ParseResult
    {
    private:
        // Provides custom sdf::ErrorCode values when parsing is done through O3DE
        inline static constexpr auto O3DESdfErrorCodeStart = static_cast<sdf::ErrorCode>(1000);

    public:
        inline static constexpr auto O3DESdfErrorParseNotStarted = static_cast<sdf::ErrorCode>(static_cast<int>(O3DESdfErrorCodeStart) + 1);

        //! Ref qualifier overloads for retrieving sdf::Root
        //! it supports a non-const lvalue overload to allow
        //! modification of the sdf::Root object directly
        sdf::Root& GetRoot() &;
        const sdf::Root& GetRoot() const&;
        sdf::Root&& GetRoot() &&;

        //! Property getters for retrieving parsing messages
        //! logged during libsdformat parsing of the URDF content
        //! This does not support a non-const lvalue overload
        //! As modification the result structure parser messages
        //! have no need to be modified inplace
        const AZStd::string& GetParseMessages() const&;
        AZStd::string&& GetParseMessages() &&;

        //! Returns the sdf::Error vector containing any parser errors
        //! This does not support a non-const lvalue overload
        //! As modification the result structure sdf Errors
        //! have no need to be modified inplace
        const sdf::Errors& GetSdfErrors() const&;
        sdf::Errors&& GetSdfErrors() &&;

        //! Returns if the parsing of the SDF file has succeeded
        //! This will return false if the sdf::Errors vector is non-empty
        explicit operator bool() const;

        //! Returns if the URDF content was modified during parsing
        bool UrdfParsedWithModifiedContent() const;

        //! Root SDF object containing parsed SDF XML content if successful
        sdf::Root m_root;
        //! Contains any messages that are output to the sdf::Console during parsing
        //! These messages are captured and provided to the caller to allow outputting in
        //! any manner they choose
        AZStd::string m_parseMessages;
        //! Vector structure populated with sdf::Error objects indicating that parsing of the .sdf/.urdf
        //! file has failed with an error
        sdf::Errors m_sdfErrors{ sdf::Error{ O3DESdfErrorParseNotStarted, std::string{ "No Parsing has occurred yet" } } };

        //! Stores the modified URDF content after parsing, empty if no modification occurred
        std::string m_modifiedURDFContent;

        //! Stores description of URDF modifications, empty if no modification occurred
        Utils::UrdfModifications m_urdfModifications;
    };
    using RootObjectOutcome = ParseResult;

    //! Parse string with URDF/SDF data and generate root SDF object.
    //! @param xmlString a string that contains URDF data (XML format).
    //! @param parserConfig structure that contains configuration options for the SDFormatter parser
    //!        The relevant ParserConfig functions for URDF importing are
    //!        URDFPreserveFixedJoint() function to prevent merging of robot links bound by fixed joint
    //!        AddURIPath() function to provide a mapping of package:// and model:// references to the local filesystem
    //! @return SDF root object containing parsed <world> or <model> tags
    RootObjectOutcome Parse(AZStd::string_view xmlString, const sdf::ParserConfig& parserConfig);
    RootObjectOutcome Parse(const std::string& xmlString, const sdf::ParserConfig& parserConfig);

    //! Parse file with URDF data and generate model.
    //! @param filePath is a path to file with URDF data that will be loaded and parsed.
    //! @param parserConfig structure that contains configuration options for the SDFormater parser
    //!        The relevant ParserConfig functions for URDF importing are
    //!        URDFPreserveFixedJoint() function to prevent merging of robot links bound by fixed joint
    //!        AddURIPath() function to provide a mapping of package:// and model:// references to the local filesystem
    //! @param settings structure that contains configuration options for the SDFAssetBuilder
    //! @return SDF root object containing parsed <world> or <model> tags
    RootObjectOutcome ParseFromFile(
        AZ::IO::PathView filePath, const sdf::ParserConfig& parserConfig, const SdfAssetBuilderSettings& settings);
} // namespace ROS2::UrdfParser
