/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/IO/SystemFile.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/function/function_template.h>
#include <AzCore/std/string/string.h>
#include <RobotImporter/URDF/UrdfParser.h>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

#include <sdf/sdf.hh>

namespace ROS2::Utils
{
    inline namespace Internal
    {
        bool FileExistsCall(const AZ::IO::PathView& filePath);
    } // namespace Internal
} // namespace ROS2::Utils

namespace ROS2::Utils
{
    //! Determine whether a given link is likely a wheel link.
    //! This can be useful to provide a good default behavior - for example, to add Vehicle Dynamics components to this link's entity.
    //! @param sdfModel Model object which is used to query the joints from SDF format data
    //! @param link the link that will be subjected to the heuristic.
    //! @return true if the link is likely a wheel link.
    bool IsWheelURDFHeuristics(const sdf::Model& model, const sdf::Link* link);

    //! The recursive function for the given link goes through URDF and finds world-to-entity transformation for us.
    //! @param link pointer to URDF link that root of robot description
    //! @param t initial transform, should be identity for non-recursive call.
    //! @returns root to entity transform
    AZ::Transform GetWorldTransformURDF(const sdf::Link* link, AZ::Transform t = AZ::Transform::Identity());

    //! Callback which is invoke for each link within a model
    //! @return Return true to continue visiting links or false to halt
    using LinkVisitorCallback = AZStd::function<bool(const sdf::Link&)>;
    //! Visit links from URDF or SDF
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query link
    //! @param visitNestedModelLinks When true recurses to any nested <model> tags of the Model object and invoke visitor on their links as well
    //! @returns void
    void VisitLinks(const sdf::Model& sdfModel, const LinkVisitorCallback& linkVisitorCB,
        bool visitNestedModelLinks = false);

    //! Retrieve all links in URDF as a map, where a key is link's name and a value is a pointer to link.
    //! Allows to retrieve a pointer to a link given it name.
    //! @param sdfModel object of SDF document corresponding to the <model> tag. It used to query links
    //! @param gatherNestedModelLinks When true recurses to any nested <model> tags of the Model object and also gathers their links as well
    //! @returns mapping from link name to link pointer
    AZStd::unordered_map<AZStd::string, const sdf::Link*> GetAllLinks(const sdf::Model& sdfModel,
        bool gatherNestedModelLinks = false);

    //! Callback which is invoke for each valid joint for a given model
    //! @return Return true to continue visiting joint or false to halt
    using JointVisitorCallback = AZStd::function<bool(const sdf::Joint&)>;
    //! Visit joints from URDF
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param visitNestedModelJoints When true recurses to any nested <model> tags of the Model object and invoke visitor on their joints as well
    //! @returns void
    void VisitJoints(const sdf::Model& sdfModel, const JointVisitorCallback& jointVisitorCB,
        bool visitNestedModelJoints = false);

    //! Retrieve all joints in URDF.
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param gatherNestedModelJoints When true recurses to any nested <model> tags of the Model object and also gathers their joints as well
    //! @returns mapping from joint name to joint pointer
    AZStd::unordered_map<AZStd::string, const sdf::Joint*> GetAllJoints(const sdf::Model& sdfModel,
        bool gatherNestedModelJoints = false);

    //! Retrieve all joints from URDF in which the specified link is a child in a sdf::Joint.
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param linkName Name of link which to query in joint objects ChildName()
    //! @param gatherNestedModelJoints When true recurses to any nested <model> tags of the Model object and also gathers their joints as well
    //! @returns vector of joints where link is a child
    AZStd::vector<const sdf::Joint*> GetJointsForChildLink(const sdf::Model& sdfModel, AZStd::string_view linkName,
        bool gatherNestedModelJoints = false);

    //! Retrieve all joints from URDF in which the specified link is a parent in a sdf::Joint.
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param linkName Name of link which to query in joint objects ParentName()
    //! @param gatherNestedModelJoints When true recurses to any nested <model> tags of the Model object and also gathers their joints as well
    //! @returns vector of joints where link is a parent
    AZStd::vector<const sdf::Joint*> GetJointsForParentLink(const sdf::Model& sdfModel, AZStd::string_view linkName,
        bool gatherNestedModelJoints = false);

    //! Retrieve all meshes referenced in URDF as unresolved URDF patches.
    //! Note that returned filenames are unresolved URDF patches.
    //! @param visual - search for visual meshes.
    //! @param colliders - search for collider meshes.
    //! @param rootLink - pointer to sdf Rootobject that corresponds to to root of robot description after it hasbeen converted from URDF to SDF
    //! @returns set of meshes' filenames.
    AZStd::unordered_set<AZStd::string> GetMeshesFilenames(const sdf::Root* root, bool visual, bool colliders);

    //! Callback used to check for file exist of a path referenced within a URDF/SDF file
    //! @param path Candidate local filesystem path to check for existence
    //! @return true should be returned if the file exist otherwise false
    using FileExistsCB = AZStd::function<bool(const AZ::IO::PathView&)>;

    //! Resolves path for an asset referenced in a URDF/SDF file.
    //! @param unresolvedPath - unresolved URDF/SDF path, example : `model://meshes/foo.dae`.
    //! @param baseFilePath - the absolute path of URDF/SDF file which contains the path that is to be resolved.
    //! @param amentPrefixPath - the string that contains available packages' path, separated by ':' signs.
    //! @param settings - the asset path resolution settings to use for attempting to locate the correct files
    //! @param fileExists - functor to check if the given file exists. Exposed for unit test, default one should be used.
    //! @returns resolved path to the referenced file within the URDF/SDF, or the passed-in path if no resolution was possible.
    AZ::IO::Path ResolveAssetPath(
        AZ::IO::Path unresolvedPath,
        const AZ::IO::PathView& baseFilePath,
        AZStd::string_view amentPrefixPath,
        const SdfAssetBuilderSettings& settings,
        const FileExistsCB& fileExists = &Internal::FileExistsCall);

    using AmentPrefixString = AZStd::fixed_string<4096>;
    AmentPrefixString GetAmentPrefixPath();
} // namespace ROS2::Utils

namespace ROS2::Utils::SDFormat
{
    //! Retrieve plugin's filename. The filepath is converted into the filename if necessary.
    //! @param plugin plugin in the parsed SDFormat data
    //! @returns filename (including extension) without path
    AZStd::string GetPluginFilename(const sdf::Plugin& plugin);

    //! Retrieve all parameters that were defined for an element in XML data that are not supported in O3DE.
    //! Allows to store the list of unsupported parameters in metadata and logs. It is typically used with sensors and plugins.
    //! @param rootElement pointer to a root Element in parsed XML data that will be a subject to heuristics
    //! @param supportedParams set of predefined parameters that are supported
    //! @returns list of unsupported parameters defined for given element
    AZStd::vector<AZStd::string> GetUnsupportedParams(
        const sdf::ElementPtr& rootElement, const AZStd::unordered_set<AZStd::string>& supportedParams);

    //! Check if plugin is supported by using it's filename. The filepath is converted into the filename if necessary.
    //! @param plugin plugin in the parsed SDFormat data
    //! @param supportedPlugins set of predefined plugins that are supported
    //! @returns true if plugin is supported
    bool IsPluginSupported(const sdf::Plugin& plugin, const AZStd::unordered_set<AZStd::string>& supportedPlugins);

    //! Given a set of SdfAssetBuilderSettings, produce an sdf::ParserConfig that can be used by the sdformat library.
    //! @param settings The input settings to use
    //! @return The output parser config to use with sdformat.
    sdf::ParserConfig CreateSdfParserConfigFromSettings(const SdfAssetBuilderSettings& settings);

} // namespace ROS2::Utils::SDFormat
