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
    //! @param link pointer to URDF/SDF link that root of robot description
    //! @param t initial transform, should be identity for non-recursive call.
    //! @returns root to entity transform
    AZ::Transform GetWorldTransformURDF(const sdf::Link* link, AZ::Transform t = AZ::Transform::Identity());

    //! Type Alias representing a "stack" of Model object that were visited on the way to the current Link/Joint Visitor Callback
    using ModelStack = AZStd::deque<AZStd::reference_wrapper<const sdf::Model>>;

    //! Callback which is invoke for each link within a model
    //! @param link reference to link being visited
    //! @param modelStack stack of references to the models and nested models that were visited to get to the current link. The stack will
    //! always contain at least one element. The back() element of the stack returns the direct model the link is attached to
    //! @return Return true to continue visiting links or false to halt
    using LinkVisitorCallback = AZStd::function<bool(const sdf::Link& link, const ModelStack& modelStack)>;
    //! Visit links from URDF/SDF
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query link
    //! @param visitNestedModelLinks When true recurses to any nested <model> tags of the Model object and invoke visitor on their links as
    //! well
    //! @returns void
    void VisitLinks(const sdf::Model& sdfModel, const LinkVisitorCallback& linkVisitorCB, bool visitNestedModelLinks = false);

    //! Retrieve all links in URDF/SDF as a map, where a key is link's fully qualified name and a value is a pointer to link.
    //! Allows to retrieve a pointer to a link given it name.
    //! @param sdfModel object of SDF document corresponding to the <model> tag. It used to query links
    //! @param gatherNestedModelLinks When true recurses to any nested <model> tags of the Model object and also gathers their links as well
    //! @returns mapping from fully qualified link name(such as "model_name::link_name") to link pointer
    AZStd::unordered_map<AZStd::string, const sdf::Link*> GetAllLinks(const sdf::Model& sdfModel, bool gatherNestedModelLinks = false);

    //! Callback which is invoke for each valid joint for a given model
    //! @param joint reference to joint being visited
    //! @param modelStack stack of references to the models and nested models that were visited to get to the current joint. The stack will
    //! always contain at least one element. The back() element of the stack returns the direct model the joint is attached to
    //! @return Return true to continue visiting joint or false to halt
    using JointVisitorCallback = AZStd::function<bool(const sdf::Joint& joint, const ModelStack& modelStack)>;
    //! Visit joints from URDF/SDF
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param visitNestedModelJoints When true recurses to any nested <model> tags of the Model object and invoke visitor on their joint as well
    //! @returns void
    void VisitJoints(const sdf::Model& sdfModel, const JointVisitorCallback& jointVisitorCB, bool visitNestedModelJoints = false);

    //! Retrieve all joints in URDF/SDF where the key is the full composed path following the name scoping proposal in SDF 1.8
    //! http://sdformat.org/tutorials?tut=composition_proposal#1-nesting-and-encapsulation
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param gatherNestedModelJoints When true recurses to any nested <model> tags of the Model object and also gathers their joints as well
    //! @returns mapping from fully qualified joint name(such as "model_name::joint_name") to joint pointer
    AZStd::unordered_map<AZStd::string, const sdf::Joint*> GetAllJoints(const sdf::Model& sdfModel, bool gatherNestedModelJoints = false);

    //! Retrieve all joints from URDF/SDF in which the specified link is a child in a sdf::Joint.
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param linkName Name of link which to query in joint objects ChildName()
    //! @param gatherNestedModelJoints When true recurses to any nested <model> tags of the Model object and also gathers their joints as well
    //! @returns vector of joints where link is a child
    AZStd::vector<const sdf::Joint*> GetJointsForChildLink(
        const sdf::Model& sdfModel, AZStd::string_view linkName, bool gatherNestedModelJoints = false);

    //! Retrieve all joints from URDF/SDF in which the specified link is a parent in a sdf::Joint.
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param linkName Name of link which to query in joint objects ParentName()
    //! @param gatherNestedModelJoints When true recurses to any nested <model> tags of the Model object and also gathers their joints as well
    //! @returns vector of joints where link is a parent
    AZStd::vector<const sdf::Joint*> GetJointsForParentLink(
        const sdf::Model& sdfModel, AZStd::string_view linkName, bool gatherNestedModelJoints = false);

    //! Visitation Enum to determine if visiting models should halt, continue to visit sibling models or continue to visit
    //! sibling and nested models of the current model
    enum class VisitModelResponse
    {
        //! Visit any nested model of the current model, and then continue to visit sibling models
        VisitNestedAndSiblings,
        //! If returned, sibling <model> to the current model will be visited,
        //! but not any nested children
        VisitSiblings,
        //! Stop visitation of future sibling <model> tags or nested <model> to the current model
        Stop,
    };

    //! Callback which is invoke for each model in the SDF
    //! This function visits any <model> tags in the root of the SDF XML content
    //! as well as any <model> tags in any <world> tags that are in the root of the SDF XML content
    //! @param model reference to SDF model object being visited
    //! @param modelStack stack of references to the models were visited to get to the current model
    //! NOTE: Unlike the Link/Joint visitor the modelStack can be empty (i.e for the root model or a model that is a child of a <world>)
    //! @return Return true to continue visiting models or false to halt
    using ModelVisitorCallback = AZStd::function<VisitModelResponse(const sdf::Model& model, const ModelStack& modelStack)>;
    //! Visit Models from URDF/SDF
    //! @param sdfRoot Root object of SDF document
    //! @param visitNestedModels When true recurses to any nested <model> tags of the Model objects and invoke the visitor on them
    //! @returns void
    void VisitModels(const sdf::Root& sdfRoot, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels = true);

    //! Retrieve all meshes referenced in URDF as unresolved URDF patches.
    //! Note that returned filenames are unresolved URDF patches.
    //! @param root reference to SDF Root object representing the root of the parsed SDF xml document
    //! @param visual search for visual meshes.
    //! @param colliders search for collider meshes.
    //! @returns set of meshes' filenames.
    AZStd::unordered_set<AZStd::string> GetMeshesFilenames(const sdf::Root& root, bool visual, bool colliders);

    //! Returns the SDF model object which contains the specified link
    //! @param root reference to SDF Root object representing the root of the parsed SDF xml document
    //! @param linkName Fully qualified name of SDF link to lookup in the SDF document
    //! A fully qualified name has the form "modelname1::nested_modelname1::linkname"
    //! This is detailed in the SDF format composition proposal: http://sdformat.org/tutorials?tut=composition_proposal#motivation
    const sdf::Model* GetModelContainingLink(const sdf::Root& root, AZStd::string_view linkName);

    //! @param root reference to SDF Root object representing the root of the parsed SDF xml document
    //! @param link SDF link object to lookup in the SDF document
    const sdf::Model* GetModelContainingLink(const sdf::Root& root, const sdf::Link& link);

    //! Returns the SDF model object which contains the specified joint
    //! @param root reference to SDF Root object representing the root of the parsed SDF xml document
    //! @param jointName Name of SDF joint to lookup in the SDF document
    //! A fully qualified name has the form "modelname1::nested_modelname1::jointname"
    //! This is detailed in the SDF format composition proposal: http://sdformat.org/tutorials?tut=composition_proposal#motivation
    const sdf::Model* GetModelContainingJoint(const sdf::Root& root, AZStd::string_view jointName);

    //! @param root reference to SDF Root object representing the root of the parsed SDF xml document
    //! @param joint SDF joint pointer to lookup in the SDF document
    const sdf::Model* GetModelContainingJoint(const sdf::Root& root, const sdf::Joint& joint);

    //! Callback used to check for file exist of a path referenced within a URDF/SDF file
    //! @param path Candidate local filesystem path to check for existence
    //! @return true should be returned if the file exist otherwise false
    using FileExistsCB = AZStd::function<bool(const AZ::IO::PathView&)>;

    //! Resolves path for an asset referenced in a URDF/SDF file.
    //! @param unresolvedPath unresolved URDF/SDF path, example : `model://meshes/foo.dae`.
    //! @param baseFilePath the absolute path of URDF/SDF file which contains the path that is to be resolved.
    //! @param amentPrefixPath the string that contains available packages' path, separated by ':' signs.
    //! @param settings the asset path resolution settings to use for attempting to locate the correct files
    //! @param fileExists functor to check if the given file exists. Exposed for unit test, default one should be used.
    //! @returns resolved path to the referenced file within the URDF/SDF, or the passed-in path if no resolution was possible.
    AZ::IO::Path ResolveAssetPath(
        AZ::IO::Path unresolvedPath,
        const AZ::IO::PathView& baseFilePath,
        AZStd::string_view amentPrefixPath,
        const SdfAssetBuilderSettings& settings,
        const FileExistsCB& fileExists = &Internal::FileExistsCall);

    using AmentPrefixString = AZStd::fixed_string<32768>;
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
    //! @param baseFilePath The base file getting parsed, which is used to help resolve file paths
    //! @return The output parser config to use with sdformat.
    sdf::ParserConfig CreateSdfParserConfigFromSettings(const SdfAssetBuilderSettings& settings, const AZ::IO::PathView& baseFilePath);

} // namespace ROS2::Utils::SDFormat
