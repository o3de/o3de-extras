/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/base.h>
#include <AzCore/std/containers/deque.h>
#include <AzCore/std/function/function_template.h>
#include <AzCore/std/reference_wrapper.h>
#include <AzCore/std/typetraits/underlying_type.h>

#include <sdf/sdf.hh>

namespace ROS2RobotImporter::Utils
{
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

    //! Callback which is invoke for each valid joint for a given model
    //! @param joint reference to joint being visited
    //! @param modelStack stack of references to the models and nested models that were visited to get to the current joint. The stack will
    //! always contain at least one element. The back() element of the stack returns the direct model the joint is attached to
    //! @return Return true to continue visiting joint or false to halt
    using JointVisitorCallback = AZStd::function<bool(const sdf::Joint& joint, const ModelStack& modelStack)>;
    //! Visit joints from URDF/SDF
    //! @param sdfModel Model object of SDF document corresponding to the <model> tag. It used to query joints
    //! @param visitNestedModelJoints When true recurses to any nested <model> tags of the Model object and invoke visitor on their joint as
    //! well
    //! @returns void
    void VisitJoints(const sdf::Model& sdfModel, const JointVisitorCallback& jointVisitorCB, bool visitNestedModelJoints = false);

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

    //! Provides overloads for comparison operators for the VisitModelResponse enum
    AZ_DEFINE_ENUM_RELATIONAL_OPERATORS(VisitModelResponse);

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
    //! @param sdfWorld World object of SDF document corresponding to the <world> tag.
    //! @param visitNestedModels When true recurses to any nested <model> tags of the Model objects and invoke the visitor on them
    //! @returns void
    void VisitModels(const sdf::World& sdfWorld, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels = true);
    //! @param sdfModel Model object corresponding to a <model> tag in the SDF.
    //! @param visitNestedModels When true recurses to any nested <model> tags of the Model objects and invoke the visitor on them
    //! @returns void
    void VisitModels(const sdf::Model& sdfModel, const ModelVisitorCallback& modelVisitorCB, bool visitNestedModels = true);
} // namespace ROS2RobotImporter::Utils
