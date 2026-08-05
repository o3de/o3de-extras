/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Transform.h>

#include <sdf/sdf.hh>

namespace ROS2RobotImporter::Utils
{
    //! Determine whether a given link is likely a wheel link.
    //! This can be useful to provide a good default behavior - for example, to add Vehicle Dynamics components to this link's entity.
    //! @param sdfModel Model object which is used to query the joints from SDF format data
    //! @param link the link that will be subjected to the heuristic.
    //! @return true if the link is likely a wheel link.
    bool IsWheelHeuristics(const sdf::Model& model, const sdf::Link* link);

    //! Returns an AZ::Transform converted from the link pose defined relative to another frame.
    //! @param semanticPose pointer to URDF/SDF link
    //! @param t initial transform, multiplied against link transform
    //! @returns Transform of link
    AZ::Transform GetLocalTransform(const sdf::SemanticPose& semanticPose, AZ::Transform t = AZ::Transform::Identity());
} // namespace ROS2RobotImporter::Utils
