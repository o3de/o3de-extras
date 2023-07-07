/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <ScriptCanvas/CodeGen/NodeableCodegen.h>
#include <ScriptCanvas/Core/Nodeable.h>
#include <ScriptCanvas/Core/NodeableNode.h>
#include <MachineLearning/INeuralNetwork.h>
#include <Source/Nodes/FeedForward.generated.h>

namespace MachineLearning
{
    class FeedForward
        : public ScriptCanvas::Nodeable
    {
        SCRIPTCANVAS_NODE_FeedForward;
    };
}
