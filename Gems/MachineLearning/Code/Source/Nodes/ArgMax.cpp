/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Nodes/ArgMax.h>
#include <Algorithms/Activations.h>

namespace MachineLearning
{
    AZStd::size_t ArgMax::In(AZ::VectorN Value)
    {
        return ArgMaxDecode(Value);
    }
}
