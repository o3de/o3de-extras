/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Nodes/OneHot.h>
#include <Algorithms/Activations.h>

namespace MachineLearning
{
    AZ::VectorN OneHot::In(AZStd::size_t Value, AZStd::size_t MaxValue)
    {
        AZ::VectorN result;
        OneHotEncode(Value, MaxValue, result);
        return result;
    }
}
