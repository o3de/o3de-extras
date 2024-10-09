/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <MachineLearning/Types.h>

namespace MachineLearning
{
    //! This is a heavier weight context suitable for backpropagation and training of models.
    struct ITrainingContext
    {
        virtual ~ITrainingContext() = default;
    };

    using ITrainingContextPtr = ITrainingContext*;
}
