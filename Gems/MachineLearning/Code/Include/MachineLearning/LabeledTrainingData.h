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
    struct LabeledTrainingData
    {
        AZ_TYPE_INFO(LabeledTrainingData, "{50DF457E-3EAC-4114-8444-023E64973AD9}");

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(class AZ::ReflectContext* context);

        AZ::VectorN m_activations; // Must be of the same dimensionality as the input layer of the model
        AZ::VectorN m_label;       // Must be of the same dimensionality as the output layer of the model
    };

    using LabeledTrainingDataSet = AZStd::vector<LabeledTrainingData>;
}
