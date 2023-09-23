/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Math/VectorN.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>

namespace MachineLearning
{
    AZ_ENUM_CLASS(LossFunctions,
        MeanSquaredError
    );

    AZ_ENUM_CLASS(ActivationFunctions,
        ReLU,
        Sigmoid,
        Softmax,
        Linear
    );

    AZ_ENUM_CLASS(AssetTypes,
        TestData,
        TestLabels,
        TrainingData, 
        TrainingLabels
    );

    class IAssetPersistenceProxy
    {
    public:
        virtual ~IAssetPersistenceProxy() = default;
        virtual bool SaveAsset() = 0;
        virtual bool LoadAsset() = 0;
    };
}
