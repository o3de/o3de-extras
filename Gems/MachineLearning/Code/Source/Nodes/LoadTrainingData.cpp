/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Nodes/LoadTrainingData.h>
#include <Assets/MnistDataLoader.h>

namespace MachineLearning
{
    ILabeledTrainingDataPtr LoadTrainingData::In(AZStd::string ImageFile, AZStd::string LabelFile)
    {
        ILabeledTrainingDataPtr result = AZStd::make_shared<MnistDataLoader>();
        result->LoadArchive(ImageFile, LabelFile);
        return result;
    }
}
