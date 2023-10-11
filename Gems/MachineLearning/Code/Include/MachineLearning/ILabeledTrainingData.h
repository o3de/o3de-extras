/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <MachineLearning/Types.h>
#include <AzCore/std/string/string.h>

namespace MachineLearning
{
    struct ILabeledTrainingData
    {
        AZ_TYPE_INFO(ILabeledTrainingData, "{50DF457E-3EAC-4114-8444-023E64973AD9}");

        virtual ~ILabeledTrainingData() = default;

        //! Loads the indicated label and data files.
        virtual bool LoadArchive(const AZ::IO::Path& imageFilename, const AZ::IO::Path& labelFilename) = 0;

        //! Returns the total number of samples contained in the training data set.
        virtual AZStd::size_t GetSampleCount() const = 0;

        //! Returns the index-th label in the training data set.
        virtual const AZ::VectorN& GetLabelByIndex(AZStd::size_t index) = 0;

        //! Returns the index-th set of activations in the training data set.
        virtual const AZ::VectorN& GetDataByIndex(AZStd::size_t index) = 0;
    };

    using ILabeledTrainingDataPtr = AZStd::shared_ptr<ILabeledTrainingData>;
}
