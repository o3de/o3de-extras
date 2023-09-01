/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <MachineLearning/INeuralNetwork.h>
#include <MachineLearning/ILabeledTrainingData.h>
#include <AzCore/std/string/string.h>
#include <AzCore/IO/FileIO.h>

namespace MachineLearning
{
    //! This wraps any training data set to restrict the range of samples to a subset of the total.
    class TrainingDataView
        : public ILabeledTrainingData
    {
    public:

        AZ_TYPE_INFO(TrainingDataView, "{BF396C77-4348-46BA-9606-275A3454738E}", ILabeledTrainingData);

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(AZ::ReflectContext* context);

        TrainingDataView() = default;
        TrainingDataView(ILabeledTrainingDataPtr sourceData);

        bool IsValid() const;
        void SetSourceData(ILabeledTrainingDataPtr sourceData);
        void SetRange(AZStd::size_t first, AZStd::size_t last);
        AZStd::size_t GetOriginalSize() const;
        void ShuffleSamples();

        //! ILabeledTrainingData interface
        //! @{
        bool LoadArchive(const AZStd::string& imageFilename, const AZStd::string& labelFilename) override;
        AZStd::size_t GetSampleCount() const override;
        const AZ::VectorN& GetLabelByIndex(AZStd::size_t index) override;
        const AZ::VectorN& GetDataByIndex(AZStd::size_t index) override;
        //! @}

        AZStd::size_t m_first = 0;
        AZStd::size_t m_last = 0;

    private:

        void FillIndicies();

        AZStd::size_t m_firstCache = 0;
        AZStd::size_t m_lastCache = 0;
        AZStd::vector<AZStd::size_t> m_indices;
        ILabeledTrainingDataPtr m_sourceData;
    };
}
