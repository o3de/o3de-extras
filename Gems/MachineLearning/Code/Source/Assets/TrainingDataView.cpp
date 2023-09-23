/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Assets/TrainingDataView.h>
#include <numeric>
#include <random>

namespace MachineLearning
{
    TrainingDataView::TrainingDataView(ILabeledTrainingDataPtr sourceData)
        : m_sourceData(sourceData)
    {
        FillIndicies();
    }

    bool TrainingDataView::IsValid() const
    {
        return m_sourceData != nullptr;
    }

    void TrainingDataView::SetSourceData(ILabeledTrainingDataPtr sourceData)
    {
        m_sourceData = sourceData;
        FillIndicies();
    }

    void TrainingDataView::SetRange(AZStd::size_t first, AZStd::size_t last)
    {
        m_first = first;
        m_last = last;
        FillIndicies();
    }

    AZStd::size_t TrainingDataView::GetOriginalSize() const
    {
        if (m_sourceData)
        {
            return m_sourceData->GetSampleCount();
        }
        return 0;
    }

    void TrainingDataView::ShuffleSamples()
    {
        std::shuffle(m_indices.begin(), m_indices.end(), std::mt19937(std::random_device{}()));
    }

    bool TrainingDataView::LoadArchive(const AZ::IO::Path& imageFilename, const AZ::IO::Path& labelFilename)
    {
        AZ_Assert(m_sourceData, "No datasource assigned to view");
        bool result = m_sourceData->LoadArchive(imageFilename, labelFilename);
        FillIndicies();
        return result;
    }

    AZStd::size_t TrainingDataView::GetSampleCount() const
    {
        return m_last - m_first;
    }

    const AZ::VectorN& TrainingDataView::GetLabelByIndex(AZStd::size_t index)
    {
        AZ_Assert(m_sourceData, "No datasource assigned to view");
        AZ_Assert(index + m_first < m_last, "Out of range index requested");
        if (m_firstCache != m_first || m_lastCache != m_last)
        {
            FillIndicies();
        }
        return m_sourceData->GetLabelByIndex(m_indices[index]);
    }

    const AZ::VectorN& TrainingDataView::GetDataByIndex(AZStd::size_t index)
    {
        AZ_Assert(m_sourceData, "No datasource assigned to view");
        AZ_Assert(index + m_first < m_last, "Out of range index requested");
        if (m_firstCache != m_first || m_lastCache != m_last)
        {
            FillIndicies();
        }
        return m_sourceData->GetDataByIndex(m_indices[index]);
    }

    void TrainingDataView::FillIndicies()
    {
        // Generate a set of training indices that we can later optionally shuffle
        m_indices.resize(m_last);
        std::iota(m_indices.begin(), m_indices.end(), m_first);
        m_firstCache = m_first;
        m_lastCache = m_last;
    }
}
