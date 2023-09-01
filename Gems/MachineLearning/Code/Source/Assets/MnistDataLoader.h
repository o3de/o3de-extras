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
    //! A class that can load the MNIST training data set.
    //! https://en.wikipedia.org/wiki/MNIST_database
    class MnistDataLoader
        : public ILabeledTrainingData
    {
    public:

        AZ_TYPE_INFO(MnistDataLoader, "{3F4C0F29-4E7E-4CAF-A331-EAC3D2D9409E}", ILabeledTrainingData);

        //! AzCore Reflection.
        //! @param context reflection context
        static void Reflect(AZ::ReflectContext* context);

        MnistDataLoader() = default;

        //! ILabeledTrainingData interface
        //! @{
        bool LoadArchive(const AZStd::string& imageFilename, const AZStd::string& labelFilename) override;
        AZStd::size_t GetSampleCount() const override;
        const AZ::VectorN& GetLabelByIndex(AZStd::size_t index) override;
        const AZ::VectorN& GetDataByIndex(AZStd::size_t index) override;
        //! @}

    private:

        bool LoadImageFile(const AZStd::string& imageFilename);
        bool LoadLabelFile(const AZStd::string& labelFilename);

        struct MnistDataHeader
        {
            uint32_t m_imageHeader = 0;
            uint32_t m_imageCount = 0;
            uint32_t m_height = 0;
            uint32_t m_width = 0;
        };

        MnistDataHeader m_dataHeader;

        AZ::IO::SystemFile m_imageFile;
        AZ::IO::SystemFile m_labelFile;

        AZStd::size_t m_imageDataStart = 0;
        AZStd::size_t m_labelDataStart = 0;

        AZStd::size_t m_currentIndex = 0xFFFFFFFF;
        AZStd::vector<uint8_t> m_imageBuffer;
        AZStd::vector<uint8_t> m_labelBuffer;

        AZ::VectorN m_imageVector;
        AZ::VectorN m_labelVector;
    };
}
