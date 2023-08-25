/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Assets/MnistDataLoader.h>
#include <Algorithms/Activations.h>
#include <AzCore/IO/FileReader.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Console/ILogger.h>
#include <AzNetworking/Utilities/Endian.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>

namespace MachineLearning
{
    void MnistDataLoader::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<MnistDataLoader>()
                ->Version(1)
                ;

            if (AZ::EditContext* editContext = serializeContext->GetEditContext())
            {
                editContext->Class<MnistDataLoader>("Parameters defining a single training data instance", "")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ;
            }
        }

        auto behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context);
        if (behaviorContext)
        {
            behaviorContext->Class<MnistDataLoader>()->
                Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)->
                Attribute(AZ::Script::Attributes::Module, "machineLearning")->
                Attribute(AZ::Script::Attributes::ExcludeFrom, AZ::Script::Attributes::ExcludeFlags::ListOnly)->
                Constructor<>()->
                Attribute(AZ::Script::Attributes::Storage, AZ::Script::Attributes::StorageType::Value)
                ;
        }
    }

    bool MnistDataLoader::LoadArchive(const AZStd::string& imageFilename, const AZStd::string& labelFilename)
    {
        return LoadImageFile(imageFilename) && LoadLabelFile(labelFilename);
    }

    AZStd::size_t MnistDataLoader::GetSampleCount() const
    {
        return m_dataHeader.m_imageCount;
    }

    const AZ::VectorN& MnistDataLoader::GetLabelByIndex(AZStd::size_t index)
    {
        OneHotEncode(m_labelBuffer[index], 10, m_labelVector);
        return m_labelVector;
    }

    AZStd::size_t MnistDataLoader::GetLabelAsValueByIndex(AZStd::size_t index)
    {
        return static_cast<AZStd::size_t>(m_labelBuffer[index]);
    }

    const AZ::VectorN& MnistDataLoader::GetDataByIndex(AZStd::size_t index)
    {
        const AZStd::size_t imageDataStride = m_dataHeader.m_height * m_dataHeader.m_width;
        m_imageVector.Resize(imageDataStride);
        for (AZStd::size_t iter = 0; iter < imageDataStride; ++iter)
        {
            m_imageVector.SetElement(iter, static_cast<float>(m_imageBuffer[index * imageDataStride + iter]) / 255.0f);
        }
        return m_imageVector;
    }

    bool MnistDataLoader::LoadImageFile(const AZStd::string& imageFilename)
    {
        AZ::IO::FixedMaxPath filePathFixed = imageFilename.c_str();
        if (AZ::IO::FileIOBase* fileIOBase = AZ::IO::FileIOBase::GetInstance())
        {
            fileIOBase->ResolvePath(filePathFixed, imageFilename.c_str());
        }

        if (!m_imageFile.Open(filePathFixed.c_str(), AZ::IO::SystemFile::SF_OPEN_READ_ONLY))
        {
            AZLOG_ERROR("Failed to load '%s'. File could not be opened.", filePathFixed.c_str());
            return false;
        }

        const AZ::IO::SizeType length = m_imageFile.Length();
        if (length == 0)
        {
            AZLOG_ERROR("Failed to load '%s'. File is empty.", filePathFixed.c_str());
            return false;
        }

        m_imageFile.Seek(0, AZ::IO::SystemFile::SF_SEEK_BEGIN);

        AZ::IO::SizeType bytesRead = m_imageFile.Read(sizeof(MnistDataHeader), &m_dataHeader);

        if (bytesRead != sizeof(MnistDataHeader))
        {
            // Failed to read the whole header
            AZLOG_ERROR("Failed to load '%s', failed to read archive header.", filePathFixed.c_str());
            m_imageFile.Close();
            return false;
        }

        m_dataHeader.m_imageHeader = ntohl(m_dataHeader.m_imageHeader);
        m_dataHeader.m_imageCount = ntohl(m_dataHeader.m_imageCount);
        m_dataHeader.m_height = ntohl(m_dataHeader.m_height);
        m_dataHeader.m_width = ntohl(m_dataHeader.m_width);

        constexpr uint32_t MnistImageHeaderValue = 2051;
        if (m_dataHeader.m_imageHeader != MnistImageHeaderValue)
        {
            // Invalid format
            AZLOG_ERROR("Failed to load '%s', file is not an MNIST archive (expected %u, encountered %u).", filePathFixed.c_str(), MnistImageHeaderValue, m_dataHeader.m_imageHeader);
            m_imageFile.Close();
            return false;
        }

        const AZStd::size_t imageDataStride = m_dataHeader.m_height * m_dataHeader.m_width;
        m_imageBuffer.resize(m_dataHeader.m_imageCount * imageDataStride);
        m_imageFile.Read(m_dataHeader.m_imageCount * imageDataStride, m_imageBuffer.data());
        return true;
    }

    bool MnistDataLoader::LoadLabelFile(const AZStd::string& labelFilename)
    {
        AZ::IO::FixedMaxPath filePathFixed = labelFilename.c_str();
        if (AZ::IO::FileIOBase* fileIOBase = AZ::IO::FileIOBase::GetInstance())
        {
            fileIOBase->ResolvePath(filePathFixed, labelFilename.c_str());
        }

        if (!m_labelFile.Open(filePathFixed.c_str(), AZ::IO::SystemFile::SF_OPEN_READ_ONLY))
        {
            AZLOG_ERROR("Failed to load '%s'. File could not be opened.", filePathFixed.c_str());
            return false;
        }

        const AZ::IO::SizeType length = m_labelFile.Length();
        if (length == 0)
        {
            AZLOG_ERROR("Failed to load '%s'. File is empty.", filePathFixed.c_str());
            return false;
        }

        m_labelFile.Seek(0, AZ::IO::SystemFile::SF_SEEK_BEGIN);

        struct MnistLabelHeader
        {
            uint32_t m_labelHeader = 0;
            uint32_t m_labelCount = 0;
        };

        MnistLabelHeader labelHeader;
        AZ::IO::SizeType bytesRead = m_labelFile.Read(sizeof(MnistLabelHeader), &labelHeader);

        if (bytesRead != sizeof(MnistLabelHeader))
        {
            // Failed to read the whole header
            AZLOG_ERROR("Failed to load '%s', failed to read label header.", filePathFixed.c_str());
            m_labelFile.Close();
            return false;
        }

        labelHeader.m_labelHeader = ntohl(labelHeader.m_labelHeader);
        labelHeader.m_labelCount = ntohl(labelHeader.m_labelCount);

        constexpr uint32_t MnistLabelHeaderValue = 2049;
        if (labelHeader.m_labelHeader != MnistLabelHeaderValue)
        {
            // Invalid format
            AZLOG_ERROR("Failed to load '%s', file is not an MNIST archive (expected %u, encountered %u).", filePathFixed.c_str(), MnistLabelHeaderValue, labelHeader.m_labelHeader);
            m_labelFile.Close();
            return false;
        }

        if (m_dataHeader.m_imageCount != labelHeader.m_labelCount)
        {
            AZLOG_ERROR("Failed to load '%s', mismatch between image count (%u) and label count (%u).", filePathFixed.c_str(), m_dataHeader.m_imageCount, labelHeader.m_labelCount);
            m_labelFile.Close();
            return false;
        }

        m_labelBuffer.resize(labelHeader.m_labelCount);
        m_labelFile.Read(labelHeader.m_labelCount, m_labelBuffer.data());
        AZLOG_INFO("Loaded MNIST archive %s containing %u samples", filePathFixed.c_str(), m_dataHeader.m_imageCount);
        return true;
    }
}
