/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <ROS2/Lidar/ClassSegmentationBus.h>
#include <Lidar/Publishing/PointCloudMessageBuilder.h>
#include <Lidar/Publishing/PointCloudMessageFormat.h>
#include <ROS2/Lidar/RaycastResults.h>

namespace ROS2
{
    class PointCloudMessageWriter
    {
    public:
        explicit PointCloudMessageWriter(const Pc2MessageFormat& format);

        void Reset(const AZStd::string& frameId, builtin_interfaces::msg::Time timeStamp, size_t count);
        void WriteResults(const RaycastResults& results);

        const Pc2Message& GetMessage();

    private:
        template<FieldFlags F, typename FT = typename FieldTraits<F>::Type>
        class MessageFieldIterator
        {
        public:
            MessageFieldIterator(void* data, size_t step);

            FT& operator*();
            MessageFieldIterator& operator++();

            bool operator==(const MessageFieldIterator& other);

        private:
            void* m_data{ nullptr };
            size_t m_step{};
        };

        template<FieldFlags F>
        MessageFieldIterator<F> CreateMessageFieldIterator(size_t fieldIndex);

        template<RaycastResultFlags R, FieldFlags F>
        AZ::Outcome<void, AZStd::string_view> AssignResultFieldValue(
            const typename ResultTraits<R>::Type& resultValue, typename FieldTraits<F>::Type& messageFieldValue);

        template<RaycastResultFlags R>
        void WriteResultIfPresent(const RaycastResults& results, FieldFlags fieldFlag, size_t index);

        // Not having 1 - 1 association between raycast results and fields allows for easier incorporation
        // of new fields at the cost of a slight increase in code complexity.
        template<RaycastResultFlags R>
        void WriteResult(const RaycastResults& results, FieldFlags fieldFlag, size_t index);

        template<RaycastResultFlags R, FieldFlags F>
        void WriteResultToMessageField(RaycastResults::ConstFieldSpan<R> fieldSpan, size_t fieldIndex);

        AZStd::vector<bool> m_isSet;
        Pc2MessageWrapper m_message;
        Pc2MessageBuilder m_messageBuilder;
    };

    template<FieldFlags F, typename FT>
    PointCloudMessageWriter::MessageFieldIterator<F, FT>::MessageFieldIterator(void* data, size_t step)
        : m_data(data)
        , m_step(step)
    {
    }

    template<FieldFlags F, typename FT>
    FT& PointCloudMessageWriter::MessageFieldIterator<F, FT>::operator*()
    {
        return *static_cast<FT*>(m_data);
    }

    template<FieldFlags F, typename FT>
    PointCloudMessageWriter::MessageFieldIterator<F, FT>& PointCloudMessageWriter::MessageFieldIterator<F, FT>::operator++()
    {
        m_data = static_cast<void*>(static_cast<char*>(m_data) + m_step);
        return *this;
    }

    template<FieldFlags F, typename FT>
    bool PointCloudMessageWriter::MessageFieldIterator<F, FT>::operator==(const MessageFieldIterator& other)
    {
        return m_data == other.m_data && m_step == other.m_step;
    }

    template<FieldFlags F>
    PointCloudMessageWriter::MessageFieldIterator<F> PointCloudMessageWriter::CreateMessageFieldIterator(size_t fieldIndex)
    {
        void* data = m_message.m_message.data.data() + m_message.m_fieldOffsets.at(fieldIndex);
        return MessageFieldIterator<F>{ data, m_message.m_message.point_step };
    }

    template<>
    inline AZ::Outcome<void, AZStd::string_view> PointCloudMessageWriter::
        AssignResultFieldValue<RaycastResultFlags::Point, FieldFlags::PositionXYZF32>(
            const ResultTraits<RaycastResultFlags::Point>::Type& resultValue,
            FieldTraits<FieldFlags::PositionXYZF32>::Type& messageFieldValue)
    {
        messageFieldValue.m_x = resultValue.GetX();
        messageFieldValue.m_y = resultValue.GetY();
        messageFieldValue.m_z = resultValue.GetZ();
        return AZ::Success();
    }

    template<>
    inline AZ::Outcome<void, AZStd::string_view> PointCloudMessageWriter::
        AssignResultFieldValue<RaycastResultFlags::Intensity, FieldFlags::IntensityF32>(
            const ResultTraits<RaycastResultFlags::Intensity>::Type& resultValue,
            FieldTraits<FieldFlags::IntensityF32>::Type& messageFieldValue)
    {
        messageFieldValue = resultValue;
        return AZ::Success();
    }

    template<>
    inline AZ::Outcome<void, AZStd::string_view> PointCloudMessageWriter::
        AssignResultFieldValue<RaycastResultFlags::Range, FieldFlags::RangeU32>(
            const ResultTraits<RaycastResultFlags::Range>::Type& resultValue, FieldTraits<FieldFlags::RangeU32>::Type& messageFieldValue)
    {
        messageFieldValue = aznumeric_cast<AZ::u32>(resultValue);
        return AZ::Success();
    }

    template<>
    inline AZ::Outcome<void, AZStd::string_view> PointCloudMessageWriter::
        AssignResultFieldValue<RaycastResultFlags::SegmentationData, FieldFlags::SegmentationData96>(
            const ResultTraits<RaycastResultFlags::SegmentationData>::Type& resultValue,
            FieldTraits<FieldFlags::SegmentationData96>::Type& messageFieldValue)
    {
        auto* classSegmentationInterface = ClassSegmentationInterface::Get();
        if (!classSegmentationInterface)
        {
            return AZ::Failure(
                "Segmentation data was requested but the Class Segmentation interface was unavailable. Unable to fetch segmentation class "
                "data. Please make sure to add the Class Segmentation Configuration Component to the Level Entity for this feature to work "
                "properly.");
        }

        const AZ::Color color = classSegmentationInterface->GetClassColor(resultValue.m_classId);
        AZ::u32 rvizColorFormat = color.GetA8() << 24 | color.GetR8() << 16 | color.GetG8() << 8 | color.GetB8();

        messageFieldValue = {
            .m_entityId = messageFieldValue.m_entityId,
            .m_rgba = rvizColorFormat,
            .m_classId = messageFieldValue.m_classId,
        };

        return AZ::Success();
    }

    template<RaycastResultFlags R>
    void PointCloudMessageWriter::WriteResultIfPresent(const RaycastResults& results, FieldFlags fieldFlag, size_t index)
    {
        if (results.IsFieldPresent<R>())
        {
            WriteResult<R>(results, fieldFlag, index);
        }
    }

    template<>
    inline void PointCloudMessageWriter::WriteResult<RaycastResultFlags::Point>(
        const RaycastResults& results, FieldFlags fieldFlag, size_t fieldIndex)
    {
        if (fieldFlag == FieldFlags::PositionXYZF32)
        {
            WriteResultToMessageField<RaycastResultFlags::Point, FieldFlags::PositionXYZF32>(
                results.GetConstFieldSpan<RaycastResultFlags::Point>().value(), fieldIndex);
        }
    }

    template<>
    inline void PointCloudMessageWriter::WriteResult<RaycastResultFlags::Intensity>(
        const RaycastResults& results, FieldFlags fieldFlag, size_t fieldIndex)
    {
        if (fieldFlag == FieldFlags::IntensityF32)
        {
            WriteResultToMessageField<RaycastResultFlags::Intensity, FieldFlags::IntensityF32>(
                results.GetConstFieldSpan<RaycastResultFlags::Intensity>().value(), fieldIndex);
        }
    }

    template<>
    inline void PointCloudMessageWriter::WriteResult<RaycastResultFlags::Range>(
        const RaycastResults& results, FieldFlags fieldFlag, size_t fieldIndex)
    {
        if (fieldFlag == FieldFlags::RangeU32)
        {
            WriteResultToMessageField<RaycastResultFlags::Range, FieldFlags::RangeU32>(
                results.GetConstFieldSpan<RaycastResultFlags::Range>().value(), fieldIndex);
        }
    }

    template<>
    inline void PointCloudMessageWriter::WriteResult<RaycastResultFlags::SegmentationData>(
        const RaycastResults& results, FieldFlags fieldFlag, size_t fieldIndex)
    {
        if (fieldFlag == FieldFlags::SegmentationData96)
        {
            WriteResultToMessageField<RaycastResultFlags::SegmentationData, FieldFlags::SegmentationData96>(
                results.GetConstFieldSpan<RaycastResultFlags::SegmentationData>().value(), fieldIndex);
        }
    }

    template<RaycastResultFlags R, FieldFlags F>
    void PointCloudMessageWriter::WriteResultToMessageField(RaycastResults::ConstFieldSpan<R> fieldSpan, size_t fieldIndex)
    {
        auto messageFieldIt = CreateMessageFieldIterator<F>(fieldIndex);
        for (auto resultIt = fieldSpan.begin(); resultIt != fieldSpan.end(); ++resultIt, ++messageFieldIt)
        {
            if (const auto outcome = AssignResultFieldValue<R, F>(*resultIt, *messageFieldIt); !outcome.IsSuccess())
            {
                AZ_Error(
                    "ROS2::PointCloudMessageWriter",
                    false,
                    "Writing result of type %u failed with the following message: %s. Skipping",
                    R,
                    outcome.GetError().data());
                return;
            }
        }
    }
} // namespace ROS2