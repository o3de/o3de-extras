/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Lidar/Publishing/PointCloudMessageBuilder.h>
#include <Lidar/Publishing/PointCloudMessageFormat.h>
#include <ROS2/Lidar/ClassSegmentationBus.h>
#include <ROS2/Lidar/RaycastResults.h>

namespace ROS2
{
    class PointCloudMessageWriter
    {
    public:
        explicit PointCloudMessageWriter(const Pc2MessageFormat& format);

        void Reset(const AZStd::string& frameId, builtin_interfaces::msg::Time timeStamp, size_t width, size_t height, bool isDense);
        void WriteResults(const RaycastResults& results, bool skipNonHits = false);

        const Pc2Message& GetMessage() const;

    private:
        template<FieldFlags F, typename FT = typename FieldTraits<F>::Type>
        class MessageFieldIterator
        {
        public:
            MessageFieldIterator(void* data, size_t step);

            FT& operator*();
            MessageFieldIterator& operator++();

            bool operator==(const MessageFieldIterator& other);
            bool operator!=(const MessageFieldIterator& other);

        private:
            void* m_data{ nullptr };
            size_t m_step{};
        };

        template<FieldFlags F>
        MessageFieldIterator<F> CreateMessageFieldIterator(size_t fieldIndex, size_t pointIndex);
        template<FieldFlags F>
        MessageFieldIterator<F> Begin(size_t fieldIndex);
        template<FieldFlags F>
        MessageFieldIterator<F> End(size_t fieldIndex);

        template<RaycastResultFlags R, FieldFlags F>
        AZ::Outcome<void, AZStd::string_view> AssignResultFieldValue(
            const typename ResultTraits<R>::Type& resultValue, typename FieldTraits<F>::Type& messageFieldValue);

        template<RaycastResultFlags R>
        bool WriteResultIfPresent(const RaycastResults& results, FieldFlags fieldFlag, size_t index, bool skipNonHits);

        void FillWithDefaultValues(FieldFlags fieldFlag, size_t fieldIndex);
        template<FieldFlags F>
        void FillWithDefaultValues(size_t fieldIndex);

        // Not having 1 - 1 association between raycast results and fields allows for easier incorporation
        // of new fields at the cost of a slight increase in code complexity.
        template<RaycastResultFlags R>
        void WriteResult(
            const RaycastResults& results,
            FieldFlags fieldFlag,
            size_t index,
            AZStd::optional<RaycastResults::ConstFieldSpan<RaycastResultFlags::IsHit>> isHit);

        template<RaycastResultFlags R, FieldFlags F>
        void WriteResultToMessageField(
            RaycastResults::ConstFieldSpan<R> fieldSpan,
            size_t fieldIndex,
            AZStd::optional<RaycastResults::ConstFieldSpan<RaycastResultFlags::IsHit>> isHit);

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

    template<FieldFlags F, typename FT>
    bool PointCloudMessageWriter::MessageFieldIterator<F, FT>::operator!=(const MessageFieldIterator& other)
    {
        return m_data != other.m_data || m_step != other.m_step;
    }

    template<FieldFlags F>
    PointCloudMessageWriter::MessageFieldIterator<F> PointCloudMessageWriter::CreateMessageFieldIterator(
        size_t fieldIndex, size_t pointIndex)
    {
        const auto pointStep = m_message.m_message.point_step;
        void* data = m_message.m_message.data.data() + m_message.m_fieldOffsets.at(fieldIndex) + pointIndex * pointStep;
        return MessageFieldIterator<F>(data, pointStep);
    }

    template<FieldFlags F>
    PointCloudMessageWriter::MessageFieldIterator<F> PointCloudMessageWriter::Begin(size_t fieldIndex)
    {
        return CreateMessageFieldIterator<F>(fieldIndex, 0U);
    }

    template<FieldFlags F>
    PointCloudMessageWriter::MessageFieldIterator<F> PointCloudMessageWriter::End(size_t fieldIndex)
    {
        return CreateMessageFieldIterator<F>(fieldIndex, m_message.GetPointCount());
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

    template<>
    inline AZ::Outcome<void, AZStd::string_view> PointCloudMessageWriter::
        AssignResultFieldValue<RaycastResultFlags::Ring, FieldFlags::RingU8>(
            const ResultTraits<RaycastResultFlags::Ring>::Type& resultValue, FieldTraits<FieldFlags::RingU8>::Type& messageFieldValue)
    {
        messageFieldValue = aznumeric_cast<AZ::u8>(resultValue);
        return AZ::Success();
    }

    template<>
    inline AZ::Outcome<void, AZStd::string_view> PointCloudMessageWriter::
        AssignResultFieldValue<RaycastResultFlags::Ring, FieldFlags::RingU16>(
            const ResultTraits<RaycastResultFlags::Ring>::Type& resultValue, FieldTraits<FieldFlags::RingU16>::Type& messageFieldValue)
    {
        messageFieldValue = aznumeric_cast<AZ::u16>(resultValue);
        return AZ::Success();
    }

    template<RaycastResultFlags R>
    bool PointCloudMessageWriter::WriteResultIfPresent(const RaycastResults& results, FieldFlags fieldFlag, size_t index, bool skipNonHits)
    {
        if (results.IsFieldPresent<R>())
        {
            const auto isHit = skipNonHits ? results.GetConstFieldSpan<RaycastResultFlags::IsHit>() : AZStd::nullopt;
            WriteResult<R>(results, fieldFlag, index, isHit);
            return true;
        }

        return false;
    }

    template<FieldFlags F>
    static bool IsZeros(const typename FieldTraits<F>::Type& value)
    {
        typename FieldTraits<F>::Type zeroValue;
        memset(&zeroValue, 0, sizeof(zeroValue));
        return memcmp(&zeroValue, &value, sizeof(zeroValue)) == 0;
    }

    template<FieldFlags F>
    void PointCloudMessageWriter::FillWithDefaultValues(size_t fieldIndex)
    {
        // We do this to avoid unnecessary work (i.e. only when the values are not zeros).
        if (IsZeros<F>(FieldTraits<F>::DefaultValue))
        {
            return;
        }

        const auto messageFieldEndIt = End<F>(fieldIndex);
        for (auto messageFieldIt = Begin<F>(fieldIndex); messageFieldIt != messageFieldEndIt; ++messageFieldIt)
        {
            *messageFieldIt = FieldTraits<F>::DefaultValue;
        }
    }

    template<>
    inline void PointCloudMessageWriter::WriteResult<RaycastResultFlags::Point>(
        const RaycastResults& results,
        FieldFlags fieldFlag,
        size_t fieldIndex,
        AZStd::optional<RaycastResults::ConstFieldSpan<RaycastResultFlags::IsHit>> isHit)
    {
        if (fieldFlag == FieldFlags::PositionXYZF32)
        {
            WriteResultToMessageField<RaycastResultFlags::Point, FieldFlags::PositionXYZF32>(
                results.GetConstFieldSpan<RaycastResultFlags::Point>().value(), fieldIndex, isHit);
        }
    }

    template<>
    inline void PointCloudMessageWriter::WriteResult<RaycastResultFlags::Intensity>(
        const RaycastResults& results,
        FieldFlags fieldFlag,
        size_t fieldIndex,
        AZStd::optional<RaycastResults::ConstFieldSpan<RaycastResultFlags::IsHit>> isHit)
    {
        if (fieldFlag == FieldFlags::IntensityF32)
        {
            WriteResultToMessageField<RaycastResultFlags::Intensity, FieldFlags::IntensityF32>(
                results.GetConstFieldSpan<RaycastResultFlags::Intensity>().value(), fieldIndex, isHit);
        }
    }

    template<>
    inline void PointCloudMessageWriter::WriteResult<RaycastResultFlags::Range>(
        const RaycastResults& results,
        FieldFlags fieldFlag,
        size_t fieldIndex,
        AZStd::optional<RaycastResults::ConstFieldSpan<RaycastResultFlags::IsHit>> isHit)
    {
        if (fieldFlag == FieldFlags::RangeU32)
        {
            WriteResultToMessageField<RaycastResultFlags::Range, FieldFlags::RangeU32>(
                results.GetConstFieldSpan<RaycastResultFlags::Range>().value(), fieldIndex, isHit);
        }
    }

    template<>
    inline void PointCloudMessageWriter::WriteResult<RaycastResultFlags::SegmentationData>(
        const RaycastResults& results,
        FieldFlags fieldFlag,
        size_t fieldIndex,
        AZStd::optional<RaycastResults::ConstFieldSpan<RaycastResultFlags::IsHit>> isHit)
    {
        if (fieldFlag == FieldFlags::SegmentationData96)
        {
            WriteResultToMessageField<RaycastResultFlags::SegmentationData, FieldFlags::SegmentationData96>(
                results.GetConstFieldSpan<RaycastResultFlags::SegmentationData>().value(), fieldIndex, isHit);
        }
    }

    template<>
    inline void PointCloudMessageWriter::WriteResult<RaycastResultFlags::Ring>(
        const RaycastResults& results,
        FieldFlags fieldFlag,
        size_t fieldIndex,
        AZStd::optional<RaycastResults::ConstFieldSpan<RaycastResultFlags::IsHit>> isHit)
    {
        if (fieldFlag == FieldFlags::RingU8)
        {
            WriteResultToMessageField<RaycastResultFlags::Ring, FieldFlags::RingU8>(
                results.GetConstFieldSpan<RaycastResultFlags::Ring>().value(), fieldIndex, isHit);
        }
        else if (fieldFlag == FieldFlags::RingU16)
        {
            WriteResultToMessageField<RaycastResultFlags::Ring, FieldFlags::RingU16>(
                results.GetConstFieldSpan<RaycastResultFlags::Ring>().value(), fieldIndex, isHit);
        }
    }

    template<RaycastResultFlags R, FieldFlags F>
    void PointCloudMessageWriter::WriteResultToMessageField(
        RaycastResults::ConstFieldSpan<R> fieldSpan,
        size_t fieldIndex,
        AZStd::optional<RaycastResults::ConstFieldSpan<RaycastResultFlags::IsHit>> isHit)
    {
        AZStd::optional<RaycastResults::ConstFieldSpan<RaycastResultFlags::IsHit>::const_iterator> isHitIt = AZStd::nullopt;
        if (isHit.has_value())
        {
            isHitIt = isHit->begin();
        }

        auto messageFieldIt = Begin<F>(fieldIndex);
        for (auto resultIt = fieldSpan.begin(); resultIt != fieldSpan.end(); ++resultIt, ++messageFieldIt)
        {
            if (!isHitIt.has_value() || *isHitIt.value())
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

            if (isHitIt.has_value())
            {
                ++(*isHitIt);
            }
        }
    }
} // namespace ROS2