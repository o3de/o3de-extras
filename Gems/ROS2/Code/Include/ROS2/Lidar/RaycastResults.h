/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Math/Vector3.h>
#include <AzCore/std/containers/span.h>
#include <AzCore/std/containers/vector.h>

namespace ROS2
{
    enum class RaycastResultFlags : AZ::u8
    {
        Point = (1 << 0), //!< return 3D point coordinates
        Range = (1 << 1), //!< return array of distances
        Intensity = (1 << 2), //!< return intensity data
    };

    //! Bitwise operators for RaycastResultFlags
    AZ_DEFINE_ENUM_BITWISE_OPERATORS(RaycastResultFlags)

    inline bool IsFlagEnabled(RaycastResultFlags flag, RaycastResultFlags flags)
    {
        return (flags & flag) == flag;
    }

    template<RaycastResultFlags F>
    struct ResultTraits;

    template<>
    struct ResultTraits<RaycastResultFlags::Point>
    {
        using Type = AZ::Vector3;
    };

    template<>
    struct ResultTraits<RaycastResultFlags::Range>
    {
        using Type = float;
    };

    template<>
    struct ResultTraits<RaycastResultFlags::Intensity>
    {
        using Type = float;
    };

    //! Class used for storing the results of a raycast.
    //! It guarantees a uniform length of all its fields.
    class RaycastResults
    {
    public:
        template<RaycastResultFlags F, typename RT = typename ResultTraits<F>::Type>
        using FieldSpan = AZStd::span<RT>;

        template<RaycastResultFlags F>
        using ConstFieldSpan = FieldSpan<F, const typename ResultTraits<F>::Type>;

        explicit RaycastResults(RaycastResultFlags flags, size_t count = 0U);
        RaycastResults(const RaycastResults& other) = default;
        RaycastResults(RaycastResults&& other);

        [[nodiscard]] bool IsEmpty() const;

        template<RaycastResultFlags F>
        [[nodiscard]] bool IsFieldPresent() const;

        [[nodiscard]] size_t GetCount() const;

        template<RaycastResultFlags F>
        AZStd::optional<ConstFieldSpan<F>> GetConstFieldSpan() const;

        template<RaycastResultFlags F>
        AZStd::optional<FieldSpan<F>> GetFieldSpan();

        void Clear();
        void Resize(size_t count);

        RaycastResults& operator=(const RaycastResults& other) = default;
        RaycastResults& operator=(RaycastResults&& other);

    private:
        template<RaycastResultFlags F, typename RT = typename ResultTraits<F>::Type>
        using FieldInternal = AZStd::optional<AZStd::vector<RT>>;

        template<RaycastResultFlags F>
        const FieldInternal<F>& GetField() const;

        template<RaycastResultFlags F>
        FieldInternal<F>& GetField();

        template<RaycastResultFlags F>
        void ResizeFieldIfPresent(size_t count);

        template<RaycastResultFlags F>
        void EnsureFlagSatisfied(RaycastResultFlags flags, size_t count);

        template<RaycastResultFlags F>
        void ClearFieldIfPresent();

        size_t m_count{};
        FieldInternal<RaycastResultFlags::Point> m_points;
        FieldInternal<RaycastResultFlags::Range> m_ranges;
        FieldInternal<RaycastResultFlags::Intensity> m_intensities;
    };

    template<RaycastResultFlags F>
    bool RaycastResults::IsFieldPresent() const
    {
        return GetField<F>().has_value();
    }

    template<RaycastResultFlags F>
    AZStd::optional<RaycastResults::ConstFieldSpan<F>> RaycastResults::GetConstFieldSpan() const
    {
        auto& field = GetField<F>();
        if (!field.has_value())
        {
            return {};
        }

        return AZStd::span(field->begin(), field->size());
    }

    template<RaycastResultFlags F>
    AZStd::optional<RaycastResults::FieldSpan<F>> RaycastResults::GetFieldSpan()
    {
        auto& field = GetField<F>();
        if (!field.has_value())
        {
            return {};
        }

        return AZStd::span(field->begin(), field->size());
    }

    template<>
    inline const RaycastResults::FieldInternal<RaycastResultFlags::Point>& RaycastResults::GetField<RaycastResultFlags::Point>() const
    {
        return m_points;
    }

    template<>
    inline const RaycastResults::FieldInternal<RaycastResultFlags::Range>& RaycastResults::GetField<RaycastResultFlags::Range>() const
    {
        return m_ranges;
    }

    template<>
    inline const RaycastResults::FieldInternal<RaycastResultFlags::Intensity>& RaycastResults::GetField<RaycastResultFlags::Intensity>()
        const
    {
        return m_intensities;
    }

    template<RaycastResultFlags F>
    RaycastResults::FieldInternal<F>& RaycastResults::GetField()
    {
        return const_cast<FieldInternal<F>&>(static_cast<const RaycastResults*>(this)->GetField<F>());
    }

    template<RaycastResultFlags F>
    void RaycastResults::ClearFieldIfPresent()
    {
        auto& field = GetField<F>();
        if (!field.has_value())
        {
            return;
        }

        field->clear();
    }

    template<RaycastResultFlags F>
    void RaycastResults::ResizeFieldIfPresent(size_t count)
    {
        auto& field = GetField<F>();
        if (!field.has_value())
        {
            return;
        }

        field->resize(count);
    }

    template<RaycastResultFlags F>
    void RaycastResults::EnsureFlagSatisfied(RaycastResultFlags flags, size_t count)
    {
        if (!IsFlagEnabled(F, flags))
        {
            return;
        }

        auto& field = GetField<F>();
        if (!field.has_value())
        {
            field = AZStd::vector<typename ResultTraits<F>::Type>(count);
        }
        else
        {
            field->resize(count);
        }
    }
} // namespace ROS2
