#pragma once

#include <AzCore/Outcome/Outcome.h>
#include <AzCore/base.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/smart_ptr/unique_ptr.h>
#include <AzCore/std/string/regex.h>
#include <ROS2/Lidar/RaycastResults.h>
#include <rclcpp/publisher.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_msgs/msg/label_info.hpp>

namespace ROS2
{
    enum class FieldFlags : AZ::u16
    {
        // clang-format off
        None                    = 0,
        PositionXYZF32          = 1,
        IntensityF32            = 1 << 1,
        TU32                    = 1 << 2,
        ReflectivityU16         = 1 << 3,
        RingU16                 = 1 << 4,
        AmbientU16              = 1 << 5,
        RangeU32                = 1 << 6,
        SegmentationData96      = 1 << 7, // A 96-bit structure is used to store segmentation data.
        Padding8                = 1 << 8,
        Padding16               = 1 << 9,
        Padding32               = 1 << 10,
        RingU8                  = 1 << 11,
        All                     = (1 << 12) - 1U,
        // clang-format on
    };

    bool IsPadding(FieldFlags fieldFlag);
    size_t GetFieldByteSize(FieldFlags fieldFlag);
    size_t GetActualFieldCount(FieldFlags fieldFlag);
    RaycastResultFlags GetFieldProvider(FieldFlags fieldFlag);
    AZStd::optional<AZStd::string> GetDefaultFieldName(FieldFlags fieldFlag);
    AZStd::optional<AZStd::string> GetDataTypeString(FieldFlags fieldFlag);

    // The field types default constructor should initialize all bits to zero.
    // This allows us to easier check if the non provided fields should be
    // overwritten with their respective default values.
    template<FieldFlags fieldFlag>
    struct FieldTraits;

    template<>
    struct FieldTraits<FieldFlags::PositionXYZF32>
    {
        struct Position
        {
            float m_x{}, m_y{}, m_z{};
        };

        using Type = Position;
        static constexpr Type DefaultValue = Type{ 0.0f, 0.0f, 0.0f };
        static constexpr RaycastResultFlags Provider = RaycastResultFlags::Point;
    };

    template<>
    struct FieldTraits<FieldFlags::IntensityF32>
    {
        using Type = float;
        static constexpr Type DefaultValue{};
        static constexpr RaycastResultFlags Provider = RaycastResultFlags::Intensity;
    };

    template<>
    struct FieldTraits<FieldFlags::TU32>
    {
        using Type = AZ::u32;
        static constexpr Type DefaultValue{};
        static constexpr RaycastResultFlags Provider = RaycastResultFlags::None;
    };

    template<>
    struct FieldTraits<FieldFlags::ReflectivityU16>
    {
        using Type = float;
        static constexpr Type DefaultValue{};
        static constexpr RaycastResultFlags Provider = RaycastResultFlags::None;
    };

    template<>
    struct FieldTraits<FieldFlags::RingU16>
    {
        using Type = AZ::u16;
        static constexpr Type DefaultValue{};
        static constexpr RaycastResultFlags Provider = RaycastResultFlags::None;
    };

    template<>
    struct FieldTraits<FieldFlags::RingU8>
    {
        using Type = AZ::u8;
        static constexpr Type DefaultValue{};
        static constexpr RaycastResultFlags Provider = RaycastResultFlags::None;
    };

    template<>
    struct FieldTraits<FieldFlags::AmbientU16>
    {
        using Type = AZ::u16;
        static constexpr Type DefaultValue{};
        static constexpr RaycastResultFlags Provider = RaycastResultFlags::None;
    };

    template<>
    struct FieldTraits<FieldFlags::RangeU32>
    {
        using Type = AZ::u32;
        static constexpr Type DefaultValue{};
        static constexpr RaycastResultFlags Provider = RaycastResultFlags::Range;
    };

    template<>
    struct FieldTraits<FieldFlags::SegmentationData96>
    {
        struct SegmentationData
        {
            AZ::s32 m_entityId;
            AZ::u32 m_rgba;
            AZ::u8 m_classId;
        };

        using Type = SegmentationData;
        static constexpr auto DefaultValue = Type{ .m_entityId = 0, .m_rgba = 0U, .m_classId = 0U };
        static constexpr RaycastResultFlags Provider = RaycastResultFlags::SegmentationData;
    };

    AZ_DEFINE_ENUM_BITWISE_OPERATORS(FieldFlags)

    namespace NameUtils
    {
        static const auto SubNameRegex = AZStd::regex(R"(\s*([^\s,]+)\s*(?:,|$))");

        bool IsNamesStringValid(const AZStd::string& namesString, size_t count);

        template<size_t N>
        AZStd::array<AZStd::string, N> ExtractFieldNameStrings(const AZStd::string& namesString)
        {
            AZ_Assert(IsNamesStringValid(namesString, N), "Programmer error. Expected a valid names string.");
            AZStd::array<AZStd::string, N> fieldNames;
            AZStd::smatch match;
            auto searchStart(namesString.cbegin());
            for (size_t i = 0; i < N; ++i)
            {
                AZStd::regex_search(searchStart, namesString.cend(), match, SubNameRegex);
                fieldNames[i] = match[1].str();
                searchStart = match.suffix().first;
            }

            return fieldNames;
        }
    } // namespace NameUtils

    struct FieldFormat
    {
        explicit FieldFormat(FieldFlags flag = FieldFlags::Padding8);
        AZ_TYPE_INFO(FieldFormat, "{2147abfd-6e16-4d1f-83a9-a840d31b0d5c}");
        static void Reflect(AZ::ReflectContext* context);

        [[nodiscard]] bool IsRegularField() const;
        AZ::Crc32 OnTypeChanged();
        AZ::Outcome<void, AZStd::string> ValidateFieldName(void* newValue, const AZ::Uuid& valueType) const;

        AZStd::string m_name;
        AZStd::string m_dataType;
        size_t m_fieldOffset{ 0U };
        FieldFlags m_fieldFlag;
    };

    using Pc2MessageFormat = AZStd::vector<FieldFormat>;
    RaycastResultFlags GetNecessaryProviders(const Pc2MessageFormat& messageFormat);

    enum class DistanceUnits
    {
        Meters,
        Centimeters,
        Millimeters,
        Custom,
    };

    AZStd::optional<float> GetUnitMultiplierValue(DistanceUnits units);
} // namespace ROS2
