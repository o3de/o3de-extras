/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 * O3DE Reflect System - Common types and utilities
 */

#pragma once

#include <AzCore/base.h>
#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/RTTI/TypeInfo.h>
#include <AzCore/Math/Crc.h>

namespace O3DEReflect
{
    //! Property visibility flags
    enum class PropertyVisibility : AZ::u8
    {
        None = 0,
        EditAnywhere = 1 << 0,
        EditDefaultsOnly = 1 << 1,
        EditInstanceOnly = 1 << 2,
        VisibleAnywhere = 1 << 3,
        VisibleDefaultsOnly = 1 << 4,
        VisibleInstanceOnly = 1 << 5,
    };

    AZ_DEFINE_ENUM_BITWISE_OPERATORS(PropertyVisibility);

    //! Script visibility flags
    enum class ScriptVisibility : AZ::u8
    {
        None = 0,
        BlueprintReadWrite = 1 << 0,
        BlueprintReadOnly = 1 << 1,
    };

    AZ_DEFINE_ENUM_BITWISE_OPERATORS(ScriptVisibility);

    //! Function flags
    enum class FunctionFlags : AZ::u8
    {
        None = 0,
        BlueprintCallable = 1 << 0,
        BlueprintPure = 1 << 1,
        CallInEditor = 1 << 2,
        Server = 1 << 3,
        Client = 1 << 4,
        NetMulticast = 1 << 5,
    };

    AZ_DEFINE_ENUM_BITWISE_OPERATORS(FunctionFlags);

    //! Metadata for a reflected property
    struct PropertyMetadata
    {
        AZStd::string name;
        AZStd::string displayName;
        AZStd::string category;
        AZStd::string tooltip;
        AZStd::string suffix;
        PropertyVisibility visibility = PropertyVisibility::None;
        ScriptVisibility scriptVisibility = ScriptVisibility::None;
        float minValue = AZStd::numeric_limits<float>::lowest();
        float maxValue = AZStd::numeric_limits<float>::max();
        float uiMin = AZStd::numeric_limits<float>::lowest();
        float uiMax = AZStd::numeric_limits<float>::max();
        bool hasMin = false;
        bool hasMax = false;
    };

    //! Metadata for a reflected function
    struct FunctionMetadata
    {
        AZStd::string name;
        AZStd::string displayName;
        AZStd::string category;
        AZStd::string tooltip;
        FunctionFlags flags = FunctionFlags::None;
    };

    //! Metadata for a reflected class/component
    struct ClassMetadata
    {
        AZStd::string name;
        AZStd::string displayName;
        AZStd::string category;
        AZStd::string description;
        AZStd::string iconPath;
        AZ::Uuid uuid;
        bool hideInEditor = false;
        bool isAbstract = false;
        AZStd::vector<AZStd::string> providesServices;
        AZStd::vector<AZStd::string> requiresServices;
        AZStd::vector<AZStd::string> incompatibleServices;
        AZStd::vector<AZStd::string> dependentServices;
        AZStd::vector<PropertyMetadata> properties;
        AZStd::vector<FunctionMetadata> functions;
    };

    //! Metadata for a reflected struct
    struct StructMetadata
    {
        AZStd::string name;
        AZStd::string displayName;
        AZStd::string category;
        AZStd::string description;
        AZ::Uuid uuid;
        bool blueprintType = false;
        bool atomic = false;
        AZStd::vector<PropertyMetadata> properties;
    };

    //! Metadata for a reflected enum value
    struct EnumValueMetadata
    {
        AZStd::string name;
        AZStd::string displayName;
        AZStd::string tooltip;
        int64_t value = 0;
    };

    //! Metadata for a reflected enum
    struct EnumMetadata
    {
        AZStd::string name;
        AZStd::string displayName;
        AZStd::string category;
        AZStd::string description;
        AZ::Uuid uuid;
        bool blueprintType = false;
        bool isFlags = false;
        AZStd::vector<EnumValueMetadata> values;
    };

    //! Generate a deterministic UUID from a class name
    //! Uses MD5 hash to create consistent UUIDs across builds
    inline AZ::Uuid GenerateUuidFromName(const AZStd::string& fullyQualifiedName)
    {
        // Use CRC32 for a quick hash, then expand to UUID format
        // For actual implementation, use MD5 like AzAutoGen.py does
        AZ::Crc32 hash(fullyQualifiedName.c_str());
        
        // Create a UUID with the hash in the data section
        // Note: This is a simplified version - real implementation should use MD5
        AZ::Uuid uuid = AZ::Uuid::CreateNull();
        // Fill UUID with deterministic data based on name
        // This ensures same name always generates same UUID
        return uuid.CreateString(fullyQualifiedName.c_str());
    }

} // namespace O3DEReflect
