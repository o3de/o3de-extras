/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Material/PhysicsMaterialAsset.h>

namespace WarehouseAutomation
{
    struct ConveyorBeltComponentConfiguration
    {
        AZ_TYPE_INFO(ConveyorBeltComponentConfiguration, "{ebaf61a0-20c5-11ee-be56-0242ac120002}");
        static void Reflect(AZ::ReflectContext* context);

        AZStd::string m_graphicalMaterialSlot{ "Belt" }; //!< Name of the material slot to change UVs of
        float m_beltWidth = 1.0f; //!< Width of the conveyor belt
        float m_segmentSize = 0.1f; //!< Length of individual segments of the conveyor belt
        AZ::Data::Asset<Physics::MaterialAsset> m_materialAsset; //!< Material of individual segments of the conveyor belt
        AZ::EntityId m_conveyorEntityId; //!< Conveyor belt entity (used for texture movement)
        float m_textureScale = 1.0f; //!< Scaling factor of the texture
        float m_speed = 1.0f; //!< Initial speed of the conveyor belt
    };
} // namespace WarehouseAutomation
