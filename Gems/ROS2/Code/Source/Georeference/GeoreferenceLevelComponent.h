/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/ComponentAdapter.h>
#include <ROS2/Georeference/GeoreferenceBus.h>

namespace ROS2
{
    struct GeoReferenceLevelConfig : public AZ::ComponentConfig
    {
        AZ_RTTI(GeoReferenceLevelConfig, "{22866de2-2d34-4510-988d-9f55a07a64c3}", AZ::ComponentConfig);

        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_enuOriginLocationEntityId; //!< EntityId of the entity that lays in the origin of the ENU coordinate system
        WGS::WGS84Coordinate m_originLocation; //!< Location of the entity that lays in the origin of the ENU coordinate system
    };

    class GeoReferenceLevelController
        : private GeoreferenceRequestsBus::Handler
        , private AZ::EntityBus::Handler
    {
    public:
        AZ_TYPE_INFO(GeoReferenceLevelController, "{60b22daa-c241-49d2-ba83-dca380c179b1}");

        static void Reflect(AZ::ReflectContext* context);

        GeoReferenceLevelController() = default;
        GeoReferenceLevelController(const GeoReferenceLevelConfig& config);

        // Controller component ...
        void Init();
        void Activate(AZ::EntityId entityId);
        void Deactivate();
        void SetConfiguration(const GeoReferenceLevelConfig& config);
        const GeoReferenceLevelConfig& GetConfiguration() const;

    private:
        // EntityBus overrides ...
        void OnEntityActivated(const AZ::EntityId& entityId) override;

        // GeoreferenceRequestsBus::Handler overrides ...
        WGS::WGS84Coordinate ConvertFromLevelToWSG84(const AZ::Vector3& xyz) override;
        AZ::Vector3 ConvertFromWSG84ToLevel(const WGS::WGS84Coordinate& latLon) override;
        AZ::Quaternion GetRotationFromLevelToENU() override;

        GeoReferenceLevelConfig m_config;
        AZ::Transform m_enuOriginTransform; //!< Transform of the entity that lays in the origin of the ENU coordinate system
    };

    using GeoReferenceLevelComponentBase = AzFramework::Components::ComponentAdapter<GeoReferenceLevelController, GeoReferenceLevelConfig>;

    class GeoReferenceLevelComponent : public GeoReferenceLevelComponentBase
    {
    public:
        AZ_COMPONENT(GeoReferenceLevelComponent, "{7dcd0112-db23-41b8-90b8-4c66c6a197e4}", AZ::Component);
        static void Reflect(AZ::ReflectContext* context);

        GeoReferenceLevelComponent(const GeoReferenceLevelConfig& config);
        GeoReferenceLevelComponent() = default;
        ~GeoReferenceLevelComponent() = default;

        // Component overrides...
        void Activate() override;
        void Deactivate() override;
    };

} // namespace ROS2
