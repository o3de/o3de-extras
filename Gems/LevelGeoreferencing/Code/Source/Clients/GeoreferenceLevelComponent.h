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
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Components/ComponentAdapter.h>
#include <Georeferencing/GeoreferenceBus.h>

namespace Georeferencing
{
    struct GeoReferenceLevelConfig : public AZ::ComponentConfig
    {
        AZ_RTTI(GeoReferenceLevelConfig, GeoReferenceLevelConfigTypeId, AZ::ComponentConfig);

        static void Reflect(AZ::ReflectContext* context);

        AZ::EntityId m_enuOriginLocationEntityId; //!< EntityId of the entity that lays in the origin of the ENU coordinate system
        WGS::WGS84Coordinate m_originLocation; //!< Location of the entity that lays in the origin of the ENU coordinate system
    };

    class GeoReferenceLevelController
        : private GeoreferenceRequestsBus::Handler
        , private GeoreferenceConfigurationRequestsBus::Handler
        , private AZ::EntityBus::Handler
        , private AZ::TransformNotificationBus::Handler
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
        WGS::WGS84Coordinate ConvertFromLevelToWGS84(const AZ::Vector3& xyz) override;
        AZ::Vector3 ConvertFromWGS84ToLevel(const WGS::WGS84Coordinate& latLon) override;
        AZ::Quaternion GetRotationFromLevelToENU() override;

        // ConfigurationRequestsBus::Handler overrides ...
        void SetOriginEntity(const AZ::EntityId& entityId) override;
        void SetOriginCoordinates(const WGS::WGS84Coordinate& origin) override;
        AZ::EntityId GetOriginEntity() override;
        WGS::WGS84Coordinate GetOriginCoordinates() override;

        // TransformNotificationBus::Handler overrides ...
        void OnTransformChanged(const AZ::Transform& local, const AZ::Transform& world) override;

        void ApplyOriginTransform(const AZ::Transform& worldTransform);

        GeoReferenceLevelConfig m_config;
        AZ::Transform m_enuOriginTransform; //!< Transform of the entity that lays in the origin of the ENU coordinate system
    };

    using GeoReferenceLevelComponentBase = AzFramework::Components::ComponentAdapter<GeoReferenceLevelController, GeoReferenceLevelConfig>;

    class GeoReferenceLevelComponent : public GeoReferenceLevelComponentBase
    {
    public:
        AZ_COMPONENT(GeoReferenceLevelComponent, GeoReferenceLevelComponentTypeId, AZ::Component);
        static void Reflect(AZ::ReflectContext* context);

        GeoReferenceLevelComponent(const GeoReferenceLevelConfig& config);
        GeoReferenceLevelComponent() = default;
        ~GeoReferenceLevelComponent() = default;

        // Component overrides...
        void Activate() override;
        void Deactivate() override;
    };

} // namespace Georeferencing
