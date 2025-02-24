/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GeoreferenceLevelComponent.h"
#include "GNSSFormatConversions.h"
#include "GeoreferenceInternalStructures.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <Georeferencing/GeoreferenceStructures.h>
namespace Georeferencing
{
    void GeoReferenceLevelConfig::Reflect(AZ::ReflectContext* context)
    {
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoReferenceLevelConfig, AZ::ComponentConfig>()
                ->Version(1)
                ->Field("EnuOriginWGS84", &GeoReferenceLevelConfig::m_originLocation)
                ->Field("EnuOriginLocationEntityId", &GeoReferenceLevelConfig::m_enuOriginLocationEntityId);

            if (auto* editContext = serialize->GetEditContext())
            {
                editContext->Class<GeoReferenceLevelConfig>("Georeference config", "Georeference config")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoReferenceLevelConfig::m_enuOriginLocationEntityId,
                        "ENU Origin Transform",
                        "ENU (East-North-Up) origin in the level")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &GeoReferenceLevelConfig::m_originLocation,
                        "ENU Origin Coordinates in WGS84",
                        "ENU Origin Coordinates in WGS84");
            }
        }
        if (AZ::BehaviorContext* behaviorContext = azrtti_cast<AZ::BehaviorContext*>(context))
        {
            behaviorContext->EBus<GeoreferenceRequestsBus>("GeoreferenceRequestsBus")
                ->Attribute(AZ::Script::Attributes::Scope, AZ::Script::Attributes::ScopeFlags::Common)
                ->Attribute(AZ::Script::Attributes::Category, "GeoreferenceRequests")
                ->Attribute(AZ::Script::Attributes::Module, "GeoreferenceRequests")
                ->Event("ConvertFromLevelToWGS84", &GeoreferenceRequests::ConvertFromLevelToWGS84)
                ->Event("ConvertFromWGS84ToLevel", &GeoreferenceRequests::ConvertFromWGS84ToLevel)
                ->Event("GetRotationFromLevelToENU", &GeoreferenceRequests::GetRotationFromLevelToENU);
        }
    }

    void GeoReferenceLevelController::Reflect(AZ::ReflectContext* context)
    {
        GeoReferenceLevelConfig::Reflect(context);

        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<GeoReferenceLevelController>()->Version(1)->Field(
                "Configuration", &GeoReferenceLevelController::m_config);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<GeoReferenceLevelController>("GeoReferenceLevelController", "Controller for GeoReferenceLevelComponent")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Manages Georeferencing of the level")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GeoReferenceLevelController::m_config);
            }
        }
    }
    GeoReferenceLevelController::GeoReferenceLevelController(const GeoReferenceLevelConfig& config)
        : m_config(config)
    {
    }

    void GeoReferenceLevelController::Init()
    {
    }

    void GeoReferenceLevelController::Activate(AZ::EntityId entityId)
    {
        AZ::EntityBus::Handler::BusConnect(m_config.m_enuOriginLocationEntityId);
        AZ::TransformNotificationBus::Handler::BusConnect(m_config.m_enuOriginLocationEntityId);
        GeoreferenceRequestsBus::Handler::BusConnect();
        GeoreferenceConfigurationRequestsBus::Handler::BusConnect();
    }

    void GeoReferenceLevelController::Deactivate()
    {
        GeoreferenceConfigurationRequestsBus::Handler::BusDisconnect();
        GeoreferenceRequestsBus::Handler::BusDisconnect();
        AZ::TransformNotificationBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
    }

    void GeoReferenceLevelController::OnEntityActivated(const AZ::EntityId& entityId)
    {
        AZ_Assert(entityId == m_config.m_enuOriginLocationEntityId, "Entity activated is not the origin entity");
        m_enuOriginTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(m_enuOriginTransform, m_config.m_enuOriginLocationEntityId, &AZ::TransformBus::Events::GetWorldTM);
        m_enuOriginTransform.Invert();
        AZ::TransformNotificationBus::Handler::BusConnect(m_config.m_enuOriginLocationEntityId);
        AZ::EntityBus::Handler::BusDisconnect();
    }

    void GeoReferenceLevelController::OnTransformChanged([[maybe_unused]] const AZ::Transform& local, const AZ::Transform& world)
    {
        ApplyOriginTransform(world);
    }

    void GeoReferenceLevelController::ApplyOriginTransform(const AZ::Transform& worldTransform)
    {
        m_enuOriginTransform = worldTransform.GetInverse();
    }

    WGS::WGS84Coordinate GeoReferenceLevelController::ConvertFromLevelToWGS84(const AZ::Vector3& xyz)
    {
        using namespace Georeferencing::Utils::GeodeticConversions;
        const auto enu = WGS::Vector3d(m_enuOriginTransform.TransformPoint(xyz));
        const auto ecef = ENUToECEF(m_config.m_originLocation, enu);
        return ECEFToWGS84(ecef);
    }

    AZ::Vector3 GeoReferenceLevelController::ConvertFromWGS84ToLevel(const WGS::WGS84Coordinate& latLon)
    {
        using namespace Georeferencing::Utils::GeodeticConversions;
        const auto ecef = WGS84ToECEF(latLon);
        const auto enu = ECEFToENU(m_config.m_originLocation, ecef);
        return m_enuOriginTransform.GetInverse().TransformPoint(enu.ToVector3f());
    };

    AZ::Quaternion GeoReferenceLevelController::GetRotationFromLevelToENU()
    {
        return m_enuOriginTransform.GetRotation();
    };

    void GeoReferenceLevelController::SetConfiguration(const GeoReferenceLevelConfig& config)
    {
        m_config = config;
        SetOriginEntity(m_config.m_enuOriginLocationEntityId);
    }

    void GeoReferenceLevelController::SetOriginEntity(const AZ::EntityId& entityId)
    {
        m_config.m_enuOriginLocationEntityId = entityId;
        AZ::Transform transform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(transform, m_config.m_enuOriginLocationEntityId, &AZ::TransformBus::Events::GetWorldTM);
        ApplyOriginTransform(transform);
        if (AZ::EntityBus::Handler::BusIsConnected())
        {
            AZ::EntityBus::Handler::BusDisconnect();
        }
        if (AZ::TransformNotificationBus::Handler::BusIsConnected())
        {
            AZ::TransformNotificationBus::Handler::BusDisconnect();
        }
        AZ::EntityBus::Handler::BusConnect(m_config.m_enuOriginLocationEntityId);
        AZ::TransformNotificationBus::Handler::BusConnect(m_config.m_enuOriginLocationEntityId);
    }

    void GeoReferenceLevelController::SetOriginCoordinates(const WGS::WGS84Coordinate& origin)
    {
        m_config.m_originLocation = origin;
    }

    AZ::EntityId GeoReferenceLevelController::GetOriginEntity()
    {
        return m_config.m_enuOriginLocationEntityId;
    }

    WGS::WGS84Coordinate GeoReferenceLevelController::GetOriginCoordinates()
    {
        return m_config.m_originLocation;
    }

    const GeoReferenceLevelConfig& GeoReferenceLevelController::GetConfiguration() const
    {
        return m_config;
    }

    GeoReferenceLevelComponent::GeoReferenceLevelComponent(const GeoReferenceLevelConfig& config)
        : GeoReferenceLevelComponentBase(config)
    {
    }

    void GeoReferenceLevelComponent::Reflect(AZ::ReflectContext* context)
    {
        GeoReferenceLevelComponentBase::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoReferenceLevelComponent, GeoReferenceLevelComponentBase>()->Version(1);
        }
    }

    void GeoReferenceLevelComponent::Activate()
    {
        GeoReferenceLevelComponentBase::Activate();
    }

    void GeoReferenceLevelComponent::Deactivate()
    {
        GeoReferenceLevelComponentBase::Deactivate();
    }
} // namespace Georeferencing
