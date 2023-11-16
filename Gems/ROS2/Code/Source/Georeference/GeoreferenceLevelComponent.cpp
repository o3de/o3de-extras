/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "GeoreferenceLevelComponent.h"
#include "GNSSFormatConversions.h"
#include <AzCore/Component/TransformBus.h>
#include <AzCore/Math/Matrix4x4.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Georeference/GeoreferenceStructures.h>

namespace ROS2
{
    void GeoReferenceLevelConfig::Reflect(AZ::ReflectContext* context)
    {
        WGS::WGS84Coordinate::Reflect(context);
        if (auto* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<GeoReferenceLevelConfig, AZ::ComponentConfig>()
                ->Version(1)
                ->Field("EnuOriginWSG84", &GeoReferenceLevelConfig::m_originLocation)
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
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "Manages spawning of robots in configurable locations")
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
        GeoreferenceRequestsBus::Handler::BusConnect();
    }

    void GeoReferenceLevelController::Deactivate()
    {
        GeoreferenceRequestsBus::Handler::BusDisconnect();
        AZ::EntityBus::Handler::BusDisconnect();
    }

    void GeoReferenceLevelController::OnEntityActivated(const AZ::EntityId& entityId)
    {
        m_enuOriginTransform = AZ::Transform::CreateIdentity();
        AZ::TransformBus::EventResult(m_enuOriginTransform, m_config.m_enuOriginLocationEntityId, &AZ::TransformBus::Events::GetWorldTM);
        m_enuOriginTransform.Invert();
        AZ::EntityBus::Handler::BusDisconnect();
    }

    WGS::WGS84Coordinate GeoReferenceLevelController::ConvertFromLevelToWSG84(const AZ::Vector3& xyz)
    {
        using namespace ROS2::Utils::GeodeticConversions;
        const auto enu = WGS::Vector3d(m_enuOriginTransform.TransformPoint(xyz));
        const auto ecef = ENUToECEF(m_config.m_originLocation, enu);
        return ECEFToWGS84(ecef);
    }

    AZ::Vector3 GeoReferenceLevelController::ConvertFromWSG84ToLevel(const WGS::WGS84Coordinate& latLon)
    {
        using namespace ROS2::Utils::GeodeticConversions;
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

} // namespace ROS2
