/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AzCore/Component/ComponentApplicationBus.h"
#include "AzCore/Component/Entity.h"
#include "AzCore/Component/EntityId.h"
#include "AzCore/Component/EntityUtils.h"
#include "AzCore/Component/TransformBus.h"
#include "AzCore/Math/Transform.h"
#include "AzCore/RTTI/RTTIMacros.h"
#include "AzCore/RTTI/TypeInfo.h"
#include "AzCore/Serialization/SerializeContext.h"
#include "AzFramework/Components/TransformComponent.h"
#include "ROS2/Frame/ROS2FrameBus.h"
#include "ROS2/ROS2GemUtilities.h"
#include "ROS2/Sensor/SensorConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameController.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <iostream>

namespace ROS2
{

    namespace Internal
    {
        AZ::TransformInterface* GetEntityTransformInterface(const AZ::Entity* entity)
        {
            if (!entity)
            {
                AZ_Error("GetEntityTransformInterface", false, "Invalid entity!");
                return nullptr;
            }

            auto* interface = Utils::GetGameOrEditorComponent<AzFramework::TransformComponent>(entity);

            return interface;
        }

        const AZ::EntityId GetFirstROS2FrameAncestorEntityId(const AZ::EntityId entityId)
        {
            AZ::EntityId parentEntityId;
            AZ::TransformBus::EventResult(parentEntityId, entityId, &AZ::TransformBus::Events::GetParentId);
            if (!parentEntityId.IsValid())
            { // We have reached the top level, there is no parent entity so there can be no parent ROS2Frame
                return parentEntityId;
            }

            bool hasFrame = false;
            ROS2FrameComponentBus::EventResult(hasFrame, parentEntityId, &ROS2FrameComponentBus::Events::IsFrame);

            if (!hasFrame)
            { // Parent entity has no ROS2Frame, but there can still be a ROS2Frame in its ancestors
                return GetFirstROS2FrameAncestorEntityId(parentEntityId);
            }

            // Found the component!
            return parentEntityId;
        }

    } // namespace Internal

    void ROS2FrameConfiguration::Reflect(AZ::ReflectContext* context)
    {
        NamespaceConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameConfiguration>()
                ->Version(1)
                ->Field("Namespace Configuration", &ROS2FrameConfiguration::m_namespaceConfiguration)
                ->Field("Frame Name", &ROS2FrameConfiguration::m_frameName)
                ->Field("Joint Name", &ROS2FrameConfiguration::m_jointNameString)
                ->Field("Publish Transform", &ROS2FrameConfiguration::m_publishTransform);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameConfiguration>("ROS2 Frame configuration", "[ROS2 Frame component configuration]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameConfiguration::m_namespaceConfiguration,
                        "Namespace Configuration",
                        "Namespace Configuration")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameConfiguration::m_frameName, "Frame Name", "Frame Name")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameConfiguration::m_jointNameString, "Joint Name", "Joint Name")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameConfiguration::m_publishTransform,
                        "Publish Transform",
                        "Publish Transform");
            }
        }
    }

    AZ::Entity* ROS2FrameConfiguration::GetEntity() const
    {
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationBus::Events::FindEntity, m_activeEntityId);

        return entity;
    }

    void ROS2FrameComponentController::Reflect(AZ::ReflectContext* context)
    {
        ROS2FrameConfiguration::Reflect(context);

        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameComponentController>()->Version(1)->Field(
                "Configuration", &ROS2FrameComponentController::m_configuration);

            AZ::EditContext* editContext = serialize->GetEditContext();
            if (editContext)
            {
                editContext->Class<ROS2FrameComponentController>("ROS2FrameConfiguration", "Configuration of the ROS2 frame")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameComponentController::m_configuration);
            }
        }
    }

    ROS2FrameComponentController::ROS2FrameComponentController(const ROS2FrameConfiguration& config)
    {
        SetConfiguration(config);
    }

    void ROS2FrameComponentController::SetConfiguration(const ROS2FrameConfiguration& config)
    {
        m_configuration = config;
    }

    const ROS2FrameConfiguration& ROS2FrameComponentController::GetConfiguration() const
    {
        return m_configuration;
    }

    void ROS2FrameComponentController::Activate(AZ::EntityId entityId)
    {
        m_configuration.m_activeEntityId = entityId;
    }

    void ROS2FrameComponentController::Deactivate()
    {
    }

    AZStd::string ROS2FrameComponentController::GetGlobalFrameName() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), AZStd::string("odom"));
    }

    bool ROS2FrameComponentController::IsTopLevel() const
    {
        return GetGlobalFrameName() == GetParentFrameID();
    }

    bool ROS2FrameComponentController::IsDynamic() const
    {
        return m_configuration.m_isDynamic;
    }

    const AZ::EntityId ROS2FrameComponentController::GetParentROS2FrameComponentId() const
    {
        return Internal::GetFirstROS2FrameAncestorEntityId(m_configuration.m_activeEntityId);
    }

    AZ::Transform ROS2FrameComponentController::GetFrameTransform() const
    {
        AZ::Transform worldFromThis{};
        AZ::TransformBus::EventResult(worldFromThis, m_configuration.m_activeEntityId, &AZ::TransformBus::Events::GetWorldTM);
        const auto parentFrameId = GetParentROS2FrameComponentId();
        if (parentFrameId.IsValid())
        {
            AZ::Transform worldFromAncestor{};
            AZ::TransformBus::EventResult(worldFromAncestor, parentFrameId, &AZ::TransformBus::Events::GetWorldTM);
            const auto ancestorFromWorld = worldFromAncestor.GetInverse();
            return ancestorFromWorld * worldFromThis;
        }
        return worldFromThis;
    }

    AZStd::string ROS2FrameComponentController::GetParentFrameID() const
    {
        const auto parentFrameEntityId = GetParentROS2FrameComponentId();
        if (parentFrameEntityId.IsValid())
        {
            AZStd::string parentFrameId;
            ROS2FrameComponentBus::EventResult(parentFrameId, parentFrameEntityId, &ROS2FrameComponentBus::Events::GetFrameID);
            return parentFrameId;
        }
        // If parent entity does not exist or does not have a ROS2FrameComponent, return ROS2 default global frame.
        return GetGlobalFrameName();
    }

    AZStd::string ROS2FrameComponentController::GetFrameID() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), m_configuration.m_frameName);
    }

    void ROS2FrameComponentController::SetFrameID(const AZStd::string& frameId)
    {
        m_configuration.m_frameName = frameId;
    }

    AZStd::string ROS2FrameComponentController::GetNamespace() const
    {
        auto parentFrameId = GetParentROS2FrameComponentId();
        AZStd::string parentNamespace;
        if (parentFrameId.IsValid())
        {
            ROS2FrameComponentBus::EventResult(parentNamespace, parentFrameId, &ROS2FrameComponentBus::Events::GetNamespace);
        }
        return m_configuration.m_namespaceConfiguration.GetNamespace(parentNamespace);
    }

    AZ::Name ROS2FrameComponentController::GetJointName() const
    {
        return AZ::Name(ROS2Names::GetNamespacedName(GetNamespace(), m_configuration.m_jointNameString).c_str());
    }

    void ROS2FrameComponentController::SetJointName(const AZStd::string& jointNameString)
    {
        m_configuration.m_jointNameString = jointNameString;
    }

    void ROS2FrameComponentController::PopulateNamespace(bool isRoot, const AZStd::string& entityName)
    {
        m_configuration.m_namespaceConfiguration.PopulateNamespace(isRoot, entityName);
    }

    void ROS2FrameComponentController::SetIsDynamic(bool value)
    {
        m_configuration.m_isDynamic = value;
    }

    bool ROS2FrameComponentController::GetPublishTransform()
    {
        return m_configuration.m_publishTransform;
    }

    void ROS2FrameComponentController::SetActiveEntityId(AZ::EntityId entityId)
    {
        m_configuration.m_activeEntityId = entityId;
    }

} // namespace ROS2
