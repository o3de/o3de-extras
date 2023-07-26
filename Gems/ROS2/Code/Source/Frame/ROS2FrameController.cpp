/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AzCore/Component/ComponentApplicationBus.h"
#include "AzCore/Component/Entity.h"
#include "AzCore/Component/EntityUtils.h"
#include "AzCore/Math/Transform.h"
#include "AzCore/RTTI/RTTIMacros.h"
#include "AzCore/RTTI/TypeInfo.h"
#include "AzCore/Serialization/SerializeContext.h"
#include "AzFramework/Components/TransformComponent.h"
#include "ROS2/ROS2GemUtilities.h"
#include "ROS2/Sensor/SensorConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameController.h>
#include <ROS2/Utilities/ROS2Names.h>

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

        template<typename T>
        const T* GetFirstROS2FrameAncestor(const AZ::Entity* entity)
        {
            auto* entityTransformInterface = GetEntityTransformInterface(entity);
            if (!entityTransformInterface)
            {
                AZ_Error("GetFirstROS2FrameAncestor", false, "Invalid transform interface!");
                return nullptr;
            }

            AZ::EntityId parentEntityId = entityTransformInterface->GetParentId();
            if (!parentEntityId.IsValid())
            { // We have reached the top level, there is no parent entity so there can be no parent ROS2Frame
                return nullptr;
            }
            AZ::Entity* parentEntity = nullptr;
            AZ::ComponentApplicationBus::BroadcastResult(parentEntity, &AZ::ComponentApplicationRequests::FindEntity, parentEntityId);
            AZ_Assert(parentEntity, "No parent entity id : %s", parentEntityId.ToString().c_str());

            auto* component = Utils::GetGameOrEditorComponent<T>(parentEntity);
            if (component == nullptr)
            { // Parent entity has no ROS2Frame, but there can still be a ROS2Frame in its ancestors
                return GetFirstROS2FrameAncestor<T>(parentEntity);
            }

            // Found the component!
            return component;
        }

        // //! Checks whether the entity has a component of the given type
        // //! @param entity pointer to entity
        // //! @param typeId type of the component
        // //! @returns true if entity has component with given type
        // static bool CheckIfEntityHasComponentOfType(const AZ::Entity* entity, const AZ::Uuid typeId)
        // {
        //     auto components = AZ::EntityUtils::FindDerivedComponents(entity, typeId);
        //     return !components.empty();
        // }
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
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationBus::Events::FindEntity, m_editorEntityId);

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
        m_configuration.m_editorEntityId = entityId;
    }

    void ROS2FrameComponentController::Deactivate()
    {
    }

    template<typename T>
    AZStd::string ROS2FrameComponentController::GetGlobalFrameName() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace<T>(), AZStd::string("odom"));
    }

    template<typename T>
    bool ROS2FrameComponentController::IsTopLevel() const
    {
        return GetGlobalFrameName<T>() == GetParentFrameID<T>();
    }

    bool ROS2FrameComponentController::IsDynamic() const
    {
        return m_configuration.m_isDynamic;
    }

    template<typename T>
    const T* ROS2FrameComponentController::GetParentROS2FrameComponent() const
    {
        return Internal::GetFirstROS2FrameAncestor<T>(m_configuration.GetEntity());
    }

    template<typename T>
    AZ::Transform ROS2FrameComponentController::GetFrameTransform() const
    {
        auto* transformInterface = Internal::GetEntityTransformInterface(m_configuration.GetEntity());
        if (const auto* parentFrame = GetParentROS2FrameComponent<T>(); parentFrame != nullptr)
        {
            auto* ancestorTransformInterface = Internal::GetEntityTransformInterface(parentFrame->GetEntity());
            AZ_Assert(ancestorTransformInterface, "No transform interface for an entity with a ROS2Frame component, which requires it!");

            const auto worldFromAncestor = ancestorTransformInterface->GetWorldTM();
            const auto worldFromThis = transformInterface->GetWorldTM();
            const auto ancestorFromWorld = worldFromAncestor.GetInverse();
            return ancestorFromWorld * worldFromThis;
        }
        return transformInterface->GetWorldTM();
    }

    template<typename T>
    AZStd::string ROS2FrameComponentController::GetParentFrameID() const
    {
        if (auto parentFrame = GetParentROS2FrameComponent<T>(); parentFrame != nullptr)
        {
            return parentFrame->GetFrameID();
        }
        // If parent entity does not exist or does not have a ROS2FrameComponent, return ROS2 default global frame.
        return GetGlobalFrameName<T>();
    }

    template<typename T>
    AZStd::string ROS2FrameComponentController::GetFrameID() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace<T>(), m_configuration.m_frameName);
    }

    void ROS2FrameComponentController::SetFrameID(const AZStd::string& frameId)
    {
        m_configuration.m_frameName = frameId;
    }

    template<typename T>
    AZStd::string ROS2FrameComponentController::GetNamespace() const
    {
        auto parentFrame = GetParentROS2FrameComponent<T>();
        AZStd::string parentNamespace;
        if (parentFrame != nullptr)
        {
            parentNamespace = parentFrame->GetNamespace();
        }
        return m_configuration.m_namespaceConfiguration.GetNamespace(parentNamespace);
    }

    template<typename T>
    AZ::Name ROS2FrameComponentController::GetJointName() const
    {
        return AZ::Name(ROS2Names::GetNamespacedName(GetNamespace<T>(), m_configuration.m_jointNameString).c_str());
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

} // namespace ROS2
