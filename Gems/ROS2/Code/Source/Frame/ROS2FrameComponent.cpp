/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ROS2FrameSystemComponent.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityUtils.h>
#include <AzCore/RTTI/ReflectContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/Json/JsonSerialization.h>
#include <AzCore/Serialization/Json/JsonSerializationResult.h>
#include <AzCore/Serialization/Json/RegistrationContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/ROS2Bus.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <rapidjson/document.h>
#include <rapidjson/stringbuffer.h>

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

        const ROS2FrameComponent* GetFirstROS2FrameAncestor(const AZ::Entity* entity)
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

            auto* component = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(parentEntity);
            if (component == nullptr)
            { // Parent entity has no ROS2Frame, but there can still be a ROS2Frame in its ancestors
                return GetFirstROS2FrameAncestor(parentEntity);
            }

            // Found the component!
            return component;
        }

        //! Checks whether the entity has a component of the given type
        //! @param entity pointer to entity
        //! @param typeId type of the component
        //! @returns true if entity has component with given type
        static bool CheckIfEntityHasComponentOfType(const AZ::Entity* entity, const AZ::Uuid typeId)
        {
            auto components = AZ::EntityUtils::FindDerivedComponents(entity, typeId);
            return !components.empty();
        }

    } // namespace Internal

    AZ::JsonSerializationResult::Result JsonFrameComponentConfigSerializer::Load(
        void* outputValue, const AZ::Uuid& outputValueTypeId, const rapidjson::Value& inputValue, AZ::JsonDeserializerContext& context)
    {
        AZ_Error(
            "ROS2FrameComponent",
            false,
            "An old version of the ROS2FrameComponent is being loaded. Manual conversion is required. The conversion script is "
            "located in: "
            "o3de-extras/Gems/ROS2/Code/Source/Frame/Conversions/FrameConversion.py");

        namespace JSR = AZ::JsonSerializationResult;

        auto configInstance = reinterpret_cast<ROS2FrameComponent*>(outputValue);
        AZ_Assert(configInstance, "Output value for JsonFrameComponentConfigSerializer can't be null.");

        JSR::ResultCode result(JSR::Tasks::ReadField);

        {
            JSR::ResultCode componentIdLoadResult = ContinueLoadingFromJsonObjectField(
                &configInstance->m_jointName, azrtti_typeid<decltype(configInstance->m_jointName)>(), inputValue, "Joint Name", context);

            result.Combine(componentIdLoadResult);
        }
        {
            JSR::ResultCode componentIdLoadResult = ContinueLoadingFromJsonObjectField(
                &configInstance->m_frameName, azrtti_typeid<decltype(configInstance->m_frameName)>(), inputValue, "Frame Name", context);

            result.Combine(componentIdLoadResult);
        }
        {
            JSR::ResultCode componentIdLoadResult = ContinueLoadingFromJsonObjectField(
                &configInstance->m_publishTransform,
                azrtti_typeid<decltype(configInstance->m_publishTransform)>(),
                inputValue,
                "Publish Transform",
                context);

            result.Combine(componentIdLoadResult);
        }
        {
            JSR::ResultCode componentIdLoadResult = ContinueLoadingFromJsonObjectField(
                &configInstance->m_namespaceConfiguration,
                azrtti_typeid<decltype(configInstance->m_namespaceConfiguration)>(),
                inputValue,
                "Namespace Configuration",
                context);

            result.Combine(componentIdLoadResult);
        }

        return context.Report(
            result,
            result.GetProcessing() != JSR::Processing::Halted ? "Successfully loaded ROS2FrameComponent information."
                                                              : "Failed to load ROS2FrameComponent information.");
    }

    AZ_CLASS_ALLOCATOR_IMPL(JsonFrameComponentConfigSerializer, AZ::SystemAllocator);

    void ROS2FrameComponent::Activate()
    {
        m_namespaceConfiguration.PopulateNamespace(IsTopLevel(), GetEntity()->GetName());

        if (m_publishTransform)
        {
            AZ_TracePrintf("ROS2FrameComponent", "Setting up %s", GetFrameID().data());

            // The frame will always be dynamic if it's a top entity.
            if (IsTopLevel())
            {
                m_isDynamic = true;
            }
            // Otherwise it'll be dynamic when it has joints and it's not a fixed joint.
            else
            {
                const bool hasJoints = Internal::CheckIfEntityHasComponentOfType(
                    m_entity, AZ::Uuid("{B01FD1D2-1D91-438D-874A-BF5EB7E919A8}")); // Physx::JointComponent;
                const bool hasFixedJoints = Internal::CheckIfEntityHasComponentOfType(
                    m_entity, AZ::Uuid("{02E6C633-8F44-4CEE-AE94-DCB06DE36422}")); // Physx::FixedJointComponent
                const bool hasArticulations = Internal::CheckIfEntityHasComponentOfType(
                    m_entity, AZ::Uuid("{48751E98-B35F-4A2F-A908-D9CDD5230264}")); // Physx::ArticulationComponent
                m_isDynamic = (hasJoints && !hasFixedJoints) || hasArticulations;
            }

            AZ_TracePrintf(
                "ROS2FrameComponent",
                "Setting up %s transform between parent %s and child %s to be published %s\n",
                IsDynamic() ? "dynamic" : "static",
                GetParentFrameID().data(),
                GetFrameID().data(),
                IsDynamic() ? "continuously to /tf" : "once to /tf_static");

            m_ros2Transform = AZStd::make_unique<ROS2Transform>(GetParentFrameID(), GetFrameID(), IsDynamic());
            if (IsDynamic())
            {
                AZ::TickBus::Handler::BusConnect();
            }
            else
            {
                m_ros2Transform->Publish(GetFrameTransform());
            }
        }
    }

    void ROS2FrameComponent::Deactivate()
    {
        if (m_publishTransform)
        {
            if (IsDynamic())
            {
                AZ::TickBus::Handler::BusDisconnect();
            }
            m_ros2Transform.reset();
        }
    }

    void ROS2FrameComponent::OnTick([[maybe_unused]] float deltaTime, [[maybe_unused]] AZ::ScriptTimePoint time)
    {
        m_ros2Transform->Publish(GetFrameTransform());
    }

    AZStd::string ROS2FrameComponent::GetGlobalFrameName() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), AZStd::string("odom"));
    }

    void ROS2FrameComponent::UpdateNamespaceConfiguration(
        const AZStd::string& ros2Namespace, NamespaceConfiguration::NamespaceStrategy strategy)
    {
        m_namespaceConfiguration.SetNamespace(ros2Namespace, strategy);
    }

    bool ROS2FrameComponent::IsTopLevel() const
    {
        return GetGlobalFrameName() == GetParentFrameID();
    }

    bool ROS2FrameComponent::IsDynamic() const
    {
        return m_isDynamic;
    }

    const ROS2FrameComponent* ROS2FrameComponent::GetParentROS2FrameComponent() const
    {
        return Internal::GetFirstROS2FrameAncestor(GetEntity());
    }

    AZ::Transform ROS2FrameComponent::GetFrameTransform() const
    {
        auto* transformInterface = Internal::GetEntityTransformInterface(GetEntity());
        if (const auto* parentFrame = GetParentROS2FrameComponent(); parentFrame != nullptr)
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

    AZStd::string ROS2FrameComponent::GetParentFrameID() const
    {
        if (auto parentFrame = GetParentROS2FrameComponent(); parentFrame != nullptr)
        {
            return parentFrame->GetFrameID();
        }
        // If parent entity does not exist or does not have a ROS2FrameComponent, return ROS2 default global frame.
        return GetGlobalFrameName();
    }

    AZStd::string ROS2FrameComponent::GetFrameID() const
    {
        return ROS2Names::GetNamespacedName(GetNamespace(), m_frameName);
    }

    void ROS2FrameComponent::SetFrameID(const AZStd::string& frameId)
    {
        m_frameName = frameId;
    }

    AZStd::string ROS2FrameComponent::GetNamespace() const
    {
        auto parentFrame = GetParentROS2FrameComponent();
        AZStd::string parentNamespace;
        if (parentFrame != nullptr)
        {
            parentNamespace = parentFrame->GetNamespace();
        }
        return m_namespaceConfiguration.GetNamespace(parentNamespace);
    }

    AZ::Name ROS2FrameComponent::GetJointName() const
    {
        return AZ::Name(ROS2Names::GetNamespacedName(GetNamespace(), m_jointName).c_str());
    }

    void ROS2FrameComponent::SetJointName(const AZStd::string& jointName)
    {
        m_jointName = jointName;
    }

    void ROS2FrameComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto jsonContext = azrtti_cast<AZ::JsonRegistrationContext*>(context))
        {
            jsonContext->Serializer<JsonFrameComponentConfigSerializer>()->HandlesType<ROS2FrameComponent>();
        }

        ROS2FrameConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameComponent, AZ::Component>()
                ->Version(1)
                ->Field("Frame Name", &ROS2FrameComponent::m_frameName)
                ->Field("Joint Name", &ROS2FrameComponent::m_jointName)
                ->Field("Publish Transform", &ROS2FrameComponent::m_publishTransform)
                ->Field("Namespace Configuration", &ROS2FrameComponent::m_namespaceConfiguration);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameComponent>(
                      "ROS2 Frame Game Component (outdated)",
                      "This is a game version of the ROS2 Frame component. This is outdated and was updated to the new "
                      "ROS2FrameEditorComponent. If you see this component a manual conversion is required.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Icons/Components/ROS2Frame.svg")
                    ->Attribute(AZ::Edit::Attributes::ViewportIcon, "Icons/Components/Viewport/ROS2Frame.svg")
                    ->Attribute(AZ::Edit::Attributes::HelpPageURL, "https://o3de.org/docs/user-guide/components/reference/ros2-frame/")
                    ->UIElement(
                        AZ::Edit::UIHandlers::Label,
                        "This component is no longer supported. Manual conversion to the ROS2FrameEditorComponent is required.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameComponent::m_frameName, "Frame Name", "Name of the frame.")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameComponent::m_jointName, "Joint Name", "Name of the joint.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameComponent::m_publishTransform,
                        "Publish Transform",
                        "Publish the transform of this frame.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameComponent::m_namespaceConfiguration,
                        "Namespace Configuration",
                        "Configuration of the namespace for this frame.");
            }
        }
    }

    void ROS2FrameComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("ROS2Frame"));
    }

    void ROS2FrameComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("TransformService"));
    }

    ROS2FrameComponent::ROS2FrameComponent(){};

    ROS2FrameComponent::ROS2FrameComponent(const ROS2FrameConfiguration& configuration)
        : m_namespaceConfiguration(configuration.m_namespaceConfiguration)
        , m_frameName(configuration.m_frameName)
        , m_jointName(configuration.m_jointName)
        , m_publishTransform(configuration.m_publishTransform)
        , m_isDynamic(configuration.m_isDynamic){};

    ROS2FrameConfiguration ROS2FrameComponent::GetConfiguration() const
    {
        ROS2FrameConfiguration configuration;
        configuration.m_namespaceConfiguration = m_namespaceConfiguration;
        configuration.m_frameName = m_frameName;
        configuration.m_jointName = m_jointName;
        configuration.m_publishTransform = m_publishTransform;
        configuration.m_isDynamic = m_isDynamic;

        return configuration;
    }
} // namespace ROS2
