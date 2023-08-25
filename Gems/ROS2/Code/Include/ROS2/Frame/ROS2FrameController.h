/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Component/Entity.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Components/ComponentAdapter.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameBus.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{

    class ROS2FrameConfiguration final : public AZ::ComponentConfig
    {
    public:
        AZ_TYPE_INFO(ROS2FrameConfiguration, "{04882f01-5451-4efa-b4f8-cd57e4b6cadf}");
        static void Reflect(AZ::ReflectContext* context);

        AZ::Entity* GetEntity() const;

        NamespaceConfiguration m_namespaceConfiguration;
        AZStd::string m_frameName = "sensor_frame";
        AZStd::string m_jointNameString;

        bool m_publishTransform = true;
        bool m_isDynamic = false;

        AZ::EntityId m_activeEntityId;
    };

    class ROS2FrameComponentController : public ROS2FrameComponentBus::Handler
    {
    public:
        AZ_TYPE_INFO(ROS2FrameComponentController, "{f96f3711-56a8-478d-b270-17681e5bb136}");

        ROS2FrameComponentController() = default;
        explicit ROS2FrameComponentController(const ROS2FrameConfiguration& config);
        ~ROS2FrameComponentController() = default;

        static void Reflect(AZ::ReflectContext* context);

        //////////////////////////////////////////////////////////////////////////
        // Controller component
        void Activate(AZ::EntityId entityId);
        void Deactivate();
        void SetConfiguration(const ROS2FrameConfiguration& config);
        const ROS2FrameConfiguration& GetConfiguration() const;
        //////////////////////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////////////////////
        // ROS2FrameComponentBus::Handler overrides
        AZStd::string GetFrameID() const override;
        void SetFrameID(const AZStd::string& frameId);
        AZ::Name GetJointName() const override;
        void SetJointName(const AZStd::string& jointNameString);
        AZStd::string GetNamespace() const override;
        AZ::Transform GetFrameTransform() const override;
        AZStd::string GetGlobalFrameName() const override;
        void UpdateNamespaceConfiguration(const AZStd::string& ns, NamespaceConfiguration::NamespaceStrategy strategy) override;
        bool IsFrame() const override;
        //////////////////////////////////////////////////////////////////////////

        bool IsTopLevel() const; //!< True if this entity does not have a parent entity with ROS2.

        //! Whether transformation to parent frame can change during the simulation, or is fixed.
        bool IsDynamic() const;

        const AZ::EntityId GetParentROS2FrameEntityId() const;

        //! Return the frame id of this frame's parent. It can be useful to determine ROS 2 transformations.
        //! @return Parent frame ID.
        //! @note This also works with top-level frames, returning a global frame name.
        //! @see GetGlobalFrameName().
        AZStd::string GetParentFrameID() const;

        void PopulateNamespace(bool isRoot, const AZStd::string& entityName);

        bool GetPublishTransform();

        void SetIsDynamic(bool value);

        void SetActiveEntityId(AZ::EntityId entityId);

    private:
        // Callback for when the reflected configuration changes.
        void ConfigurationChange() const;

        ROS2FrameConfiguration m_configuration;
    };

} // namespace ROS2
