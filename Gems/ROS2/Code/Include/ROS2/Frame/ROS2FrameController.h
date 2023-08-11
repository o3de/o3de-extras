/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include "AzCore/Component/Entity.h"
#include "AzCore/Component/EntityId.h"
#include "ROS2/Frame/ROS2FrameBus.h"
#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Components/ComponentAdapter.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
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

    class ROS2FrameComponentController
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

        //! Get a frame id, which is needed for any ROS2 message with a Header
        //! @return Frame id which includes the namespace, ready to send in a ROS2 message
        AZStd::string GetFrameID() const;

        //! Set a above-mentioned frame id
        void SetFrameID(const AZStd::string& frameId);

        //! Get the joint name including the namespace
        //! @note Supplementary metadata for Joint components, necessary in some cases for joints addressed by name in ROS 2
        //! @return The namespaced joint name, ready to send in a ROS2 message
        AZ::Name GetJointName() const;

        //! Set the joint name
        //! @note May be populated during URDF import or set by the user in the Editor view
        //! @param jointNameString does not include the namespace. The namespace prefix is added automatically.
        void SetJointName(const AZStd::string& jointNameString);

        //! Get a namespace, which should be used for any publisher or subscriber in the same entity.
        //! @return A complete namespace (including parent namespaces)
        AZStd::string GetNamespace() const;

        //! Get a transform between this frame and the next frame up in hierarchy.
        //! @return If the parent frame is found, return a Transform between this frame and the parent.
        //! Otherwise, return a global Transform.
        //! @note Parent frame is not the same as parent Transform: there could be many Transforms in between without ROS2Frame
        //! components.
        AZ::Transform GetFrameTransform() const;

        //! Global frame name in ros2 ecosystem.
        //! @return The name of the global frame with namespace attached. It is typically "odom", "map", "world".
        AZStd::string GetGlobalFrameName() const;

        bool IsTopLevel() const; //!< True if this entity does not have a parent entity with ROS2.

        //! Whether transformation to parent frame can change during the simulation, or is fixed.
        bool IsDynamic() const;

        const AZ::EntityId GetParentROS2FrameComponentId() const;

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
        ROS2FrameConfiguration m_configuration;
    };

} // namespace ROS2
