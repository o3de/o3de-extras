/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/std/string/string.h>
#include <AzFramework/Physics/Material/PhysicsMaterialManager.h>
#include <RobotImporter/Assets/AssetTypes.h>
#include <RobotImporter/ReferencedAssetsRequestBus.h>
#include <sdf/Collision.hh>
#include <sdf/Model.hh>

namespace ROS2RobotImporter
{
    using BuildReadyCallback = AZStd::function<void()>;

    //! Populates a given entity with all the contents of the <collider> tag in robot description.
    class CollidersMaker
    {
    public:
        //! Construct the class based on session id.
        //! @param sessionId id of current import session, allowing communication with asset and status busses.
        CollidersMaker(const ImportSessionId sessionId);

        //! Prevent copying of existing CollidersMaker
        CollidersMaker(const CollidersMaker& other) = delete;

        //! Add zero, one or many collider elements (depending on link content).
        //! @param model An SDF model object provided by libsdformat from a parsed URDF/SDF
        //! @param link A parsed SDF tree link node which could hold information about colliders.
        //! @param entityId A non-active entity which will be affected.
        void AddColliders(const sdf::Model& model, const sdf::Link* link, AZ::EntityId entityId);

    private:
        void FindWheelMaterial();
        void AddCollider(
            const sdf::Collision* collision,
            const AZStd::string& modelUri,
            AZ::EntityId entityId,
            const AZStd::string& generatedName,
            const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset);
        void AddColliderToEntity(
            const sdf::Collision* collision,
            const AZStd::string& modelUri,
            AZ::EntityId entityId,
            const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset) const;

        AZ::Data::Asset<Physics::MaterialAsset> m_wheelMaterial;
        const ImportSessionId m_sessionId;
    };
} // namespace ROS2RobotImporter
