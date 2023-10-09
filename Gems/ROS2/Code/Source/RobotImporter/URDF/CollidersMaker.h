/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "UrdfParser.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/parallel/atomic.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/parallel/thread.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzFramework/Physics/Material/PhysicsMaterialId.h>
#include <AzFramework/Physics/Material/PhysicsMaterialManager.h>
#include <RobotImporter/Utils/SourceAssetsStorage.h>

namespace ROS2
{
    using BuildReadyCallback = AZStd::function<void()>;

    //! Populates a given entity with all the contents of the <collider> tag in robot description.
    class CollidersMaker
    {
    public:
        //! Construct the class based on SDF asset mapping.
        //! @param sdfAssetsMapping a prepared mapping of Assets used by the source URDF/SDF.
        CollidersMaker(const AZStd::shared_ptr<Utils::UrdfAssetMap>& sdfAssetsMapping);

        //! Prevent copying of existing CollidersMaker
        CollidersMaker(const CollidersMaker& other) = delete;

        //! Add zero, one or many collider elements (depending on link content).
        //! @param model An SDF model object provided by libsdformat from a parsed URDF/SDF
        //! @param link A parsed SDF tree link node which could hold information about colliders.
        //! @param entityId A non-active entity which will be affected.
        void AddColliders(const sdf::Model& model, const sdf::Link* link, AZ::EntityId entityId);
        //! Sends meshes required for colliders to asset processor.
        //! @param buildReadyCb Function to call when the processing finishes.
        void ProcessMeshes(BuildReadyCallback notifyBuildReadyCb);

    private:
        void FindWheelMaterial();
        void AddCollider(
            const sdf::Collision* collision,
            AZ::EntityId entityId,
            const AZStd::string& generatedName,
            const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset);
        void AddColliderToEntity(
            const sdf::Collision* collision, AZ::EntityId entityId, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset) const;

        AZ::Data::Asset<Physics::MaterialAsset> m_wheelMaterial;
        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetsMapping;
    };
} // namespace ROS2
