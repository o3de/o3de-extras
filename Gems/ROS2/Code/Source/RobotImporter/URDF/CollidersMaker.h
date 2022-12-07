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
        //! Construct the class based on URDF asset mapping.
        //! @param urdfAssetsMapping a prepared mapping of Assets used by the source URDF.
        CollidersMaker(const AZStd::shared_ptr<Utils::UrdfAssetMap>& urdfAssetsMapping);

        //! Deleted copy constructor.
        CollidersMaker(const CollidersMaker& other) = delete;

        ~CollidersMaker();

        //! Builds .pxmeshes for every collider in link collider mesh.
        //! @param link A parsed URDF tree link node which could hold information about colliders.
        void BuildColliders(urdf::LinkSharedPtr link);
        //! Add zero, one or many collider elements (depending on link content).
        //! @param link A parsed URDF tree link node which could hold information about colliders.
        //! @param entityId A non-active entity which will be affected.
        void AddColliders(urdf::LinkSharedPtr link, AZ::EntityId entityId);
        //! Sends meshes required for colliders to asset processor.
        //! @param buildReadyCb Function to call when the processing finishes.
        void ProcessMeshes(BuildReadyCallback notifyBuildReadyCb);

    private:
        void FindWheelMaterial();
        void BuildCollider(urdf::CollisionSharedPtr collision);
        void AddCollider(
            urdf::CollisionSharedPtr collision,
            AZ::EntityId entityId,
            const AZStd::string& generatedName,
            const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset);
        void AddColliderToEntity(
            urdf::CollisionSharedPtr collision, AZ::EntityId entityId, const AZ::Data::Asset<Physics::MaterialAsset>& materialAsset) const;

        AZStd::thread m_buildThread;
        AZStd::mutex m_buildMutex;
        AZStd::vector<AZ::IO::Path> m_meshesToBuild;
        AZStd::atomic_bool m_stopBuildFlag;
        AZ::Data::Asset<Physics::MaterialAsset> m_wheelMaterial;
        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetsMapping;
    };
} // namespace ROS2
