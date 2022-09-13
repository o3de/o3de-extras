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
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/parallel/atomic.h>
#include <AzCore/std/parallel/mutex.h>
#include <AzCore/std/parallel/thread.h>

namespace ROS2
{
    using BuildReadyCallback = std::function<void()>;

    //! Populates a given entity with all the contents of the <collider> tag in robot description.
    class CollidersMaker
    {
    public:
        CollidersMaker(AZStd::string modelPath);
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
        void BuildCollider(urdf::CollisionSharedPtr collision);
        void AddCollider(urdf::CollisionSharedPtr collision, AZ::EntityId entityId, const AZStd::string& generatedName);
        void AddColliderToEntity(urdf::CollisionSharedPtr collision, AZ::EntityId entityId);
        AZ::IO::Path GetFullURDFMeshPath(AZ::IO::Path modelPath, AZ::IO::Path meshPath);

        AZStd::string m_modelPath;

        AZStd::thread m_buildThread;
        AZStd::mutex m_buildMutex;
        AZStd::vector<AZ::IO::Path> m_meshesToBuild;
        AZStd::atomic_bool m_stopBuildFlag;
    };
} // namespace ROS2
