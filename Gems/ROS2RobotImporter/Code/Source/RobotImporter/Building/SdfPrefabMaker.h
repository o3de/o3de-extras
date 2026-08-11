/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "Makers/ArticulationsMaker.h"
#include "Makers/CollidersMaker.h"
#include "Makers/InertialsMaker.h"
#include "Makers/JointsMaker.h"
#include "Makers/RobotControlMaker.h"
#include "Makers/SensorsMaker.h"
#include "Makers/VisualsMaker.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/optional.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/Prefab/PrefabIdTypes.h>
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>
#include <RobotImporter/RobotImporterStatusBus.h>

namespace ROS2RobotImporter
{
    //! Encapsulates constructive mapping of SDF elements to a complete prefab with entities and components
    class SdfPrefabMaker
    {
    public:
        //! Construct PrefabMaker from arguments.
        //! @param root parsed SDF root object.
        //! @param prefabPath path to the prefab which will be created as a result of import.
        //! @param sessionId id of current import session, allowing communication with asset and status busses.
        //! @param useArticulations allows sdfImporter to create PhysXArticulations instead of multiple rigid bodies and joints.
        SdfPrefabMaker(
            const sdf::Root* root,
            AZStd::string prefabPath,
            const ImportSessionId sessionId,
            bool useArticulations = false,
            AZStd::optional<AZ::Transform> spawnPosition = AZStd::nullopt);

        ~SdfPrefabMaker() = default;

        //! On prefab creation this will contain a prefab template id when successful,
        //! and an error string on failure.
        using CreatePrefabTemplateResult = AZ::Outcome<AzToolsFramework::Prefab::TemplateId, AZStd::string>;

        //! Create and return a prefab template corresponding to the SDF model as set through the constructor.
        //! This will also instantiate the prefab template into the level.
        //! @return result which is either the prefab template id containing the imported model or an error message.
        CreatePrefabTemplateResult CreatePrefabFromUrdfOrSdf();

        //! Create and return a prefab template id corresponding to the URDF/SDF model/world set in the constructor.
        //! @return result which is either the prefab template id or an error message.
        CreatePrefabTemplateResult CreatePrefabTemplateFromUrdfOrSdf();

        //! Get path to the prefab resulting from the import.
        //! @return path to the prefab.
        const AZStd::string& GetPrefabPath() const;

    private:
        AzToolsFramework::Prefab::PrefabEntityResult CreateEntityForModel(const sdf::Model& model);
        AzToolsFramework::Prefab::PrefabEntityResult AddEntitiesForLink(
            const sdf::Link& link,
            const sdf::Model* attachedModel,
            AZ::EntityId parentEntityId,
            AZStd::vector<AZ::EntityId>& createdEntities);
        static void MoveEntityToDefaultSpawnPoint(const AZ::EntityId& rootEntityId, AZStd::optional<AZ::Transform> spawnPosition);

        // Returns if SDF document contains any model objects, be that at the root or within an <world> tag
        bool ContainsModel() const;

        const sdf::Root* m_root;
        AZStd::string m_prefabPath;
        VisualsMaker m_visualsMaker;
        CollidersMaker m_collidersMaker;
        SensorsMaker m_sensorsMaker;
        InertialsMaker m_inertialsMaker;
        JointsMaker m_jointsMaker;
        ArticulationsMaker m_articulationsMaker;
        RobotControlMaker m_controlMaker;

        const ImportSessionId m_sessionId;
        bool m_useArticulations{ false };

        const AZStd::optional<AZ::Transform> m_spawnPosition;
    };
} // namespace ROS2RobotImporter
