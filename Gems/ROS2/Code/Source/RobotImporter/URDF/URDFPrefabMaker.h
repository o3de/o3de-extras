/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ArticulationsMaker.h"
#include "CollidersMaker.h"
#include "InertialsMaker.h"
#include "JointsMaker.h"
#include "UrdfParser.h"
#include "VisualsMaker.h"
#include <AzCore/Component/EntityId.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/Prefab/PrefabIdTypes.h>
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>
#include <RobotImporter/Utils/SourceAssetsStorage.h>
#include <optional>

namespace ROS2
{
    //! Encapsulates constructive mapping of URDF elements to a complete prefab with entities and components
    class URDFPrefabMaker
    {
    public:
        //! Construct URDFPrefabMaker from arguments.
        //! @param modelFilePath path to the source URDF model.
        //! @param model parsed model.
        //! @param prefabPath path to the prefab which will be created as a result of import.
        //! @param urdfAssetsMapping prepared mapping of URDF meshes to Assets.
        //! @param useArticulations allows urdfImporter to create PhysXArticulations instead of multiple rigid bodies and joints.
        URDFPrefabMaker(
            const AZStd::string& modelFilePath,
            urdf::ModelInterfaceSharedPtr model,
            AZStd::string prefabPath,
            const AZStd::shared_ptr<Utils::UrdfAssetMap> urdfAssetsMapping,
            bool useArticulations = false,
            AZStd::optional<AZ::Transform> spawnPosition = AZStd::nullopt);

        ~URDFPrefabMaker() = default;

        //! On prefab creation this will contain a prefab template id when successful,
        //! and an error string on failure.
        using CreatePrefabTemplateResult = AZ::Outcome<AzToolsFramework::Prefab::TemplateId, AZStd::string>;

        //! Create and return a prefab template corresponding to the URDF model as set through the constructor.
        //! This will also instantiate the prefab template into the level.
        //! @return result which is either the prefab template id containing the imported model or an error message.
        CreatePrefabTemplateResult CreatePrefabFromURDF();

        //! Create and return a prefab template id corresponding to the URDF model set in the constructor.
        //! @return result which is either the prefab template id or an error message.
        CreatePrefabTemplateResult CreatePrefabTemplateFromURDF();

        //! Get path to the prefab resulting from the import.
        //! @return path to the prefab.
        const AZStd::string& GetPrefabPath() const;

        //! Get descriptive status of import.
        //! A string with the status, which can be understood by the user.
        AZStd::string GetStatus();

    private:
        AzToolsFramework::Prefab::PrefabEntityResult AddEntitiesForLink(urdf::LinkSharedPtr link, AZ::EntityId parentEntityId, AZStd::vector<AZ::EntityId>& createdEntities);
        void BuildAssetsForLink(urdf::LinkSharedPtr link);
        void AddRobotControl(AZ::EntityId rootEntityId);
        static void MoveEntityToDefaultSpawnPoint(const AZ::EntityId& rootEntityId, AZStd::optional<AZ::Transform> spawnPosition);

        urdf::ModelInterfaceSharedPtr m_model;
        AZStd::string m_prefabPath;
        VisualsMaker m_visualsMaker;
        CollidersMaker m_collidersMaker;
        InertialsMaker m_inertialsMaker;
        JointsMaker m_jointsMaker;
        ArticulationsMaker m_articulationsMaker;

        AZStd::mutex m_statusLock;
        AZStd::multimap<AZStd::string, AZStd::string> m_status;

        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetsMapping;
        bool m_useArticulations{ false };

        const AZStd::optional<AZ::Transform> m_spawnPosition;
    };
} // namespace ROS2
