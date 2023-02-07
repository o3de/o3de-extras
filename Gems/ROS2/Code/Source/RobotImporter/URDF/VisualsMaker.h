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
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <RobotImporter/Utils/SourceAssetsStorage.h>

namespace ROS2
{
    //! Populates a given entity with all the contents of the <visual> tag in robot description
    class VisualsMaker
    {
    public:
        VisualsMaker(
            const std::map<std::string, urdf::MaterialSharedPtr>& materials,
            const AZStd::shared_ptr<Utils::UrdfAssetMap>& urdfAssetsMapping);

        //! Add zero, one or many visual elements to a given entity (depending on link content).
        //! Note that a sub-entity will be added to hold each visual (since they can have different transforms).
        //! @param link A parsed URDF tree link node which could hold information about visuals.
        //! @param entityId A non-active entity which will be affected.
        void AddVisuals(urdf::LinkSharedPtr link, AZ::EntityId entityId) const;

    private:
        void AddVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId, const AZStd::string& generatedName) const;
        void AddVisualToEntity(urdf::VisualSharedPtr visual, AZ::EntityId entityId) const;
        void AddMaterialForVisual(urdf::VisualSharedPtr visual, AZ::EntityId entityId) const;

        AZStd::unordered_map<AZStd::string, urdf::MaterialSharedPtr> m_materials;
        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetsMapping;
    };
} // namespace ROS2
