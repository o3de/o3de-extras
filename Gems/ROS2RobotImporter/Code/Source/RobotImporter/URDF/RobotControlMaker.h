/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/ComponentBus.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/containers/set.h>
#include <ROS2/RobotImporter/SDFormatModelPluginImporterHook.h>

#include <sdf/sdf.hh>

namespace ROS2
{
    //! Populates a given entity with all the contents related to robot control.
    //! It is used to create ROS2RobotControlComponent and similar components if
    class RobotControlMaker
    {
    public:
        //! Adds model control plugins and sets it accordingly based on the SDFormat description.
        //! @param model A parsed SDF model which could hold information about a model control plugin.
        //! @param entityId A non-active entity which will be affected.
        //! @param createdEntities A map of all created entities that can be used by plugins.
        void AddControlPlugins(const sdf::Model& model, AZ::EntityId entityId, const SDFormat::CreatedEntitiesMap& createdEntities);

        //! Get a reference to collection of status messages (read-only)
        //! @return A reference to set containing status messages.
        const AZStd::set<AZStd::string>& GetStatusMessages() const;

    private:
        AZStd::set<AZStd::string> m_status;

        using ControlHookCallOutcome = AZ::Outcome<void, AZStd::string>;
        ControlHookCallOutcome AddPlugin(
            AZ::EntityId entityId, const sdf::Plugin& plugin, const sdf::Model& model, const SDFormat::CreatedEntitiesMap& createdEntities);
    };
} // namespace ROS2
