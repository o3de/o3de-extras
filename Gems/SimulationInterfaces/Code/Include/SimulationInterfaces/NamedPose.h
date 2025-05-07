/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "SimulationInterfacesTypeIds.h"
#include <AzCore/Math/Transform.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/string/string.h>

namespace SimulationInterfaces
{
    //! A named pose defined in the simulation for certain purposes such as spawning.
    //! @see <a href="https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/NamedPose.msg">NamedPose.msg</a>
    struct NamedPose
    {
        AZ_RTTI(NamedPose, NamedPoseTypeId);
        static void Reflect(AZ::ReflectContext* context)
        {
            if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serializeContext->Class<NamedPose>()
                    ->Version(0)
                    ->Field("name", &NamedPose::m_name)
                    ->Field("description", &NamedPose::m_description)
                    ->Field("tags", &NamedPose::m_tags)
                    ->Field("pose", &NamedPose::m_pose);

                if (auto editContext = serializeContext->GetEditContext())
                {
                    editContext->Class<NamedPose>("Named Pose", "Configuration of named pose")
                        ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                        ->DataElement(AZ::Edit::UIHandlers::Default, &NamedPose::m_description, "Description", "");
                }
            }
        }
        NamedPose() = default;
        NamedPose(
            const AZStd::string& name,
            const AZStd::string& description,
            const AZStd::vector<AZStd::string>& tags,
            const AZ::Transform& pose)
            : m_name(name)
            , m_description(description)
            , m_tags(tags)
            , m_pose(pose)
        {
        }
        virtual ~NamedPose() = default;
        AZStd::string m_name;
        AZStd::string m_description;
        AZStd::vector<AZStd::string> m_tags;
        AZ::Transform m_pose;
    };

} // namespace SimulationInterfaces
