/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "SimulationInterfaces/NamedPoseManagerRequestBus.h"
#include <AzCore/Component/Component.h>
#include <AzCore/Component/EntityId.h>
#include <AzCore/Component/TickBus.h>
#include <AzCore/Component/TransformBus.h>
#include <AzCore/std/containers/set.h>
#include <AzCore/std/containers/unordered_map.h>
#include <SimulationInterfaces/NamedPose.h>
#include <SimulationInterfaces/SimulationInterfacesTypeIds.h>

namespace SimulationInterfaces
{
    class NamedPoseComponent
        : public AZ::Component
        , public NamedPoseComponentRequestBus::Handler
    {
    public:
        AZ_COMPONENT(NamedPoseComponent, NamedPoseComponentTypeId, AZ::Component);
        NamedPoseComponent() = default;
        NamedPoseComponent(const NamedPose& config);
        ~NamedPoseComponent() override = default;

        void Activate() override;
        void Deactivate() override;
        static void Reflect(AZ::ReflectContext* context);

    private:
        NamedPose GetConfiguration() override
        {
            return m_config;
        }

        NamedPose m_config;
    };
} // namespace SimulationInterfaces
