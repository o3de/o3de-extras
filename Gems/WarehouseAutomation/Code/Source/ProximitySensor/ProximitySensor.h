/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <Atom/RPI.Public/AuxGeom/AuxGeomDraw.h>
#include <AzCore/Math/Vector3.h>

namespace WarehouseAutomation
{
    //! Simple proximity sensor based on raycasting
    //! This component publishes a bool topic depending on the object presence
    class ProximitySensor
        : public AZ::Component
        , public AZ::TickBus::Handler
    {
    public:
        AZ_COMPONENT(ProximitySensor, "{1f7b51f6-9450-4da4-9636-672a056e8812}", AZ::Component);
        ProximitySensor();
        ~ProximitySensor() = default;

        static void Reflect(AZ::ReflectContext* context);
        //////////////////////////////////////////////////////////////////////////
        // Component overrides
        void Activate() override;
        void Deactivate() override;
        //////////////////////////////////////////////////////////////////////////

    private:
        //////////////////////////////////////////////////////////////////////////
        // AZ::TickBus::Handler overrides
        void OnTick(float deltaTime, AZ::ScriptTimePoint time) override;
        //////////////////////////////////////////////////////////////////////////

        void Visualize();
        void DetectionCheck();

        bool m_visualize{ true };
        float m_frequency{ 10.f };

        AZ::Vector3 m_detectionDirection{ AZ::Vector3::CreateAxisX() };
        float m_detectionDistance{ 1.f };
        std::optional<AZ::Vector3> m_position;
        float m_timeElapsedSinceLastTick{ 0.f };

        AZ::RPI::AuxGeomDrawPtr m_drawQueue;
    };
} // namespace WarehouseAutomation
