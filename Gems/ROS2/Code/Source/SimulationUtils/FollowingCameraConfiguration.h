/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/EntityId.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! A structure capturing configuration of Following Camera.
    struct FollowingCameraConfiguration
    {
        AZ_TYPE_INFO(FollowingCameraConfiguration, "{605fec3d-0152-44f3-b885-669cdcf201eb}");
        static void Reflect(AZ::ReflectContext* context);

        AZStd::vector<AZ::EntityId> m_predefinedViews; //!< List of predefined views.
        int m_defaultView{ 0 }; //!< Index of the default view.
        int m_smoothingBuffer = 30; //!< Number of past transforms used to smooth, larger value gives smoother result, but more lag
        float m_zoomSpeed = 0.06f; //!< Speed of zooming
        float m_rotationSpeed = 0.05f; //!< Rotation Speed around the target
        bool m_lockZAxis = false; //!< Lock the Z axis of the camera
        const float m_opticalAxisTranslationMin = 0.0f; //!< Minimum zoom distance
    };
} // namespace ROS2
