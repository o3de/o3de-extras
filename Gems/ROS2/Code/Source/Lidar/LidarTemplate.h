/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>

namespace ROS2
{
    //! Configuration reflecting a specific Lidar model.
    //! This is meant to capture differences between different Lidars available on the market.
    //! @note Current implementation is simplified. Rays in real lidars are often not uniformly
    //! distributed among angular range, there is noise etc.
    struct LidarTemplate
    {
    public:
        AZ_TYPE_INFO(LidarTemplate, "{9E9EF583-733D-4450-BBA0-ADD4D1BEFBF2}");
        static void Reflect(AZ::ReflectContext* context);

        // TODO - implement more models. Add at least one realistic model.
        enum LidarModel
        {
            Generic3DLidar
        };

        LidarModel m_model;
        AZStd::string m_name;
        float m_minHAngle = 0.0f;
        float m_maxHAngle = 0.0f;
        float m_minVAngle = 0.0f;
        float m_maxVAngle = 0.0f;
        unsigned int m_layers = 0;
        unsigned int m_numberOfIncrements = 0;
        float m_maxRange = 0.0f;
        bool m_addPointsAtMax = false;
    };
} // namespace ROS2
