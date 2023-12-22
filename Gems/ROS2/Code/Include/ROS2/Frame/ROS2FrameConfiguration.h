/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/RTTI/RTTI.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzCore/std/string/string.h>
#include <ROS2/Frame/NamespaceConfiguration.h>

namespace ROS2
{
    class ROS2FrameConfiguration final : public AZ::ComponentConfig
    {
    public:
        AZ_TYPE_INFO(ROS2FrameConfiguration, "{04882f01-5451-4efa-b4f8-cd57e4b6cadf}");
        static void Reflect(AZ::ReflectContext* context);

        NamespaceConfiguration m_namespaceConfiguration;
        AZStd::string m_frameName = "sensor_frame";
        AZStd::string m_jointName;

        bool m_publishTransform = true;
        bool m_isDynamic = false;

        //! Sets the effective namespace shown in the Editor.
        //! @param effectiveNamespace namespace to be set.
        void SetEffectiveNamespace(const AZStd::string& effectiveNamespace);

    private:
        AZStd::string m_effectiveNamespace = "";
    };

} // namespace ROS2
