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
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    //! Configuration data for ROS2FrameComponent.
    //!
    //! This class encapsulates all configuration options for ROS2 frame management,
    //! including namespace handling, frame naming, joint naming, and transform publishing settings.
    class ROS2FrameConfiguration final : public AZ::ComponentConfig
    {
    public:
        AZ_TYPE_INFO(ROS2FrameConfiguration, ROS2FrameConfigurationTypeId);
        static void Reflect(AZ::ReflectContext* context);

        NamespaceConfiguration m_namespaceConfiguration; //!< Namespace management configuration
        AZStd::string m_frameName = "sensor_frame"; //!< Base frame name (without namespace)
        AZStd::string m_jointName; //!< Joint name for this frame

        bool m_publishTransform = true; //!< Whether to publish transforms to ROS2
        bool m_isDynamic = false; //!< Whether frame should be treated as dynamic (auto-determined)
        bool m_forceDynamic = false; //!< Force frame to be dynamic regardless of joints

        //! Update the effective namespace displayed in the Editor.
        //! @param effectiveNamespace The computed namespace to display
        void SetEffectiveNamespace(const AZStd::string& effectiveNamespace);

    private:
        AZStd::string m_effectiveNamespace; //!< Computed namespace for display
        AZStd::string m_fullName = m_frameName; //!< Full namespaced frame name for display
    };

} // namespace ROS2
