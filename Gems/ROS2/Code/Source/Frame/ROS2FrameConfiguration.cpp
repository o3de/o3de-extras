/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Utilities/ROS2Names.h>

namespace ROS2
{
    void ROS2FrameConfiguration::Reflect(AZ::ReflectContext* context)
    {
        NamespaceConfiguration::Reflect(context);
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ROS2FrameConfiguration>()
                ->Version(1)
                ->Field("Namespace Configuration", &ROS2FrameConfiguration::m_namespaceConfiguration)
                ->Field("Frame Name", &ROS2FrameConfiguration::m_frameName)
                ->Field("Joint Name", &ROS2FrameConfiguration::m_jointName)
                ->Field("Publish Transform", &ROS2FrameConfiguration::m_publishTransform);

            if (AZ::EditContext* ec = serialize->GetEditContext())
            {
                ec->Class<ROS2FrameConfiguration>("ROS2 Frame configuration", "[ROS2 Frame component configuration]")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameConfiguration::m_namespaceConfiguration,
                        "Namespace Configuration",
                        "Namespace Configuration")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameConfiguration::m_frameName, "Frame Name", "Frame Name")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameConfiguration::m_jointName, "Joint Name", "Joint Name")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameConfiguration::m_publishTransform,
                        "Publish Transform",
                        "Publish Transform")
                    ->UIElement(AZ::Edit::UIHandlers::Label, "Effective namespace", "")
                    ->Attribute(AZ::Edit::Attributes::ValueText, &ROS2FrameConfiguration::m_effectiveNamespace);
            }
        }
    }

    void ROS2FrameConfiguration::SetEffectiveNamespace(const AZStd::string& effectiveNamespace)
    {
        m_effectiveNamespace = effectiveNamespace;
    }

} // namespace ROS2
