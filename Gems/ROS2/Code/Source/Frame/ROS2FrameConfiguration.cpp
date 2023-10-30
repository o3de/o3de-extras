/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AzCore/Component/ComponentApplicationBus.h"
#include "AzCore/Component/Entity.h"
#include "AzCore/Component/EntityId.h"
#include "AzCore/Component/EntityUtils.h"
#include "AzCore/Component/TransformBus.h"
#include "AzCore/Math/Transform.h"
#include "AzCore/RTTI/RTTIMacros.h"
#include "AzCore/RTTI/ReflectContext.h"
#include "AzCore/RTTI/TypeInfo.h"
#include "AzCore/Serialization/SerializeContext.h"
#include "AzFramework/Components/TransformComponent.h"
#include "ROS2/Frame/ROS2FrameBus.h"
#include "ROS2/Frame/ROS2FrameSystemBus.h"
#include "ROS2/ROS2GemUtilities.h"
#include "ROS2/Sensor/SensorConfiguration.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <ROS2/Frame/NamespaceConfiguration.h>
#include <ROS2/Frame/ROS2FrameConfiguration.h>
#include <ROS2/Utilities/ROS2Names.h>
#include <iostream>

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
                ->Field("Joint Name", &ROS2FrameConfiguration::m_jointNameString)
                ->Field("Publish Transform", &ROS2FrameConfiguration::m_publishTransform)
                ->Field("Effective namespace", &ROS2FrameConfiguration::m_effectiveNamespace);

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
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameConfiguration::m_jointNameString, "Joint Name", "Joint Name")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &ROS2FrameConfiguration::m_publishTransform,
                        "Publish Transform",
                        "Publish Transform")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &ROS2FrameConfiguration::m_effectiveNamespace, "Effective namespace", "")
                    ->Attribute(AZ::Edit::Attributes::ReadOnly, true);
            }
        }
    }

} // namespace ROS2
