/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotControl/RobotConfiguration.h"
#include <AzCore/Component/ComponentApplicationBus.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzFramework/Physics/Components/SimulatedBodyComponentBus.h>

namespace ROS2
{
    AZ::Outcome<void, AZStd::string> RobotConfiguration::ValidateField(void* newValue, const AZ::Uuid& valueType)
    {
        // Check if the object type is valid
        if (azrtti_typeid<AZ::EntityId>() != valueType)
        {
            AZ_Assert(false, "Unexpected value type");
            return AZ::Failure(AZStd::string("Trying to set an entity ID to something that isn't an entity ID."));
        }

        // Check if entity id is valid
        AZ::EntityId actualEntityId = *reinterpret_cast<AZ::EntityId*>(newValue);
        if (!actualEntityId.IsValid())
        {
            return AZ::Failure(AZStd::string("Invalid entity ID."));
        }

        // Check if entity exists
        AZ::Entity* entity = nullptr;
        AZ::ComponentApplicationBus::BroadcastResult(entity, &AZ::ComponentApplicationBus::Events::FindEntity, actualEntityId);
        if (entity == nullptr)
        {
            return AZ::Failure(AZStd::string("Can't find entity."));
        }

        // Check if object is a physics simulation body
        AzPhysics::SimulatedBody* simulatedBody = nullptr;
        AzPhysics::SimulatedBodyComponentRequestsBus::EventResult(
            simulatedBody, actualEntityId, &AzPhysics::SimulatedBodyComponentRequests::GetSimulatedBody);

        if (simulatedBody == nullptr)
        {
            return AZ::Failure(AZStd::string("Object must be a simulation body."));
        }

        return AZ::Success();
    }

    void RobotConfiguration::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<RobotConfiguration>()
                ->Version(1)
                ->Field("Body", &RobotConfiguration::m_body)
                ->Field("Front left wheel", &RobotConfiguration::m_wheelFrontLeft)
                ->Field("Front right wheel", &RobotConfiguration::m_wheelFrontRight)
                ->Field("Rear left wheel", &RobotConfiguration::m_wheelBackLeft)
                ->Field("Rear right wheel", &RobotConfiguration::m_wheelBackRight);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<RobotConfiguration>("Robot control", "Handles robot control")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_body, "Body", "Robot body")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &RobotConfiguration::ValidateField)
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_wheelFrontLeft, "Front left wheel", "Robot wheel")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &RobotConfiguration::ValidateField)
                    ->Attribute(AZ::Edit::Attributes::Visibility, false) // not functional yet
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_wheelFrontRight, "Front right wheel", "Robot wheel")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &RobotConfiguration::ValidateField)
                    ->Attribute(AZ::Edit::Attributes::Visibility, false) // not functional yet
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_wheelBackLeft, "Back left wheel", "Robot wheel")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &RobotConfiguration::ValidateField)
                    ->Attribute(AZ::Edit::Attributes::Visibility, false) // not functional yet
                    ->DataElement(AZ::Edit::UIHandlers::Default, &RobotConfiguration::m_wheelBackRight, "Back right wheel", "Robot wheel")
                    ->Attribute(AZ::Edit::Attributes::ChangeValidate, &RobotConfiguration::ValidateField)
                    ->Attribute(AZ::Edit::Attributes::Visibility, false) // not functional yet
                    ;
            }
        }
    }
} // namespace ROS2
