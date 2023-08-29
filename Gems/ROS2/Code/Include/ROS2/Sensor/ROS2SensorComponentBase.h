/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Serialization/EditContext.h>
#include <ROS2/Frame/ROS2FrameComponent.h>
#include <ROS2/ROS2GemUtilities.h>
#include <ROS2/Sensor/Events/EventSourceAdapter.h>
#include <ROS2/Sensor/SensorConfiguration.h>

namespace ROS2
{
    template<class EventSourceT>
    class ROS2SensorComponentBase : public AZ::Component
    {
    public:
        using SensorBaseType = ROS2SensorComponentBase<EventSourceT>;

        AZ_COMPONENT_DECL((ROS2SensorComponentBase, AZ_CLASS));

        static void Reflect(AZ::ReflectContext* context)
        {
            SensorConfiguration::Reflect(context);

            // Verify during review -> multiple sensor implementation, using the same EventSourceT, will call this Reflect - this can be a
            // problem.
            EventSourceAdapter<EventSourceT>::Reflect(context);

            if (auto serialize = azrtti_cast<AZ::SerializeContext*>(context))
            {
                serialize->Class<ROS2SensorComponentBase<EventSourceT>, AZ::Component>()
                    ->Version(1)
                    ->Field("Sensor configuration", &ROS2SensorComponentBase<EventSourceT>::m_sensorConfiguration)
                    ->Field("Source adapter configuration", &ROS2SensorComponentBase<EventSourceT>::m_eventSourceAdapter);

                if (auto editContext = serialize->GetEditContext())
                {
                    editContext->Class<ROS2SensorComponentBase<EventSourceT>>("ROS2 Sensor Component Base", "Base component for sensors")
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &ROS2SensorComponentBase<EventSourceT>::m_sensorConfiguration,
                            "Sensor configuration",
                            "Sensor configuration")
                        ->DataElement(
                            AZ::Edit::UIHandlers::Default,
                            &ROS2SensorComponentBase<EventSourceT>::m_eventSourceAdapter,
                            "Source adapter configuration",
                            "Source adapter configuration");
                }
            }
        }

        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
        {
            required.push_back(AZ_CRC_CE("ROS2Frame"));
        }

        virtual ~ROS2SensorComponentBase() = default;

        void Activate() override
        {
        }

        void Deactivate() override
        {
        }

    protected:
        //! Returns a complete namespace for this sensor topics and frame ids.
        [[nodiscard]] AZStd::string GetNamespace() const
        {
            auto ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
            return ros2Frame->GetNamespace();
        }

        //! Returns this sensor frame ID. The ID contains namespace.
        [[nodiscard]] AZStd::string GetFrameID() const
        {
            auto ros2Frame = Utils::GetGameOrEditorComponent<ROS2FrameComponent>(GetEntity());
            return ros2Frame->GetFrameID();
        }

        SensorConfiguration m_sensorConfiguration;
        EventSourceAdapter<EventSourceT> m_eventSourceAdapter;
        typename EventSourceAdapter<EventSourceT>::SourceEventHandlerType m_sourceEventHandler;
        typename EventSourceAdapter<EventSourceT>::SourceEventHandlerType m_adaptedEventHandler;
    };

    AZ_COMPONENT_IMPL_INLINE(
        (ROS2SensorComponentBase, AZ_CLASS), "ROS2SensorComponentBase", "{2DF9A652-DF5D-43B1-932F-B6A838E36E97}", AZ::Component)
} // namespace ROS2