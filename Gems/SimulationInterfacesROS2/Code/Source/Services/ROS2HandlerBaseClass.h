/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "AzCore/std/optional.h"
#include "Utils/RegistryUtils.h"
#include <AzCore/std/containers/unordered_set.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/service.hpp>
#include <simulation_interfaces/msg/simulator_features.hpp>
namespace SimulationInterfacesROS2
{
    // common interface to store all feature handlers in common container
    class IROS2HandlerBase
    {
    public:
        virtual ~IROS2HandlerBase() = default;
        virtual AZStd::unordered_set<AZ::u8> GetProvidedFeatures() = 0;
        virtual AZStd::string_view GetTypeName() const = 0;
        virtual AZStd::string_view GetDefaultName() const = 0;
    };

    //! base for each ros2 service handler, forces declaration of features provided by the handler
    //! combined informations along all ROS 2 handlers gives information about simulation features
    //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
    using SimulationFeatures = simulation_interfaces::msg::SimulatorFeatures;
    template<typename RosServiceType>
    class ROS2HandlerBase : public virtual IROS2HandlerBase
    {
    public:
        using Request = typename RosServiceType::Request;
        using Response = typename RosServiceType::Response;
        using ServiceHandle = std::shared_ptr<rclcpp::Service<RosServiceType>>;
        virtual ~ROS2HandlerBase() = default;
        void CreateService(rclcpp::Node::SharedPtr& node)
        {
            // get the service name from the type name
            AZStd::string serviceName = RegistryUtilities::GetServiceName(GetTypeName());

            if (serviceName.empty())
            {
                // if the service name is empty, use the default name
                serviceName = GetDefaultName();
            }

            const std::string serviceNameStr{ serviceName.c_str(), serviceName.size() };
            m_serviceHandle = node->create_service<RosServiceType>(
                serviceNameStr,
                [this](
                    const ServiceHandle service_handle,
                    const std::shared_ptr<rmw_request_id_t> header,
                    const std::shared_ptr<Request> request)
                {
                    auto response = HandleServiceRequest(header, *request);
                    // if no response passed it means, that handleServiceRequest will send response in defined callback after time consuming
                    // task
                    if (response.has_value())
                    {
                        service_handle->send_response(*header, response.value());
                    }
                });
        }
        [[nodiscard]] ServiceHandle& GetServiceHandle()
        {
            return m_serviceHandle;
        }

    protected:
        //! This function is called when a service request is received.
        virtual AZStd::optional<Response> HandleServiceRequest(const std::shared_ptr<rmw_request_id_t> header, const Request& request) = 0;

        //! return features id defined by the handler, ids must follow the definition inside standard:
        //! @see https://github.com/ros-simulation/simulation_interfaces/blob/main/msg/SimulatorFeatures.msg
        AZStd::unordered_set<AZ::u8> GetProvidedFeatures() override
        {
            return {};
        };

    private:
        ServiceHandle m_serviceHandle;
    };

} // namespace SimulationInterfacesROS2
