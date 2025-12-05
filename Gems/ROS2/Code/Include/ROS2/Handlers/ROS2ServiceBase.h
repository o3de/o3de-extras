/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "HandlersRegistryUtils.h"
#include "IROS2HandlerBase.h"
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/optional.h>
#include <rclcpp/service.hpp>

namespace ROS2
{
    //! Base for each ROS 2 service handler
    template<typename RosServiceType>
    class ROS2ServiceBase : public virtual IROS2HandlerBase
    {
    public:
        using Request = typename RosServiceType::Request;
        using Response = typename RosServiceType::Response;
        using ServiceHandle = std::shared_ptr<rclcpp::Service<RosServiceType>>;
        virtual ~ROS2ServiceBase() = default;

        void Initialize(rclcpp::Node::SharedPtr& node) override
        {
            CreateService(node);
        }

        void SendResponse(Response response)
        {
            AZ_Assert(m_serviceHandle, "Failed to get m_serviceHandle");
            AZ_Assert(m_lastRequestHeader, "Failed to get last request header ptr");
            m_serviceHandle->send_response(*m_lastRequestHeader, response);
        }

        bool IsValid() const override
        {
            return m_serviceHandle != nullptr;
        }

    protected:
        //! This function is called when a service request is received.
        virtual AZStd::optional<Response> HandleServiceRequest(const std::shared_ptr<rmw_request_id_t> header, const Request& request) = 0;

    private:
        void CreateService(rclcpp::Node::SharedPtr& node)
        {
            // get the service name from the type name
            // passing an empty string to settings registry disables ROS 2 action
            AZStd::optional<AZStd::string> serviceName = HandlersRegistryUtils::GetName(GetTypeName());

            // do not create a ROS 2 action if the value is empty
            if (serviceName.has_value() && serviceName.value().empty())
            {
                AZ_Trace("ROS2 Gem", "Service name for type %s is set to empty string, service won't be created", GetTypeName().data());
                return;
            }

            if (!serviceName.has_value())
            {
                // if the service name is empty, use the default name
                serviceName = GetDefaultName();
            }

            const std::string serviceNameStr{ serviceName.value().c_str(), serviceName.value().size() };
            m_serviceHandle = node->create_service<RosServiceType>(
                serviceNameStr,
                [this](
                    const ServiceHandle service_handle,
                    const std::shared_ptr<rmw_request_id_t> header,
                    const std::shared_ptr<Request> request)
                {
                    m_lastRequestHeader = header;
                    auto response = HandleServiceRequest(header, *request);
                    // if no response passed it means, that handleServiceRequest will send response in defined callback after time consuming
                    // task, header needs to be cached
                    if (response.has_value())
                    {
                        service_handle->send_response(*header, response.value());
                    }
                });
        }

        std::shared_ptr<rmw_request_id_t> m_lastRequestHeader = nullptr;
        ServiceHandle m_serviceHandle;
    };

} // namespace ROS2
