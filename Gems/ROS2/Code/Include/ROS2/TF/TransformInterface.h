/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <builtin_interfaces/msg/time.hpp>
#include <AzCore/Math/Transform.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/string/string.h>
#include <ROS2/ROS2TypeIds.h>

namespace ROS2
{
    class TFInterfaceRequests
    {
    public:
        AZ_RTTI(TFInterfaceRequests, TFInterfaceTypeId);

        //! Gets transform from source frame to target frame, at given time using ROS tf2 library.
        //! If the transform is not available, it will return an error message.
        //! @param source - source frame name
        //! @param target - target frame name
        //! @param time - time to get the transform at, if not provided, uses most recent time
        //! @return AZ::Outcome with transform if successful, or error message if failed
        virtual AZ::Outcome<AZ::Transform, AZStd::string> GetTransform(
            const AZStd::string& source, const AZStd::string& target, const builtin_interfaces::msg::Time& time) = 0;

        //! Gets the latest transform from source frame to target frame, using ROS tf2 library.
        //! This method does not require a specific time and will return the most recent transform available.
        //! @param source - source frame name
        //! @param target - target frame name
        //! @return AZ::Transform representing the latest transform from source to target frame.
        //! If the transform is not available, it will return an identity transform.
        //! @note This method is useful for scripting use-case
        virtual AZ::Transform GetLatestTransform(const AZStd::string& source, const AZStd::string& target) = 0;

        //! Publishes a transform between source and target frames with current time stamp.
        //! @param source - source frame name
        //! @param target - target frame name
        //! @param transform - AZ::Transform representing the transformation from source to target frame
        //! @param isDynamic - if true, the transform is dynamic and should be published continuously
        //! @note This method is useful for scripting use-case
        virtual void PublishTransform(const AZStd::string& source, const AZStd::string& target, const AZ::Transform& transform, bool isDynamic) = 0;

    };
    class TFInterfaceTraits
        : public AZ::EBusTraits
        , public TFInterfaceRequests
    {
    public:
        static constexpr AZ::EBusAddressPolicy AddressPolicy = AZ::EBusAddressPolicy::Single;
        static const AZ::EBusHandlerPolicy HandlerPolicy = AZ::EBusHandlerPolicy::Single;
    };
    using TFInterface = AZ::Interface<TFInterfaceRequests>;
    using TFInterfaceBus = AZ::EBus<TFInterfaceTraits>;
} // namespace ROS2
