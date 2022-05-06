/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRSystemInterface.h>

namespace XR
{
    class InstanceDescriptor
        : public AZ::RPI::XRInstanceDescriptor
    {
    public:
        AZ_RTTI(InstanceDescriptor, "{1C457924-56A4-444F-BC72-4D31A097BA70}");

        InstanceDescriptor() = default;
        virtual ~InstanceDescriptor() = default;

        //any extra info a generic xr instance descriptor needs
    };

    class Instance
    {
    public:
        AZ_RTTI(Instance, "{1C457924-56A4-444F-BC72-4D31A097BA70}");

        Instance() = default;
        virtual ~Instance() = default;

        AZStd::intrusive_ptr<InstanceDescriptor> m_descriptor;

        virtual AZ::RHI::ResultCode InitInstanceInternal();
    };

} // namespace XR
