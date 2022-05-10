/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <XR/XRinput.h>
#include <OpenXRVk_Platform.h>

namespace OpenXRVk
{
    class InputDescriptor final
        : public XR::InputDescriptor
    {
    public:
        AZ_CLASS_ALLOCATOR(InputDescriptor, AZ::SystemAllocator, 0);
        AZ_RTTI(InputDescriptor, "{5CE5E693-775B-42A5-9B32-7C1006C69975}", XR::InputDescriptor);

        InputDescriptor() = default;
        virtual ~InputDescriptor() = default;

        //any extra info for a generic xr InputDescriptor
    };

    // Class that will help manage XrActionSet/XrAction
    class Input final
        : public XR::Input
    {
    public:
        AZ_CLASS_ALLOCATOR(Input, AZ::SystemAllocator, 0);
        AZ_RTTI(Input, "{97ADD1FE-27DF-4F36-9F61-683F881F9477}", XR::Input);

        Input() = default;
        virtual ~Input() = default;

        static AZStd::intrusive_ptr<XR::Input> Create();

        AZ::RHI::ResultCode Init() override;
        void InitializeActions() override;
        void PollActions() override;
        void PollEvents() override;
        AZ::RHI::ResultCode InitInternal() override;

    private:
        struct InputState
        {
            XrActionSet actionSet{ XR_NULL_HANDLE };
            XrAction grabAction{ XR_NULL_HANDLE };
            XrAction poseAction{ XR_NULL_HANDLE };
            XrAction vibrateAction{ XR_NULL_HANDLE };
            XrAction quitAction{ XR_NULL_HANDLE };
            AZStd::array<XrPath, 2> handSubactionPath;
            AZStd::array<XrSpace, 2> handSpace;
            AZStd::array<float, 2> handScale = { { 1.0f, 1.0f } };
            AZStd::array<XrBool32, 2> handActive;
        };
        InputState m_input;
    };
}
