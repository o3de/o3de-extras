/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <Atom/RPI.Public/XR/XRinput.h>
#include <openxr/openxr.h>
#include <openxr/openxr_platform.h>
#include <openxr/openxr_reflection.h>

namespace AZ
{
    namespace OpenXRVk
    {
        // Class that will help manage XrActionSet/XrAction
        class Input final
            : public AZ::RPI::XR::Input
        {
        public:
            static AZStd::intrusive_ptr<AZ::RPI::XR::Input> Create();

            AZ::RPI::XR::ResultCode Init(AZ::RPI::XR::Input::Descriptor descriptor) override;
            void InitializeActions() override;
            void PollActions() override;
            void PollEvents() override;

        private:
            struct InputState
            {
                XrActionSet actionSet{ XR_NULL_HANDLE };
                XrAction grabAction{ XR_NULL_HANDLE };
                XrAction poseAction{ XR_NULL_HANDLE };
                XrAction vibrateAction{ XR_NULL_HANDLE };
                XrAction quitAction{ XR_NULL_HANDLE };
                AZStd::array<XrPath, Side::COUNT> handSubactionPath;
                AZStd::array<XrSpace, Side::COUNT> handSpace;
                AZStd::array<float, Side::COUNT> handScale = { { 1.0f, 1.0f } };
                AZStd::array<XrBool32, Side::COUNT> handActive;
            };
            InputState m_input;
        };
    } // namespace OpenXRVk
} // namespace AZ
