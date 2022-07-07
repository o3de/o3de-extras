/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>

#include <XR/XRBase.h>
#include <XR/XRInput.h>
#include <XR/XRObject.h>
#include <XR/XRSpace.h>

namespace XR
{
    class Device;

    // This class will be responsible for creating XR::Session and
    // all the code around managing the session state
    class Session
        : public XR::Object
    {
    public:
        AZ_CLASS_ALLOCATOR(Session, AZ::SystemAllocator, 0);
        AZ_RTTI(Session, "{E7276FE1-94B8-423A-9C1D-1BCF1A0066BC}");

        struct Descriptor
        {
            AZ::RHI::ValidationMode m_validationMode = AZ::RHI::ValidationMode::Disabled;
            Ptr<Device> m_device;
            Ptr<Instance> m_instance;
        };

        Session() = default;
        virtual ~Session() = default;
        
        //! Initialize the XrSession object which is responsible for creating
        //! XrInput and XrSpace
        AZ::RHI::ResultCode Init(const Descriptor& descriptor);

        //! Get the descriptor for the class
        const Descriptor& GetDescriptor() const;

        //! Get the Xr Input object
        Input* GetInput() const;

        //! Get the Xr Space object
        Space* GetSpace() const;
        
        //! Return true if session is running
        virtual bool IsSessionRunning() const = 0;

        //! Return true if session is focused
        virtual bool IsSessionFocused() const = 0;

        //! Return true if a restart is requested
        virtual bool IsRestartRequested() const = 0;
    
        //! Return true if render loop skip is requested
        virtual bool IsExitRenderLoopRequested() const = 0;

        //! Poll events and process the pending messages accordingly
        virtual void PollEvents() = 0;

        //! All the back end object to initialize properly
        virtual AZ::RHI::ResultCode InitInternal(AZ::RHI::XRSessionDescriptor* descriptor) = 0;

        //! Allow the back-end to cache the controller space data
        virtual void LocateControllerSpace(AZ::u32 handIndex) = 0;

        //! Api to retrieve the controller space data
        virtual AZ::RPI::PoseData GetControllerPose(AZ::u32 handIndex) const = 0;

        //! Api to retrieve the controller scale data
        virtual float GetControllerScale(AZ::u32 handIndex) const = 0;

        //! Api to retrieve the front view space data
        virtual AZ::RPI::PoseData GetViewFrontPose() const = 0;

    private:

        ///////////////////////////////////////////////////////////////////
        // XR::Object
        void Shutdown() override;
        ///////////////////////////////////////////////////////////////////

        //! Called when the XR instance is being shutdown.
        virtual void ShutdownInternal() = 0;

        Ptr<Input> m_input;
        Ptr<Space> m_space;
        Descriptor m_descriptor;
    };
} // namespace XR
