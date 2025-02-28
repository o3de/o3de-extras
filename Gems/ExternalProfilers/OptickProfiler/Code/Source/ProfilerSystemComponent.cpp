/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ProfilerSystemComponent.h"

#include <AzCore/IO/FileIO.h>
#include <AzCore/RTTI/BehaviorContext.h>
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>
#include <optick.h>

namespace OptickProfiler
{
    static constexpr AZ::Crc32 profilerServiceCrc = AZ_CRC_CE("ProfilerService");

    void ProfilerSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (AZ::SerializeContext* serialize = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serialize->Class<ProfilerSystemComponent, AZ::Component>()->Version(0);
        }
    }

    void ProfilerSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(profilerServiceCrc);
    }

    void ProfilerSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(profilerServiceCrc);
    }

    void ProfilerSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
    }

    void ProfilerSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    ProfilerSystemComponent::ProfilerSystemComponent()
    {
        if (AZ::Debug::ProfilerSystemInterface::Get() == nullptr)
        {
            AZ::Debug::ProfilerSystemInterface::Register(this);
        }
    }

    ProfilerSystemComponent::~ProfilerSystemComponent()
    {
        if (AZ::Debug::ProfilerSystemInterface::Get() == this)
        {
            AZ::Debug::ProfilerSystemInterface::Unregister(this);
        }
    }

    void ProfilerSystemComponent::Activate()
    {
        m_cpuProfiler.Init();
    }

    void ProfilerSystemComponent::Deactivate()
    {
        m_cpuProfiler.Shutdown();
    }

    bool ProfilerSystemComponent::IsActive() const
    {
        return false;
    }

    void ProfilerSystemComponent::SetActive([[maybe_unused]] bool enabled)
    {
    }

    bool ProfilerSystemComponent::CaptureFrame([[maybe_unused]] const AZStd::string& outputFilePath)
    {
        return false;
    }

    bool ProfilerSystemComponent::StartCapture(AZStd::string outputFilePath)
    {
        if (!m_cpuCaptureInProgress && Optick::StartCapture())
        {
            m_captureFile = AZStd::move(outputFilePath);
            m_cpuCaptureInProgress = true;
            return true;
        }
        return false;
    }

    bool ProfilerSystemComponent::EndCapture()
    {
        if (m_cpuCaptureInProgress && Optick::StopCapture())
        {
            m_cpuCaptureInProgress = false;
            return Optick::SaveCapture(m_captureFile.c_str());
        }
        return false;
    }

    bool ProfilerSystemComponent::IsCaptureInProgress() const
    {
        return m_cpuCaptureInProgress;
    }

} // namespace OptickProfiler
