/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "CpuProfiler.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Debug/ProfilerBus.h>

namespace TracyProfiler
{
    class ProfilerSystemComponent
        : public AZ::Component
        , protected AZ::Debug::ProfilerRequests
    {
    public:
        AZ_COMPONENT(ProfilerSystemComponent, "{839DAD12-4571-4484-B71F-2133A5BC1137}");

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        ProfilerSystemComponent();
        ~ProfilerSystemComponent();

    protected:
        // AZ::Component interface implementation
        void Activate() override;
        void Deactivate() override;

        // ProfilerRequests interface implementation
        bool IsActive() const override;
        void SetActive(bool active) override;
        bool CaptureFrame(const AZStd::string& outputFilePath) override;
        bool StartCapture(AZStd::string outputFilePath) override;
        bool EndCapture() override;
        bool IsCaptureInProgress() const override;

    private:
        AZStd::atomic_bool m_cpuCaptureInProgress{ false };
        CpuProfiler m_cpuProfiler;
    };

}
