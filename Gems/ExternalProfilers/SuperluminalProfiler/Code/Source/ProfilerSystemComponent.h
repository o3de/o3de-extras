/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "SuperluminalProfilerEventForwarder.h"

#include <AzCore/Component/Component.h>
#include <AzCore/Debug/ProfilerBus.h>

namespace SuperluminalProfiler
{
    class ProfilerSystemComponent
        : public AZ::Component
        , protected AZ::Debug::ProfilerRequests
    {
    public:
        AZ_COMPONENT(ProfilerSystemComponent, "{C920A0CC-A053-4A0E-8550-DC44FF03A2D1}");

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
        SuperluminalProfilerEventForwarder m_eventForwarder;
    };

} // namespace SuperluminalProfiler
