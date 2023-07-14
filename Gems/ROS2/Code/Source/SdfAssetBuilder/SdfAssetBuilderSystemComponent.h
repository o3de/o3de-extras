/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#pragma once

#include <AzCore/Component/Component.h>

namespace AZ
{
    class ReflectContext;
}

namespace ROS2
{
    class SdfAssetBuilder;

    /// System component for registering and managing the SdfAssetBuilder.
    class SdfAssetBuilderSystemComponent : public AZ::Component
    {
    public:
        AZ_COMPONENT(SdfAssetBuilderSystemComponent, "{0171F119-993A-4949-B62D-430291B01A41}");
        static void Reflect(AZ::ReflectContext* context);

        SdfAssetBuilderSystemComponent();
        ~SdfAssetBuilderSystemComponent();

    private:
        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // Asset builder for Sdf assets
        AZStd::unique_ptr<SdfAssetBuilder> m_sdfAssetBuilder;
    };
} // namespace ROS2
