/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */
#include <AzCore/Serialization/SerializeContext.h>
#include <SdfAssetBuilder/SdfAssetBuilderSystemComponent.h>
#include <SdfAssetBuilder/SdfAssetBuilder.h>

namespace ROS2
{
    void SdfAssetBuilderSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<SdfAssetBuilderSystemComponent, AZ::Component>()
                ->Version(0)
                ->Attribute(AZ::Edit::Attributes::SystemComponentTags, AssetBuilderSDK::ComponentTags::AssetBuilder)
             ;
        }
    }

    void SdfAssetBuilderSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("SdfAssetBuilderService"));
    }

    void SdfAssetBuilderSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("SdfAssetBuilderService"));
    }

    void SdfAssetBuilderSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        // This doesn't require any services to exist before startup.
    }

    void SdfAssetBuilderSystemComponent::GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
        // If the asset services exist at all, they should be started first so that the Sdf builder can register with them correctly.
        dependent.push_back(AZ_CRC_CE("AssetDatabaseService"));
        dependent.push_back(AZ_CRC_CE("AssetCatalogService"));
    }

    void SdfAssetBuilderSystemComponent::Activate()
    {
        m_sdfAssetBuilder.RegisterBuilder();
    }

    void SdfAssetBuilderSystemComponent::Deactivate()
    {
    }

} // namespace ROS2
