/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "LidarRegistrarSystemComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/EditContextConstants.inl>
#include <AzCore/Serialization/SerializeContext.h>

namespace ROS2
{
    LidarRegistrarSystemComponent::LidarRegistrarSystemComponent()
    {
        if (LidarRegistrarInterface::Get() == nullptr)
        {
            LidarRegistrarInterface::Register(this);
        }
    }

    LidarRegistrarSystemComponent::~LidarRegistrarSystemComponent()
    {
        if (LidarRegistrarInterface::Get() == this)
        {
            LidarRegistrarInterface::Unregister(this);
        }
    }

    void LidarRegistrarSystemComponent::Init()
    {
    }

    void LidarRegistrarSystemComponent::Activate()
    {
        m_physxLidarSystem.Activate();
    }

    void LidarRegistrarSystemComponent::Deactivate()
    {
    }

    void LidarRegistrarSystemComponent::Reflect(AZ::ReflectContext* context)
    {
        if (auto serializeContext = azrtti_cast<AZ::SerializeContext*>(context))
        {
            serializeContext->Class<LidarRegistrarSystemComponent, AZ::Component>()->Version(0);

            if (AZ::EditContext* ec = serializeContext->GetEditContext())
            {
                ec->Class<LidarRegistrarSystemComponent>(
                      "Lidar Registrar", "Manages the LidarSystem registration and stores their metadata.")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("System"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2")
                    ->Attribute(AZ::Edit::Attributes::AutoExpand, true);
            }
        }
    }

    void LidarRegistrarSystemComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC_CE("LidarRegistrarService"));
    }

    void LidarRegistrarSystemComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC_CE("LidarRegistrarService"));
    }

    void LidarRegistrarSystemComponent::GetRequiredServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC_CE("ROS2Service"));
    }

    void LidarRegistrarSystemComponent::GetDependentServices([[maybe_unused]] AZ::ComponentDescriptor::DependencyArrayType& dependent)
    {
    }

    void LidarRegistrarSystemComponent::RegisterLidarSystem(const char* name, const char* description, const LidarSystemFeatures& features)
    {
        AZ_Assert(
            !m_registeredLidarSystems.contains(AZ_CRC(name)),
            "A lidar system with the provided name already exists. Please choose a different name.");
        m_registeredLidarSystems.emplace(AZ_CRC(name), LidarSystemMetaData{ name, description, features });
    }

    const AZStd::vector<AZStd::string> LidarRegistrarSystemComponent::GetRegisteredLidarSystems()
    {
        AZStd::vector<AZStd::string> lidarSystemList;
        lidarSystemList.reserve(m_registeredLidarSystems.size());
        for (auto lidarSystem : m_registeredLidarSystems)
        {
            lidarSystemList.push_back(lidarSystem.second.m_name);
        }

        return lidarSystemList;
    }

    const LidarSystemMetaData& LidarRegistrarSystemComponent::GetLidarSystemMetaData(const AZStd::string& name)
    {
        auto lidarSystem = m_registeredLidarSystems.find(AZ_CRC(name));
        AZ_Assert(lidarSystem != m_registeredLidarSystems.end(), "No registered lidar system matches the provided name.");
        return lidarSystem->second;
    }
} // namespace ROS2
