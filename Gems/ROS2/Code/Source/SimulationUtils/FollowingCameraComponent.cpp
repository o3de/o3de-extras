/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FollowingCameraComponent.h"
#include <AzCore/Serialization/EditContext.h>
#include <AzCore/Serialization/SerializeContext.h>
#include <AzFramework/Input/Devices/Keyboard/InputDeviceKeyboard.h>
#include <MathConversion.h>

namespace ROS2
{

    // Default keyboard mapping for predefined views.
    const AZStd::unordered_map<AzFramework::InputChannelId, int> KeysToView{
        { AzFramework::InputDeviceKeyboard::Key::Alphanumeric1, 0 }, { AzFramework::InputDeviceKeyboard::Key::Alphanumeric2, 1 },
        { AzFramework::InputDeviceKeyboard::Key::Alphanumeric3, 2 }, { AzFramework::InputDeviceKeyboard::Key::Alphanumeric4, 3 },
        { AzFramework::InputDeviceKeyboard::Key::Alphanumeric5, 4 }, { AzFramework::InputDeviceKeyboard::Key::Alphanumeric6, 5 },
        { AzFramework::InputDeviceKeyboard::Key::Alphanumeric7, 6 }, { AzFramework::InputDeviceKeyboard::Key::Alphanumeric8, 7 },
        { AzFramework::InputDeviceKeyboard::Key::Alphanumeric9, 8 }, { AzFramework::InputDeviceKeyboard::Key::Alphanumeric0, 9 }
    };

    FollowingCameraComponent::FollowingCameraComponent(const FollowingCameraConfiguration& configuration)
        : m_configuration(configuration)
    {
    }



    void FollowingCameraComponent::Reflect(AZ::ReflectContext* reflection)
    {
        FollowingCameraConfiguration::Reflect(reflection);
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(reflection);
        if (serializeContext)
        {
            serializeContext->Class<FollowingCameraComponent, AZ::Component>()->Version(1)->Field(
                "FollowingCameraConfiguration", &FollowingCameraComponent::m_configuration);
            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<FollowingCameraComponent>("Following Camera", "Camera following entity with predefined views")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->Attribute(AZ::Edit::Attributes::AppearsInAddComponentMenu, AZ_CRC("Game"))
                    ->Attribute(AZ::Edit::Attributes::Category, "ROS2 Utilities")
                    ->Attribute(AZ::Edit::Attributes::Icon, "Editor/Icons/Components/Camera.svg")
                    ->UIElement(AZ::Edit::UIHandlers::Label, "", "")
                    ->Attribute(
                        AZ::Edit::Attributes::ValueText,
                        "This Component allows to switch camera view between predefined views. "
                        "It also allows to zoom in/out and rotate around parent transformation. "
                        "Use 0-9 keys to switch views and W, S, A, D keys to manipulate current view.")
                    ->DataElement(
                        AZ::Edit::UIHandlers::Default,
                        &FollowingCameraComponent::m_configuration,
                        "FollowingCameraConfiguration",
                        "FollowingCameraConfiguration")
                    ->Attribute(AZ::Edit::Attributes::Visibility, AZ::Edit::PropertyVisibility::ShowChildrenOnly);
            }
        }
    }

    void FollowingCameraComponent::GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided)
    {
        provided.push_back(AZ_CRC("FollowingCameraService"));
    }

    void FollowingCameraComponent::GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible)
    {
        incompatible.push_back(AZ_CRC("FollowingCameraService"));
    }

    void FollowingCameraComponent::GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required)
    {
        required.push_back(AZ_CRC("TransformService"));
        required.push_back(AZ_CRC("CameraService"));
    }

    void FollowingCameraComponent::Activate()
    {
        if (m_configuration.m_predefinedViews.size() == 0)
        {
            AZ_Warning("FollowingCameraComponent", false, "No predefined views");
            return;
        }
        if (m_configuration.m_defaultView < m_configuration.m_predefinedViews.size())
        {
            m_currentView = m_configuration.m_predefinedViews[m_configuration.m_defaultView];
        }
        InputChannelEventListener::Connect();
        AZ::TickBus::Handler::BusConnect();
    }

    void FollowingCameraComponent::Deactivate()
    {
        AZ::TickBus::Handler::BusDisconnect();
        InputChannelEventListener::Disconnect();
    }

    AZ::Transform FollowingCameraComponent::RemoveTiltFromTransform(AZ::Transform transform)
    {
        const AZ::Vector3 axisX = transform.GetBasisX();
        const AZ::Vector3 axisY = transform.GetBasisY();

        const AZ::Matrix3x3 projectionOnXY {AZ::Matrix3x3::CreateFromColumns(AZ::Vector3::CreateAxisX(), AZ::Vector3::CreateAxisY(), AZ::Vector3::CreateZero())};

        const AZ::Vector3 newAxisZ = AZ::Vector3::CreateAxisZ(); // new axis Z points up

        // project axisX on the XY plane
        const AZ::Vector3 projectedAxisX = (projectionOnXY * axisX);
        const AZ::Vector3 projectedAxisY = (projectionOnXY * axisY);

        AZ::Vector3 newAxisX = AZ::Vector3::CreateZero();
        AZ::Vector3 newAxisY = AZ::Vector3::CreateZero();

        // get 3rd vector of the new basis from the cross product of the projected vectors.
        // Primarily we want to use the projectedAxisX as the newAxisX, but if it is zero-length, we use the projectedAxisY as the newAxisY.
        if (!projectedAxisX.IsZero())
        {
            newAxisX = projectedAxisX.GetNormalized();
            newAxisY = newAxisZ.Cross(newAxisX);
        }
        else
        {
            newAxisY = projectedAxisY.GetNormalized();
            newAxisX = newAxisY.Cross(newAxisZ);
        }
        // apply rotation using created basis
        transform.SetRotation(AZ::Quaternion::CreateFromBasis(newAxisX, newAxisY, newAxisZ));
        return transform;
    }

    void FollowingCameraComponent::CacheTransform(const AZ::Transform& transform, float deltaTime)
    {
        // update the smoothing buffer
        m_lastTranslationsBuffer.push_back(AZStd::make_pair(transform.GetTranslation(), deltaTime));
        m_lastRotationsBuffer.push_back(AZStd::make_pair(transform.GetRotation(), deltaTime));

        if (m_lastTranslationsBuffer.size() > m_configuration.m_smoothingBuffer)
        {
            m_lastTranslationsBuffer.pop_front();
        }
        if (m_lastRotationsBuffer.size() > m_configuration.m_smoothingBuffer)
        {
            m_lastRotationsBuffer.pop_front();
        }
    }
    void FollowingCameraComponent::OnTick(float deltaTime, AZ::ScriptTimePoint /*time*/)
    {
        AZ_Warning("FollowingCameraComponent", m_currentView.IsValid(), "View is not valid");
        if (!m_currentView.IsValid())
        {
            return;
        }
        // obtain the current view transform
        AZ::Transform target_local_transform;
        AZ::Transform target_world_transform;

        AZ::TransformBus::Event(m_currentView, &AZ::TransformBus::Events::GetLocalAndWorld, target_local_transform, target_world_transform);

        // get parent's transform
        const AZ::Transform parent_transform = target_world_transform * target_local_transform.GetInverse();

        const AZ::Transform transformToCache = m_configuration.m_lockZAxis ? RemoveTiltFromTransform(parent_transform) : parent_transform;
        CacheTransform(transformToCache, deltaTime);

        // get the averaged translation and quaternion
        AZ::Transform filtered_parent_transform = { SmoothTranslation(), SmoothRotation(), 1.f };

        auto modifiedTransformZoom = AZ::Transform::CreateIdentity();
        modifiedTransformZoom.SetTranslation(AZ::Vector3::CreateAxisY(m_opticalAxisTranslation));

        // adjust the camera's transform
        //  - rotation is applied in the parent's frame
        //  - translation is applied in the camera's frame
        AZ::Transform filteredTransformAdjusted = filtered_parent_transform *
            AZ::Transform::CreateFromQuaternion(AZ::Quaternion::CreateFromAxisAngle(AZ::Vector3::CreateAxisZ(1.f), m_rotationOffset)) *
            target_local_transform * modifiedTransformZoom;

        // apply the transform to the camera
        AZ::TransformBus::Event(GetEntityId(), &AZ::TransformBus::Events::SetWorldTM, filteredTransformAdjusted);
    }

    AZ::Vector3 FollowingCameraComponent::AverageVector(const AZStd::deque<AZStd::pair<AZ::Vector3, float>>& buffer) const
    {
        AZ::Vector3 sum{ 0 };
        float normalization{ 0 };
        for (const auto& p : buffer)
        {
            sum += p.first * p.second;
            normalization += p.second;
        }
        return sum / normalization;
    }

    AZ::Vector3 FollowingCameraComponent::SmoothTranslation() const
    {
        return AverageVector(m_lastTranslationsBuffer);
    }

    AZ::Quaternion FollowingCameraComponent::SmoothRotation() const
    {
        AZ::Quaternion q = m_lastRotationsBuffer.front().first;
        for (int i = 1; i < m_lastRotationsBuffer.size(); i++)
        {
            q = q.Slerp(m_lastRotationsBuffer[i].first, m_lastRotationsBuffer[i].second);
        }
        return q;
    }

    bool FollowingCameraComponent::OnInputChannelEventFiltered(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputDeviceId& deviceId = inputChannel.GetInputDevice().GetInputDeviceId();

        if (AzFramework::InputDeviceKeyboard::IsKeyboardDevice(deviceId) && inputChannel.IsStateBegan())
        {
            OnKeyboardEvent(inputChannel);
        }

        return false;
    }

    void FollowingCameraComponent::OnKeyboardEvent(const AzFramework::InputChannel& inputChannel)
    {
        const AzFramework::InputChannelId& channelId = inputChannel.GetInputChannelId();
        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericW)
        {
            m_opticalAxisTranslation += m_configuration.m_zoomSpeed;
            m_opticalAxisTranslation = AZStd::min(m_opticalAxisTranslation, m_configuration.m_opticalAxisTranslationMin);
            return;
        }
        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericS)
        {
            m_opticalAxisTranslation -= m_configuration.m_zoomSpeed;
            return;
        }
        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericA)
        {
            m_rotationOffset -= m_configuration.m_rotationSpeed;
            return;
        }
        if (channelId == AzFramework::InputDeviceKeyboard::Key::AlphanumericD)
        {
            m_rotationOffset += m_configuration.m_rotationSpeed;
            return;
        }

        // channelId is a numeric key (Key::Alphanumeric0-Key::Alphanumeric9)
        if (auto it = KeysToView.find(channelId); it != KeysToView.end())
        {
            if (int viewId = it->second; viewId < m_configuration.m_predefinedViews.size())
            {
                m_currentView = m_configuration.m_predefinedViews[viewId];
                m_lastTranslationsBuffer.clear();
                m_lastRotationsBuffer.clear();
            }
        }
    }
} // namespace ROS2
