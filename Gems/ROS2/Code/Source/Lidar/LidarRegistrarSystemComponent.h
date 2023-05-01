/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <Lidar/LidarSystem.h>
#include <ROS2/Lidar/LidarRegistrarBus.h>

namespace ROS2
{
    //! A Component that manages LidarSystems' registration and storage of their metadata.
    class LidarRegistrarSystemComponent
        : public AZ::Component
        , protected LidarRegistrarRequestBus::Handler
    {
    public:
        AZ_COMPONENT(LidarRegistrarSystemComponent, "{78cba3f1-db2c-46de-9c3d-c40dd72f2f1e}");
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        LidarRegistrarSystemComponent();
        virtual ~LidarRegistrarSystemComponent();

    protected:
        void Init() override;
        void Activate() override;
        void Deactivate() override;

        // LidarRegistrarRequestBus overrides
        void RegisterLidarSystem(const char* name, const char* description, const LidarSystemFeatures& features) override;
        AZStd::vector<AZStd::string> GetRegisteredLidarSystems() const override;
        const LidarSystemMetaData* GetLidarSystemMetaData(const AZStd::string& name) const override;

    private:
        LidarSystem m_physxLidarSystem;
        AZStd::unordered_map<AZ::Crc32, LidarSystemMetaData> m_registeredLidarSystems;
    };

    namespace Details
    {
        AZStd::string GetDefaultLidarSystem();
    } // namespace Details

} // namespace ROS2
