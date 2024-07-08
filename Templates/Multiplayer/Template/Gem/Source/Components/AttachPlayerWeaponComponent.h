/*
 * Copyright (c) Contributors to the Open 3D Engine Project
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <AzCore/Math/Transform.h>
#include <AzFramework/Spawnable/Spawnable.h>
#include <AzFramework/Spawnable/SpawnableEntitiesInterface.h>

namespace ${SanitizedCppName}
{
    //! @class AttachPlayerWeaponComponent
    //! @brief Spawns a non-network gun prefab and attaches it to the player entity.
    class AttachPlayerWeaponComponent final
        : public AZ::Component
    {
    public:
        AZ_COMPONENT(${SanitizedCppName}::AttachPlayerWeaponComponent, "{${Random_Uuid}}");

        static void Reflect(AZ::ReflectContext* context);

        //! AZ::Component overrides.
        //! @{
        void Activate() override;
        void Deactivate() override;
        //! }@

    private:
        AZStd::shared_ptr<AzFramework::EntitySpawnTicket> m_weaponTicket;
        AZ::Data::Asset<AzFramework::Spawnable> m_weaponAsset;
        AZStd::string m_boneToAttachTo;
        AZ::Transform m_attachmentTransform;
    };
}
