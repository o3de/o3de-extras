/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ImportSession.h"

#include <AzCore/std/parallel/atomic.h>

namespace ROS2RobotImporter
{
    namespace
    {
        AZStd::atomic<ImportSessionId> s_nextSessionId{ 0u };
    }

    ImportSession::ImportSession(Assets::ReferencedAssetMap referencedAssetMap)
        : m_referencedAssetMap(AZStd::move(referencedAssetMap))
        , m_Id(++s_nextSessionId)
    {
        RobotImporterAssetsRequestBus::Handler::BusConnect(m_Id);
    }

    ImportSession::~ImportSession()
    {
        RobotImporterAssetsRequestBus::Handler::BusDisconnect();
    }

    ImportSessionId ImportSession::GetId() const
    {
        return m_Id;
    }

    AZStd::optional<Assets::ReferencedAsset> ImportSession::FindRefferencedAssets(AZ::IO::Path modelUri) const
    {
        auto it = m_referencedAssetMap.find(modelUri);
        if (it != m_referencedAssetMap.end())
        {
            return it->second;
        }

        return AZStd::nullopt;
    }

} // namespace ROS2RobotImporter
