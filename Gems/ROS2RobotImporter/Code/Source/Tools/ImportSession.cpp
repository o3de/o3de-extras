/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ImportSession.h"

#include <AzCore/std/parallel/atomic.h>
#include <AzCore/std/string/conversions.h>

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
        RobotImporterStatusRequestBus::Handler::BusConnect(m_Id);
    }

    ImportSession::~ImportSession()
    {
        RobotImporterAssetsRequestBus::Handler::BusDisconnect();
        RobotImporterStatusRequestBus::Handler::BusDisconnect();
    }

    ImportSessionId ImportSession::GetId() const
    {
        return m_Id;
    }

    AZStd::string ImportSession::GetStatus()
    {
        AZStd::string report;
        AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);

        // Print warnings first
        constexpr unsigned int articulationsLimit = 64;
        if (m_articulationsCounter >= articulationsLimit)
        {
            report += "\n## 💡 Note: the number of articulations (" + AZStd::to_string(m_articulationsCounter) +
                ") might not be supported by the physics engine.\n";
        }

        report += "# The following components were found and parsed:\n";

        const AZStd::unordered_map<ImportStatusMessageType, AZStd::string> names = {
            { ImportStatusMessageType::Model, "Models" },
            { ImportStatusMessageType::Link, "Links" },
            { ImportStatusMessageType::Joint, "Joints" },
            { ImportStatusMessageType::Sensor, "Sensors" },
            { ImportStatusMessageType::SensorPlugin, "Sensor plugins" },
            { ImportStatusMessageType::ModelPlugin, "Model plugins" }
        };
        auto it = m_status.begin();
        auto end = m_status.end();
        while (it != end)
        {
            const auto key = it->first;
            report += "\n## " + names.at(key) + ":\n";

            do
            {
                report += "- " + it->second + "\n";
                if (++it == end)
                {
                    break;
                }
            } while (it->first == key);
        }
        return report;
    }

    void ImportSession::OnImportStatusMessage(ImportStatusMessageType messageType, const AZStd::string& message)
    {
        AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
        m_status.emplace(messageType, message);
    }

    void ImportSession::OnArticulatedLinkCreated()
    {
        AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
        m_articulationsCounter++;
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