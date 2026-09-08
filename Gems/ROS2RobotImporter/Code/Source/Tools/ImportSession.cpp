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
        , m_id(++s_nextSessionId)
    {
        ReferencedAssetsRequestBus::Handler::BusConnect(m_id);
        StatusAggregationRequestBus::Handler::BusConnect(m_id);
    }

    ImportSession::~ImportSession()
    {
        ReferencedAssetsRequestBus::Handler::BusDisconnect();
        StatusAggregationRequestBus::Handler::BusDisconnect();
    }

    ImportSessionId ImportSession::GetId() const
    {
        return m_id;
    }

    AZStd::string ImportSession::GetStatus() const
    {
        AZStd::string report;
        AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);

        // Print warnings first
        constexpr unsigned int articulationsLimit = 64;
        if (m_articulationsCounter >= articulationsLimit)
        {
            report += "\n## ⚠️ Note: the number of articulations (" + AZStd::to_string(m_articulationsCounter) +
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

    void ImportSession::ReportImportStatusMessage(ImportStatusMessageType messageType, const AZStd::string& message)
    {
        AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
        m_status.emplace(messageType, message);
    }

    void ImportSession::ReportArticulatedLinkCreated()
    {
        AZStd::lock_guard<AZStd::mutex> lck(m_statusLock);
        m_articulationsCounter++;
    }

    AZStd::optional<Assets::ReferencedAsset> ImportSession::FindReferencedAssets(
        const AZStd::string& modelUri, const AZ::IO::Path& assetUri) const
    {
        const AZ::IO::Path modelAssetKey = (modelUri.empty()) ? assetUri : AZ::IO::Path(modelUri + "/" + assetUri.String());
        if (auto it = m_referencedAssetMap.find(modelAssetKey); it != m_referencedAssetMap.end())
        {
            return it->second;
        }

        return AZStd::nullopt;
    }

} // namespace ROS2RobotImporter
