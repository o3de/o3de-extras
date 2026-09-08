/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/containers/map.h>
#include <AzCore/std/parallel/mutex.h>
#include <RobotImporter/ReferencedAssetsRequestBus.h>
#include <RobotImporter/StatusAggregationRequestBus.h>

namespace ROS2RobotImporter
{
    class ImportSession
        : private ReferencedAssetsRequestBus::Handler
        , private StatusAggregationRequestBus::Handler
    {
    public:
        explicit ImportSession(Assets::ReferencedAssetMap referencedAssetMap);
        ImportSession(const ImportSession& other) = delete;
        ImportSession(ImportSession&& other) = delete;
        ImportSession& operator=(const ImportSession& other) = delete;
        ImportSession& operator=(ImportSession&& other) = delete;
        ~ImportSession();

        ImportSessionId GetId() const;

        //! Get descriptive status of import.
        //! A string with the status in Markdown format.
        AZStd::string GetStatus() const;

    private:
        void ReportImportStatusMessage(ImportStatusMessageType messageType, const AZStd::string& message) override;
        void ReportArticulatedLinkCreated() override;
        AZStd::optional<Assets::ReferencedAsset> FindReferencedAssets(
            const AZStd::string& modelUri, const AZ::IO::Path& assetUri) const override;

        //! Assets referenced by the source file, resolved before the import started.
        const Assets::ReferencedAssetMap m_referencedAssetMap;

        //! Address of the buses of this import, used by the makers to reach this session.
        const ImportSessionId m_id;

        //! Guards the members below, which are modified by the makers.
        mutable AZStd::mutex m_statusLock;

        //! Messages reported by the makers, kept grouped by the type of the element they refer to.
        AZStd::multimap<ImportStatusMessageType, AZStd::string> m_status;

        //! Number of the created articulation links, tracked in order to warn the user if the count becomes excessive.
        unsigned int m_articulationsCounter{ 0u };
    };

} // namespace ROS2RobotImporter
