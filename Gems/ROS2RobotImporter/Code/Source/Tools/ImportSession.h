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
#include <RobotImporter/RobotImporterAssetsBus.h>
#include <RobotImporter/RobotImporterStatusBus.h>

namespace ROS2RobotImporter
{
    class ImportSession
        : public RobotImporterAssetsRequestBus::Handler
        , public RobotImporterStatusRequestBus::Handler
    {
    public:
        explicit ImportSession(Assets::ReferencedAssetMap referencedAssetMap);
        CollidersMaker(const CollidersMaker& other) = delete;
        ~ImportSession();

        ImportSessionId GetId() const;

        //! Get descriptive status of import.
        //! A string with the status in Markdown format.
        AZStd::string GetStatus();

    private:
        void OnImportStatusMessage(ImportStatusMessageType, const AZStd::string&) override;
        void OnArticulatedLinkCreated() override;
        AZStd::optional<Assets::ReferencedAsset> FindReferencedAssets(const AZ::IO::Path& modelUri) const override;

        const Assets::ReferencedAssetMap m_referencedAssetMap;
        const ImportSessionId m_id;

        AZStd::mutex m_statusLock;
        AZStd::multimap<ImportStatusMessageType, AZStd::string> m_status;
        unsigned int m_articulationsCounter{ 0u };
    };

} // namespace ROS2RobotImporter