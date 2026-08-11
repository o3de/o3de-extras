/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <RobotImporter/RobotImporterAssetsBus.h>

namespace ROS2RobotImporter
{
    class ImportSession
        : public RobotImporterAssetsRequestBus::Handler
    {
    public:
        explicit ImportSession(Assets::ReferencedAssetMap referencedAssetMap);
        ~ImportSession();

        ImportSessionId GetId() const;

    private:
        AZStd::optional<Assets::ReferencedAsset> FindRefferencedAssets(AZ::IO::Path modelUri) const override;

        const Assets::ReferencedAssetMap m_referencedAssetMap;
        const ImportSessionId m_Id;
    };

} // namespace ROS2RobotImporter
