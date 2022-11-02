/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>
#include <atomic>
#include <functional>

#include "RobotImporter/URDF/URDFPrefabMaker.h"

namespace ROS2
{
    //! Robot importer configuration structure
    struct RobotImporterConfig
    {
        AZStd::string urdfFilePath; //!< Path to the robot definition file to import
        AZStd::string prefabFilePath; //!< Path to the prefab file to create
    };

    //! Handles importing of a robot definition file
    class RobotImporter
    {
    public:
        enum LogLevel
        {
            Info,
            Error
        };
        using LoggerFunction = AZStd::function<void(LogLevel, AZStd::string_view)>;

        //! Constructs robot importer
        //! @param infoLogger Function that is called to log some information
        RobotImporter(RobotImporter::LoggerFunction logger);

        //! Parses URDF file and starts asset import process
        //! This function starts a thread that initializes and waits for the asset import process to finish
        //! Caller is expected to use ParseURDFAndStartLoadingAssets to check if the import process is finished
        //! and proceed with the prefab creation.
        //! @param config Configuration for the import process
        void ParseURDFAndStartLoadingAssets(const RobotImporterConfig& config);

        //! Checks if assets are loaded and if so, proceeds with prefab creation
        //! This function should be called from the main thread repeatedly after ParseURDFAndStartLoadingAssets
        //! It will poll the asset import process and if it is finished, it will proceed with prefab creation
        //! @param importFinishedCb Function that is called when the import process is finished
        void CheckIfAssetsWereLoadedAndCreatePrefab(std::function<void()> importFinishedCb);

        AZStd::string GetProgress()
        {
            if (m_prefabMaker)
            {
                return m_prefabMaker->getStatus();
            }
            return "";
        }

    private:
        std::atomic_bool m_isProcessingAssets;
        std::atomic_bool m_loadingURDFFailed;
        std::optional<URDFPrefabMaker> m_prefabMaker;
        LoggerFunction m_logger;
    };

} // namespace ROS2
