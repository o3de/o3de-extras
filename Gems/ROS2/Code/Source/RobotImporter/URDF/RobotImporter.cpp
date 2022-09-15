/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotImporter.h"
#include "RobotImporter/RobotImporterWidget.h"

namespace ROS2
{
    RobotImporter::RobotImporter(LoggerFunction logger)
        : m_logger(std::move(logger))
        , m_isProcessingAssets(false)
        , m_loadingURDFFailed(false)
    {
    }

    void RobotImporter::ParseURDFAndStartLoadingAssets(const RobotImporterConfig& config)
    {
        m_prefabMaker.reset();
        m_loadingURDFFailed.store(false);
        m_logger(LogLevel::Info, "Importing robot definition file: " + config.urdfFilePath);

        urdf::ModelInterfaceSharedPtr urdfModel = UrdfParser::ParseFromFile(config.urdfFilePath);
        if (!urdfModel)
        {
            m_loadingURDFFailed.store(true);
            m_logger(LogLevel::Error, "Failed to parse the robot definition file");
            return;
        }

        m_logger(LogLevel::Info, "Processing URDF model...");

        m_prefabMaker.emplace(config.urdfFilePath, urdfModel, config.prefabFilePath);

        m_isProcessingAssets = true;
        m_prefabMaker->LoadURDF(
            [this]
            {
                m_isProcessingAssets = false;
            });
    }

    void RobotImporter::CheckIfAssetsWereLoadedAndCreatePrefab(std::function<void()> importFinishedCb)
    {
        if (m_loadingURDFFailed)
        {
            importFinishedCb();
            return;
        }
        AZ_Assert(m_prefabMaker, "Prefab maker is not initialized");
        if (m_prefabMaker)
        {
            if (m_isProcessingAssets)
            {
                return;
            }
            auto outcome = m_prefabMaker->CreatePrefabFromURDF();
            if (!outcome)
            {
                auto errorMessage = AZStd::string::format("Importing robot definition failed with error: %s", outcome.GetError().c_str());
                m_logger(LogLevel::Error, errorMessage);
                importFinishedCb();
                return;
            }
            m_logger(LogLevel::Info, AZStd::string::format("Imported %s", m_prefabMaker->GetPrefabPath().c_str()));
            importFinishedCb();
        }
    }
} // namespace ROS2
