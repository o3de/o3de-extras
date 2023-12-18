/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ROS2/Spawner/SpawnerInfo.h"
#include <AzCore/Component/Entity.h>
#include <qcombobox.h>
#if !defined(Q_MOC_RUN)
#include <AzCore/Math/Crc.h>
#include <AzCore/std/string/string.h>
#include <QCheckBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QWizardPage>
#endif

namespace ROS2
{
    class RobotImporterWidget;
    class PrefabMakerPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit PrefabMakerPage(RobotImporterWidget* parent);
        void SetProposedPrefabName(const AZStd::string prefabName);
        AZStd::string GetPrefabName() const;
        void ReportProgress(const AZStd::string& progressForUser);
        void SetSuccess(bool success);
        bool isComplete() const override;
        AZStd::optional<AZ::Transform> getSelectedSpawnPoint() const;
    Q_SIGNALS:
        void onCreateButtonPressed();

    private:
        static bool IsZeroPoint(AZStd::string spawnPointName);

        static constexpr AZStd::string_view zeroPoint = "Simulation origin";

        bool m_success;
        QLineEdit* m_prefabName;
        QPushButton* m_createButton;
        QTextEdit* m_log;
        QComboBox* m_spawnPointsComboBox;
        SpawnPointInfoMap m_spawnPointsInfos;
        RobotImporterWidget* m_parentImporterWidget;
    };
} // namespace ROS2
