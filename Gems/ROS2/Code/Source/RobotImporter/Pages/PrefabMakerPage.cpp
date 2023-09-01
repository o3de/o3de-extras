/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PrefabMakerPage.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Debug/Trace.h>
#include <AzCore/EBus/Results.h>
#include <AzCore/Math/Transform.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/string/string.h>

#include <ROS2/Spawner/SpawnerBus.h>
#include <ROS2/Spawner/SpawnerInfo.h>
#include <RobotImporter/RobotImporterWidget.h>
#include <optional>
#include <qcombobox.h>
#include <qvariant.h>

namespace ROS2
{
    PrefabMakerPage::PrefabMakerPage(RobotImporterWidget* parent)
        : QWizardPage(parent)
        , m_success(false)
        , m_parentImporterWidget(parent)
    {
        AZ::EBusAggregateResults<AZStd::unordered_map<AZStd::string, SpawnPointInfo>> allActiveSpawnPoints;
        SpawnerRequestsBus::BroadcastResult(allActiveSpawnPoints, &SpawnerRequestsBus::Events::GetAllSpawnPointInfos);

        m_spawnPointsComboBox = new QComboBox(this);
        m_spawnPointsInfos = allActiveSpawnPoints.values;

        for (int i = 0; i < allActiveSpawnPoints.values.size(); i++)
        {
            for (const auto& element : allActiveSpawnPoints.values[i])
            {
                m_spawnPointsComboBox->addItem(element.first.c_str(), QVariant(i));
            }
        }

        m_prefabName = new QLineEdit(this);
        m_createButton = new QPushButton(tr("Create Prefab"), this);
        m_log = new QTextEdit(this);

        setTitle(tr("Prefab creation"));
        QVBoxLayout* layout = new QVBoxLayout;
        QHBoxLayout* layoutInner = new QHBoxLayout;
        layoutInner->addWidget(m_prefabName);
        layoutInner->addWidget(m_createButton);
        layout->addLayout(layoutInner);
        QLabel* spawnPointListLabel;
        if (allActiveSpawnPoints.values.size() == 0)
        {
            spawnPointListLabel = new QLabel("Select spawn position (No spawn positions were detected)", this);
        }
        else
        {
            spawnPointListLabel = new QLabel("Select spawn position", this);
        }
        layout->addWidget(spawnPointListLabel);
        layout->addWidget(m_spawnPointsComboBox);
        layout->addWidget(m_log);
        setLayout(layout);
        connect(m_createButton, &QPushButton::pressed, this, &PrefabMakerPage::onCreateButtonPressed);
    }

    void PrefabMakerPage::setProposedPrefabName(const AZStd::string prefabName)
    {
        m_prefabName->setText(QString::fromUtf8(prefabName.data(), int(prefabName.size())));
    }

    AZStd::string PrefabMakerPage::getPrefabName() const
    {
        return AZStd::string(m_prefabName->text().toUtf8().constData());
    }

    void PrefabMakerPage::reportProgress(const AZStd::string& progressForUser)
    {
        m_log->setText(QString::fromUtf8(progressForUser.data(), int(progressForUser.size())));
    }
    void PrefabMakerPage::setSuccess(bool success)
    {
        m_success = success;
        emit completeChanged();
    }
    bool PrefabMakerPage::isComplete() const
    {
        return m_success;
    }
    AZStd::optional<AZ::Transform> PrefabMakerPage::getSelectedSpawnPoint() const
    {
        if (!m_spawnPointsInfos.empty())
        {
            int vectorIndex = m_spawnPointsComboBox->currentData().toInt();
            AZStd::string mapKey(m_spawnPointsComboBox->currentText().toStdString().c_str());
            auto& map = m_spawnPointsInfos[vectorIndex];
            if (auto spawnInfo = map.find(mapKey);
                spawnInfo != map.end())
            {
                return spawnInfo->second.pose;
            }
        }
        return AZStd::nullopt;
    }
} // namespace ROS2
