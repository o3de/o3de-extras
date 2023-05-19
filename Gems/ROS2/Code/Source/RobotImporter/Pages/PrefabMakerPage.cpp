/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PrefabMakerPage.h"
#include "Spawner/ROS2SpawnerInterface.h"
#include <AzCore/Component/Entity.h>
#include <AzCore/Debug/Trace.h>
#include <RobotImporter/RobotImporterWidget.h>
#include <cstdint>
#include <qcombobox.h>
#include <qlabel.h>

namespace ROS2
{

    PrefabMakerPage::PrefabMakerPage(RobotImporterWidget* parent)
        : QWizardPage(parent)
        , m_parentImporterWidget(parent)
        , m_success(false)
    {
        auto spawnerInterface = ROS2::SpawnerInterface::Get();
        if (spawnerInterface != nullptr)
        {
            m_spawnPointsInfos = spawnerInterface->GetAllSpawnPoints();
        }
        else
        {
            AZ_TracePrintf("PrefabMakerPage", "Spawner interface not found");
        }
        m_spawnPointsList = new QComboBox(this);
        for (uint32_t i = 0; i < m_spawnPointsInfos.size(); i++)
        {
            m_spawnPointsList->addItem(m_spawnPointsInfos[i].first.c_str(), QVariant(i + 1));
        }

        m_prefabName = new QLineEdit(this);
        m_createButton = new QPushButton(tr("Create Prefab"), this);
        m_log = new QTextEdit(this);
        m_useArticulation = new QCheckBox(tr("Use articulation for joints and rigid bodies"), this);
        setTitle(tr("Prefab creation"));
        QVBoxLayout* layout = new QVBoxLayout;
        QHBoxLayout* layoutInner = new QHBoxLayout;
        layoutInner->addWidget(m_prefabName);
        layoutInner->addWidget(m_createButton);
        layout->addLayout(layoutInner);
        layout->addWidget(m_useArticulation);
        auto spawnPointListLabel = new QLabel("Select spawn position", this);
        layout->addWidget(spawnPointListLabel);
        layout->addWidget(m_spawnPointsList);
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
    bool PrefabMakerPage::IsUseArticulations() const
    {
        return m_useArticulation->isChecked();
    }
    AZStd::shared_ptr<AZ::Transform> PrefabMakerPage::getSelectedSpawnPoint() const
    {
        return m_spawnPointsInfos[m_spawnPointsList->currentIndex()].second;
    }

} // namespace ROS2
