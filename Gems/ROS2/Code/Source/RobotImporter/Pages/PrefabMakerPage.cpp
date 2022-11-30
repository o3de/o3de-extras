/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "PrefabMakerPage.h"
#include "RobotImporter/RobotImporterWidget.h"

namespace ROS2
{

    PrefabMakerPage::PrefabMakerPage(RobotImporterWidget* parent)
        : QWizardPage(parent)
        , m_parentImporterWidget(parent)
        , m_success(false)
    {
        m_prefabName = new QLineEdit(this);
        m_createButton = new QPushButton(tr("Create Prefab"), this);
        m_log = new QTextEdit(this);
        setTitle(tr("Prefab creation"));
        QVBoxLayout* layout = new QVBoxLayout;
        QHBoxLayout* layoutInner = new QHBoxLayout;
        layoutInner->addWidget(m_prefabName);
        layoutInner->addWidget(m_createButton);
        layout->addLayout(layoutInner);
        layout->addWidget(m_log);
        this->setLayout(layout);
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

} // namespace ROS2
