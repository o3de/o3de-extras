/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "RobotDescriptionPage.h"
#include <QVBoxLayout>

namespace ROS2
{
    RobotDescriptionPage::RobotDescriptionPage(QWizard* parent)
        : QWizardPage(parent)
        , m_success(false)
        , m_warning(false)
    {
        m_urdfName = new QLineEdit(this);
        m_saveButton = new QPushButton(tr("Save URDF"), this);
        m_showButton = new QPushButton(tr("Show URDF"), this);
        m_log = new QTextEdit(this);
        m_log->acceptRichText();
        m_log->setReadOnly(true);

        QVBoxLayout* layout = new QVBoxLayout;
        QHBoxLayout* layoutInner = new QHBoxLayout;
        layoutInner->addWidget(m_urdfName);
        layoutInner->addWidget(m_saveButton);
        layoutInner->addWidget(m_showButton);

        layout->addWidget(m_log);
        layout->addLayout(layoutInner);

        setTitle(tr("URDF/SDF opening results:"));
        setLayout(layout);
        connect(m_saveButton, &QPushButton::pressed, this, &RobotDescriptionPage::onSaveModifiedUrdfPressed);
        connect(m_showButton, &QPushButton::pressed, this, &RobotDescriptionPage::onShowModifiedUrdfPressed);
    }

    void RobotDescriptionPage::ReportParsingResult(const QString& status, bool isSuccess, bool isWarning)
    {
        m_log->setMarkdown(status);
        m_success = isSuccess;
        m_warning = isWarning;

        if ((isSuccess && isWarning == false) || isSuccess == false)
        {
            m_urdfName->setDisabled(true);
            m_saveButton->setDisabled(true);
            m_showButton->setDisabled(true);
        }
        else
        {
            m_urdfName->setDisabled(false);
            m_saveButton->setDisabled(false);
            m_showButton->setDisabled(false);
        }

        emit completeChanged();
    }

    void RobotDescriptionPage::SetModifiedUrdfName(const AZStd::string prefabName)
    {
        m_urdfName->setText(QString::fromUtf8(prefabName.data(), int(prefabName.size())));
    }

    AZStd::string RobotDescriptionPage::GetModifiedUrdfName() const
    {
        return AZStd::string(m_urdfName->text().toUtf8().constData());
    }

    bool RobotDescriptionPage::isComplete() const
    {
        return m_success;
    }

    bool RobotDescriptionPage::isWarning() const
    {
        return m_warning;
    }
} // namespace ROS2
