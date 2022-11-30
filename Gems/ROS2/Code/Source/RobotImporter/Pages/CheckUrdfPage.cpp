/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CheckUrdfPage.h"
namespace ROS2
{
    CheckUrdfPage::CheckUrdfPage(QWizard* parent)
        : QWizardPage(parent)
        , m_success(false)
    {
        m_log = new QTextEdit(this);
        setTitle(tr("URDF opening results:"));
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_log);
        m_log->acceptRichText();
        m_log->setReadOnly(true);
        this->setLayout(layout);
    }

    void CheckUrdfPage::ReportURDFResult(const QString& status, bool isSuccess)
    {
        m_log->setMarkdown(status);
        m_success = isSuccess;
        emit completeChanged();
    }

    bool CheckUrdfPage::isComplete() const
    {
        return m_success;
    }
} // namespace ROS2
