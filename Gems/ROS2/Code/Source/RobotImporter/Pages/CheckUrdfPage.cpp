/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CheckUrdfPage.h"
#include <QVBoxLayout>

namespace ROS2
{
    CheckUrdfPage::CheckUrdfPage(QWizard* parent)
        : QWizardPage(parent)
        , m_success(false)
        , m_warning(false)
    {
        m_log = new QTextEdit(this);
        setTitle(tr("URDF/SDF opening results:"));
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_log);
        m_log->acceptRichText();
        m_log->setReadOnly(true);
        setLayout(layout);
    }

    void CheckUrdfPage::ReportURDFResult(const QString& status, bool isSuccess, bool isWarning)
    {
        m_log->setMarkdown(status);
        m_success = isSuccess;
        m_warning = isWarning;
        emit completeChanged();
    }

    bool CheckUrdfPage::isComplete() const
    {
        return m_success;
    }

    bool CheckUrdfPage::isWarning() const
    {
        return m_warning;
    }
} // namespace ROS2
