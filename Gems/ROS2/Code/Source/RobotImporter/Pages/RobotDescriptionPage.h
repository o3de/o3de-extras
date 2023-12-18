/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include <QLabel>
#include <QString>
#include <QTextEdit>
#include <QWizardPage>
#endif

namespace ROS2
{
    class RobotDescriptionPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit RobotDescriptionPage(QWizard* parent);
        void ReportParsingResult(const QString& status, bool isSuccess, bool isWarning = false);
        bool isComplete() const override;
        bool isWarning() const;

    private:
        QTextEdit* m_log;
        QString m_fileName;
        bool m_success;
        bool m_warning;
    };
} // namespace ROS2
