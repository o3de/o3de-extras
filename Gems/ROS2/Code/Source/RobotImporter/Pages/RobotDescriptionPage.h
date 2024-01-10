/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>

#if !defined(Q_MOC_RUN)
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
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
        void SetModifiedUrdfName(const AZStd::string prefabName);
        AZStd::string GetModifiedUrdfName() const;
        bool isComplete() const override;
        bool isWarning() const;
    Q_SIGNALS:
        void onSaveModifiedUrdfPressed();
        void onShowModifiedUrdfPressed();

    private:
        QTextEdit* m_log;
        QString m_fileName;
        QLineEdit* m_urdfName;
        QPushButton* m_saveButton;
        QPushButton* m_showButton;
        bool m_success;
        bool m_warning;
    };
} // namespace ROS2
