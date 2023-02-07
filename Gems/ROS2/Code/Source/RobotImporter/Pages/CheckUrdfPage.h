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
    class CheckUrdfPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit CheckUrdfPage(QWizard* parent);
        void ReportURDFResult(const QString& result, bool isSuccess);
        bool isComplete() const override;

    private:
        QTextEdit* m_log;
        QString m_fileName;
        bool m_success;
    };
} // namespace ROS2
