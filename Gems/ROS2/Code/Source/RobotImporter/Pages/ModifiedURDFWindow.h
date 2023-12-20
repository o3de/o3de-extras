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
#include <QPushButton>
#include <QString>
#include <QTextEdit>
#include <QWindow>
#endif

namespace ROS2
{
    class ModifiedURDFWindow : public QWidget
    {
        Q_OBJECT
    public:
        ModifiedURDFWindow();
        void SetUrdfData(const QString& urdf);

    protected:
        void closeEvent(QCloseEvent* event) override;

    private:
        QTextEdit* m_log;
    };
} // namespace ROS2
