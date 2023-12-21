/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "ModifiedUrdfWindow.h"
#include <AzCore/Utils/Utils.h>
#include <QCloseEvent>
#include <QVBoxLayout>

namespace ROS2
{
    ModifiedUrdfWindow::ModifiedUrdfWindow()
    {
        m_log = new QTextEdit();
        m_log->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
        setWindowTitle(tr("Modified URDF data"));
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_log);
        m_log->acceptRichText();
        m_log->setReadOnly(true);
        this->setLayout(layout);
    }

    void ModifiedUrdfWindow::closeEvent([[maybe_unused]] QCloseEvent* event)
    {
        event->ignore();
        this->hide();
    }

    void ModifiedUrdfWindow::SetUrdfData(const std::string& urdf)
    {
        m_urdf = AZStd::move(urdf);
        m_log->setText(QString::fromStdString(m_urdf));
    }

    const std::string& ModifiedUrdfWindow::GetUrdfData() const
    {
        return m_urdf;
    }
} // namespace ROS2
