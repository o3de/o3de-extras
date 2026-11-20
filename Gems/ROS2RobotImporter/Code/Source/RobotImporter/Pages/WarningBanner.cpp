/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "WarningBanner.h"
#include <QHBoxLayout>
#include <QIcon>

namespace ROS2RobotImporter
{
    WarningBanner::WarningBanner(QWidget* parent)
        : QFrame(parent)
    {
        setFrameShape(QFrame::StyledPanel);
        setStyleSheet(QStringLiteral("QFrame { background-color: #4D3B00; border: 1px solid #E5A000; border-radius: 4px; }"));

        QHBoxLayout* layout = new QHBoxLayout(this);

        QLabel* icon = new QLabel(this);
        icon->setPixmap(QIcon(QStringLiteral(":/stylesheet/img/logging/warning.svg")).pixmap(24, 24));
        icon->setStyleSheet(QStringLiteral("QLabel { border: none; }"));
        icon->setAlignment(Qt::AlignTop);
        layout->addWidget(icon);

        m_text = new QLabel(this);
        m_text->setWordWrap(true);
        m_text->setTextFormat(Qt::RichText);
        m_text->setTextInteractionFlags(Qt::TextBrowserInteraction);
        m_text->setStyleSheet(QStringLiteral("QLabel { border: none; }"));
        layout->addWidget(m_text, 1);

        hide();
    }

    void WarningBanner::SetWarning(const QString& message)
    {
        m_text->setText(message);
        show();
    }

    void WarningBanner::ClearWarning()
    {
        m_text->clear();
        hide();
    }
} // namespace ROS2RobotImporter
