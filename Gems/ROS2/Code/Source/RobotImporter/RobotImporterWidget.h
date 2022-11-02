/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include "RobotImporter/URDF/RobotImporter.h"
#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <QCheckBox>
#include <QFileDialog>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QTimer>
#include <QWidget>
#endif

namespace ROS2
{
    class URDFPrefabMaker;
    //! Handles UI for the process of URDF importing
    class RobotImporterWidget : public QWidget
    {
        Q_OBJECT
    public:
        explicit RobotImporterWidget(QWidget* parent = nullptr);

    private:
        //! Report an error to the user.
        //! Populates the log, sets status information in the status label and shows an error popup with the message
        //! @param errorMessage error message to display to the user
        void ReportError(const AZStd::string& errorMessage);

        //! Report an information to the user.
        //! Populates the log and sets status information in the status label
        //! @param infoMessage info message to display to the user
        void ReportInfo(const AZStd::string& infoMessage);
        QLabel m_statusLabel;
        QTextEdit m_statusText;
        QPushButton m_selectFileButton;
        QTimer m_importerUpdateTimer;

        RobotImporter m_robotImporter;

        void ImporterTimerUpdate();
    };
} // namespace ROS2
