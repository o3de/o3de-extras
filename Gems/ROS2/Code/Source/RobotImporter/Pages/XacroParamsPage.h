/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include <QFileDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QTableWidget>
#include <QWizardPage>
#include <RobotImporter/xacro/XacroUtils.h>
#endif

namespace ROS2
{
    class RobotImporterWidget;

    //!  Wizard page that allows user to modify Xacro parameters
    class XacroParamsPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit XacroParamsPage(RobotImporterWidget* parent);

        bool isComplete() const override;

        void SetXacroParameters(const Utils::xacro::Params& params);
        Utils::xacro::Params GetXacroParameters() const;

    private:
        Utils::xacro::Params m_defaultParams;
        QTableWidget* m_table {};
    };
} // namespace ROS2
