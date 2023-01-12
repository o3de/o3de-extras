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
    class XacroParamsPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit XacroParamsPage(RobotImporterWidget* parent);

        void setSuccess(bool success);
        bool isComplete() const override;

        void SetParameters(const Utils::xacro::Params& params);
        Utils::xacro::Params GetParams() const;

    private:
        Utils::xacro::Params m_defaultParams;
        QTableWidget* m_table;
    };
} // namespace ROS2
