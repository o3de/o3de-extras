/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include <QFrame>
#include <QLabel>
#include <QString>
#endif

namespace ROS2RobotImporter
{
    //! Highlighted strip that carries a warning at the top of a wizard page.
    //! The banner stays hidden until a warning is set, so a page adds it to its layout unconditionally.
    class WarningBanner : public QFrame
    {
    public:
        explicit WarningBanner(QWidget* parent = nullptr);

        void SetWarning(const QString& message);
        void ClearWarning();

    private:
        QLabel* m_text{};
    };
} // namespace ROS2RobotImporter
