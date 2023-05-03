/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include <AzCore/Math/Crc.h>
#include <AzCore/std/string/string.h>
#include <QLabel>
#include <QWizardPage>
#endif

namespace ROS2
{
    class IntroPage : public QWizardPage
    {
        Q_OBJECT
    public:
        IntroPage(QWizard* parent);

    private:
        QLabel* m_label;
    };
} // namespace ROS2
