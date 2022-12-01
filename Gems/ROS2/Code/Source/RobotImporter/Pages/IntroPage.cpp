/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "IntroPage.h"
#include <QVBoxLayout>

namespace ROS2
{
    IntroPage::IntroPage(QWizard* parent)
        : QWizardPage(parent)
    {
        setTitle(QObject::tr("Introduction"));

        m_label = new QLabel(QObject::tr("This wizard allows you to build robot prefab out of URDF description file."
                                         " Before processing, please, make sure that all necessary assets that are"
                                         " used by URDF file are processed by Asset Processor."
                                         " Open 3D Engine can only use files in its internal"
                                         " <a href=\"https://www.o3de.org/docs/user-guide/assets/asset-types/\">format</a>."
                                         " The tool, called Asset Processor allows processing source assets (e.g, meshes, textures)"
                                         " to supported internal format. To learn more about asset processors click"
                                         " <a href=\"https://www.o3de.org/docs/user-guide/assets/asset-processor/\">here</a>."
                                         " URDF Importer will find correct meshes in product assets during import."));
        m_label->setTextFormat(Qt::RichText);
        m_label->setOpenExternalLinks(true);
        m_label->setWordWrap(true);

        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_label);
        setLayout(layout);
    }

} // namespace ROS2
