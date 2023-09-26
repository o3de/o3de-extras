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

        m_label = new QLabel(
            QObject::tr("This wizard allows you to build a robot prefab using a URDF description file or object/environment prefab using an SDF file."
                        " Before processing, please make sure that all of the robot's description packages have been built and sourced."
                        " Details can be found <a "
                        "href=\"https://www.o3de.org/docs/user-guide/interactivity/robotics/importing-robot/"
                        "#loading-the-robot-definition-file-with-robot-importer\">here</a>."
                        " The Open 3D Engine can only use files in its internal <a "
                        "href=\"https://www.o3de.org/docs/user-guide/assets/asset-types/\">format</a>."
                        "During the import process, the assets will be imported and processed."
                        "A level must be opened before using the URDF/SDF Importer."));
        m_label->setTextFormat(Qt::RichText);
        m_label->setOpenExternalLinks(true);
        m_label->setWordWrap(true);

        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_label);
        setLayout(layout);
    }

} // namespace ROS2
