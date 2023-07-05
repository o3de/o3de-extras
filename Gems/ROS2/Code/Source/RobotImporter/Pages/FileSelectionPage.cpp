/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FileSelectionPage.h"
#include <AzCore/Utils/Utils.h>
#include <QFileInfo>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace ROS2
{
    FileSelectionPage::FileSelectionPage(QWizard* parent)
        : QWizardPage(parent)
    {
        m_fileDialog = new QFileDialog(this);
        m_fileDialog->setDirectory(QString::fromUtf8(AZ::Utils::GetProjectPath().data()));
        m_fileDialog->setNameFilter("URDF, XACRO, SDF (*.urdf *.xacro *.sdf)");
        m_button = new QPushButton("...", this);
        m_textEdit = new QLineEdit("", this);
        m_copyFiles = new QCheckBox(tr("Import meshes during URDF load"), this);
        m_copyFiles->setCheckState(Qt::CheckState::Checked);
        setTitle(tr("Load URDF file"));
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addStretch();
        layout->addWidget(new QLabel(tr("URDF file path to load : "), this));
        QHBoxLayout* layout_in = new QHBoxLayout;
        layout_in->addWidget(m_button);
        layout_in->addWidget(m_textEdit);
        layout->addLayout(layout_in);
        layout->addWidget(m_copyFiles);
        layout->addStretch();
        this->setLayout(layout);
        connect(m_button, &QPushButton::pressed, this, &FileSelectionPage::onLoadButtonPressed);
        connect(m_fileDialog, &QFileDialog::fileSelected, this, &FileSelectionPage::onFileSelected);
        connect(m_textEdit, &QLineEdit::editingFinished, this, &FileSelectionPage::onEditingFinished);
        FileSelectionPage::onEditingFinished();
    }

    void FileSelectionPage::onLoadButtonPressed()
    {
        m_fileDialog->show();
    }

    void FileSelectionPage::onFileSelected(const QString& file)
    {
        QFileInfo urdfFile(file);
        m_textEdit->setText(file);
        m_fileExists = urdfFile.exists() && urdfFile.isFile();
        emit completeChanged();
    }

    void FileSelectionPage::onEditingFinished()
    {
        QFileInfo urdfFile(m_textEdit->text());
        m_fileExists = urdfFile.exists() && urdfFile.isFile();
        emit completeChanged();
    }

    bool FileSelectionPage::isComplete() const
    {
        return m_fileExists;
    }

    bool FileSelectionPage::getIfCopyAssetsDuringUrdfImport() const
    {
        return m_copyFiles->isChecked();
    }
} // namespace ROS2
