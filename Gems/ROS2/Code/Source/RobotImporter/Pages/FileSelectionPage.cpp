/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FileSelectionPage.h"
#include <AzCore/Utils/Utils.h>
#include <QHBoxLayout>
#include <QVBoxLayout>

namespace ROS2
{
    FileSelectionPage::FileSelectionPage(QWizard* parent)
        : QWizardPage(parent)
    {
        m_fileDialog = new QFileDialog(this);
        m_fileDialog->setDirectory(QString::fromUtf8(AZ::Utils::GetProjectPath().data()));
        m_fileDialog->setNameFilter("URDF (*.urdf)");
        m_button = new QPushButton("...", this);
        m_textEdit = new QLineEdit("", this);
        setTitle(tr("Load URDF file"));
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addStretch();
        layout->addWidget(new QLabel(tr("URDF file path to load : "), this));
        QHBoxLayout* layout_in = new QHBoxLayout;
        layout_in->addWidget(m_button);
        layout_in->addWidget(m_textEdit);
        layout->addLayout(layout_in);
        layout->addStretch();
        this->setLayout(layout);
        connect(m_button, &QPushButton::pressed, this, &FileSelectionPage::onLoadButtonPressed);
        connect(m_fileDialog, &QFileDialog::fileSelected, this, &FileSelectionPage::onFileSelected);
        connect(m_textEdit, &QLineEdit::textChanged, this, &FileSelectionPage::onTextChanged);
        FileSelectionPage::onTextChanged(m_textEdit->text());
    }

    void FileSelectionPage::onLoadButtonPressed()
    {
        m_fileDialog->show();
    }

    void FileSelectionPage::onFileSelected(const QString& file)
    {
        m_textEdit->setText(file);
    }

    void FileSelectionPage::onTextChanged(const QString& text)
    {
        m_fileExists = QFileInfo::exists(text);
        emit completeChanged();
    }

    bool FileSelectionPage::isComplete() const
    {
        return m_fileExists;
    };
} // namespace ROS2
