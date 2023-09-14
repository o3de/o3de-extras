/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FileSelectionPage.h"
#include <AzCore/Utils/Utils.h>
#include <AzToolsFramework/UI/PropertyEditor/ReflectedPropertyEditor.hxx>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>

#include <QFileInfo>
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QSettings>

namespace ROS2
{
    static constexpr const char FileSelectionPageDefaultFile[] = "RobotImporter/SelectFileDefaultFile";

    FileSelectionPage::FileSelectionPage(QWizard* parent)
        : QWizardPage(parent)
        , m_sdfAssetBuilderSettings(AZStd::make_unique<SdfAssetBuilderSettings>())
    {
        m_fileDialog = new QFileDialog(this);
        m_fileDialog->setNameFilter("URDF, XACRO, SDF, WORLD (*.urdf *.xacro *.sdf *.world)");
        // Whenever the selected file is successfully changed via the File Dialog or the Text Edit widget,
        // save the full file name with path into the QSettings so that it defaults correctly the next time it is opened.
        connect(this, &QWizardPage::completeChanged, [this]()
        {
            if (m_fileExists)
            {
                QSettings settings;
                const QString absolutePath = m_textEdit->text();
                settings.setValue(FileSelectionPageDefaultFile, absolutePath);
            }
        });

        m_button = new QPushButton("...", this);
        m_textEdit = new QLineEdit("", this);
        setTitle(tr("Load URDF/SDF file"));
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addStretch();
        layout->addWidget(new QLabel(tr("URDF/SDF file path to load : "), this));
        QHBoxLayout* layoutHorizontal = new QHBoxLayout;
        layoutHorizontal->addWidget(m_button);
        layoutHorizontal->addWidget(m_textEdit);
        layout->addLayout(layoutHorizontal);

        // Seed the SDF Asset Builder Settings with the values from the Settings Registry
        m_sdfAssetBuilderSettings->LoadSettings();
        // Create a reflected property editor that can modify the SDF AssetBuilder Settings
        m_sdfAssetBuilderSettingsEditor = new AzToolsFramework::ReflectedPropertyEditor(this);
        AZ::ComponentApplicationRequests* componentApplicationRequests = AZ::Interface<AZ::ComponentApplicationRequests>::Get();
        AZ::SerializeContext* serializeContext{ componentApplicationRequests ? componentApplicationRequests->GetSerializeContext() : nullptr };
        AZ_Assert(serializeContext != nullptr, "Serialize Context is missing. It is required for the SDF Asset Builder Settings Editor");
        constexpr bool enableScrollBars = true;
        m_sdfAssetBuilderSettingsEditor->Setup(serializeContext, nullptr, enableScrollBars);
        m_sdfAssetBuilderSettingsEditor->AddInstance(m_sdfAssetBuilderSettings.get());
        m_sdfAssetBuilderSettingsEditor->InvalidateAll();
        layout->addWidget(m_sdfAssetBuilderSettingsEditor);

        this->setLayout(layout);
        connect(m_button, &QPushButton::pressed, this, &FileSelectionPage::onLoadButtonPressed);
        connect(m_fileDialog, &QFileDialog::fileSelected, this, &FileSelectionPage::onFileSelected);
        connect(m_textEdit, &QLineEdit::editingFinished, this, &FileSelectionPage::onEditingFinished);
        FileSelectionPage::onEditingFinished();
    }

    FileSelectionPage::~FileSelectionPage() = default;

    void FileSelectionPage::RefreshDefaultPath()
    {
        // The first time this dialog ever gets opened, default to the project's root directory.
        // Once a URDF/SDF file has been selected or typed in, change the default directory to the location of that file.
        // This gets stored in QSettings, so it will persist between Editor runs.
        QSettings settings;
        QString defaultFile(settings.value(FileSelectionPageDefaultFile).toString());
        if (!defaultFile.isEmpty() && QFile(defaultFile).exists())
        {
            // Set both the default directory and the default file in that directory.
            m_fileDialog->setDirectory(QFileInfo(defaultFile).absolutePath());
            m_fileDialog->selectFile(QFileInfo(defaultFile).fileName());
        }
        else
        {
            // No valid file was found, so default back to the current project path.
            m_fileDialog->setDirectory(QString::fromUtf8(AZ::Utils::GetProjectPath().c_str()));
            m_fileDialog->selectFile("");
        }
    }

    void FileSelectionPage::onLoadButtonPressed()
    {
        // Refresh the default path in the file dialog every time it is opened so that
        // any changes in the text edit box are reflected in its default path and any Cancel
        // pressed to escape from the file dialog *don't* change its default path.
        RefreshDefaultPath();
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

    QString FileSelectionPage::getFileName() const
    {
        return isComplete() ? m_textEdit->text(): QString{};
    }

    const SdfAssetBuilderSettings& FileSelectionPage::GetSdfAssetBuilderSettings() const
    {
        return *m_sdfAssetBuilderSettings;
    }
} // namespace ROS2
