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

namespace ROS2
{
    FileSelectionPage::FileSelectionPage(QWizard* parent)
        : QWizardPage(parent)
        , m_sdfAssetBuilderSettings(AZStd::make_unique<SdfAssetBuilderSettings>())
    {
        m_fileDialog = new QFileDialog(this);
        m_fileDialog->setDirectory(QString::fromUtf8(AZ::Utils::GetProjectPath().data()));
        m_fileDialog->setNameFilter("URDF, XACRO, SDF, WORLD (*.urdf *.xacro *.sdf *.world)");
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
        // Make sure the SDF Asset Builder settings are expanded by default
        m_sdfAssetBuilderSettingsEditor->ExpandAll();
        layout->addWidget(m_sdfAssetBuilderSettingsEditor);

        this->setLayout(layout);
        connect(m_button, &QPushButton::pressed, this, &FileSelectionPage::onLoadButtonPressed);
        connect(m_fileDialog, &QFileDialog::fileSelected, this, &FileSelectionPage::onFileSelected);
        connect(m_textEdit, &QLineEdit::editingFinished, this, &FileSelectionPage::onEditingFinished);
        FileSelectionPage::onEditingFinished();
    }

    FileSelectionPage::~FileSelectionPage() = default;

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

    QString FileSelectionPage::getFileName() const
    {
        return isComplete() ? m_textEdit->text(): QString{};
    }

    const SdfAssetBuilderSettings& FileSelectionPage::GetSdfAssetBuilderSettings() const
    {
        return *m_sdfAssetBuilderSettings;
    }
} // namespace ROS2
