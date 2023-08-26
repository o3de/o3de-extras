/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include <QCheckBox>
#include <QFileDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QString>
#include <QWizardPage>
#endif

namespace AzToolsFramework
{
    class ReflectedPropertyEditor;
}

namespace ROS2
{
    struct SdfAssetBuilderSettings;
}


namespace ROS2
{
    class FileSelectionPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit FileSelectionPage(QWizard* parent);
        // The destructor is defaulted in the cpp file to allow the unique_ptr<SdfAssetBuilderSettings>
        // to not need a complete type in the header
        ~FileSelectionPage();

        bool isComplete() const override;

        QString getFileName() const
        {
            if (m_fileExists)
            {
                return m_textEdit->text();
            }
            return "";
        }
        bool getIfCopyAssetsDuringUrdfImport() const;

        const SdfAssetBuilderSettings& GetSdfAssetBuilderSettings() const;

    private:
        QFileDialog* m_fileDialog;
        QPushButton* m_button;
        QLineEdit* m_textEdit;
        QCheckBox* m_copyFiles;

        AZStd::unique_ptr<SdfAssetBuilderSettings> m_sdfAssetBuilderSettings;
        AzToolsFramework::ReflectedPropertyEditor* m_sdfAssetBuilderSettingsEditor{};
        void onLoadButtonPressed();

        void onFileSelected(const QString& file);

        void onEditingFinished();

        bool m_fileExists{ false };
    };
} // namespace ROS2
