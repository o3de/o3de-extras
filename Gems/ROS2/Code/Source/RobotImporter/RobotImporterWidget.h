/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include "Pages/CheckAssetPage.h"
#include "Pages/CheckUrdfPage.h"
#include "Pages/FileSelectionPage.h"
#include "Pages/IntroPage.h"
#include "Pages/PrefabMakerPage.h"
#include "Pages/XacroParamsPage.h"

#include "URDF/URDFPrefabMaker.h"
#include "URDF/UrdfParser.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/std/containers/unordered_map.h>
#include <RobotImporter/Utils/RobotImporterUtils.h>
#include <RobotImporter/xacro/XacroUtils.h>

#include <AzToolsFramework/API/ToolsApplicationAPI.h>
#include <AzToolsFramework/Prefab/PrefabFocusInterface.h>
#include <QCheckBox>
#include <QFileDialog>
#include <QFileSystemModel>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTextEdit>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>
#include <QWizard>
#include <QWizardPage>
#endif

namespace ROS2
{
    class RobotImporterWidget;
    class URDFPrefabMaker;

    //! Handles UI for the process of URDF importing
    class RobotImporterWidget : public QWizard
    {
        Q_OBJECT
    public:
        explicit RobotImporterWidget(QWidget* parent = nullptr);
        void CreatePrefab(AZStd::string prefabName);

    private:
        int nextId() const override;
        bool validateCurrentPage() override;
        void OpenUrdf();
        void OnUrdfCreated();
        void onCreateButtonPressed();

        IntroPage* m_introPage;
        FileSelectionPage* m_fileSelectPage;
        CheckUrdfPage* m_checkUrdfPage;
        CheckAssetPage* m_assetPage;
        PrefabMakerPage* m_prefabMakerPage;
        XacroParamsPage* m_xacroParamsPage;
        AZ::IO::Path m_urdfPath;
        urdf::ModelInterfaceSharedPtr m_parsedUrdf;

        //! User's choice to copy meshes during urdf import
        bool m_importAssetWithUrdf{ false };

        /// mapping from urdf path to asset source
        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetsMapping;
        AZStd::unique_ptr<URDFPrefabMaker> m_prefabMaker;
        AZStd::unordered_set<AZStd::string> m_meshNames;

        /// Xacro params
        Utils::xacro::Params m_params;

        void onCurrentIdChanged(int id);
        void FillAssetPage();
        void FillPrefabMakerPage();

        //! Checks if the importedPrefabFilename is the same as focused prefab name.
        //! @param importedPrefabFilename name of imported prefab
        //! @return True if names of prefabs are identical or an erorr occured during validation
        bool CheckCyclicalDependency(AZ::IO::Path importedPrefabFilename);

        //! Report an error to the user.
        //! Populates the log, sets status information in the status label and shows an error popup with the message
        //! @param errorMessage error message to display to the user
        void ReportError(const QString& errorMessage);

        //! Returns if file is xacro.
        //! @param filename path to check
        bool IsFileXacro(const AZ::IO::Path& filename) const;

        //! Returns if file is urdf.
        //! @param filename path to check
        bool IsFileUrdf(const AZ::IO::Path& filename) const;

        //! Returns capitalized extension.
        //! @param filename path to check
        AZStd::string GetCapitalizedExtension(const AZ::IO::Path& filename) const;

        static constexpr QWizard::WizardButton PrefabCreationButtonId{ QWizard::CustomButton1 };
        static constexpr QWizard::WizardOption HavePrefabCreationButton{ QWizard::HaveCustomButton1 };
    };
} // namespace ROS2
