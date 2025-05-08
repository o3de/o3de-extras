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
#include "Pages/FileSelectionPage.h"
#include "Pages/IntroPage.h"
#include "Pages/ModifiedURDFWindow.h"
#include "Pages/PrefabMakerPage.h"
#include "Pages/RobotDescriptionPage.h"
#include "Pages/XacroParamsPage.h"

#include "URDF/URDFPrefabMaker.h"
#include "URDF/UrdfParser.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/parallel/thread.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <RobotImporter/FixURDF/URDFModifications.h>
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
        void onSaveModifiedUrdfPressed();
        void onShowModifiedUrdfPressed();

        IntroPage* m_introPage;
        FileSelectionPage* m_fileSelectPage;
        RobotDescriptionPage* m_robotDescriptionPage;
        CheckAssetPage* m_assetPage;
        PrefabMakerPage* m_prefabMakerPage;
        XacroParamsPage* m_xacroParamsPage;
        ModifiedURDFWindow* m_modifiedUrdfWindow;
        AZ::IO::Path m_urdfPath;
        sdf::Root m_parsedSdf{};

        //! User's choice to copy meshes during urdf import
        bool m_importAssetWithUrdf{ false };

        /// mapping from urdf path to asset source
        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetsMapping;
        AZStd::shared_ptr<AZStd::thread> m_copyReferencedAssetsThread;

        AZStd::unique_ptr<URDFPrefabMaker> m_prefabMaker;
        Utils::AssetFilenameReferences m_assetNames;

        /// Xacro params
        Utils::xacro::Params m_params;

        // Assets ready to be processed by asset processor
        AZStd::set<AZ::IO::Path> m_toProcessAssets;

        void onCurrentIdChanged(int id);
        void FillAssetPage();
        void FillPrefabMakerPage();

        //! Checks if the asset is finished processing by asset processor.
        //! @param assetGlobalPath global path to the asset.
        //! @return True if asset is finished processing, false if it an error occurred, and AZ::Failure if the asset is not finished
        //! processing.
        static AZ::Outcome<bool> CheckIfAssetFinished(const AZStd::string& assetGlobalPath);

        //! Checks all assets that are in the m_toProcessAssets set and emits signals based on results.
        void CheckToProcessAssets();

        //! Checks if the asset is finished processing by asset processor. Timer callback.
        void RefreshTimerElapsed();
        QTimer* m_refreshTimerCheckAssets{};
        //! Variable used to start the timer as the timer start function cannot be called from a different thread.
        AZStd::atomic_bool m_shouldCheckAssets{ false };

        //! Checks if the importedPrefabFilename is the same as focused prefab name.
        //! @param importedPrefabFilename name of imported prefab
        //! @return True if names of prefabs are identical or an error occurred during validation
        bool CheckCyclicalDependency(AZ::IO::Path importedPrefabFilename);

        //! Report an error to the user.
        //! Populates the log, sets status information in the status label and shows an error popup with the message
        //! @param errorMessage error message to display to the user
        void ReportError(const QString& errorMessage);

        void AddModificationWarningsToReportString(QString& report, const UrdfParser::RootObjectOutcome& parsedSdfOutcome);

        static constexpr QWizard::WizardButton PrefabCreationButtonId{ QWizard::CustomButton1 };
        static constexpr QWizard::WizardOption HavePrefabCreationButton{ QWizard::HaveCustomButton1 };
    };
} // namespace ROS2
