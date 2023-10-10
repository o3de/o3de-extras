/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/IO/FileIO.h>
#include <AzCore/IO/Path/Path.h>
#include <AzCore/Math/Uuid.h>
#include <AzCore/Utils/Utils.h>

#include "FixURDF/URDFModifications.h"
#include "RobotImporterWidget.h"
#include <QApplication>
#include <QScreen>
#include <QTranslator>
#include <SdfAssetBuilder/SdfAssetBuilderSettings.h>
#include <URDF/URDFPrefabMaker.h>
#include <URDF/UrdfParser.h>
#include <Utils/ErrorUtils.h>
#include <Utils/FilePath.h>
#include <Utils/RobotImporterUtils.h>

namespace ROS2
{
    RobotImporterWidget::RobotImporterWidget(QWidget* parent)
        : QWizard(parent)
    {
        m_introPage = new IntroPage(this);
        m_fileSelectPage = new FileSelectionPage(this);
        m_checkUrdfPage = new CheckUrdfPage(this);
        m_assetPage = new CheckAssetPage(this);
        m_prefabMakerPage = new PrefabMakerPage(this);
        m_xacroParamsPage = new XacroParamsPage(this);

        addPage(m_introPage);
        addPage(m_fileSelectPage);
        addPage(m_xacroParamsPage);
        addPage(m_checkUrdfPage);
        addPage(m_assetPage);
        addPage(m_prefabMakerPage);

        connect(this, &QWizard::currentIdChanged, this, &RobotImporterWidget::onCurrentIdChanged);
        connect(m_prefabMakerPage, &QWizardPage::completeChanged, this, &RobotImporterWidget::OnUrdfCreated);
        connect(m_prefabMakerPage, &PrefabMakerPage::onCreateButtonPressed, this, &RobotImporterWidget::onCreateButtonPressed);
        connect(
            this,
            &QWizard::customButtonClicked,
            this,
            [this](int id)
            {
                if (id == PrefabCreationButtonId)
                {
                    this->onCreateButtonPressed();
                }
            });

        void onCreateButtonPressed();

        setWindowTitle(tr("Robot Import Wizard"));
        connect(
            this,
            &QDialog::finished,
            [this](int id)
            {
                AZ_Printf("page", "QDialog::finished : %d", id);
                parentWidget()->close();
            });
    }

    void RobotImporterWidget::OnUrdfCreated()
    {
        // hide cancel and back buttons when last page succeed
        if (currentPage() == m_prefabMakerPage && m_prefabMakerPage->isComplete())
        {
            QWizard::button(QWizard::CancelButton)->hide();
            QWizard::button(QWizard::BackButton)->hide();
            QWizard::button(PrefabCreationButtonId)->hide();
        }
    }

    void RobotImporterWidget::AddModificationWarningsToReportString(QString& report, const UrdfParser::RootObjectOutcome& parsedSdfOutcome)
    {
        // This is a URDF only path, and therefore the report text does not mention SDF
        report += "# " + tr("The URDF was parsed, though results were modified to be compatible with SDFormat") + "\n";

        if (!parsedSdfOutcome.m_urdfModifications.missingInertias.empty())
        {
            report += "## " + tr("Inertial information in the following links is missing, reset to default: ") + "\n";
            for (const auto& modifiedTag : parsedSdfOutcome.m_urdfModifications.missingInertias)
            {
                report += " - " + QString::fromUtf8(modifiedTag.linkName.data(), static_cast<int>(modifiedTag.linkName.size())) + "\n";
            }
            report += "\n";
        }

        if (!parsedSdfOutcome.m_urdfModifications.incompleteInertias.empty())
        {
            report +=
                "## " + tr("Inertial information in the following links is incomplete, set default values for listed subtags: ") + "\n";
            for (const auto& modifiedTag : parsedSdfOutcome.m_urdfModifications.incompleteInertias)
            {
                report += " - " + QString::fromUtf8(modifiedTag.linkName.data(), static_cast<int>(modifiedTag.linkName.size())) + ": ";

                for (const auto& tag : modifiedTag.missingTags)
                {
                    report += QString::fromUtf8(tag.data(), static_cast<int>(tag.size())) + ", ";
                }

                report += "\n";
            }
            report += "\n";
        }

        if (!parsedSdfOutcome.m_urdfModifications.duplicatedJoints.empty())
        {
            report += "## " + tr("The following joints were renamed to avoid duplication") + "\n";
            for (const auto& modifiedTag : parsedSdfOutcome.m_urdfModifications.duplicatedJoints)
            {
                report += " - " + QString::fromUtf8(modifiedTag.oldName.data(), static_cast<int>(modifiedTag.oldName.size())) + " -> " +
                    QString::fromUtf8(modifiedTag.newName.data(), static_cast<int>(modifiedTag.newName.size())) + "\n";
            }
        }

        report += "\n# " + tr("The modified URDF code:") + "\n";
        report += "```\n" + QString::fromStdString(parsedSdfOutcome.m_modifiedURDFContent) + "```\n";
    }

    void RobotImporterWidget::OpenUrdf()
    {
        UrdfParser::RootObjectOutcome parsedSdfOutcome;
        QString report;
        if (!m_urdfPath.empty())
        {
            // Read the SDF Settings from PrefabMakerPage
            const SdfAssetBuilderSettings& sdfBuilderSettings = m_fileSelectPage->GetSdfAssetBuilderSettings();

            // Set the parser config settings for URDF content
            sdf::ParserConfig parserConfig = Utils::SDFormat::CreateSdfParserConfigFromSettings(sdfBuilderSettings, m_urdfPath);

            if (Utils::IsFileXacro(m_urdfPath))
            {
                Utils::xacro::ExecutionOutcome outcome =
                    Utils::xacro::ParseXacro(m_urdfPath.String(), m_params, parserConfig, sdfBuilderSettings);
                // Store off the URDF parsing outcome which will be output later in this function
                parsedSdfOutcome = AZStd::move(outcome.m_urdfHandle);
                if (outcome)
                {
                    report += "# " + tr("XACRO execution succeeded") + "\n";
                    m_assetPage->ClearAssetsList();
                }
                else
                {
                    if (outcome.m_succeed)
                    {
                        report += "# " + tr("XACRO execution succeeded, but URDF parsing failed") + "\n";
                    }
                    else
                    {
                        report += "# " + tr("XACRO parsing failed") + "\n";
                        report += "\n\n## " + tr("Command called") + "\n\n`" + QString::fromUtf8(outcome.m_called.data()) + "`";
                        report += "\n\n" + tr("Process failed");
                        report += "\n\n## " + tr("Error output") + "\n\n";
                        report += "```\n";
                        if (outcome.m_logErrorOutput.size())
                        {
                            report += QString::fromUtf8(outcome.m_logErrorOutput.data(), static_cast<int>(outcome.m_logErrorOutput.size()));
                        }
                        else
                        {
                            report += tr("(EMPTY)");
                        }
                        report += "\n```";
                        report += "\n\n## " + tr("Standard output") + "\n\n";
                        report += "```\n";
                        if (outcome.m_logStandardOutput.size())
                        {
                            report +=
                                QString::fromUtf8(outcome.m_logStandardOutput.data(), static_cast<int>(outcome.m_logStandardOutput.size()));
                        }
                        else
                        {
                            report += tr("(EMPTY)");
                        }
                        report += "\n```";
                        m_checkUrdfPage->ReportURDFResult(report, false);
                        return;
                    }
                }
            }
            else if (Utils::IsFileUrdfOrSdf(m_urdfPath))
            {
                // standard URDF
                parsedSdfOutcome = UrdfParser::ParseFromFile(m_urdfPath, parserConfig, sdfBuilderSettings);
            }
            else
            {
                AZ_Assert(false, "Unknown file extension : %s \n", m_urdfPath.c_str());
            }

            AZStd::string log;
            const bool urdfParsedSuccess{ parsedSdfOutcome };
            const bool urdfParsedWithWarnings{ parsedSdfOutcome.UrdfParsedWithModifiedContent() };
            if (urdfParsedSuccess)
            {
                if (urdfParsedWithWarnings)
                {
                    AddModificationWarningsToReportString(report, parsedSdfOutcome);
                }
                else
                {
                    report += "# " + tr("The URDF/SDF was parsed and opened successfully") + "\n";
                    AZ_Printf("Wizard", "Wizard skips m_checkUrdfPage since there is no errors in URDF\n");
                }
                m_parsedSdf = AZStd::move(parsedSdfOutcome.GetRoot());
                m_prefabMaker.reset();
                // Report the status of skipping this page
                m_assetNames = Utils::GetReferencedAssetFilenames(m_parsedSdf);
                m_assetPage->ClearAssetsList();
            }
            else
            {
                log = Utils::JoinSdfErrorsToString(parsedSdfOutcome.GetSdfErrors());
                report += "# " + tr("The URDF/SDF was not opened") + "\n";
                report += tr("URDF/SDF parser returned following errors:") + "\n\n";
            }
            if (!log.empty())
            {
                report += "`";
                report += QString::fromUtf8(log.data(), int(log.size()));
                report += "`";
            }
            m_checkUrdfPage->ReportURDFResult(report, urdfParsedSuccess, urdfParsedWithWarnings);
            const auto& messages = parsedSdfOutcome.GetParseMessages();
            if (!messages.empty())
            {
                report += "\n\n";
                report += tr("URDF/SDF parser returned following messages:") + "\n\n";
                report += "```bash\n";
                report += QString::fromUtf8(messages.c_str(), int(messages.size()));
                report += "\n```\n";
                AZ_Printf("RobotImporterWidget", "SDF Stream: %s\n", messages.c_str());
            }
            m_checkUrdfPage->ReportURDFResult(report, urdfParsedSuccess);
        }
    }

    void RobotImporterWidget::onCurrentIdChanged(int id)
    {
        AZ_Printf("Wizard", "Wizard at page %d", id);

        if (currentPage() == m_assetPage)
        {
            FillAssetPage();
        }
        else if (currentPage() == m_prefabMakerPage)
        {
            FillPrefabMakerPage();
        }
    }

    void RobotImporterWidget::FillAssetPage()
    {
        if (m_assetPage->IsEmpty())
        {
            AZ::Uuid::FixedString dirSuffix;
            if (!m_params.empty())
            {
                auto paramsUuid = AZ::Uuid::CreateNull();
                for (auto& [key, value] : m_params)
                {
                    paramsUuid += AZ::Uuid::CreateName(key);
                    paramsUuid += AZ::Uuid::CreateName(value);
                }

                dirSuffix = paramsUuid.ToFixedString();
            }

            // Read the SDF Settings from PrefabMakerPage
            const SdfAssetBuilderSettings& sdfBuilderSettings = m_fileSelectPage->GetSdfAssetBuilderSettings();

            if (m_importAssetWithUrdf)
            {
                m_urdfAssetsMapping = AZStd::make_shared<Utils::UrdfAssetMap>(Utils::CopyReferencedAssetsAndCreateAssetMap(
                    m_assetNames, m_urdfPath.String(), sdfBuilderSettings, dirSuffix));
            }
            else
            {
                m_urdfAssetsMapping =
                    AZStd::make_shared<Utils::UrdfAssetMap>(Utils::FindReferencedAssets(m_assetNames, m_urdfPath.String(), sdfBuilderSettings));
                for (const auto& [assetPath, assetReferenceType] : m_assetNames)
                {
                    if (m_urdfAssetsMapping->contains(assetPath))
                    {
                        const auto& asset = m_urdfAssetsMapping->at(assetPath);
                        bool visual = (assetReferenceType & Utils::ReferencedAssetType::VisualMesh) == Utils::ReferencedAssetType::VisualMesh;
                        bool collider = (assetReferenceType & Utils::ReferencedAssetType::ColliderMesh) == Utils::ReferencedAssetType::ColliderMesh;
                        if (visual || collider)
                        {
                            Utils::CreateSceneManifest(asset.m_availableAssetInfo.m_sourceAssetGlobalPath, collider, visual);
                        }
                    }
                }
            };

            for (const auto& [assetPath, assetReferenceType] : m_assetNames)
            {
                AZ::Uuid sourceAssetUuid = AZ::Uuid::CreateNull();
                QString type = tr("Unknown");
                AZStd::optional<AZ::Crc32> crc;
                AZStd::optional<AZStd::string> sourcePath;
                AZStd::optional<AZStd::string> resolvedPath;

                if (m_urdfAssetsMapping->contains(assetPath))
                {
                    bool visual = (assetReferenceType & Utils::ReferencedAssetType::VisualMesh) == Utils::ReferencedAssetType::VisualMesh;
                    bool collider = (assetReferenceType & Utils::ReferencedAssetType::ColliderMesh) == Utils::ReferencedAssetType::ColliderMesh;
                    bool texture = (assetReferenceType & Utils::ReferencedAssetType::Texture) == Utils::ReferencedAssetType::Texture;
                    if (visual && collider)
                    {
                        type = tr("Visual and Collider Mesh");
                    }
                    else if (visual)
                    {
                        type = tr("Visual Mesh");
                    }
                    else if (collider)
                    {
                        type = tr("Collider Mesh");
                    }
                    else if (texture)
                    {
                        type = tr("Texture");
                    }

                    const auto& asset = m_urdfAssetsMapping->at(assetPath);
                    sourceAssetUuid = asset.m_availableAssetInfo.m_sourceGuid;
                    sourcePath = asset.m_availableAssetInfo.m_sourceAssetRelativePath.String();
                    resolvedPath = asset.m_resolvedUrdfPath.String();
                    crc = asset.m_urdfFileCRC;
                }
                m_assetPage->ReportAsset(sourceAssetUuid, assetPath, type, sourcePath, crc, resolvedPath);
            }
            m_assetPage->StartWatchAsset();
        }
    }

    void RobotImporterWidget::FillPrefabMakerPage()
    {
        // Use the URDF/SDF file name stem the prefab name
        AZStd::string robotName = AZStd::string::format("%.*s.prefab", AZ_PATH_ARG(m_urdfPath.Stem()));
        m_prefabMakerPage->setProposedPrefabName(robotName);
        QWizard::button(PrefabCreationButtonId)->setText(tr("Create Prefab"));
        QWizard::setOption(HavePrefabCreationButton, true);
    }

    bool RobotImporterWidget::validateCurrentPage()
    {
        // If SDF file are desired to be brought in via the RobotImporter workflow
        // an OpenSdf function would need to be added
        if (currentPage() == m_fileSelectPage)
        {
            m_params.clear();
            m_urdfPath = AZStd::string(m_fileSelectPage->getFileName().toUtf8().constData());
            if (Utils::IsFileXacro(m_urdfPath))
            {
                m_params = Utils::xacro::GetParameterFromXacroFile(m_urdfPath.String());
                AZ_Printf("RobotImporterWidget", "Xacro has %d arguments\n", m_params.size());
                m_xacroParamsPage->SetXacroParameters(m_params);
            }
            // no need to wait for param page - parse urdf now, nextId will skip unnecessary pages
            if (const bool isFileXacroUrdfOrSdf = Utils::IsFileXacroOrUrdfOrSdf(m_urdfPath); m_params.empty() && isFileXacroUrdfOrSdf)
            {
                OpenUrdf();
            }
            m_importAssetWithUrdf = m_fileSelectPage->GetSdfAssetBuilderSettings().m_importReferencedMeshFiles;
        }
        if (currentPage() == m_xacroParamsPage)
        {
            m_params = m_xacroParamsPage->GetXacroParameters();
            if (const bool isFileXacroUrdfOrSdf = Utils::IsFileXacroOrUrdfOrSdf(m_urdfPath); isFileXacroUrdfOrSdf)
            {
                OpenUrdf();
            }
        }
        if (currentPage() == m_introPage)
        {
            AZ::EntityId levelEntityId;
            AzToolsFramework::ToolsApplicationRequestBus::BroadcastResult(
                levelEntityId, &AzToolsFramework::ToolsApplicationRequests::GetCurrentLevelEntityId);

            AZ::Entity* levelEntity{ nullptr };
            AZ::ComponentApplicationBus::BroadcastResult(levelEntity, &AZ::ComponentApplicationRequests::FindEntity, levelEntityId);

            if (!levelEntityId.IsValid() || levelEntity == nullptr)
            {
                QMessageBox noLevelLoadedMessage;
                noLevelLoadedMessage.critical(0, "No level opened", "A level must be opened before using the Robot Importer");
                noLevelLoadedMessage.setFixedSize(500, 200);

                return false;
            }
        }
        return currentPage()->validatePage();
    }

    int RobotImporterWidget::nextId() const
    {
        if ((currentPage() == m_fileSelectPage && m_params.empty()) || currentPage() == m_xacroParamsPage)
        {
            if (!m_checkUrdfPage->isWarning())
            {
                return m_xacroParamsPage->nextId();
            }
            if (m_checkUrdfPage->isComplete())
            {
                if (m_assetNames.empty())
                {
                    // skip two pages when urdf/sdf is parsed without problems, and it has no assets
                    return m_assetPage->nextId();
                }
                else
                {
                    // skip one page when urdf/sdf is parsed without problems
                    return m_checkUrdfPage->nextId();
                }
            }
            if (m_params.empty())
            {
                return m_xacroParamsPage->nextId();
            }
        }
        return currentPage()->nextId();
    }

    void RobotImporterWidget::CreatePrefab(AZStd::string prefabName)
    {
        const AZ::IO::Path prefabPathRelative(AZ::IO::Path("Assets") / "Importer" / prefabName);
        const AZ::IO::Path prefabPath(AZ::IO::Path(AZ::Utils::GetProjectPath()) / prefabPathRelative);
        bool fileExists = AZ::IO::FileIOBase::GetInstance()->Exists(prefabPath.c_str());

        if (CheckCyclicalDependency(prefabPathRelative))
        {
            m_prefabMakerPage->setSuccess(false);
            return;
        }
        if (fileExists)
        {
            QMessageBox msgBox;
            msgBox.setText(tr("Prefab with this name already exists"));
            msgBox.setInformativeText(tr("Do you want to overwrite existing prefab?"));
            msgBox.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
            msgBox.setDefaultButton(QMessageBox::Cancel);
            int ret = msgBox.exec();
            if (ret == QMessageBox::Cancel)
            {
                m_prefabMakerPage->setSuccess(false);
                return;
            }
        }

        const auto& sdfAssetBuilderSettings = m_fileSelectPage->GetSdfAssetBuilderSettings();
        const bool useArticulation = sdfAssetBuilderSettings.m_useArticulations;
        m_prefabMaker = AZStd::make_unique<URDFPrefabMaker>(
            m_urdfPath.String(),
            &m_parsedSdf,
            prefabPath.String(),
            m_urdfAssetsMapping,
            useArticulation,
            m_prefabMakerPage->getSelectedSpawnPoint());

        auto prefabOutcome = m_prefabMaker->CreatePrefabFromUrdfOrSdf();
        if (prefabOutcome.IsSuccess())
        {
            AZStd::string status = m_prefabMaker->GetStatus();
            m_prefabMakerPage->reportProgress(status);
            m_prefabMakerPage->setSuccess(true);
        }
        else
        {
            AZStd::string status = "Failed to create prefab\n";
            status += prefabOutcome.GetError() + "\n";
            status += m_prefabMaker->GetStatus();
            m_prefabMakerPage->reportProgress(status);
            m_prefabMakerPage->setSuccess(false);
        }
    }

    void RobotImporterWidget::onCreateButtonPressed()
    {
        CreatePrefab(m_prefabMakerPage->getPrefabName());
    }

    bool RobotImporterWidget::CheckCyclicalDependency(AZ::IO::Path importedPrefabPath)
    {
        AzFramework::EntityContextId contextId;
        AzFramework::EntityIdContextQueryBus::BroadcastResult(contextId, &AzFramework::EntityIdContextQueryBus::Events::GetOwningContextId);

        AZ_Printf("CheckCyclicalDependency", "CheckCyclicalDependency %s\n", importedPrefabPath.Native().c_str());
        auto focusInterface = AZ::Interface<AzToolsFramework::Prefab::PrefabFocusInterface>::Get();

        if (!focusInterface)
        {
            ReportError(tr("Imported prefab could not be validated.\nImport aborted."));
            return true;
        }

        auto focusedPrefabInstance = focusInterface->GetFocusedPrefabInstance(contextId);

        if (!focusedPrefabInstance)
        {
            ReportError(tr("Imported prefab could not be validated.\nImport aborted."));
            return true;
        }

        auto focusPrefabFilename = focusedPrefabInstance.value().get().GetTemplateSourcePath();

        if (focusPrefabFilename == importedPrefabPath)
        {
            ReportError(tr(
                "Cyclical dependency detected.\nSelected URDF/SDF model is currently being edited. Exit prefab edit mode and try again."));
            return true;
        }

        return false;
    }

    void RobotImporterWidget::ReportError(const QString& errorMessage)
    {
        QMessageBox::critical(this, QObject::tr("Error"), errorMessage);
        AZ_Error("RobotImporterWidget", false, "%s", errorMessage.toUtf8().constData());
    }

} // namespace ROS2
