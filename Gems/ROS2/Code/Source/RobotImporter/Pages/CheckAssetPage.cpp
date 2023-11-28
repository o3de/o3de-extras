/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CheckAssetPage.h"
#include "RobotImporter/Utils/SourceAssetsStorage.h"
#include <AzCore/Math/MathStringConversions.h>
#include <AzCore/std/parallel/lock.h>
#include <AzFramework/Asset/AssetSystemBus.h>
#include <QHeaderView>
#include <QPushButton>
#include <QVBoxLayout>

namespace ROS2
{
    namespace Columns
    {
        constexpr int SdfMeshPath{ 0 };
        constexpr int ResolvedMeshPath{ 1 };
        constexpr int SourceAsset{ 3 };
        constexpr int ProductAsset{ 2 };
        constexpr int Type{ 4 };
    } // namespace Columns

    CheckAssetPage::CheckAssetPage(QWizard* parent)
        : QWizardPage(parent)
        , m_success(true)
        , m_missingCount(0)
        , m_failureIcon(QStringLiteral(":/stylesheet/img/logging/failure.svg"))
        , m_okIcon(QStringLiteral(":/stylesheet/img/logging/valid.svg"))
        , m_processingIcon(QStringLiteral(":/stylesheet/img/logging/processing.svg"))
    {
        m_table = new QTableWidget(parent);
        SetTitle();
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_table);
        m_table->setEnabled(true);
        m_table->setAlternatingRowColors(true);
        m_table->setMinimumHeight(800);
        m_table->setMinimumWidth(1250);
        m_table->horizontalHeader()->setStretchLastSection(true);
        m_table->setCornerButtonEnabled(false);
        m_table->setSortingEnabled(false);
        m_table->setColumnCount(5);
        m_table->setShowGrid(true);
        m_table->setMouseTracking(true);
        m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_table->setSelectionMode(QAbstractItemView::SingleSelection);
        // Set the header items.
        QTableWidgetItem* headerItem = new QTableWidgetItem(tr("URDF/SDF asset path"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(Columns::SdfMeshPath, headerItem);
        headerItem = new QTableWidgetItem(tr("Resolved asset from URDF/SDF"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(Columns::ResolvedMeshPath, headerItem);
        headerItem = new QTableWidgetItem(tr("Type"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(Columns::Type, headerItem);
        headerItem = new QTableWidgetItem(tr("Source asset"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(Columns::SourceAsset, headerItem);
        headerItem = new QTableWidgetItem(tr("Product asset"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(Columns::ProductAsset, headerItem);
        m_table->horizontalHeader()->resizeSection(Columns::SdfMeshPath, 200);
        m_table->horizontalHeader()->resizeSection(Columns::ResolvedMeshPath, 350);
        m_table->horizontalHeader()->resizeSection(Columns::Type, 50);
        m_table->horizontalHeader()->resizeSection(Columns::SourceAsset, 400);
        m_table->horizontalHeader()->resizeSection(Columns::ProductAsset, 400);
        m_table->verticalHeader()->hide();
        connect(m_table, &QTableWidget::cellDoubleClicked, this, &CheckAssetPage::DoubleClickRow);
        this->setLayout(layout);
        m_refreshTimer = new QTimer(this);
        m_refreshTimer->setInterval(250);
        m_refreshTimer->setSingleShot(false);
        connect(m_refreshTimer, &QTimer::timeout, this, &CheckAssetPage::RefreshTimerElapsed);
    }

    void CheckAssetPage::SetTitle()
    {
        if (m_missingCount == 0)
        {
            setTitle(tr("Resolved assets"));
        }
        else
        {
            setTitle(tr("There are ") + QString::number(m_missingCount) + tr(" unresolved assets"));
        }
    }

    bool CheckAssetPage::isComplete() const
    {
        return m_success;
    };

    void CheckAssetPage::ReportAsset(const AZStd::string unresolvedFileName, const Utils::UrdfAsset& urdfAsset, const QString& type)
    {
        int rowId = m_table->rowCount();
        m_table->setRowCount(rowId + 1);

        SetTitle();
        const AZStd::string crcStr = AZStd::to_string(urdfAsset.m_urdfFileCRC);

        QTableWidgetItem* p =
            createCell(true, QString::fromUtf8(urdfAsset.m_urdfPath.String().data(), urdfAsset.m_urdfPath.String().size()));
        if (urdfAsset.m_urdfFileCRC != AZ::Crc32())
        {
            p->setToolTip(tr("CRC for file : ") + QString::fromUtf8(crcStr.data(), crcStr.size()));
        }
        m_table->setItem(rowId, Columns::SdfMeshPath, p);

        if (!urdfAsset.m_resolvedUrdfPath.empty())
        {
            m_table->setItem(
                rowId,
                Columns::ResolvedMeshPath,
                createCell(
                    true, QString::fromUtf8(urdfAsset.m_resolvedUrdfPath.String().data(), urdfAsset.m_resolvedUrdfPath.String().size())));
            m_table->item(rowId, Columns::ResolvedMeshPath)->setIcon(m_okIcon);
        }
        else
        {
            m_table->setItem(rowId, Columns::ResolvedMeshPath, createCell(false, tr("Not found")));
            m_table->item(rowId, Columns::ResolvedMeshPath)->setIcon(m_failureIcon);
        }

        m_table->setItem(rowId, Columns::Type, createCell(true, type));

        m_assetsToColumnIndex[unresolvedFileName] = rowId;
    }

    void CheckAssetPage::StartWatchAsset(
        AZStd::shared_ptr<Utils::UrdfAssetMap> urdfAssetMap, AZStd::shared_ptr<AZStd::mutex> urdfAssetMapMutex)
    {
        m_urdfAssetMap = urdfAssetMap;
        m_urdfAssetMapMutex = urdfAssetMapMutex;
        m_refreshTimer->start();
    }

    QTableWidgetItem* CheckAssetPage::createCell(bool isOk, const QString& text)
    {
        QTableWidgetItem* p = new QTableWidgetItem(text);
        if (!isOk)
        {
            p->setBackground(Qt::darkRed);
        }
        p->setToolTip(text);
        p->setFlags(Qt::ItemIsEnabled | Qt::ItemIsSelectable);
        return p;
    }

    void CheckAssetPage::ClearAssetsList()
    {
        m_assetsToColumnIndex.clear();
        m_assetsFinished.clear();
        m_assetsPaths.clear();
        m_table->setRowCount(0);
        m_missingCount = 0;
        m_failedCount = 0;
        m_refreshTimer->stop();
    }

    bool CheckAssetPage::IsEmpty() const
    {
        return m_assetsToColumnIndex.empty();
    }

    void CheckAssetPage::DoubleClickRow(int row, [[maybe_unused]] int col)
    {
        if (!m_urdfAssetMap)
        {
            return;
        }
        AZStd::lock_guard<AZStd::mutex> lock(*m_urdfAssetMapMutex);
        for (const auto& [assetPath, columnId] : m_assetsToColumnIndex)
        {
            if (columnId == row && (*m_urdfAssetMap).contains(assetPath))
            {
                auto productAssetRelativePath = (*m_urdfAssetMap)[assetPath].m_availableAssetInfo.m_productAssetRelativePath;
                if (productAssetRelativePath.empty())
                {
                    return;
                }
                AzFramework::AssetSystemRequestBus::Broadcast(
                    &AzFramework::AssetSystem::AssetSystemRequests::ShowInAssetProcessor, productAssetRelativePath.String());
            }
        }
    }

    void CheckAssetPage::RefreshTimerElapsed()
    {
        for (const auto& [unresolvedAssetPath, rowId] : m_assetsToColumnIndex)
        {
            Utils::UrdfAsset urdfAsset;
            {
                AZStd::lock_guard<AZStd::mutex> lock(*m_urdfAssetMapMutex);
                auto urdfAssetIt = m_urdfAssetMap->find(unresolvedAssetPath);
                if (urdfAssetIt == m_urdfAssetMap->end())
                {
                    continue;
                }
                urdfAsset = urdfAssetIt->second;
            }
            auto copyStatus = urdfAsset.m_copyStatus;
            if (!m_assetsFinished.contains(unresolvedAssetPath))
            {
                if (copyStatus == Utils::CopyStatus::Unresolvable)
                {
                    m_table->setItem(rowId, Columns::ResolvedMeshPath, createCell(false, tr("Unable to resolve mesh path")));
                    m_table->item(rowId, Columns::ResolvedMeshPath)->setIcon(m_failureIcon);
                    m_assetsFinished.insert(unresolvedAssetPath);
                    m_failedCount++;
                }
                else if (copyStatus == Utils::CopyStatus::Failed)
                {
                    m_table->setItem(rowId, Columns::ProductAsset, createCell(false, tr("Failed to copy mesh")));
                    m_table->item(rowId, Columns::ProductAsset)->setIcon(m_failureIcon);
                    m_assetsFinished.insert(unresolvedAssetPath);
                    m_failedCount++;
                }
                else if (copyStatus == Utils::CopyStatus::Copying)
                {
                    m_table->setItem(rowId, Columns::ProductAsset, createCell(true, tr("Copying")));
                    m_table->item(rowId, Columns::ProductAsset)->setIcon(m_processingIcon);
                }
                else if (copyStatus == Utils::CopyStatus::Copied || copyStatus == Utils::CopyStatus::Exists)
                {
                    auto copiedText = tr("Copied, waiting to be processed");
                    auto foundText = tr("Found file, waiting to be processed");

                    m_table->setItem(
                        rowId, Columns::ProductAsset, createCell(true, copyStatus == Utils::CopyStatus::Copied ? copiedText : foundText));
                    m_table->item(rowId, Columns::ProductAsset)->setIcon(m_processingIcon);
                    m_table->setItem(
                        rowId, Columns::SourceAsset, createCell(true, urdfAsset.m_availableAssetInfo.m_sourceAssetRelativePath.c_str()));
                }

                if (copyStatus == Utils::CopyStatus::Copied || copyStatus == Utils::CopyStatus::Exists)
                {
                    // Execute for all found source assets that are not finished yet.
                    const AZStd::string& sourceAssetFullPath = urdfAsset.m_availableAssetInfo.m_sourceAssetGlobalPath.c_str();
                    using namespace AzToolsFramework;
                    using namespace AzToolsFramework::AssetSystem;

                    AZ::Outcome<AssetSystem::JobInfoContainer> result = AZ::Failure();
                    AssetSystemJobRequestBus::BroadcastResult(
                        result, &AssetSystemJobRequestBus::Events::GetAssetJobsInfo, sourceAssetFullPath, true);
                    if (result)
                    {
                        bool allFinished = true;
                        bool productAssetFailed = false;
                        JobInfoContainer& allJobs = result.GetValue();
                        for (const JobInfo& job : allJobs)
                        {
                            if (job.m_status == JobStatus::Queued || job.m_status == JobStatus::InProgress)
                            {
                                allFinished = false;
                            }
                            if (job.m_status == JobStatus::Failed)
                            {
                                productAssetFailed = true;
                            }
                        }
                        if (allFinished)
                        {
                            if (!productAssetFailed)
                            {
                                const AZStd::vector<AZStd::string> productPaths =
                                    Utils::GetProductAssets(urdfAsset.m_availableAssetInfo.m_sourceGuid);
                                QString text;
                                for (const auto& productPath : productPaths)
                                {
                                    text += QString::fromUtf8(productPath.data(), productPath.size()) + " ";
                                }
                                m_table->setItem(rowId, Columns::ProductAsset, createCell(true, text));
                                m_table->item(rowId, Columns::ProductAsset)->setIcon(m_okIcon);
                            }
                            else
                            {
                                m_table->setItem(rowId, Columns::ProductAsset, createCell(false, tr("Failed")));
                                m_table->item(rowId, Columns::ProductAsset)->setIcon(m_failureIcon);
                                m_failedCount++;
                            }
                            m_assetsFinished.insert(unresolvedAssetPath);
                        }
                    }
                }
            }
        }

        if (m_assetsFinished.size() == m_assetsToColumnIndex.size())
        {
            m_refreshTimer->stop();
            if (m_failedCount == 0 && m_missingCount == 0)
            {
                setTitle(tr("All assets were processed"));
            }
            else
            {
                setTitle(
                    tr("There are ") + QString::number(m_missingCount) + tr(" unresolved assets.") + tr("There are ") +
                    QString::number(m_failedCount) + tr(" failed asset processor jobs."));
            }
        }
    }
} // namespace ROS2
