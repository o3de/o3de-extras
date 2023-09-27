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
        QTableWidgetItem* headerItem = new QTableWidgetItem(tr("URDF/SDF mesh path"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(Columns::SdfMeshPath, headerItem);
        headerItem = new QTableWidgetItem(tr("Resolved mesh from URDF/SDF"));
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
            setTitle(tr("Resolved meshes"));
        }
        else
        {
            setTitle(tr("There are ") + QString::number(m_missingCount) + tr(" unresolved meshes"));
        }
    }

    bool CheckAssetPage::isComplete() const
    {
        return m_success;
    };

    void CheckAssetPage::ReportAsset(
        const AZ::Uuid assetUuid,
        const AZStd::string sdfPath,
        const QString& type,
        const AZStd::optional<AZStd::string>& assetSourcePath,
        const AZStd::optional<AZ::Crc32>& crc32,
        const AZStd::optional<AZStd::string>& resolvedSdfPath)
    {
        int i = m_table->rowCount();
        m_table->setRowCount(i + 1);

        bool isOk = assetSourcePath.has_value() && resolvedSdfPath.has_value();
        if (!isOk)
        {
            m_missingCount++;
        }
        SetTitle();
        AZStd::string crcStr;
        if (crc32)
        {
            crcStr = AZStd::to_string(*crc32);
        }
        QTableWidgetItem* p = createCell(isOk, QString::fromUtf8(sdfPath.data(), sdfPath.size()));
        if (crc32 != AZ::Crc32())
        {
            p->setToolTip(tr("CRC for file : ") + QString::fromUtf8(crcStr.data(), crcStr.size()));
        }
        m_table->setItem(i, Columns::SdfMeshPath, p);

        if (resolvedSdfPath)
        {
            m_table->setItem(
                i, Columns::ResolvedMeshPath, createCell(true, QString::fromUtf8(resolvedSdfPath->data(), resolvedSdfPath->size())));
        }
        else
        {
            m_table->setItem(i, Columns::ResolvedMeshPath, createCell(false, tr("Not found")));
        }

        m_table->setItem(i, Columns::Type, createCell(isOk, type));

        if (assetSourcePath && !assetSourcePath->empty())
        {
            m_table->setItem(
                i, Columns::SourceAsset, createCell(true, QString::fromUtf8(assetSourcePath->data(), assetSourcePath->size())));
        }
        else
        {
            m_table->setItem(i, Columns::SourceAsset, createCell(false, tr("Not found")));
        }

        if (isOk)
        {
            m_table->item(i, Columns::ResolvedMeshPath)->setIcon(m_okIcon);
        }
        else
        {
            m_table->item(i, Columns::ResolvedMeshPath)->setIcon(m_failureIcon);
            m_table->setItem(i, Columns::ProductAsset, createCell(false, QString()));
        }
        m_assetsPaths.push_back(assetSourcePath ? *assetSourcePath : AZStd::string());
        m_assetsUuids.push_back(assetUuid);
    }

    void CheckAssetPage::StartWatchAsset()
    {
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
        m_assetsUuids.clear();
        m_assetsUuidsFinished.clear();
        m_assetsPaths.clear();
        m_table->setRowCount(0);
        m_missingCount = 0;
        m_failedCount = 0;
        m_refreshTimer->stop();
    }

    bool CheckAssetPage::IsEmpty() const
    {
        return m_assetsUuids.empty();
    }

    void CheckAssetPage::DoubleClickRow(int row, [[maybe_unused]] int col)
    {
        AZ_Printf("CheckAssetPage", "Clicked on row", row);
        if (row < m_assetsPaths.size())
        {
            AzFramework::AssetSystemRequestBus::Broadcast(
                &AzFramework::AssetSystem::AssetSystemRequests::ShowInAssetProcessor, m_assetsPaths[row]);
        }
    }

    void CheckAssetPage::RefreshTimerElapsed()
    {
        for (int i = 0; i < m_assetsUuids.size(); i++)
        {
            const AZ::Uuid& assetUuid = m_assetsUuids[i];
            const AZStd::string& sourceAssetFullPath = m_assetsPaths[i];
            if (!m_assetsUuidsFinished.contains(assetUuid))
            {
                using namespace AzToolsFramework;
                using namespace AzToolsFramework::AssetSystem;

                AZ::Outcome<AssetSystem::JobInfoContainer> result = AZ::Failure();
                AssetSystemJobRequestBus::BroadcastResult(
                    result, &AssetSystemJobRequestBus::Events::GetAssetJobsInfo, sourceAssetFullPath, true);
                if (result)
                {
                    bool allFinished = true;
                    bool failed = false;
                    JobInfoContainer& allJobs = result.GetValue();
                    for (const JobInfo& job : allJobs)
                    {
                        if (job.m_status == JobStatus::Queued || job.m_status == JobStatus::InProgress)
                        {
                            allFinished = false;
                        }
                        if (job.m_status == JobStatus::Failed)
                        {
                            failed = true;
                            m_failedCount++;
                        }
                    }
                    if (allFinished)
                    {
                        if (!failed)
                        {
                            const AZStd::string productRelPathVisual = Utils::GetModelProductAsset(assetUuid);
                            const AZStd::string productRelPathCollider = Utils::GetPhysXMeshProductAsset(assetUuid);
                            QString text = QString::fromUtf8(productRelPathVisual.data(), productRelPathVisual.size()) + " " +
                                QString::fromUtf8(productRelPathCollider.data(), productRelPathCollider.size());
                            m_table->setItem(i, Columns::ProductAsset, createCell(true, text));
                            m_table->item(i, Columns::ProductAsset)->setIcon(m_okIcon);
                        }
                        else
                        {
                            m_table->setItem(i, Columns::ProductAsset, createCell(false, tr("Failed")));
                            m_table->item(i, Columns::ProductAsset)->setIcon(m_failureIcon);
                        }
                        m_assetsUuidsFinished.insert(assetUuid);
                    }
                }
            }
        }
        if (m_assetsUuidsFinished.size() == m_assetsUuids.size())
        {
            m_refreshTimer->stop();
            if (m_failedCount == 0 && m_missingCount == 0)
            {
                setTitle(tr("All meshes were processed"));
            }
            else
            {
                setTitle(
                    tr("There are ") + QString::number(m_missingCount) + tr(" unresolved meshes.") + tr("There are ") +
                    QString::number(m_failedCount) + tr(" failed asset processor jobs."));
            }
        }
    }
} // namespace ROS2
