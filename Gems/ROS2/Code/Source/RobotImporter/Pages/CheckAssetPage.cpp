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
        m_table->setMinimumHeight(500);
        m_table->setMinimumWidth(1000);
        m_table->horizontalHeader()->setStretchLastSection(true);
        m_table->setCornerButtonEnabled(false);
        m_table->setSortingEnabled(false);
        m_table->setColumnCount(5);
        m_table->setShowGrid(true);
        m_table->setMouseTracking(true);
        m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_table->setSelectionMode(QAbstractItemView::SingleSelection);
        // Set the header items.
        QTableWidgetItem* headerItem = new QTableWidgetItem(tr("URDF mesh path"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(0, headerItem);
        headerItem = new QTableWidgetItem(tr("Resolved mesh from URDF"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(1, headerItem);
        headerItem = new QTableWidgetItem(tr("Type"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(2, headerItem);
        headerItem = new QTableWidgetItem(tr("Source asset"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(3, headerItem);
        headerItem = new QTableWidgetItem(tr("Product asset"));
        headerItem->setTextAlignment(Qt::AlignVCenter | Qt::AlignLeft);
        m_table->setHorizontalHeaderItem(4, headerItem);
        m_table->horizontalHeader()->resizeSection(0, 200);
        m_table->horizontalHeader()->resizeSection(1, 350);
        m_table->horizontalHeader()->resizeSection(2, 50);
        m_table->horizontalHeader()->resizeSection(3, 400);
        m_table->horizontalHeader()->resizeSection(4, 400);
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
        const AZStd::string urdfPath,
        const QString& type,
        const AZStd::string assetSourcePath,
        const AZ::Crc32& crc32,
        const AZStd::string resolvedUrdfPath)
    {
        int i = m_table->rowCount();
        m_table->setRowCount(i + 1);

        bool isOk = !assetSourcePath.empty();
        if (!isOk)
        {
            m_missingCount++;
        }
        SetTitle();
        AZStd::string crcStr = AZStd::to_string(crc32);
        QTableWidgetItem* p = createCell(isOk, QString::fromUtf8(urdfPath.data(), urdfPath.size()));
        p->setToolTip(tr("CRC for file : ") + QString::fromUtf8(crcStr.data(), crcStr.size()));
        m_table->setItem(i, 0, p);
        m_table->setItem(i, 1, createCell(isOk, QString::fromUtf8(resolvedUrdfPath.data(), resolvedUrdfPath.size())));
        m_table->setItem(i, 2, createCell(isOk, type));
        m_table->setItem(i, 3, createCell(isOk, QString::fromUtf8(assetSourcePath.data(), assetSourcePath.size())));
        if (isOk)
        {
            m_table->item(i, 1)->setIcon(m_okIcon);
        }
        else
        {
            m_table->item(i, 1)->setIcon(m_failureIcon);
        }
        if (!assetSourcePath.empty())
        {
            m_assetsPaths.push_back(assetSourcePath);
            m_assetsUuids.push_back(assetUuid);
        }
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

    void CheckAssetPage::DoubleClickRow(int row, int col)
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
                        m_table->setItem(i, 4, createCell(true, text));
                        m_table->item(i, 4)->setIcon(m_okIcon);
                    }
                    else
                    {
                        m_table->setItem(i, 4, createCell(false, tr("Failed")));
                        m_table->item(i, 4)->setIcon(m_failureIcon);
                    }
                    m_assetsUuidsFinished.insert(assetUuid);
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
