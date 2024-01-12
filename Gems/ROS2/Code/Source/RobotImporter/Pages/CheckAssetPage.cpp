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
        m_assetsPaths.clear();
        m_table->setRowCount(0);
        m_missingCount = 0;
        m_failedCount = 0;
    }

    bool CheckAssetPage::IsEmpty() const
    {
        return m_assetsToColumnIndex.empty();
    }

    int CheckAssetPage::GetRowIndex(const AZStd::string& unresolvedFileName)
    {
        auto it = m_assetsToColumnIndex.find(unresolvedFileName);
        if (it == m_assetsToColumnIndex.end())
        {
            return -1;
        }
        return it->second;
    }

    void CheckAssetPage::OnAssetCopyStatusChanged(
        const Utils::CopyStatus& status, const AZStd::string& unresolvedFileName, const AZStd::string assetPath)
    {
        int rowId = GetRowIndex(unresolvedFileName);
        if (rowId == -1)
        {
            return;
        }

        switch (status)
        {
        case Utils::CopyStatus::Unresolvable:
            m_table->setItem(rowId, Columns::ResolvedMeshPath, createCell(false, tr("Unable to resolve mesh path")));
            m_table->item(rowId, Columns::ResolvedMeshPath)->setIcon(m_failureIcon);
            m_table->setItem(rowId, Columns::ProductAsset, createCell(true, ""));
            m_missingCount++;
            break;
        case Utils::CopyStatus::Failed:
            m_table->setItem(rowId, Columns::ProductAsset, createCell(false, tr("Failed to copy mesh")));
            m_table->item(rowId, Columns::ProductAsset)->setIcon(m_failureIcon);
            m_failedCount++;
            break;
        case Utils::CopyStatus::Copying:
            m_table->setItem(rowId, Columns::ProductAsset, createCell(true, tr("Copying")));
            m_table->item(rowId, Columns::ProductAsset)->setIcon(m_processingIcon);
            break;
        case Utils::CopyStatus::Copied:
            m_table->setItem(rowId, Columns::ProductAsset, createCell(true, tr("Copied, waiting to be processed")));
            m_table->item(rowId, Columns::ProductAsset)->setIcon(m_processingIcon);
            m_table->setItem(rowId, Columns::SourceAsset, createCell(true, assetPath.c_str()));
            break;
        case Utils::CopyStatus::Exists:
            m_table->setItem(rowId, Columns::ProductAsset, createCell(true, tr("Found file, waiting to be processed")));
            m_table->item(rowId, Columns::ProductAsset)->setIcon(m_processingIcon);

            m_table->setItem(rowId, Columns::SourceAsset, createCell(true, assetPath.c_str()));
            break;
        }
    }

    void CheckAssetPage::OnAssetProcessStatusChanged(
        const AZStd::string& unresolvedFileName, const Utils::UrdfAsset& urdfAsset, bool isError)
    {
        int rowId = GetRowIndex(unresolvedFileName);
        if (rowId == -1)
        {
            return;
        }

        if (!isError)
        {
            const AZStd::vector<AZStd::string> productPaths = Utils::GetProductAssets(urdfAsset.m_availableAssetInfo.m_sourceGuid);
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
        }
    }

    void CheckAssetPage::DoubleClickRow(int row, [[maybe_unused]] int col)
    {
        if (!m_urdfAssetMap)
        {
            return;
        }
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

} // namespace ROS2
