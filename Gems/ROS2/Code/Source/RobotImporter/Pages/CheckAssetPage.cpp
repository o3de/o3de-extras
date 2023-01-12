/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "CheckAssetPage.h"
#include <AzCore/Math/MathStringConversions.h>
#include <QHeaderView>
#include <QVBoxLayout>
namespace ROS2
{

    CheckAssetPage::CheckAssetPage(QWizard* parent)
        : QWizardPage(parent)
        , m_success(true)
        , m_missingCount(0)
    {
        m_table = new QTableWidget(parent);
        SetTitle();
        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_table);
        m_table->setColumnCount(4);
        m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
        m_table->setShowGrid(true);
        m_table->setSelectionMode(QAbstractItemView::SingleSelection);
        m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_table->setHorizontalHeaderLabels({ tr("URDF mesh path"), tr("CRC"), tr("Type"), tr("Asset source") });
        m_table->horizontalHeader()->setStretchLastSection(true);
        this->setLayout(layout);
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
        const QString& urdfPath, const QString& type, const QString& assetSourcePath, const AZ::Crc32& crc32, const QString& tooltip)
    {
        int i = m_table->rowCount();
        m_table->setRowCount(i + 1);

        bool isOk = !assetSourcePath.isEmpty();
        if (!isOk)
        {
            m_missingCount++;
        }
        SetTitle();
        AZStd::string crcStr = AZStd::to_string(crc32);
        QTableWidgetItem* p = createCell(isOk, urdfPath);
        p->setToolTip(tr("Resolved to : ") + tooltip);
        m_table->setItem(i, 0, p);
        m_table->setItem(i, 1, createCell(isOk, QString::fromUtf8(crcStr.data(), crcStr.size())));
        m_table->setItem(i, 2, createCell(isOk, type));
        m_table->setItem(i, 3, createCell(isOk, assetSourcePath));
    }

    QTableWidgetItem* CheckAssetPage::createCell(bool isOk, const QString& text)
    {
        QTableWidgetItem* p = new QTableWidgetItem(text);
        if (!isOk)
        {
            p->setBackground(Qt::red);
        }
        p->setFlags(Qt::NoItemFlags);
        return p;
    }

    void CheckAssetPage::ClearAssetsList()
    {
        m_table->setRowCount(0);
        m_missingCount = 0;
    }

} // namespace ROS2
