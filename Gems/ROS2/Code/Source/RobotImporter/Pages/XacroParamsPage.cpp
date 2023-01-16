/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "XacroParamsPage.h"
#include <RobotImporter/RobotImporterWidget.h>

namespace ROS2
{

    XacroParamsPage::XacroParamsPage(RobotImporterWidget* parent)
        : QWizardPage(parent)
    {
        setTitle(tr("Xacro Parameters"));
        m_table = new QTableWidget(this);

        m_table->setColumnCount(2);
        m_table->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
        m_table->setShowGrid(true);
        m_table->setSelectionMode(QAbstractItemView::SingleSelection);
        m_table->setSelectionBehavior(QAbstractItemView::SelectRows);
        m_table->setHorizontalHeaderLabels({ tr("Parameter"), tr("Value") });
        m_table->horizontalHeader()->setStretchLastSection(true);

        QVBoxLayout* layout = new QVBoxLayout;
        layout->addWidget(m_table);
        setLayout(layout);
    }

    bool XacroParamsPage::isComplete() const
    {
        return true;
    }

    void XacroParamsPage::SetXacroParameters(const Utils::xacro::Params& params)
    {
        m_defaultParams = params;
        m_table->setRowCount(0);
        for (const auto& [name, value] : params)
        {
            const int rowIndex = m_table->rowCount();
            m_table->setRowCount(rowIndex + 1);
            QTableWidgetItem* p1 = new QTableWidgetItem(QString::fromUtf8(name.data()));
            QTableWidgetItem* p2 = new QTableWidgetItem(QString::fromUtf8(value.data()));
            m_table->setItem(rowIndex, 0, p1);
            m_table->setItem(rowIndex, 1, p2);
        }
    }

    Utils::xacro::Params XacroParamsPage::GetXacroParameters() const
    {
        Utils::xacro::Params params;
        const int rowCount = m_table->rowCount();
        for (int i = 0; i < rowCount; i++)
        {
            const auto* p1 = m_table->item(i, 0);
            const auto* p2 = m_table->item(i, 1);
            AZ_Assert(p1, "cell should exist");
            AZ_Assert(p2, "cell should exist");
            const AZStd::string name(p1->text().toUtf8().constData());
            const AZStd::string value(p2->text().toUtf8().constData());
            auto it = m_defaultParams.find(name);
            if (it != m_defaultParams.end())
            {
                if (it->second != value)
                {
                    params[name] = value;
                }
            }
        }
        AZ_Printf("XacroParamsPage", "number of modified parameters %d\n", params.size());
        return params;
    }

} // namespace ROS2
