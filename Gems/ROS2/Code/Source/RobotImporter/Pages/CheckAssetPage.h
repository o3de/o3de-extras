/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include "Utils/SourceAssetsStorage.h"
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Math/Crc.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/shared_ptr.h>
#include <AzCore/std/string/string.h>
#include <QLabel>
#include <QString>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTimer>
#include <QVector>
#include <QWizardPage>
#endif

namespace ROS2
{
    class CheckAssetPage : public QWizardPage
    {
        Q_OBJECT
    public:
        explicit CheckAssetPage(QWizard* parent);

        //! Function reports assets that will be copied/processed by asset processor.
        void ReportAsset(const AZStd::string unresolvedFileName, const Utils::UrdfAsset& urdfAsset, const QString& type);
        void ClearAssetsList();
        bool IsEmpty() const;
        bool isComplete() const override;

        void OnAssetCopyStatusChanged(
            const Utils::CopyStatus& status, const AZStd::string& unresolvedFileName, const AZStd::string assetPath);

        void OnAssetProcessStatusChanged(const AZStd::string& unresolvedFileName, const Utils::UrdfAsset& urdfAsset, bool isError);

    private:
        bool m_success;
        QTableWidget* m_table{};
        QTableWidgetItem* createCell(bool isOk, const QString& text);
        QLabel* m_numberOfAssetLabel{};
        unsigned int m_missingCount{ 0 };
        unsigned int m_failedCount{ 0 };
        void SetTitle();
        AZStd::unordered_map<AZStd::string, int> m_assetsToColumnIndex; //!< Map of unresolved asset to column index in the table.
        AZStd::unordered_map<AZ::Uuid, AZStd::string> m_assetsPaths; //! Map of asset UUIDs to asset source paths.
        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetMap;

        void DoubleClickRow(int row, int col);
        QIcon m_failureIcon;
        QIcon m_okIcon;
        QIcon m_processingIcon;

        int GetRowIndex(const AZStd::string& unresolvedFileName);
    };
} // namespace ROS2
