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
#include <AzCore/std/parallel/mutex.h>
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
        void StartWatchAsset(AZStd::shared_ptr<Utils::UrdfAssetMap> urdfAssetMap, AZStd::shared_ptr<AZStd::mutex> urdfAssetMapMutex);

    private:
        bool m_success;
        QTimer* m_refreshTimer{};
        QTableWidget* m_table{};
        QTableWidgetItem* createCell(bool isOk, const QString& text);
        QLabel* m_numberOfAssetLabel{};
        unsigned int m_missingCount{ 0 };
        unsigned int m_failedCount{ 0 };
        void SetTitle();
        AZStd::unordered_map<AZStd::string, int> m_assetsToColumnIndex; //!< Map of unresolved asset to column index in the table.
        AZStd::unordered_map<AZ::Uuid, AZStd::string> m_assetsPaths; //! Map of asset UUIDs to asset source paths.
        AZStd::unordered_set<AZStd::string>
            m_assetsFinished; //!< Set of asset unresolved paths that have been processed by asset processor.
        AZStd::shared_ptr<Utils::UrdfAssetMap> m_urdfAssetMap;
        AZStd::shared_ptr<AZStd::mutex> m_urdfAssetMapMutex;
        void DoubleClickRow(int row, int col);
        void RefreshTimerElapsed();
        QIcon m_failureIcon;
        QIcon m_okIcon;
        QIcon m_processingIcon;
    };
} // namespace ROS2
