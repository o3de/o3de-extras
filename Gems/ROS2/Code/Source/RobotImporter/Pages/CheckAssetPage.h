/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#if !defined(Q_MOC_RUN)
#include <AzCore/Asset/AssetCommon.h>
#include <AzCore/Math/Crc.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/std/containers/vector.h>
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

        //! Function reports assets that are will be processed by asset processor.
        void ReportAsset(
            const AZ::Uuid assetUuid,
            const AZStd::string urdfPath,
            const QString& type,
            const AZStd::string assetSourcePath,
            const AZ::Crc32& crc32,
            const AZStd::string resolvedUrdfPath);
        void ClearAssetsList();
        bool IsEmpty() const;
        bool isComplete() const override;
        void StartWatchAsset();

    private:
        bool m_success;
        QTimer* m_refreshTimer{};
        QTableWidget* m_table{};
        QTableWidgetItem* createCell(bool isOk, const QString& text);
        QLabel* m_numberOfAssetLabel{};
        unsigned int m_missingCount{ 0 };
        unsigned int m_failedCount{ 0 };
        void SetTitle();
        AZStd::vector<AZ::Uuid> m_assetsUuids;
        AZStd::vector<AZStd::string> m_assetsPaths;
        AZStd::unordered_set<AZ::Uuid> m_assetsUuidsFinished;
        void DoubleClickRow(int row, int col);
        void RefreshTimerElapsed();
        QIcon m_failureIcon;
        QIcon m_okIcon;
    };
} // namespace ROS2
