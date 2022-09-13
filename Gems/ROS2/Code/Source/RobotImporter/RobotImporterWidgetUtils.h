/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/IO/Path/Path.h>
#include <QWidget>
#include <optional>

namespace ROS2::RobotImporterWidgetUtils
{
    enum ExistingPrefabAction
    {
        Overwrite,
        CreateWithNewName,
        Cancel
    };

    //! Get valid path to the existing URDF file from the user
    //! @return valid path to the existing URDF file or empty optional if the user canceled the operation
    //! @param parent - parent widget for the widgets used inside this function
    AZStd::optional<AZStd::string> QueryUserForURDFPath(QWidget* parent = nullptr);

    //! Validate whether a path exists. If yes, ask user to take a proper action to provide correct path.
    //! @param path - path to validate
    //! @param parent - parent widget for the widgets used inside this function
    //! @return Valid path or an empty optional if it was not possible or user cancelled.
    AZStd::optional<AZ::IO::Path> ValidatePrefabPathExistenceAndQueryUserForNewIfNecessary(
        const AZ::IO::Path& path, QWidget* parent = nullptr);
} // namespace ROS2::RobotImporterWidgetUtils
