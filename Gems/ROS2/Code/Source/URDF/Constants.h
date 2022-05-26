/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace ROS2
{
    namespace Fbx
    {
        //! The struct contains all constant parameters used in FBX files.
        //! @note Constant values could be moved to configuration file.
        struct Constants
        {
            struct FbxHeader
            {
                static constexpr int headerVersion = 1003;
                static constexpr int fileVersion = 7500;
                static constexpr const char* creatorName = "O3DE ROS2 Gem";
                static constexpr int timeStampVersion = 1000;
                static constexpr int metaDataVersion = 100;
                static constexpr const char* metaDataTitle = "";
                static constexpr int sceneInfoVersion = 100;
                static constexpr const char* applicationName = "O3DE";
                static constexpr const char* applicationVersion = "2022";
                static constexpr const char* documentActiveAnimStackName = "Take 001";
                static constexpr const char* dummyPath = "/dummy_path.fbx";
                static constexpr const char* fileCreationDate = "01/01/2022 00:00:00.000";
            };

            struct GlobalSettings
            {
                static constexpr int version = 1000;
                static constexpr int defaultTimeSpan = 1924423250;
                static constexpr const char* defaultCamera = "Producer Perspective";
                static constexpr int timeMode = 11;
                static constexpr int timeProtocol = 2;
                static constexpr int snapOnFrameMode = 0;
                static constexpr int customFrameRate = -1;
                static constexpr int currentTimeMarker = -1;
            };

            struct Material
            {
                static constexpr int defaultVersion = 102;
                static constexpr const char* defaultShadingModel = "phong";
                static constexpr float defaultDiffuseFactor = 0.9;
                static constexpr float defaultOpacity = 1.0;
                static constexpr float defaultReflectivity = 0.0;
            };

            struct Object
            {
                static constexpr int modelVersion = 232;
                static constexpr int geometryVersion = 102;
                static constexpr int layerElementNormalVersion = 102;
                static constexpr int layerElementUvVersion = 101;
            };

            // Other
            static constexpr int definitionsVersion = 100;
            static constexpr const char* defaultConnectionType = "OO";
        };
    } // namespace Fbx
} // namespace ROS2