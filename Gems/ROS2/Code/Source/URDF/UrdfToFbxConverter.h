/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>

#include <AzCore/std/string/string.h>
#include <AzCore/std/containers/map.h>
#include <AzCore/Memory/SystemAllocator.h>

#include "FbxGenerator.h"
#include "UrdfParser.h"

namespace ROS2
{
    //! Class for conversion from URDF to Filmbox (.fbx) files
    class UrdfToFbxConverter
    {
    public:
        AZ_CLASS_ALLOCATOR(UrdfToFbxConverter, AZ::SystemAllocator, 0);

        AZStd::string Convert(const AZStd::string & urdfString);

        AZStd::string ConvertAndSaveToFile(const AZStd::string & urdfString, const AZStd::string & filePath);

    private:
        void AddLinkToFbxGenerator(const urdf::Link & urdfLink);
        void AddMaterialsToFbxGenerator(const urdf::ModelInterfaceSharedPtr & urdfModel);

        Fbx::FbxGenerator m_generator;
        AZStd::map<AZStd::string, Id> m_materialNamesToIds;
        AZStd::map<AZStd::string, Id> m_objectNameToObjectId;
    };

} // namespace ROS2