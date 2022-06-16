/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "UrdfToFbxConverter.h"

#include <AzCore/std/string/string.h>
#include <AzCore/Console/Console.h>

namespace ROS2
{
    AZStd::string UrdfToFbxConverter::Convert(const AZStd::string & urdfString)
    {
        // 1. Parse URDF
        const auto urdf = UrdfParser::Parse(urdfString.data());

        // 2. Add materials to FBX
        AddMaterialsToFbxGenerator(urdf);

        // 3. Add links from URDF based structure to FBX generator in Depth First order
        const auto root = urdf->getRoot();

        std::stack<urdf::Link> stack;
        stack.push(*root);

        while(!stack.empty())
        {
            const auto link = stack.top();
            stack.pop();

            AddLinkToFbxGenerator(link);

            // Add childs to stack
            for (const auto & child : link.child_links)
            {
                stack.push(*child);
            }
        }

        return m_generator.GetFbxString().data();
    }

    AZStd::string UrdfToFbxConverter::ConvertAndSaveToFile(
        const AZStd::string & urdfString, const AZStd::string & filePath)
    {
        const auto fbxContent = Convert(urdfString);
        m_generator.SaveToFile(filePath.data());

        return fbxContent;
    }

    void UrdfToFbxConverter::AddLinkToFbxGenerator(const urdf::Link & link)
    {
        const auto linkGeometry = link.visual->geometry;

        if (linkGeometry->type == urdf::Geometry::BOX)
        {
            auto boxGeometry = std::dynamic_pointer_cast<urdf::Box>(linkGeometry);
            const double cubeSize = boxGeometry->dim.x; // TODO: Handle box in FBX instead of cube
            AZStd::string materialName(link.visual->material_name.c_str());
            auto objectId = m_generator.AddCubeObject(link.name.c_str(), cubeSize, m_materialNamesToIds[materialName]);
            m_objectNameToObjectId[link.name.c_str()] = objectId;
        }
        else
        {
            AZ_Warning(__func__, false, "Only box geometry is supported.");
        }

        // Set proper relations between links (only root has no parent)
        // TODO: handle joints tranformations
        if (const auto & parent = link.getParent())
        {
            const auto parentId = m_objectNameToObjectId[parent->name.c_str()];
            const auto childId = m_objectNameToObjectId[link.name.c_str()];
            m_generator.SetRelationBetweenObjects(parentId, childId);
        }
    }

    void UrdfToFbxConverter::AddMaterialsToFbxGenerator(const urdf::ModelInterfaceSharedPtr & urdfModel)
    {
        if (!urdfModel)
        {
            AZ_Error(__func__, false, "Missing URDF model.");
            return;
        }

        for (const auto & e : urdfModel->materials_)
        {
            const auto material = e.second;
            const AZStd::string materialName(material->name.c_str());
            const auto materialColor = material->color;
            const Fbx::Color fbxColor(materialColor.r, materialColor.g, materialColor.b);
            const auto materialId = m_generator.AddMaterial(materialName, fbxColor);
            m_materialNamesToIds[materialName] = materialId;

            AZ_Printf(__func__, "Add new material: %s", materialName.c_str());
        }
    }

} // namespace ROS2