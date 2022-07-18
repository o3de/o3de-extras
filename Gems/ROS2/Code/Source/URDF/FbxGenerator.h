/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Memory/SystemAllocator.h>
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/smart_ptr/make_shared.h>
#include <AzCore/std/string/string.h>

#include "FbxNode.h"
#include "UniqueIdGenerator.h"

namespace ROS2
{
    namespace Fbx
    {
        //! Define type of the FBX file.
        enum class FileType
        {
            Text,
            Binary
        };

        //! RGB color
        struct Color
        {
            Color(float r, float g, float b)
                : r(r)
                , g(g)
                , b(b){};
            float r = 0;
            float g = 0;
            float b = 0;
        };

        //! The class represents the FBX file structure generator and operations on that files.
        //!
        //! FBX file has a tree based structure.
        //! Top structure of the FBX file consists of the following sections:
        //!   - FBXHeaderExtension (mandatory) - metadata of file.
        //!   - GlobalSettings (mandatory) - general data properties.
        //!   - Documents (optional) - ?
        //!   - References (optional) - ?
        //!   - Definitions (optional) - ?
        //!   - Objects (optional) - static data storage like objects, geometries, textures and materials.
        //!   - Connections (optional) - defines connections between data defined in Objects.
        //!                              For example which object uses specific material.
        //!   - Takes (optional) - animations definitions.
        //!
        //! Additional documentation about FBX structure:
        //! https://web.archive.org/web/20160605023014/https://wiki.blender.org/index.php/User:Mont29/Foundation/FBX_File_Structure
        //! https://banexdevblog.wordpress.com/2014/06/23/a-quick-tutorial-about-the-fbx-ascii-format/
        //!
        //! Example FBX file
        //! https://www.ics.uci.edu/~djp3/classes/2014_03_ICS163/tasks/arMarker/Unity/arMarker/Assets/CactusPack/Meshes/Sprites/Rock_Medium_SPR.fbx
        class FbxGenerator
        {
        public:
            AZ_CLASS_ALLOCATOR(FbxGenerator, AZ::SystemAllocator, 0);

            //! Save the current FBX structure to file.
            //! @note only ASCII version is supported.
            //! @param filePath is a path of the generated file.
            //! @param type of the generated FBX file.
            void SaveToFile(const AZStd::string& filePath, FileType type = FileType::Text);

            //! Get the string with FBX data.
            //! @return The string with ASCII version of current FBX structure.
            AZStd::string GetFbxString();

            //! Reset the internal data used for FBX file structure creation.
            void Reset();

            //! Add cube object.
            //! Notice: First added object is attached to the root node.
            //! @note TODO: generalization for other types of objects e.g. cuboid, cylinder
            //! @note TODO: handle textures.
            //! @param objectName A name of created object.
            //! @param size A size of the cube in meters.
            //! @param materialId The id of the created previously material that will be used in object.
            //! @return id of created object.
            Id AddCubeObject(const AZStd::string& objectName, double size, Id materialId);

            //! Add default material and return its id.
            //! @note TODO: add more material parameters.
            //! @return id of created material.
            Id AddMaterial(const AZStd::string& materialName, const Color& color);

            //! Create relation between objects
            //! @param parentId The id of the parent object in created relation.
            //! @param parentId The id of the child object in created relation.
            void SetRelationBetweenObjects(Id parentId, Id childId);

        private:
            //! Represents the connection between two objects.
            struct Connection
            {
                AZ_CLASS_ALLOCATOR(Connection, AZ::SystemAllocator, 0);

                Connection(Id parent, Id child, AZStd::string connectionType)
                    : parentId(parent)
                    , childId(child)
                    , type(connectionType)
                {
                }

                Id parentId = -1;
                Id childId = -1;
                AZStd::string type = "OO";
            };

            //! Generate the FBX file structure.
            void GenerateFbxStructure();

            // Default FBX file header
            Node GetFbxHeaderExtension() const;
            Node GetTimeStamp() const;
            Node GetSceneInfo() const;
            Node GetMetaData() const;

            // Default global settings
            Node GetGlobalSettings() const;

            Node GetDocuments() const;
            Node GetDefinitions() const;

            // Objects creation
            NodeWithId CreateModel(const AZStd::string& modelName) const;
            NodeWithId CreateMaterial(const AZStd::string& name, const Color& color) const;
            NodeWithId CreateGeometryCube(double size = 1.0) const;

            // Generate connections based on the m_connections.
            Node GenerateConnections() const;

            static constexpr Id rootId = 0;

            AZStd::vector<Node> m_basicNodes;
            bool m_nodesUpdated = false;
            AZStd::vector<Connection> m_connections;

            AZStd::shared_ptr<Node> m_objects = AZStd::make_shared<Node>("Objects");
            bool m_first_object = true;
        };

    } // namespace Fbx
} // namespace ROS2