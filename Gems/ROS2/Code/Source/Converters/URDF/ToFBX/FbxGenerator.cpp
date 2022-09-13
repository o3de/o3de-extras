/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "FbxGenerator.h"

#include <chrono>
#include <ctime>
#include <fstream>

#include <AzCore/Console/Console.h>

#include "Constants.h"

namespace ROS2
{
    namespace Fbx
    {
        void FbxGenerator::SaveToFile(const AZStd::string& filePath, FileType type)
        {
            // TODO: add support for binary files
            if (type != FileType::Text)
            {
                std::runtime_error(std::string(__func__) + ": Only text file type is supported!");
            }

            std::ofstream file(filePath.c_str());
            if (!file.is_open())
            {
                std::runtime_error(std::string(__func__) + ": Unable to open file: " + std::string(filePath.c_str()));
            }

            file << GetFbxString().c_str();
            AZ_Printf("Fbx", "Data structure saved to file: %s", filePath.c_str());
        }

        AZStd::string FbxGenerator::GetFbxString()
        {
            if (!m_nodesUpdated)
            {
                GenerateFbxStructure();
            }

            AZStd::string data;
            // Traverse tree in depth-first order and generate string
            for (const auto& n : m_basicNodes)
            {
                data += n.ToString(0);
            }

            return data;
        }

        void FbxGenerator::Reset()
        {
            m_basicNodes.clear();
            m_nodesUpdated = false;
            m_connections.clear();
            m_objects = AZStd::make_shared<Node>("Objects");
        }

        Id FbxGenerator::AddCubeObject(const AZStd::string& objectName, double size, Id materialId)
        {
            m_nodesUpdated = false;

            const auto model = CreateModel(objectName);
            m_objects->AddChild(model.node);

            const auto geometry = CreateGeometryCube(size);
            m_objects->AddChild(std::move(geometry.node));

            // Attach first object to the root node
            if (m_first_object)
            {
                m_connections.push_back(Connection(rootId, model.id, Constants::defaultConnectionType));
                m_first_object = false;
            }

            m_connections.push_back(Connection(model.id, geometry.id, Constants::defaultConnectionType));

            // TODO: Would be great to add validation if materialId exists.
            m_connections.push_back(Connection(model.id, materialId, Constants::defaultConnectionType));

            return model.id;
        }

        Id FbxGenerator::AddMaterial(const AZStd::string& materialName, const Color& color)
        {
            m_nodesUpdated = false;

            const auto material = CreateMaterial(materialName, color);
            m_objects->AddChild(material.node);
            return material.id;
        }

        void FbxGenerator::SetRelationBetweenObjects(Id parentId, Id childId)
        {
            m_nodesUpdated = false;
            m_connections.push_back(Connection(parentId, childId, Constants::defaultConnectionType));
        }

        void FbxGenerator::GenerateFbxStructure()
        {
            m_basicNodes.clear();

            m_basicNodes.push_back(GetFbxHeaderExtension());
            m_basicNodes.push_back(GetGlobalSettings());
            m_basicNodes.push_back(GetDocuments());
            m_basicNodes.push_back(Node("References"));
            m_basicNodes.push_back(GetDefinitions());
            m_basicNodes.push_back(*m_objects);
            m_basicNodes.push_back(GenerateConnections());
            m_basicNodes.push_back(Node("Takes"));

            m_nodesUpdated = true;
        }

        Node FbxGenerator::GetFbxHeaderExtension() const
        {
            Node fbxHeader("FBXHeaderExtension");
            fbxHeader.AddChild("FBXHeaderVersion", Constants::FbxHeader::headerVersion);
            fbxHeader.AddChild("FBXVersion", Constants::FbxHeader::fileVersion);
            fbxHeader.AddChild(GetTimeStamp());
            fbxHeader.AddChild("Creator", Constants::FbxHeader::creatorName);
            fbxHeader.AddChild(GetSceneInfo());

            return fbxHeader;
        }

        Node FbxGenerator::GetGlobalSettings() const
        {
            Node globalSettings("GlobalSettings");
            globalSettings.AddChild("Version", Constants::GlobalSettings::version);

            Node properties("Properties70");
            properties.AddChild(Node("P", { "UpAxis", "int", "Integer", "", 1 }));
            properties.AddChild(Node("P", { "UpAxisSign", "int", "Integer", "", 1 }));
            properties.AddChild(Node("P", { "FrontAxis", "int", "Integer", "", 2 }));
            properties.AddChild(Node("P", { "FrontAxisSign", "int", "Integer", "", 1 }));
            properties.AddChild(Node("P", { "CoordAxis", "int", "Integer", "", 0 }));
            properties.AddChild(Node("P", { "CoordAxisSign", "int", "Integer", "", 1 }));
            properties.AddChild(Node("P", { "OriginalUpAxis", "int", "Integer", "", 1 }));
            properties.AddChild(Node("P", { "OriginalUpAxisSign", "int", "Integer", "", 1 }));
            properties.AddChild(Node("P", { "UnitScaleFactor", "double", "Number", "", 1 }));
            properties.AddChild(Node("P", { "OriginalUnitScaleFactor", "double", "Number", "", 1 }));
            properties.AddChild(Node("P", { "AmbientColor", "ColorRGB", "Color", "", 0, 0, 0 }));
            properties.AddChild(Node("P", { "DefaultCamera", "KString", "", "", Constants::GlobalSettings::defaultCamera }));
            properties.AddChild(Node("P", { "TimeMode", "enum", "", "", Constants::GlobalSettings::timeMode }));
            properties.AddChild(Node("P", { "TimeProtocol", "enum", "", "", Constants::GlobalSettings::timeProtocol }));
            properties.AddChild(Node("P", { "SnapOnFrameMode", "enum", "", "", Constants::GlobalSettings::snapOnFrameMode }));
            properties.AddChild(Node("P", { "TimeSpanStart", "KTime", "Time", "", Constants::GlobalSettings::defaultTimeSpan }));
            properties.AddChild(Node("P", { "TimeSpanStop", "KTime", "Time", "", Constants::GlobalSettings::defaultTimeSpan }));
            properties.AddChild(Node("P", { "CustomFrameRate", "double", "Number", "", Constants::GlobalSettings::customFrameRate }));
            properties.AddChild(Node("P", { "TimeMarker", "Compound", "", "" }));
            properties.AddChild(Node("P", { "CurrentTimeMarker", "int", "Integer", "", Constants::GlobalSettings::currentTimeMarker }));

            globalSettings.AddChild(std::move(properties));

            return globalSettings;
        }

        Node FbxGenerator::GetDocuments() const
        {
            Node documents("Documents");
            documents.AddChild("Count", 1);

            Node document("Document", { "", "Scene" });

            Node properties("Properties70");
            properties.AddChild(Node("P", { "SourceObject", "object", "", "" }));
            properties.AddChild(Node("P", { "ActiveAnimStackName", "KString", "", Constants::FbxHeader::documentActiveAnimStackName }));

            document.AddChild(std::move(properties));
            document.AddChild("RootNode", 0);
            return documents;
        }

        Node FbxGenerator::GetDefinitions() const
        {
            Node definitions("Definitions");
            definitions.AddChild("Version", Constants::definitionsVersion);
            definitions.AddChild("Count", 3);
            definitions.AddChild(Node("ObjectType", { "GlobalSettings" }, { Node("Count", { 1 }) }));
            definitions.AddChild(Node("ObjectType", { "Model" }, { Node("Count", { 1 }) }));
            definitions.AddChild(Node("ObjectType", { "Geometry" }, { Node("Count", { 1 }) }));
            definitions.AddChild(Node("ObjectType", { "Material" }, { Node("Count", { 1 }) }));

            return definitions;
        }

        NodeWithId FbxGenerator::CreateModel(const AZStd::string& modelName) const
        {
            const int modelId = UniqueIdGenerator::GetUniqueId();

            Node model("Model", { modelId, modelName, "Mesh" });
            model.AddChild("Version", Constants::Object::modelVersion);
            model.AddChild("Culling", "CullingOff");

            Node properties("Properties70");
            properties.AddChild(Node("P", { "RotationActive", "bool", "", "", 1 }));
            properties.AddChild(Node("P", { "InheritType", "enum", "", "", 1 }));
            properties.AddChild(Node("P", { "ScalingMax", "Vector3D", "Vector", "", 0, 0, 0 }));
            properties.AddChild(Node("P", { "DefaultAttributeIndex", "int", "Integer", "", 0 }));
            properties.AddChild(Node("P", { "Lcl Scaling", "Lcl Scaling", "", "A", 100, 100, 100 }));
            properties.AddChild(Node("P", { "currentUVSet", "KString", "", "U", "map1" }));
            model.AddChild(std::move(properties));

            return NodeWithId(modelId, model);
        }

        NodeWithId FbxGenerator::CreateMaterial(const AZStd::string& name, const Color& color) const
        {
            const int materialId = UniqueIdGenerator::GetUniqueId();

            Node material("Material", { materialId, name, "" });
            material.AddChild("Version", Constants::Material::defaultVersion);
            material.AddChild("ShadingModel", Constants::Material::defaultShadingModel);
            material.AddChild("Multilayer", 0);

            // TODO: set more parameters of material
            Node properties("Properties70");
            properties.AddChild(Node("P", { "AmbientColor", "Color", "", "A", 0, 0, 0 }));
            properties.AddChild(Node("P", { "DiffuseColor", "Color", "", "A", 1, 1, 1 }));
            properties.AddChild(Node("P", { "DiffuseFactor", "Number", "", "A", Constants::Material::defaultDiffuseFactor }));
            properties.AddChild(Node("P", { "TransparencyFactor", "Number", "", "A", 1 }));
            properties.AddChild(Node("P", { "SpecularColor", "Color", "", "A", color.r, color.g, color.b }));
            properties.AddChild(Node("P", { "ReflectionFactor", "Number", "", "A", 0.5 }));
            properties.AddChild(Node("P", { "Emissive", "Vector3D", "Vector", "", 0, 0, 0 }));
            properties.AddChild(Node("P", { "Ambient", "Vector3D", "Vector", "", 0, 0, 0 }));
            properties.AddChild(Node("P", { "Diffuse", "Vector3D", "Vector", "", 0.9, 0.9, 0.9 }));
            properties.AddChild(Node("P", { "Specular", "Vector3D", "Vector", "", 0.5, 0.5, 0.5 }));
            properties.AddChild(Node("P", { "Shininess", "double", "Number", "", 20 }));
            properties.AddChild(Node("P", { "Opacity", "double", "Number", "", Constants::Material::defaultOpacity }));
            properties.AddChild(Node("P", { "Reflectivity", "double", "Number", "", Constants::Material::defaultReflectivity }));
            material.AddChild(std::move(properties));

            return NodeWithId(materialId, material);
        }

        NodeWithId FbxGenerator::CreateGeometryCube(double size) const
        {
            // Syntax of geometry
            // Geometry: "name", "Mesh" {
            //      Vertices: [...]
            //      PolygonVertexIndex: [...]
            //      LayerElementNormal: { }
            //      LayerElementUV: { }
            // }
            const int geometryId = UniqueIdGenerator::GetUniqueId();

            // Example cube geometry
            Node geometry("Geometry", { geometryId, "Geometry::cube", "Mesh" });
            geometry.AddChild("GeometryVersion", Constants::Object::geometryVersion);

            // Vertices
            // Single vertex v1: v1_x, v1_y, v1_z
            // Multiple vertices: v1_x, v1_y, v1_z, v2_x, v2_y, v2_z, ..., vn_x, vn_y, vn_z
            // More details: https://banexdevblog.wordpress.com/2014/06/23/a-quick-tutorial-about-the-fbx-ascii-format/
            Node vertices("Vertices", { RawString("*24") });
            vertices.AddChild(Node("a", { -size, -size, size,  size, -size, size,  -size, size,  size,  size, size,  size,
                                          -size, size,  -size, size, size,  -size, -size, -size, -size, size, -size, -size }));
            geometry.AddChild(vertices);

            // Indices of four-sided polygons (quads)
            Node polygonVertexIndex("PolygonVertexIndex", { RawString("*24") });
            polygonVertexIndex.AddChild(Node("a", { 0, 1, 3, -3, 2, 3, 5, -5, 4, 5, 7, -7, 6, 7, 1, -1, 1, 7, 5, -4, 6, 0, 2, -5 }));
            geometry.AddChild(polygonVertexIndex);

            // Edges
            Node edges("Edges", { RawString("*12") });
            edges.AddChild(Node("a", { 0, 2, 6, 10, 3, 1, 7, 5, 11, 9, 15, 13 }));
            geometry.AddChild(edges);

            // Normals
            auto layerElementNormal = Node("LayerElementNormal", { 0 });
            layerElementNormal.AddChild("Version", Constants::Object::layerElementNormalVersion);
            layerElementNormal.AddChild("Name", "");
            layerElementNormal.AddChild("MappingInformationType", "ByPolygonVertex");
            layerElementNormal.AddChild("ReferenceInformationType", "Direct");

            Node normals("Normals", { RawString("*72") });
            normals.AddChild(Node("a", { 0, 0, 1,  0, 0, 1,  0, 0, 1,  0, 0, 1,  0,  1,  0, 0,  1,  0, 0,  1,  0, 0,  1,  0,
                                         0, 0, -1, 0, 0, -1, 0, 0, -1, 0, 0, -1, 0,  -1, 0, 0,  -1, 0, 0,  -1, 0, 0,  -1, 0,
                                         1, 0, 0,  1, 0, 0,  1, 0, 0,  1, 0, 0,  -1, 0,  0, -1, 0,  0, -1, 0,  0, -1, 0,  0 }));
            layerElementNormal.AddChild(normals);

            Node normalsSw("NormalsW", { RawString("*24") });
            normalsSw.AddChild(Node("a", { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1 }));
            layerElementNormal.AddChild(normalsSw);

            geometry.AddChild(layerElementNormal);

            // UV
            auto layerElementUV = Node("LayerElementUV", { 0 });
            layerElementUV.AddChild("Version", Constants::Object::layerElementUvVersion);
            layerElementUV.AddChild("Name", "map1");
            layerElementUV.AddChild("MappingInformationType", "ByPolygonVertex");
            layerElementUV.AddChild("ReferenceInformationType", "IndexToDirect");

            Node uv("UV", { RawString("*28") });
            uv.AddChild(Node("a", { 0.375, 0,    0.625, 0, 0.375, 0.25, 0.625, 0.25, 0.375, 0.5,  0.625, 0.5, 0.375, 0.75,
                                    0.625, 0.75, 0.375, 1, 0.625, 1,    0.875, 0,    0.875, 0.25, 0.125, 0,   0.125, 0.25 }));
            layerElementUV.AddChild(uv);

            Node uvIndex("UVIndex", { RawString("*28") });
            uvIndex.AddChild(Node("a", { 0, 1, 3, 2, 2, 3, 5, 4, 4, 5, 7, 6, 6, 7, 9, 8, 1, 10, 11, 3, 12, 0, 2, 13 }));
            layerElementUV.AddChild(uvIndex);

            geometry.AddChild(layerElementUV);

            return NodeWithId(geometryId, geometry);
        }

        Node FbxGenerator::GenerateConnections() const
        {
            Node connections("Connections");
            for (const auto& c : m_connections)
            {
                connections.AddChild(Node("C", { c.type, c.childId, c.parentId }));
            }

            return connections;
        }

        Node FbxGenerator::GetTimeStamp() const
        {
            auto now = std::chrono::system_clock::now();
            std::time_t now_c = std::chrono::system_clock::to_time_t(now);
            struct tm* parts = std::localtime(&now_c);

            Node timeStamp("CreationTimeStamp");
            timeStamp.AddChild("Version", Constants::FbxHeader::timeStampVersion);
            timeStamp.AddChild("Year", 1900 + parts->tm_year);
            timeStamp.AddChild("Month", 1 + parts->tm_mon);
            timeStamp.AddChild("Day", parts->tm_mday);
            timeStamp.AddChild("Hour", parts->tm_hour);
            timeStamp.AddChild("Minute", parts->tm_min);
            timeStamp.AddChild("Second", 0);
            timeStamp.AddChild("Millisecond", 0);

            return timeStamp;
        }

        Node FbxGenerator::GetSceneInfo() const
        {
            Node sceneInfo("SceneInfo");
            sceneInfo.AddProperty("SceneInfo::GlobalInfo");
            sceneInfo.AddProperty("UserData");
            sceneInfo.AddChild("Type", "UserData");
            sceneInfo.AddChild("Version", Constants::FbxHeader::sceneInfoVersion);
            sceneInfo.AddChild(GetMetaData());

            Node properties("Properties70");
            properties.AddChild(Node("P", { "DocumentUrl", "KString", "Url", "", Constants::FbxHeader::dummyPath }));
            properties.AddChild(Node("P", { "SrcDocumentUrl", "KString", "Url", "", Constants::FbxHeader::dummyPath }));
            properties.AddChild(Node("P", { "Original", "Compound", "", "" }));
            properties.AddChild(Node("P", { "Original|ApplicationVendor", "KString", "", "", Constants::FbxHeader::applicationName }));
            properties.AddChild(Node("P", { "Original|ApplicationName", "KString", "", "", Constants::FbxHeader::applicationName }));
            properties.AddChild(Node("P", { "Original|ApplicationVersion", "KString", "", "", Constants::FbxHeader::applicationVersion }));
            properties.AddChild(Node("P", { "Original|DateTime_GMT", "DateTime", "", "", Constants::FbxHeader::fileCreationDate }));
            properties.AddChild(Node("P", { "Original|FileName", "KString", "", "", Constants::FbxHeader::dummyPath }));
            properties.AddChild(Node("P", { "LastSaved", "Compound", "", "" }));
            properties.AddChild(Node("P", { "LastSaved|ApplicationVendor", "KString", "", "", Constants::FbxHeader::applicationName }));
            properties.AddChild(Node("P", { "LastSaved|ApplicationName", "KString", "", "", Constants::FbxHeader::applicationName }));
            properties.AddChild(Node("P", { "LastSaved|ApplicationVersion", "KString", "", "", Constants::FbxHeader::applicationVersion }));
            properties.AddChild(Node("P", { "LastSaved|DateTime_GMT", "DateTime", "", "", Constants::FbxHeader::fileCreationDate }));
            properties.AddChild(Node("P", { "Original|ApplicationActiveProject", "KString", "", "", Constants::FbxHeader::dummyPath }));
            properties.AddChild(Node("P", { "Original|ApplicationNativeFile", "KString", "", "", Constants::FbxHeader::dummyPath }));

            sceneInfo.AddChild(std::move(properties));

            return sceneInfo;
        }

        Node FbxGenerator::GetMetaData() const
        {
            Node metaData("MetaData");
            metaData.AddChild("Version", Constants::FbxHeader::metaDataVersion);
            metaData.AddChild("Title", Constants::FbxHeader::metaDataTitle);
            metaData.AddChild("Subject", "");
            metaData.AddChild("Author", "");
            metaData.AddChild("Keywords", "");
            metaData.AddChild("Revision", "");
            metaData.AddChild("Comment", "");

            return metaData;
        }

    } // namespace Fbx
} // namespace ROS2