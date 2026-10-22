/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include "AssetPathResolver.h"
#include <AzCore/IO/Path/Path.h>
#include <AzCore/IO/SystemFile.h>
#include <AzCore/StringFunc/StringFunc.h>
#include <AzCore/Utils/Utils.h>
#include <RobotImporter/Queries/SdfVisitors.h>

namespace ROS2RobotImporter::Assets
{
    inline namespace Internal
    {
        bool FileExistsCall(const AZ::IO::PathView& filePath)
        {
            AZ::IO::FixedMaxPath pathStorage(filePath);
            return AZ::IO::SystemFile::Exists(pathStorage.c_str());
        };
    } // namespace Internal

    ReferencedAssetMap GetReferencedAssetFilenames(const sdf::Root& root)
    {
        ReferencedAssetMap referencedAssetMap;
        auto GetAssetsFromModel =
            [&referencedAssetMap](const sdf::Model& model, const SDFormat::ModelStack&) -> SDFormat::VisitModelResponse
        {
            AZStd::string modelUri(model.Uri().c_str(), model.Uri().size());
            const auto addFilenameFromGeometry =
                [&referencedAssetMap, &modelUri](const sdf::Geometry* geometry, ReferencedAssetType assetType)
            {
                if (geometry->Type() == sdf::GeometryType::MESH)
                {
                    if (auto mesh = geometry->MeshShape(); mesh)
                    {
                        const AZ::IO::Path assetUri(mesh->Uri().c_str());
                        const AZStd::string modelAssetUri = (modelUri.empty()) ? assetUri.String() : modelUri + "/" + assetUri.String();
                        if (referencedAssetMap.contains(modelAssetUri))
                        {
                            referencedAssetMap[modelAssetUri].m_assetType |= assetType;
                        }
                        else
                        {
                            ReferencedAsset asset;
                            asset.m_assetType = assetType;
                            asset.m_modelUri = modelUri;
                            asset.m_assetUri = assetUri;
                            referencedAssetMap.emplace(modelAssetUri, AZStd::move(asset));
                        }
                    }
                }
            };

            const auto addFilenamesFromMaterial = [&referencedAssetMap, &modelUri](const sdf::Material* material)
            {
                // Only PBR entries on a material have filenames that need to be added.
                if ((!material) || (!material->PbrMaterial()))
                {
                    return;
                }

                if (auto pbr = material->PbrMaterial(); pbr)
                {
                    auto pbrWorkflow = pbr->Workflow(sdf::PbrWorkflowType::METAL);
                    if (!pbrWorkflow)
                    {
                        pbrWorkflow = pbr->Workflow(sdf::PbrWorkflowType::SPECULAR);
                        if (!pbrWorkflow)
                        {
                            return;
                        }
                    }

                    const auto emplaceTexture = [&referencedAssetMap, &modelUri](const std::string& texturePath)
                    {
                        ReferencedAsset asset;
                        asset.m_assetType = ReferencedAssetType::Texture;
                        asset.m_modelUri = modelUri;
                        asset.m_assetUri = AZ::IO::Path(texturePath.c_str());

                        const AZStd::string modelAssetUri =
                            (modelUri.empty()) ? asset.m_assetUri.String() : modelUri + "/" + asset.m_assetUri.String();
                        referencedAssetMap.emplace(modelAssetUri, AZStd::move(asset));
                    };

                    if (auto texture = pbrWorkflow->AlbedoMap(); !texture.empty())
                    {
                        emplaceTexture(texture);
                    }

                    if (auto texture = pbrWorkflow->NormalMap(); !texture.empty())
                    {
                        emplaceTexture(texture);
                    }

                    if (auto texture = pbrWorkflow->AmbientOcclusionMap(); !texture.empty())
                    {
                        emplaceTexture(texture);
                    }

                    if (auto texture = pbrWorkflow->EmissiveMap(); !texture.empty())
                    {
                        emplaceTexture(texture);
                    }

                    if (pbrWorkflow->Type() == sdf::PbrWorkflowType::METAL)
                    {
                        if (auto texture = pbrWorkflow->RoughnessMap(); !texture.empty())
                        {
                            emplaceTexture(texture);
                        }

                        if (auto texture = pbrWorkflow->MetalnessMap(); !texture.empty())
                        {
                            emplaceTexture(texture);
                        }
                    }
                }
            };

            const auto processLink = [&addFilenameFromGeometry, &addFilenamesFromMaterial](const sdf::Link* link)
            {
                for (uint64_t index = 0; index < link->VisualCount(); index++)
                {
                    addFilenameFromGeometry(link->VisualByIndex(index)->Geom(), ReferencedAssetType::VisualMesh);
                    addFilenamesFromMaterial(link->VisualByIndex(index)->Material());
                }

                for (uint64_t index = 0; index < link->CollisionCount(); index++)
                {
                    addFilenameFromGeometry(link->CollisionByIndex(index)->Geom(), ReferencedAssetType::ColliderMesh);
                }
            };

            for (uint64_t index = 0; index < model.LinkCount(); index++)
            {
                processLink(model.LinkByIndex(index));
            }

            return SDFormat::VisitModelResponse::VisitNestedAndSiblings;
        };

        SDFormat::VisitModels(root, GetAssetsFromModel);

        return referencedAssetMap;
    }

    AZ::IO::Path ResolveAmentPrefixPath(AZ::IO::Path unresolvedPath, AZStd::string_view amentPrefixPath, const FileExistsCB& fileExistsCB)
    {
        AZStd::vector<AZ::IO::Path> amentPrefixPaths;

        // Parse the AMENT_PREFIX_PATH environment variable into a set of distinct paths.
        auto AmentPrefixPathVisitor = [&amentPrefixPaths](AZStd::string_view prefixPath)
        {
            amentPrefixPaths.push_back(prefixPath);
        };
        // Note this code only works on Unix platforms
        // For Windows this will not work as the drive letter has a colon in it (C:\)
        AZ::StringFunc::TokenizeVisitor(amentPrefixPath, AmentPrefixPathVisitor, ':');

        AZ::IO::PathView strippedPath;

        // The AMENT_PREFIX_PATH is only used for lookups if the URI starts with "model://" or "package://"
        constexpr AZStd::string_view ValidAmentPrefixes[] = { "model://", "package://" };
        for (const auto& prefix : ValidAmentPrefixes)
        {
            // Perform a case-sensitive check to look for the prefix.
            constexpr bool prefixMatchCaseSensitive = true;
            if (AZ::StringFunc::StartsWith(unresolvedPath.Native(), prefix, prefixMatchCaseSensitive))
            {
                strippedPath = AZ::IO::PathView(unresolvedPath).Native().substr(prefix.size());
                break;
            }
        }

        // If no valid prefix was found, or if the URI *only* contains the prefix, return an empty result.
        if (strippedPath.empty())
        {
            return {};
        }

        // If the remaining path is an absolute path, it shouldn't get resolved with AMENT_PREFIX_PATH. return an empty result.
        if (strippedPath.IsAbsolute())
        {
            return {};
        }

        // Check to see if the relative part of the URI path refers to a location
        // within each <ament prefix path>/share directory
        for (const AZ::IO::Path& amentPrefixPath : amentPrefixPaths)
        {
            auto pathIter = strippedPath.begin();
            AZ::IO::PathView packageName = *pathIter;
            const AZ::IO::Path amentSharePath = amentPrefixPath / "share";
            const AZ::IO::Path packageManifestPath = amentSharePath / packageName / "package.xml";

            // Given a path like 'ambulance/meshes/model.stl', it will be considered a match if
            // <ament prefix path>/share/ambulance/package.xml exists and
            // <ament prefix path>/share/ambulance/meshes/model.stl exists.
            if (const AZ::IO::Path candidateResolvedPath = amentSharePath / strippedPath;
                fileExistsCB(packageManifestPath) && fileExistsCB(candidateResolvedPath))
            {
                AZ_Trace(
                    "ResolveAssetPath",
                    R"(Resolved using AMENT_PREFIX_PATH: "%.*s" -> "%.*s")"
                    "\n",
                    AZ_PATH_ARG(unresolvedPath),
                    AZ_PATH_ARG(candidateResolvedPath));
                return candidateResolvedPath;
            }
        }

        // No resolution was found, return an empty result.
        return {};
    }

    /// Finds global path from URDF/SDF path
    AZ::IO::Path ResolveAssetPath(
        AZ::IO::Path unresolvedPath,
        const AZ::IO::PathView& baseFilePath,
        AZStd::string_view amentPrefixPath,
        const SdfAssetBuilderSettings& settings,
        const FileExistsCB& fileExistsCB)
    {
        AZ_Printf("ResolveAssetPath", "ResolveAssetPath with %s\n", unresolvedPath.c_str());

        const auto& pathResolverSettings = settings.m_resolverSettings;

        // If the settings tell us to try the AMENT_PREFIX_PATH, use that first to try and resolve path.
        if (pathResolverSettings.m_useAmentPrefixPath)
        {
            if (AZ::IO::Path amentResolvedPath = ResolveAmentPrefixPath(unresolvedPath, amentPrefixPath, fileExistsCB);
                !amentResolvedPath.empty())
            {
                return amentResolvedPath;
            }
        }

        // Append all ancestor directories from the root file to the candidate replacement paths if the settings enable using
        // them for path resolution and the root file isn't empty.
        AZStd::vector<AZ::IO::Path> ancestorPaths;
        if (!baseFilePath.empty() && pathResolverSettings.m_useAncestorPaths)
        {
            // The first time through this loop, fileAncestorPath contains the full file name ('/a/b/c.sdf') so
            // ParentPath() will return the path containing the file ('/a/b'). Each iteration will walk up the path
            // to the root, including the root, before stopping ('/a', '/').
            AZ::IO::Path fileAncestorPath = baseFilePath;
            do
            {
                fileAncestorPath = fileAncestorPath.ParentPath();
                ancestorPaths.emplace_back(fileAncestorPath);
            } while (fileAncestorPath != fileAncestorPath.RootPath());
        }

        // Loop through each prefix in the builder settings and attempt to resolve it as either an absolute path
        // or a relative path that's relative in some way to the base file.
        for (const auto& [prefix, replacements] : pathResolverSettings.m_uriPrefixMap)
        {
            // Note this is a case-sensitive check to match the exact URI scheme
            // If that is not desired, then this code should be updated to read
            // a value from the Setting Registry indicating whether the uriPrefix matching
            // should be case sensitive
            constexpr bool uriPrefixMatchCaseSensitive = true;
            // If the path doesn't start with the given prefix, move on to the next prefix
            if (!AZ::StringFunc::StartsWith(unresolvedPath.Native(), prefix, uriPrefixMatchCaseSensitive))
            {
                continue;
            }

            // Strip the number of characters from the Uri scheme from beginning of the path
            AZ::IO::PathView strippedUriPath = AZ::IO::PathView(unresolvedPath).Native().substr(prefix.size());

            // Loop through each replacement path for this prefix, attach it to the front, and look for matches.
            for (const auto& replacement : replacements)
            {
                AZ::IO::Path replacedUriPath(replacement);
                replacedUriPath /= strippedUriPath;

                // If we successfully matched the prefix, and the replacement path is completely empty, we don't need to look any further.
                // There's no match.
                if (replacedUriPath.empty())
                {
                    AZ_Trace(
                        "ResolveAssetPath",
                        R"(Resolved Path is empty: "%.*s" -> "")"
                        "\n",
                        AZ_PATH_ARG(unresolvedPath));
                    return {};
                }

                // If the replaced path is an absolute path, if it exists, return it.
                // If it doesn't exist, keep trying other replacements.
                if (replacedUriPath.IsAbsolute())
                {
                    if (fileExistsCB(replacedUriPath))
                    {
                        AZ_Trace(
                            "ResolveAssetPath",
                            R"(Resolved Absolute Path: "%.*s" -> "%.*s")"
                            "\n",
                            AZ_PATH_ARG(unresolvedPath),
                            AZ_PATH_ARG(replacedUriPath));
                        return replacedUriPath;
                    }
                    else
                    {
                        // The file didn't exist, so continue.
                        continue;
                    }
                }

                // The URI path is not absolute, so attempt to append it to the ancestor directories of the URDF/SDF file
                for (const AZ::IO::Path& ancestorPath : ancestorPaths)
                {
                    if (const AZ::IO::Path candidateResolvedPath = ancestorPath / replacedUriPath; fileExistsCB(candidateResolvedPath))
                    {
                        AZ_Trace(
                            "ResolveAssetPath",
                            R"(Resolved using ancestor paths: "%.*s" -> "%.*s")"
                            "\n",
                            AZ_PATH_ARG(unresolvedPath),
                            AZ_PATH_ARG(candidateResolvedPath));
                        return candidateResolvedPath;
                    }
                }
            }
        }

        // At this point, the path has no identified URI prefix. If it's an absolute path, try to locate and return it.
        // Otherwise, return an empty path as an error.
        if (unresolvedPath.IsAbsolute())
        {
            if (fileExistsCB(unresolvedPath))
            {
                AZ_Trace(
                    "ResolveAssetPath",
                    R"(Resolved Absolute Path: "%.*s")"
                    "\n",
                    AZ_PATH_ARG(unresolvedPath));
                return unresolvedPath;
            }
            else
            {
                AZ_Trace(
                    "ResolveAssetPath",
                    R"(Failed to resolve Absolute Path: "%.*s")"
                    "\n",
                    AZ_PATH_ARG(unresolvedPath));
                return {};
            }
        }

        // The path is a relative path, so use the directory containing the base URDF/SDF file as the root path,
        // and if the file can be found successfully, return the path. Otherwise, return an empty path as an error.
        const AZ::IO::Path relativePath = AZ::IO::Path(baseFilePath.ParentPath()) / unresolvedPath;

        if (fileExistsCB(relativePath))
        {
            AZ_Trace(
                "ResolveAssetPath",
                R"(Resolved Relative Path: "%.*s" -> "%.*s")"
                "\n",
                AZ_PATH_ARG(unresolvedPath),
                AZ_PATH_ARG(relativePath));
            return relativePath;
        }

        AZ_Trace(
            "ResolveAssetPath",
            R"(Failed to resolve Relative Path: "%.*s" -> "%.*s")"
            "\n",
            AZ_PATH_ARG(unresolvedPath),
            AZ_PATH_ARG(relativePath));
        return {};
    }

    AmentPrefixString GetAmentPrefixPath()
    {
        // Support reading the AMENT_PREFIX_PATH environment variable on Unix/Windows platforms
        auto StoreAmentPrefixPath = [](char* buffer, size_t size) -> size_t
        {
            auto getEnvOutcome = AZ::Utils::GetEnv(AZStd::span(buffer, size), "AMENT_PREFIX_PATH");

            if (!getEnvOutcome.IsSuccess())
            {
                if (getEnvOutcome.GetError().m_errorCode == AZ::Utils::GetEnvErrorCode::EnvNotSet)
                {
                    AZ_Error("ReferencedAssetMap", false, "AMENT_PREFIX_PATH is not set in the environment.");
                }
                else if (getEnvOutcome.GetError().m_errorCode == AZ::Utils::GetEnvErrorCode::BufferTooSmall)
                {
                    AZ_Error(
                        "ReferencedAssetMap",
                        false,
                        "AMENT_PREFIX_PATH is too long (%zu), maximum permissible size is %zu ",
                        getEnvOutcome.GetError().m_requiredSize,
                        size);
                }
                else
                {
                    AZ_Error("ReferencedAssetMap", false, "AMENT_PREFIX_PATH is not found.");
                }
            }

            return getEnvOutcome ? getEnvOutcome.GetValue().size() : 0;
        };
        AmentPrefixString amentPrefixPath;
        amentPrefixPath.resize_and_overwrite(amentPrefixPath.capacity(), StoreAmentPrefixPath);

        return amentPrefixPath;
    }

} // namespace ROS2RobotImporter::Assets
