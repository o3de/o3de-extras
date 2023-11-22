/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/Outcome/Outcome.h>
#include <AzCore/std/containers/unordered_map.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/function/function_template.h>
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/Prefab/PrefabPublicInterface.h>

namespace sdf
{
    inline namespace v13
    {
        class Link;
        class Model;
        class Plugin;
    } // namespace v13
} // namespace sdf

namespace ROS2::SDFormat
{
    //! Buffer for created Entities per link
    using CreatedEntitiesMap = AZStd::unordered_map<const sdf::Link*, AzToolsFramework::Prefab::PrefabEntityResult>;

    //! Hook used in ROS2 Gem Robot Importer to create ROS2 components based on SDFormat model description.
    //! It implements the parameters mapping between the SDFormat model plugin and the particular O3DE component.
    //! It should be registered as a serialization attribute in the component's reflection to make the it visible in the SDFormat model
    //! parser.
    struct ModelPluginImporterHook
    {
        AZ_TYPE_INFO(ModelPluginImporterHook, "{f55496c8-d48d-4895-b63c-cd069298bfda}");

        //! Names of plugins associated with the model in SDFormat description that are supported by the particular O3DE component.
        //! Multiple SDFormat plugins can map to one O3DE component.
        AZStd::unordered_set<AZStd::string> m_pluginNames;

        //! Names of the plugin's parameters associated with the sensor in SDFormat description that are supported
        //! by the particular O3DE component.
        AZStd::unordered_set<AZStd::string> m_supportedPluginParams;

        //! Registered function that is invoked when a plugin of the specified type is processed by the SDFormat Importer.
        //! @param AZ::Entity& a reference to the Entity in which one or more O3DE components are created by the importer
        //! @param sdf::Plugin& a reference to the plugins data in SDFormat, which is used to configure O3DE components
        //! @param sdf::Model& a reference to the models data in SDFormat, might be used to configure O3DE components
        //! @param CreatedEntitiesMap& a reference to the map of all created entities, might be used configure O3DE components
        using ErrorString = AZStd::string;
        using ConvertPluginOutcome = AZ::Outcome<void, ErrorString>;
        using ConvertSDFModelPluginCallback =
            AZStd::function<ConvertPluginOutcome(AZ::Entity&, const sdf::Plugin&, const sdf::Model&, const CreatedEntitiesMap&)>;
        ConvertSDFModelPluginCallback m_sdfPluginToComponentCallback;
    };

    //! Buffer for ModelPluginImporterHook data
    using ModelPluginImporterHooksStorage = AZStd::vector<ROS2::SDFormat::ModelPluginImporterHook>;
} // namespace ROS2::SDFormat
