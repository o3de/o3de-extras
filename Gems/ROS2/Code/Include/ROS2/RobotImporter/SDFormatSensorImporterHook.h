/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Entity.h>
#include <AzCore/std/containers/unordered_set.h>
#include <AzCore/std/function/function_template.h>
#include <AzCore/std/string/string.h>

namespace sdf
{
    inline namespace v13
    {
        enum class SensorType;
        class Sensor;
    } // namespace v13
} // namespace sdf

namespace ROS2::SDFormat
{
    //! Hook used in ROS2 Gem Robot Importer to create ROS2 sensor components based on SDF model description file.
    //! See the <sensor> element at http://sdformat.org/spec?ver=1.10&elem=sensor for more details.
    struct SensorImporterHook
    {
        AZ_TYPE_INFO(SensorImporterHook, "{23f189e3-8da4-4498-9b89-1ef6c900940a}");

        //! Types of the sensor in the SDF file that can be reused in O3DE.
        //! Multiple SDF sensors can map to one O3DE component.
        AZStd::unordered_set<sdf::SensorType> m_sensorTypes;

        //! Names of the options of the sensor in the SDF file that can be reused in O3DE.
        AZStd::unordered_set<AZStd::string> m_sensorOptions;

        //! Names of the plugins associated with the sensor in the SDF file that can be reused in O3DE.
        //! Multiple SDF plugins can map to one O3DE component.
        AZStd::unordered_set<AZStd::string> m_pluginNames;

        //! Names of the options of the plugin associated with the sensor in the SDF file that can be reused in O3DE.
        AZStd::unordered_set<AZStd::string> m_pluginOptions;

        //! Registered function, that is invoked when an plugin with the specified name
        //! is processed by the SDF Importer.
        //! A reference to an Entity is supplied which can be populated with one or more O3DE Components
        //! that were converted using the SDF Plugin data.
        using ConvertSDFSensorCallback = AZStd::function<void(AZ::Entity&, const sdf::Sensor&)>;
        ConvertSDFSensorCallback m_sdfSensorToComponentCallback;
    };
} // namespace ROS2::SDFormat
