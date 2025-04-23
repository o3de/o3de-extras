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
    //! Hook used in ROS2 Gem Robot Importer to create ROS2 sensor components based on SDFormat model description.
    //! It implements the parameters mapping between the SDFormat sensor and the particular O3DE component.
    //! It should be registered as a serialization attribute in the component's reflection to make the it visible in the SDFormat model
    //! parser. See the sensor element at http://sdformat.org/spec?ver=1.10&elem=sensor for more details on SDFormat.
    struct SensorImporterHook
    {
        AZ_TYPE_INFO(SensorImporterHook, "{23f189e3-8da4-4498-9b89-1ef6c900940a}");

        //! Types of sensors in SDFormat description that can be mapped to the particular O3DE component.
        //! Multiple SDFormat sensors can map to one O3DE component.
        AZStd::unordered_set<sdf::SensorType> m_sensorTypes;

        //! Names of the sensor's parameters in SDFormat description that are supported by the particular O3DE component.
        AZStd::unordered_set<AZStd::string> m_supportedSensorParams;

        //! Names of plugins associated with the sensor in SDFormat description that are supported by the particular O3DE component.
        //! Multiple SDFormat plugins can map to one O3DE component.
        AZStd::unordered_set<AZStd::string> m_pluginNames;

        //! Names of the plugin's parameters associated with the sensor in SDFormat description that are supported
        //! by the particular O3DE component.
        AZStd::unordered_set<AZStd::string> m_supportedPluginParams;

        //! Registered function that is invoked when a sensor of the specified type is processed by the SDFormat Importer.
        //! @param AZ::Entity& a reference to the Entity in which one or more O3DE component is created by the importer
        //! @param sdf::Sensor& a reference to the sensor data in SDFormat, which is used to configure O3DE component
        using ErrorString = AZStd::string;
        using ConvertSensorOutcome = AZ::Outcome<void, ErrorString>;
        using ConvertSDFSensorCallback = AZStd::function<ConvertSensorOutcome(AZ::Entity&, const sdf::Sensor&)>;
        ConvertSDFSensorCallback m_sdfSensorToComponentCallback;
    };

    //! Buffer for SensorImporterHook data
    using SensorImporterHooksStorage = AZStd::vector<ROS2::SDFormat::SensorImporterHook>;
} // namespace ROS2::SDFormat
