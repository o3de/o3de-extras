/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include "ROS2RobotImporterSystemComponent.h"
#include <AzCore/std/string/string.h>
#include <AzToolsFramework/Entity/EditorEntityContextBus.h>
#include <ROS2/RobotImporter/RobotImporterBus.h>
#include <ROS2/RobotImporter/SDFormatModelPluginImporterHook.h>
#include <ROS2/RobotImporter/SDFormatSensorImporterHook.h>
#include <RobotImporter/Utils/SourceAssetsStorage.h>
namespace ROS2
{

    //! Editor component for RobotImporter widget
    class ROS2RobotImporterEditorSystemComponent
        : public ROS2RobotImporterSystemComponent
        , private AzToolsFramework::EditorEvents::Bus::Handler
        , private RobotImporterRequestBus::Handler
    {
    public:
        AZ_COMPONENT(ROS2RobotImporterEditorSystemComponent, "{1cc069d0-72f9-411e-a94b-9159979e5a0c}", ROS2RobotImporterSystemComponent);
        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);

        ROS2RobotImporterEditorSystemComponent() = default;
        ~ROS2RobotImporterEditorSystemComponent() = default;

    private:
        // Component overrides ...
        void Activate() override;
        void Deactivate() override;

        // AzToolsFramework::EditorEvents::Bus::Handler overrides ...
        void NotifyRegisterViews() override;

        // RobotImporterRequestsBus::Handler overrides ..
        bool GeneratePrefabFromFile(const AZStd::string_view filePath, bool importAssetWithUrdf, bool useArticulation) override;
        const SDFormat::SensorImporterHooksStorage& GetSensorHooks() const override;
        const SDFormat::ModelPluginImporterHooksStorage& GetModelPluginHooks() const override;

        // Method for parsing attributes used for both sensor importer hooks and model plugin importer hooks
        template<typename T>
        bool ParseAttributes(T& outputBuffer, const AZ::SerializeContext::ClassData* classData, const AZStd::string& attributeName)
        {
            auto* attribute = AZ::FindAttribute(AZ::Crc32(attributeName.c_str()), classData->m_attributes);
            if (attribute == nullptr)
            {
                return true;
            }

            AZ::AttributeReader reader(nullptr, attribute);
            T readData;
            if (reader.Read<T>(readData))
            {
                outputBuffer.insert(outputBuffer.end(), readData.begin(), readData.end());
            }

            return false;
        }

        // Timeout for loop waiting for assets to be built
        static constexpr AZStd::chrono::seconds assetLoopTimeout = AZStd::chrono::seconds(30);

        // Cache for storing sensor importer hooks (read only once)
        SDFormat::SensorImporterHooksStorage m_sensorHooks;

        // Cache for storing model plugin importer hooks (read only once)
        SDFormat::ModelPluginImporterHooksStorage m_modelPluginHooks;
    };
} // namespace ROS2
