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

        //! Callback for finding hooks defined in class reflection for certain SerializeContext attribute. This callback should be used
        //! with EnumerateAll method and it is meant to be called only once, as the search results are copied and can be reused.
        //! @param outputBuffer output buffer where hooks are copied
        //! @param classData serialization context in which queried attribute might exist.
        //! @param attributeName name of the attribute used to store hooks.
        //! @return this method returns always true, to enforce EnumerateAll method to iterate over all classes
        template<typename T>
        bool CopyHooksCallback(T& outputBuffer, const AZ::SerializeContext::ClassData* classData, const AZStd::string& attributeName)
        {
            auto* attribute = AZ::FindAttribute(AZ::Crc32(attributeName.c_str()), classData->m_attributes);
            if (attribute == nullptr)
            {
                return true; // attribute not found, keep iterating
            }

            AZ::AttributeReader reader(nullptr, attribute);
            T readData;
            if (reader.Read<T>(readData))
            {
                outputBuffer.insert(outputBuffer.end(), readData.begin(), readData.end());
            }

            return true; // attribute found, keep iterating to find hooks declared in other classes
        }

        // Timeout for loop waiting for assets to be built
        static constexpr AZStd::chrono::seconds assetLoopTimeout = AZStd::chrono::seconds(30);

        // Cache for storing sensor importer hooks (read only once)
        SDFormat::SensorImporterHooksStorage m_sensorHooks;

        // Cache for storing model plugin importer hooks (read only once)
        SDFormat::ModelPluginImporterHooksStorage m_modelPluginHooks;
    };
} // namespace ROS2
