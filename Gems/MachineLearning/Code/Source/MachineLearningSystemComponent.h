/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/Component.h>
#include <MachineLearning/IMachineLearning.h>

namespace MachineLearning
{
    class MachineLearningSystemComponent
        : public AZ::Component
        , protected MachineLearningRequestBus::Handler
//        , public AZ::Interface<IMachineLearning>::Registrar
    {
    public:
        AZ_COMPONENT_DECL(MachineLearningSystemComponent);

        static void Reflect(AZ::ReflectContext* context);

        static void GetProvidedServices(AZ::ComponentDescriptor::DependencyArrayType& provided);
        static void GetIncompatibleServices(AZ::ComponentDescriptor::DependencyArrayType& incompatible);
        static void GetRequiredServices(AZ::ComponentDescriptor::DependencyArrayType& required);
        static void GetDependentServices(AZ::ComponentDescriptor::DependencyArrayType& dependent);

        MachineLearningSystemComponent();
        ~MachineLearningSystemComponent();

    protected:

        //! AZ::Component interface
        //! @{
        void Init() override;
        void Activate() override;
        void Deactivate() override;
        //! @}

        //! IMachineLearning interface
        //! @{
        void RegisterModel(INeuralNetworkPtr model) override;
        void UnregisterModel(INeuralNetworkPtr model) override;
        ModelSet& GetModelSet() override;
        //! @}

    private:

        ModelSet m_registeredModels;
    };
}
