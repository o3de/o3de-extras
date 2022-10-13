/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/Component/ComponentBus.h>

namespace OpenXRVk
{
    class XRCameraMovementRequests
        : public AZ::ComponentBus
    {
    public:
        AZ_RTTI(OpenXRVk::XRCameraMovementRequests, "{90D22228-2EAB-44FE-A7A0-693983DABEC6}");

        // Put your public request methods here.
        
        // Put notification events here. Examples:
        // void RegisterEvent(AZ::EventHandler<...> notifyHandler);
        // AZ::Event<...> m_notifyEvent1;
        
    };

    using XRCameraMovementRequestBus = AZ::EBus<XRCameraMovementRequests>;

} // namespace OpenXRVk
