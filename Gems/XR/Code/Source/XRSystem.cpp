/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzCore/Interface/Interface.h>
#include <AzCore/Debug/Profiler.h>
#include <AzCore/Console/IConsole.h>
#include <AzCore/Settings/SettingsRegistry.h>
#include <XR/XRFactory.h>
#include <XR/XRSystem.h>
#include <XR/XRUtils.h>
#include <Atom/RHI/Image.h>
#include <Atom/RHI/ImagePool.h>
#include <Atom/RHI/RHISystemInterface.h>
#include <Atom/RHI.Reflect/VariableRateShadingEnums.h>
#include <Atom/RHI.Reflect/Viewport.h>
#include <Atom/RPI.Public/Image/AttachmentImage.h>
#include <Atom/RPI.Reflect/Asset/AssetUtils.h>
#include <Atom/RPI.Reflect/Image/AttachmentImageAsset.h>
#include <Atom/RPI.Reflect/Image/AttachmentImageAssetCreator.h>
#include <Atom/RPI.Reflect/Pass/PassTemplate.h>


namespace XR
{
#if AZ_TRAIT_OS_IS_HOST_OS_PLATFORM
    AZ_CVAR(
        bool,
        r_EnableHostRenderPipelineOnXR,
        true,
        nullptr,
        AZ::ConsoleFunctorFlags::Null,
        "When an XR system is present in a host platform, this will enable the regular render pipeline on the host PC as well "
        "(true by default).");
#endif

    void System::Init(const System::Descriptor& descriptor)
    {
        m_validationMode = descriptor.m_validationMode;
        AZ::SystemTickBus::Handler::BusConnect();
    }

    AZ::RHI::ResultCode System::InitInstance()
    {
        if (GetInstance())
        {
            return GetInstance()->Init(m_validationMode);
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::u32 System::GetNumPhysicalDevices() const
    {
        return m_instance->GetNumPhysicalDevices();
    }

    AZ::RHI::ResultCode System::GetXRPhysicalDevice(AZ::RHI::XRPhysicalDeviceDescriptor* physicalDeviceDescriptor, int32_t index)
    {
        AZ_Error("XR", physicalDeviceDescriptor, "The descriptor is null");
        if (physicalDeviceDescriptor)
        {
            return GetInstance()->GetXRPhysicalDevice(physicalDeviceDescriptor, index);
        }

        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode System::CreateDevice(AZ::RHI::XRDeviceDescriptor* instanceDescriptor)
    {
        if (!m_device)
        {
            m_device = Factory::Get().CreateDevice();
            AZ_Assert(m_device, "XR Device not created");
            if (m_device->Init(Device::Descriptor{ m_validationMode, GetInstance()}) == AZ::RHI::ResultCode::Success)
            {
                return m_device->InitDeviceInternal(instanceDescriptor);
            }
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode System::CreateSession(AZ::RHI::XRSessionDescriptor* sessionDescriptor)
    {
        if (!m_session)
        {
            m_session = Factory::Get().CreateSession();
            AZ_Assert(m_session, "Session not created");
            AZ::RHI::ResultCode result = m_session->Init(Session::Descriptor{ m_validationMode, m_device, GetInstance() });
            if (result == AZ::RHI::ResultCode::Success)
            {
                return m_session->InitInternal(sessionDescriptor);
            }
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode System::CreateSwapChain()
    {
        if (!m_swapChain)
        {
            m_swapChain = Factory::Get().CreateSwapChain();
            AZ_Assert(m_swapChain, "XR SwapChain not created");
            return m_swapChain->Init(SwapChain::Descriptor{ m_validationMode, GetInstance(), m_session, m_device });
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::RHI::ResultCode System::GetSwapChainImage(AZ::RHI::XRSwapChainDescriptor* swapchainDescriptor) const
    {
        AZ_Assert(m_swapChain, "SwapChain is null");
        if (m_swapChain)
        {
            return m_swapChain->GetSwapChainImage(swapchainDescriptor);
        }
        return AZ::RHI::ResultCode::Fail;
    }

    AZ::u32 System::GetSwapChainWidth(AZ::u32 viewIndex) const
    {
        AZ_Assert(m_swapChain, "SwapChain is null");
        if (m_swapChain)
        {
            return m_swapChain->GetSwapChainWidth(viewIndex);
        }
        return 0;
    }

    AZ::u32 System::GetSwapChainHeight(AZ::u32 viewIndex) const
    {
        AZ_Assert(m_swapChain, "SwapChain is null");
        if (m_swapChain)
        {
            return m_swapChain->GetSwapChainHeight(viewIndex);
        }
        return 0;
    }

    AZ::RHI::Format System::GetSwapChainFormat(AZ::u32 viewIndex) const
    {
        AZ_Assert(m_swapChain, "SwapChain is null");
        if (m_swapChain)
        {
            return m_swapChain->GetSwapChainFormat(viewIndex);
        }
        return AZ::RHI::Format::Unknown;
    }

    void System::OnSystemTick()
    {
        if (m_session)
        {
            m_session->PollEvents();
        }
    }
    
    void System::BeginFrame()
    {
        if (m_device && m_session && m_session->IsSessionRunning())
        {
            m_isInFrame = m_device->BeginFrame();
        }
    }

    void System::EndFrame()
    {
        if (m_isInFrame)
        {
            m_device->EndFrame(m_swapChain);
            m_isInFrame = false;
        }
    }

    void System::PostFrame()
    {
        if (m_device && m_session && m_session->IsSessionRunning())
        {
            m_device->PostFrame();
        }
    }

    void System::AcquireSwapChainImage(AZ::u32 viewIndex)
    {
        if (m_isInFrame && m_device->ShouldRender())
        {
            m_device->AcquireSwapChainImage(viewIndex, m_swapChain.get());
        }
    }

    AZ::u32 System::GetNumViews() const
    {
        if (m_swapChain)
        {
            return m_swapChain->GetNumViews();
        }

        AZ_Warning("XRSystem", false, "SwapChain is null");
        return 0;
    }

    AZ::u32 System::GetCurrentImageIndex(AZ::u32 viewIndex) const
    {
        SwapChain::View* viewSwapchain = m_swapChain->GetView(viewIndex);
        return viewSwapchain->m_activeImageIndex;
    }

    bool System::ShouldRender() const
    {
        if (m_session && m_session->IsSessionRunning())
        { 
            return m_device->ShouldRender();
        }
        return false;
    }

    AZ::RHI::ResultCode System::GetViewFov(AZ::u32 viewIndex, AZ::RPI::FovData& outFovData) const
    {
        return m_device->GetViewFov(viewIndex, outFovData);
    }

    AZ::RHI::ResultCode System::GetViewPose(AZ::u32 viewIndex, AZ::RPI::PoseData& outPoseData) const
    {
        return m_device->GetViewPose(viewIndex, outPoseData);
    }

    AZ::RHI::ResultCode System::GetControllerPose(AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetControllerPose(handIndex, outPoseData);
        }
        return AZ::RHI::ResultCode::NotReady;
    }

    AZ::RHI::ResultCode System::GetControllerStagePose(AZ::u32 handIndex, AZ::RPI::PoseData& outPoseData) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetControllerStagePose(handIndex, outPoseData);
        }
        return AZ::RHI::ResultCode::NotReady;
    }

    AZ::RHI::ResultCode System::GetViewFrontPose(AZ::RPI::PoseData& outPoseData) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetViewFrontPose(outPoseData);
        }
        return AZ::RHI::ResultCode::NotReady;
    }

    AZ::RHI::ResultCode System::GetViewLocalPose(AZ::RPI::PoseData& outPoseData) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetViewLocalPose(outPoseData);
        }
        return AZ::RHI::ResultCode::NotReady;
    }

    float System::GetControllerScale(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetControllerScale(handIndex);
        }
        return 1.0f;
    }

    float System::GetSqueezeState(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetSqueezeState(handIndex);
        }
        return 0.0f;
    }

    float System::GetTriggerState(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetTriggerState(handIndex);
        }
        return 0.0f;
    }

    AZ::Data::Instance<AZ::RPI::AttachmentImage> System::InitPassFoveatedAttachment(const AZ::RPI::PassTemplate& passTemplate, const AZ::RHI::XRFoveatedLevel* level) const
    {
        // Need to fill the contents of the Variable shade rating image.
        // Find the Shading Rate Attachment
        AZ::Data::Asset<AZ::RPI::AttachmentImageAsset> vrsImageAsset;
        for (const auto& imageAttachment : passTemplate.m_imageAttachments)
        {
            if (AZ::RHI::CheckBitsAll(imageAttachment.m_imageDescriptor.m_bindFlags, AZ::RHI::ImageBindFlags::ShadingRate))
            {
                vrsImageAsset = AZ::RPI::AssetUtils::LoadAssetById<AZ::RPI::AttachmentImageAsset>(imageAttachment.m_assetRef.m_assetId, AZ::RPI::AssetUtils::TraceLevel::Error);
                break;
            }
        }

        AZ::Data::Instance<AZ::RPI::AttachmentImage> textureAsset;
        if (vrsImageAsset && vrsImageAsset.IsReady())
        {
            AZ::RHI::Device* device = AZ::RHI::RHISystemInterface::Get()->GetDevice();
            // Resize the image to match the proper tile size
            AZ::u32 outputWidth = GetSwapChainWidth(1);
            AZ::u32 outputHeight = GetSwapChainHeight(1);

            const auto& tileSize = device->GetLimits().m_shadingRateTileSize;
            AZ::RHI::ImageDescriptor imageDescriptor = vrsImageAsset->GetImageDescriptor();
            imageDescriptor.m_size.m_width = aznumeric_cast<uint32_t>(ceil(static_cast<float>(outputWidth) / tileSize.m_width));
            imageDescriptor.m_size.m_height = aznumeric_cast<uint32_t>(ceil(static_cast<float>(outputHeight) / tileSize.m_height));

            // Find the appropriate format for the image
            for (uint32_t i = 0; i < static_cast<uint32_t>(AZ::RHI::Format::Count); ++i)
            {
                AZ::RHI::Format format = static_cast<AZ::RHI::Format>(i);
                AZ::RHI::FormatCapabilities capabilities = device->GetFormatCapabilities(format);
                if (AZ::RHI::CheckBitsAll(capabilities, AZ::RHI::FormatCapabilities::ShadingRate))
                {
                    imageDescriptor.m_format = format;
                    break;
                }
            }

            // Create the new asset with the proper size and format and register it in the AZ::RPI::AttachmentImage database
            AZ::RPI::AttachmentImageAssetCreator imageAssetCreator;
            imageAssetCreator.Begin(vrsImageAsset->GetId());
            imageAssetCreator.SetImageDescriptor(imageDescriptor);
            imageAssetCreator.SetName(vrsImageAsset->GetName(), vrsImageAsset->HasUniqueName());

            AZ::Data::Asset<AZ::RPI::AttachmentImageAsset> asset;
            if (imageAssetCreator.End(asset))
            {
                textureAsset = AZ::RPI::AttachmentImage::FindOrCreate(asset);
                AZ::RHI::XRFoveatedLevel foveatedType = AZ::RHI::XRFoveatedLevel::None;
                if (level)
                {
                    foveatedType = *level;
                }
                else
                {
                    // Check settings registry for the foveated level
                    if (AZ::SettingsRegistryInterface* settingsRegistry = AZ::SettingsRegistry::Get())
                    {
                        if (AZ::u64 foveatedLevel; settingsRegistry->Get(foveatedLevel, AZ::RHI::XRFoveatedLevelKey))
                        {
                            AZ_Assert(
                                foveatedLevel <= static_cast<AZ::u64>(AZ::RHI::XRFoveatedLevel::High),
                                "Invalid foveated level %d", static_cast<int>(foveatedLevel));
                            foveatedType = static_cast<AZ::RHI::XRFoveatedLevel>(foveatedLevel);
                        }
                    }
                }
                // Fill up the contents of the shading rate image
                InitVariableRateShadingImageContent(textureAsset->GetRHIImage(), foveatedType);
            }
        }

        return textureAsset;
    }

    float System::GetXButtonState() const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetXButtonState();
        }
        return 0.0f;
    }

    float System::GetYButtonState() const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetYButtonState();
        }
        return 0.0f;
    }

    float System::GetAButtonState() const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetAButtonState();
        }
        return 0.0f;
    }

    float System::GetBButtonState() const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetBButtonState();
        }
        return 0.0f;
    }

    float System::GetXJoyStickState(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetXJoyStickState(handIndex);
        }
        return 0.0f;
    }

    float System::GetYJoyStickState(AZ::u32 handIndex) const
    {
        if (m_session->IsSessionRunning())
        {
            return m_session->GetYJoyStickState(handIndex);
        }
        return 0.0f;
    }

    AZ::Matrix4x4 System::CreateStereoscopicProjection(float angleLeft, float angleRight,
                                                 float angleBottom, float angleTop, 
                                                 float nearDist, float farDist, bool reverseDepth)
    {
        return XR::CreateStereoscopicProjection(angleLeft, angleRight, angleBottom, angleTop, nearDist, farDist, reverseDepth);
    }

    AZ::RHI::XRRenderingInterface* System::GetRHIXRRenderingInterface()
    {
        return this;
    }

    void System::Shutdown()
    {
        AZ::SystemTickBus::Handler::BusDisconnect();
        m_instance = nullptr;
        m_device = nullptr;
    }

    bool System::IsDefaultRenderPipelineNeeded() const
    {
        // While there is an XR system the default render pipeline is only needed on host platforms,
        // in case we also render on PC as well as in the XR device.
#if AZ_TRAIT_OS_IS_HOST_OS_PLATFORM
        return true;
#else
        return false;
#endif
    }

    bool System::IsDefaultRenderPipelineEnabledOnHost() const
    {
#if AZ_TRAIT_OS_IS_HOST_OS_PLATFORM
        return r_EnableHostRenderPipelineOnXR;
#else
        return false;
#endif
    }

    AZ::RHI::ResultCode System::InitVariableRateShadingImageContent(AZ::RHI::Image* image, AZ::RHI::XRFoveatedLevel level) const
    {
        AZ_Assert(image, "Null variable rate shading image");
        const auto& imageDescriptor = image->GetDescriptor();
        uint32_t width = imageDescriptor.m_size.m_width;
        uint32_t height = imageDescriptor.m_size.m_height;
        uint32_t formatSize = GetFormatSize(imageDescriptor.m_format);
        uint32_t bufferSize = width * height * formatSize;

        // Get a list of supported shading rates so we always write a valid one
        const AZ::RHI::Device& device = image->GetDevice();
        const auto& features = device.GetFeatures();
        AZ::RHI::ShadingRate supportedRates[static_cast<int>(AZ::RHI::ShadingRate::Count)];
        AZ::RHI::ShadingRate lastSupported = AZ::RHI::ShadingRate::Rate1x1;
        for (int i = 0; i < AZ_ARRAY_SIZE(supportedRates); ++i)
        {
            if (AZ::RHI::CheckBitsAll(features.m_shadingRateMask, static_cast<AZ::RHI::ShadingRateFlags>(AZ_BIT(i))))
            {
                supportedRates[i] = static_cast<AZ::RHI::ShadingRate>(i);
                lastSupported = supportedRates[i];
            }
            else
            {
                supportedRates[i] = lastSupported;
            }
        }

        // Divide the image in a grid of "gridSize" and each cell will have a shading rate.
        constexpr uint32_t gridSize = 8;
        AZStd::vector<uint8_t> shadingRatePatternData(bufferSize);
        AZ::RHI::ShadingRate rateGrid[gridSize][gridSize];
        // Initialize the whole image with the normal rate. FYI - memset only works with 0 (i.e Rate1x1).
        ::memset(rateGrid, static_cast<int>(AZ::RHI::ShadingRate::Rate1x1), sizeof(rateGrid));

        // Helper function to fill up the grid
        auto fillFunc = [&](AZ::RHI::ShadingRate rate, int col, int row, int colCount, int rowCount)
        {
            for (int i = col; i < col + colCount; ++i)
            {
                for (int ii = row; ii < row + rowCount; ++ii)
                {
                    rateGrid[i][ii] = supportedRates[static_cast<uint32_t>(rate)];
                    // The image is symmetric on the vertical axis
                    rateGrid[gridSize - 1 - i][ii] = supportedRates[static_cast<uint32_t>(rate)];
                }
            }
        };

        // Each level has it's own shading rates and regions
        switch (level)
        {
        case AZ::RHI::XRFoveatedLevel::Low:
        {
            //  _______________________________________________
            // |_4x4_|________________2x2________________|_4x4_|
            // |     | 1x2 |                       | 1x2 |     |
            // |     |_____|                       |_____|     |
            // |     |                                   |     |
            // | 2x2 |                1x1                | 2x2 |
            // |     |                                   |     |
            // |     |___________________________________|     |
            // |______________________2x2______________________|
            //
            fillFunc(AZ::RHI::ShadingRate::Rate4x4, 0, 0, 1, 1);
            fillFunc(AZ::RHI::ShadingRate::Rate2x2, 1, 0, 3, 1);
            fillFunc(AZ::RHI::ShadingRate::Rate2x2, 0, 1, 1, gridSize - 1);
            fillFunc(AZ::RHI::ShadingRate::Rate1x2, 1, 1, 1, 2);
            fillFunc(AZ::RHI::ShadingRate::Rate2x2, 1, gridSize - 1, 3, 1);
        }
        break;
        case AZ::RHI::XRFoveatedLevel::Medium:
        {
            //    1    2     3    4      5    6      7     8
            //  _______________________________________________
            // |______4x4_______|____2x2____|____4x4___________|    1
            // |     |     |                       |     |     |    2
            // |     |     |                       |     |     |    3
            // | 4x4 | 2x2 |                       | 2x2 | 4x4 |    4
            // |     |     |          1x1          |     |     |    5
            // |     |     |                       |     |     |    6
            // |_____|_____|_______________________|_____|_____|    7
            // |_______4x4______|____2x2____|____4x4___________|    8
            //
            fillFunc(AZ::RHI::ShadingRate::Rate4x4, 0, 0, 3, 1);
            fillFunc(AZ::RHI::ShadingRate::Rate4x4, 0, 1, 1, 6);
            fillFunc(AZ::RHI::ShadingRate::Rate4x4, 0, gridSize - 1, 3, 1);
            fillFunc(AZ::RHI::ShadingRate::Rate2x2, 1, 1, 1, 6);
            fillFunc(AZ::RHI::ShadingRate::Rate2x2, 3, 0, 1, 1);
            fillFunc(AZ::RHI::ShadingRate::Rate2x2, 3, gridSize - 1, 1, 1);
        }
        break;
        case AZ::RHI::XRFoveatedLevel::High:
        {
            //    1    2     3    4      5    6      7     8
            //  _______________________________________________
            // |______________________4x4 _____________________|    1
            // |     |     |     |           |     |     |     |    2
            // |     |     |     |           |     |     |     |    3
            // | 4x4 | 4x2 | 2x2 |           | 2x2 | 4x2 | 4x4 |    4
            // |     |     |     |    1x1    |     |     |     |    5
            // |     |     |     |           |     |     |     |    6
            // |_____|_____|_____|___________|_____|_____|_____|    7
            // |______________________4x4______________________|    8
            fillFunc(AZ::RHI::ShadingRate::Rate4x4, 0, 0, 4, 1);
            fillFunc(AZ::RHI::ShadingRate::Rate4x4, 0, 1, 1, 6);
            fillFunc(AZ::RHI::ShadingRate::Rate4x4, 0, gridSize - 1, 4, 1);
            fillFunc(AZ::RHI::ShadingRate::Rate4x2, 1, 1, 1, 6);
            fillFunc(AZ::RHI::ShadingRate::Rate2x2, 2, 1, 1, 6);
        }
        break;
        case AZ::RHI::XRFoveatedLevel::None:
            // Intentionally leave the rate grid with default values
            break;
        default:
            AZ_Assert(false, "Invalid AZ::RHI::XRFoveatedLevel value %d", level);
            return AZ::RHI::ResultCode::InvalidArgument;
        }

        uint8_t* ptrData = shadingRatePatternData.data();
        float widthRegion = width / static_cast<float>(gridSize);
        float heightRegion = height / static_cast<float>(gridSize);
        for (uint32_t y = 0; y < height; y++)
        {
            for (uint32_t x = 0; x < width; x++)
            {
                auto val = device.ConvertShadingRate(rateGrid[static_cast<uint32_t>(x / widthRegion)][static_cast<uint32_t>(y / heightRegion)]);
                ::memcpy(ptrData, &val, formatSize);
                ptrData += formatSize;
            }
        }

        AZ::RHI::ImageUpdateRequest request;
        request.m_image = image;
        request.m_sourceData = shadingRatePatternData.data();
        request.m_sourceSubresourceLayout =
            AZ::RHI::ImageSubresourceLayout(AZ::RHI::Size(width, height, 1), height, width * formatSize, bufferSize, 1, 1);

        AZ::RHI::ImagePool* imagePool = azrtti_cast<AZ::RHI::ImagePool*>(image->GetPool());
        return imagePool->UpdateImageContents(request);
    }

    Instance* System::GetInstance()
    {
        if (!m_instance)
        {
            m_instance = AZ::Interface<Instance>::Get();
        }
        return m_instance.get();
    }
}
