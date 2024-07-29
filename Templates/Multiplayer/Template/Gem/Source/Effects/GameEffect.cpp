/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Source/Effects/GameEffect.h>
#include <AzCore/Console/IConsole.h>


namespace ${SanitizedCppName}
{
    AZ_CVAR(bool, cl_KillEffectOnRestart, false, nullptr, AZ::ConsoleFunctorFlags::Null, "Controls whether to kill or terminate current effects on restart");

    void GameEffect::Reflect(AZ::ReflectContext* context)
    {
        AZ::SerializeContext* serializeContext = azrtti_cast<AZ::SerializeContext*>(context);
        if (serializeContext)
        {
            serializeContext->Class<GameEffect>()
                ->Version(1)
                ->Field("ParticleAsset", &GameEffect::m_particleAssetId)
                ->Field("AudioTrigger", &GameEffect::m_audioTrigger)
                ->Field("EffectOffset", &GameEffect::m_effectOffset);

            AZ::EditContext* editContext = serializeContext->GetEditContext();
            if (editContext)
            {
                editContext->Class<GameEffect>("GameEffect", "A single game effect, consisting of a particle effect and a sound trigger pair")
                    ->ClassElement(AZ::Edit::ClassElements::EditorData, "")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GameEffect::m_particleAssetId, "ParticleAsset", "The particle effect to play upon effect trigger")
                    // O3DE doesn't come with a built-in particle fx system. Filter based on your game's particular particle fx asset type.
                    //#if AZ_TRAIT_CLIENT
                    //    ->Attribute(AZ_CRC_CE("SupportedAssetTypes"), []() { return AZStd::vector<AZ::Data::AssetType>({ MyParticleFx::AssetTypeId }); })
                    //#endif
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GameEffect::m_audioTrigger, "AudioTrigger", "The audio trigger name of the sound to play upon effect trigger")
                    ->DataElement(AZ::Edit::UIHandlers::Default, &GameEffect::m_effectOffset, "EffectOffset", "The offset to apply when triggering an effect");
            }
        }
    }

    GameEffect::~GameEffect()
    {
        Destroy();
    }

    GameEffect::GameEffect(const GameEffect& gameEffect)
    {
        *this = gameEffect;
    }

    GameEffect& GameEffect::operator=(const GameEffect& gameEffect)
    {
        // Make sure the current emitter is destroyed before copying new settings over this one.
        Destroy();

        // Only copy the effect settings, but leave it in an uninitialized state. Each GameEffect instance should
        // have its own emitter to manipulate and move around.
        m_particleAssetId = gameEffect.m_particleAssetId;
        m_audioTrigger = gameEffect.m_audioTrigger;
        m_effectOffset = gameEffect.m_effectOffset;

        return *this;
    }

    void GameEffect::Destroy()
    {
#if AZ_TRAIT_CLIENT
        // @todo O3DE doesn't come with a built-in particle fx system.
        // Clean up particle fx here...
        
        // Clean up audio
        if (m_audioSystem && m_audioProxy)
        {
            m_audioSystem->RecycleAudioProxy(m_audioProxy);
        }

        // Clear all of these out so that we know we need to call Initialize() again.
        m_audioSystem = nullptr;
        m_audioProxy = nullptr;
        m_audioTriggerId = INVALID_AUDIO_CONTROL_ID;
#endif
    }

    void GameEffect::Initialize([[maybe_unused]] EmitterType emitterType)
    {
#if AZ_TRAIT_CLIENT
        AZ_Assert(!IsInitialized(), "Destroy() needs to be called before calling Initialize() for a second time.");
        if (IsInitialized())
        {
            return;
        }

        m_audioSystem = AZ::Interface<Audio::IAudioSystem>::Get();
        m_emitterType = emitterType;

        // @todo O3DE doesn't come with a built-in particle fx system.
        // Warm up or load any particle effects here...

        // Audio
        if (m_audioSystem != nullptr)
        {
            m_audioProxy = m_audioSystem->GetAudioProxy();
            m_audioProxy->Initialize(m_audioTrigger.c_str(), this);
            m_audioProxy->SetObstructionCalcType(Audio::ObstructionType::Ignore);
            m_audioTriggerId = m_audioSystem->GetAudioTriggerID(m_audioTrigger.c_str());
        }
#endif
    }

    bool GameEffect::IsInitialized() const
    {
#if AZ_TRAIT_CLIENT
        return false; // @todo return true if the particle fx was successfully initialized
#else
        return true;
#endif
    }

    bool GameEffect::SetAttribute([[maybe_unused]] const char* attributeName, [[maybe_unused]] float value) const
    {
#if AZ_TRAIT_CLIENT
        AZ_Assert(m_emitterType == EmitterType::ReusableEmitter, "SetAttribute only supports reusable emitters.");
        // @todo O3DE doesn't come with a built
        // Set particle fx emitter attributes here and return true if successful...

#endif
        return false;
    }

    bool GameEffect::SetAttribute([[maybe_unused]] const char* attributeName, [[maybe_unused]] const AZ::Vector2& value) const
    {
#if AZ_TRAIT_CLIENT
        AZ_Assert(m_emitterType == EmitterType::ReusableEmitter, "SetAttribute only supports reusable emitters.");
        // @todo O3DE doesn't come with a built
        // Set particle fx emitter attributes here and return true if successful...

#endif
        return false;
    }

    bool GameEffect::SetAttribute([[maybe_unused]] const char* attributeName, [[maybe_unused]] const AZ::Vector3& value) const
    {
#if AZ_TRAIT_CLIENT
        AZ_Assert(m_emitterType == EmitterType::ReusableEmitter, "SetAttribute only supports reusable emitters.");
        // @todo O3DE doesn't come with a built
        // Set particle fx emitter attributes here and return true if successful...

#endif
        return false;
    }

    bool GameEffect::SetAttribute([[maybe_unused]] const char* attributeName, [[maybe_unused]] const AZ::Vector4& value) const
    {
#if AZ_TRAIT_CLIENT
        AZ_Assert(m_emitterType == EmitterType::ReusableEmitter, "SetAttribute only supports reusable emitters.");
        // @todo O3DE doesn't come with a built
        // Set particle fx emitter attributes here and return true if successful...
#endif
        return false;
    }

    void GameEffect::TriggerEffect([[maybe_unused]] const AZ::Transform& transform) const
    {
#if AZ_TRAIT_CLIENT
        const AZ::Vector3 offsetPosition = transform.TransformPoint(m_effectOffset);

        // @todo O3DE doesn't come with a built
        // Trigger the particle fx here...


        // Trigger audio
        if ((m_audioProxy != nullptr) && (m_audioTriggerId != INVALID_AUDIO_CONTROL_ID))
        {
            m_audioProxy->SetPosition(offsetPosition);
            m_audioProxy->ExecuteTrigger(m_audioTriggerId);
        }
#endif
    }

    void GameEffect::StopEffect() const
    {
        #if AZ_TRAIT_CLIENT
            // @todo O3DE doesn't come with a built
            // Stop any particle fx here...

        #endif
    }

    const AZ::Vector3& GameEffect::GetEffectOffset() const
    {
        return m_effectOffset;
    }
}
