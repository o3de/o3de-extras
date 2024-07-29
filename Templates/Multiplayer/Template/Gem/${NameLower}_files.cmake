
set(FILES
    Source/Components/AttachPlayerWeaponComponent.h
    Source/Components/AttachPlayerWeaponComponent.cpp
    Source/Components/ExampleFilteredEntityComponent.h
    Source/Components/ExampleFilteredEntityComponent.cpp
    Source/Components/GameplayEffectsComponent.cpp
    Source/Components/GameplayEffectsComponent.h
    Source/Components/NetworkAiComponent.cpp
    Source/Components/NetworkAiComponent.h
    Source/Components/NetworkAnimationComponent.cpp
    Source/Components/NetworkAnimationComponent.h
    Source/Components/NetworkHealthComponent.cpp
    Source/Components/NetworkHealthComponent.h
    Source/Components/NetworkRandomComponent.cpp
    Source/Components/NetworkRandomComponent.h
    Source/Components/NetworkWeaponsComponent.cpp
    Source/Components/NetworkWeaponsComponent.h
    Source/Components/NetworkSimplePlayerCameraComponent.cpp
    Source/Components/NetworkSimplePlayerCameraComponent.h
    Source/Components/NetworkPlayerMovementComponent.cpp
    Source/Components/NetworkPlayerMovementComponent.h
    Source/Components/NetworkPrefabSpawnerComponent.cpp
    Source/Components/NetworkPrefabSpawnerComponent.h

    Source/Effects/GameEffect.cpp
    Source/Effects/GameEffect.h

    Source/Weapons/BaseWeapon.cpp
    Source/Weapons/BaseWeapon.h
    Source/Weapons/IWeapon.h
    Source/Weapons/ProjectileWeapon.cpp
    Source/Weapons/ProjectileWeapon.h
    Source/Weapons/TraceWeapon.cpp
    Source/Weapons/TraceWeapon.h
    Source/Weapons/WeaponGathers.cpp
    Source/Weapons/WeaponGathers.h
    Source/Weapons/WeaponTypes.cpp
    Source/Weapons/WeaponTypes.h
    Source/Weapons/SceneQuery.cpp
    Source/Weapons/SceneQuery.h

    Include/${Name}/${Name}Bus.h
    Include/${Name}/${Name}TypeIds.h
    Include/${Name}/GameplayEffectsNotificationBus.h
    Include/${Name}/NetworkPrefabSpawnerInterface.h

    Source/${Name}SystemComponent.cpp
    Source/${Name}SystemComponent.h
    Source/${Name}Types.h
)
