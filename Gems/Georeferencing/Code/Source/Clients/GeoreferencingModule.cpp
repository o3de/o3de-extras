
#include <Georeferencing/GeoreferencingTypeIds.h>
#include <GeoreferencingModuleInterface.h>
#include "GeoreferencingSystemComponent.h"

namespace Georeferencing
{
    class GeoreferencingModule
        : public GeoreferencingModuleInterface
    {
    public:
        AZ_RTTI(GeoreferencingModule, GeoreferencingModuleTypeId, GeoreferencingModuleInterface);
        AZ_CLASS_ALLOCATOR(GeoreferencingModule, AZ::SystemAllocator);
    };
}// namespace Georeferencing

#if defined(O3DE_GEM_NAME)
AZ_DECLARE_MODULE_CLASS(AZ_JOIN(Gem_, O3DE_GEM_NAME), Georeferencing::GeoreferencingModule)
#else
AZ_DECLARE_MODULE_CLASS(Gem_Georeferencing, Georeferencing::GeoreferencingModule)
#endif
