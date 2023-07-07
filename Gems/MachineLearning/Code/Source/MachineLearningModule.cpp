
#include <MachineLearning/MachineLearningTypeIds.h>
#include <MachineLearningModuleInterface.h>
#include "MachineLearningSystemComponent.h"

namespace MachineLearning
{
    class MachineLearningModule
        : public MachineLearningModuleInterface
    {
    public:
        AZ_RTTI(MachineLearningModule, MachineLearningModuleTypeId, MachineLearningModuleInterface);
        AZ_CLASS_ALLOCATOR(MachineLearningModule, AZ::SystemAllocator);
    };
}// namespace MachineLearning

AZ_DECLARE_MODULE_CLASS(Gem_MachineLearning, MachineLearning::MachineLearningModule)
