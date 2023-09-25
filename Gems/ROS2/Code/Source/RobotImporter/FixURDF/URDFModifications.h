
#pragma once
#include <AzCore/std/containers/vector.h>
#include <AzCore/std/string/string.h>

namespace ROS2::Utils
{
    struct MissingInertia
    {
        AZStd::string linkName;

        ~MissingInertia() = default;
    };

    struct IncompleteInertia
    {
        AZStd::string linkName;
        AZStd::vector<AZStd::string> missingTags;
        // AZStd::vector<AZStd::string> unnknownTags;
    };

    struct DuplicatedJoint
    {
        AZStd::string oldName;
        AZStd::string newName;
    };

    struct UrdfModifications
    {
        AZStd::vector<MissingInertia> missingInertias;
        AZStd::vector<IncompleteInertia> incompleteInertias;
        AZStd::vector<DuplicatedJoint> duplicatedJoints;
    };

} // namespace ROS2::Utils
