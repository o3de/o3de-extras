/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

#include <AzCore/std/string/string.h>
namespace SimulationInterfaces::Utils
{
    //! Convert a relative path to a URI
    //! relative path: "path/to/file.txt"
    //! URI: "product_asset:///path/to/file.txt"
    AZStd::string RelPathToUri(AZStd::string_view relPath);
    AZStd::string UriToRelPath(AZStd::string_view relPath);

} // namespace SimulationInterfaces::Utils
