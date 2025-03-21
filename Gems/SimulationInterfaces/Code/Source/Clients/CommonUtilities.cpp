/*
* Copyright (c) Contributors to the Open 3D Engine Project.
* For complete copyright and license terms please see the LICENSE at the root of this distribution.
*
* SPDX-License-Identifier: Apache-2.0 OR MIT
*
 */

#include "CommonUtilities.h"

namespace SimulationInterfaces::Utils
{
    const char* const ProductAssetPrefix = "product_asset:///";
    AZStd::string RelPathToUri(AZStd::string_view relPath)
    {
        AZStd::string uri = relPath;
        AZStd::replace(uri.begin(), uri.end(), '\\', '/');
        uri.insert(0, ProductAssetPrefix);
        return uri;
    }

    AZStd::string UriToRelPath(AZStd::string_view uri)
    {

        if (uri.starts_with(ProductAssetPrefix))
        {
            const AZStd::string_view productAssetPrefix{ ProductAssetPrefix };
            return uri.substr(productAssetPrefix.length());

        }
        return {};
    }
}