/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <XR/XRUtils.h>

namespace XR
{
    AZ::Matrix4x4 CreateStereoscopicProjection(float angleLeft, float angleRight, float angleBottom, float angleTop, float nearDist, float farDist, bool reverseDepth)
    {
        AZ::Matrix4x4 result;
        AZ_MATH_ASSERT(nearDist > 0.0f, "Near plane distance must be greater than 0");
        
        const float left = tanf(angleLeft);
        const float right = tanf(angleRight);
        const float bottom = tanf(angleBottom);
        const float top = tanf(angleTop);

        const float invfn = 1.0f / (farDist - nearDist);
        const float tanAngleWidth = right - left;
        const float tanAngleHeight = (top - bottom);

        //Stretch more horizontally and vertically
        //Generate asymmetric or off-center projection matrix
        //Right handed coord system as Openxr provides data in that system
        //Handle the case where farDist is less than nearDist
        if(farDist > nearDist)
        { 
            result.SetRow(0, 2.0f / tanAngleWidth, 0.0f, (left + right) / tanAngleWidth, 0.0f);
            result.SetRow(1, 0.0f, 2.0f / tanAngleHeight, (top + bottom) / tanAngleHeight, 0.0f);
            if(reverseDepth)
            { 
                result.SetRow(2, 0.0f, 0.0f, 2.0f * nearDist * invfn, 2.0f * farDist * nearDist * invfn);
            }
            else
            {
                result.SetRow(2, 0.0f, 0.0f, -1.0f * (farDist + nearDist) * invfn, -2.0f * farDist * nearDist * invfn);
            }
            result.SetRow(3, 0.0f, 0.0f, -1.0f, 0.0f);
        }
        else
        {
            // place the far plane at infinity
            result.SetRow(0, 2.0f / tanAngleWidth, 0.0f, (left + right) / tanAngleWidth, 0.0f);
            result.SetRow(1, 0.0f, 2.0f / tanAngleHeight, (top + bottom) / tanAngleHeight, 0.0f);
            if (reverseDepth)
            {
                result.SetRow(2, 0.0f, 0.0f, 0.0f, 2.0f * nearDist);
            }
            else
            {
                result.SetRow(2, 0.0f, 0.0f, -1.0f, -2.0f * nearDist);
            }
            result.SetRow(3, 0.0f, 0.0f, -1.0f, 0.0f);
        }

        return result;
    }
}
