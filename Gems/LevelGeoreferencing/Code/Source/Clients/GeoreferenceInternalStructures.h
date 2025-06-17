/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root
 * of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once
#include <AzCore/Math/Vector3.h>

namespace Georeferencing::WGS
{
    //! Vector3d is a 3D vector with double precision.
    //! It is used to represent coordinates in ECEF or ENU coordinate systems.
    struct Vector3d
    {
        Vector3d() = default;
        Vector3d(double x, double y, double z);
        explicit Vector3d(const AZ::Vector3& xyz);
        [[nodiscard]] AZ::Vector3 ToVector3f() const;
        Vector3d operator+(Vector3d const& v) const;
        Vector3d operator-(Vector3d const& v) const;
        double m_x = 0.0; //!< X coordinate in meters.
        double m_y = 0.0; //! Y coordinate in meters.
        double m_z = 0.0; //! Z coordinate in meters.
    };
} // namespace Georeferencing::WGS
