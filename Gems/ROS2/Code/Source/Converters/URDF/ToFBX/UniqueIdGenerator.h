/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#pragma once

namespace ROS2
{
    using Id = int;

    //! Generate unique, positive id
    class UniqueIdGenerator
    {
    public:
        static Id GetUniqueId()
        {
            m_counter++;
            return static_cast<Id>(m_counter);
        }

        static void Reset()
        {
            m_counter = 0;
        }

    private:
        inline static int m_counter = 0;
    };
} // namespace ROS2