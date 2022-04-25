/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <Atom/RPI.Public/XR/XRFactory.h>

namespace AZ
{
    namespace RPI
    {
        namespace XR
        {
            //! Registers the global factory instance.
            void Factory::Register(Factory* /*instance*/)
            {
            }

            //! Unregisters the global factory instance.
            void Factory::Unregister(Factory* /*instance*/)
            {
            }

            //! Access the global factory instance.
            Factory& Factory::Get()
            {
                return;
            }
        } // namespace XR
    } // namespace RPI
} // namespace AZ
