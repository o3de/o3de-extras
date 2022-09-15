/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzTest/AzTest.h>
#include "XRTest.h"

#if !AZ_TRAIT_DISABLE_ALL_XR_TESTS

void XRTest::SetUp()
{
    SetupInternal();
}

void XRTest::TearDown()
{
    TearDownInternal();
}

TEST_F(XRTest, PassThisTest)
{
    EXPECT_TRUE(true);
}

#endif // !AZ_TRAIT_DISABLE_ALL_XR_TESTS

AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
