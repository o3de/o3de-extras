/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <AzTest/AzTest.h>

#include "OpemXRVkTest.h"

#if !O3DE_TRAIT_DISABLE_ALL_OPENXRVK_TESTS

void OpenXRVkTestFixture::SetUp()
{

}

void OpenXRVkTestFixture::TearDown()
{

}

#if O3DE_TRAIT_DISABLE_FAILED_OPENXRVK_TESTS
TEST_F(OpenXRVkTestFixture, DISABLED_ExpectTrue)
#else
TEST_F(OpenXRVkTestFixture, ExpectTrue)
#endif // O3DE_TRAIT_DISABLE_FAILED_OPENXRVK_TESTS
{
    EXPECT_TRUE(false);
}

#endif // !O3DE_TRAIT_DISABLE_ALL_OPENXRVK_TESTS

AZ_UNIT_TEST_HOOK(DEFAULT_UNIT_TEST_ENV);
