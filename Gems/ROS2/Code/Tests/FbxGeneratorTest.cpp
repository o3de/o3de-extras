/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <URDF/FbxGenerator.h>

#include <AzCore/UnitTest/TestTypes.h>
#include <AzTest/AzTest.h>

namespace UnitTest
{

    using namespace ROS2::Fbx;

    class FbxGeneratorTest : public AllocatorsTestFixture
    {
    public:
        void PrintFbxContent(const AZStd::string& str)
        {
            std::cout << __func__ << " fbx data:"
                      << "\n---------------\n"
                      << str.c_str() << "\n---------------\n";
        }
    };

    TEST_F(FbxGeneratorTest, BasicStructureGeneration)
    {
        FbxGenerator generator;

        const auto fbxStr = generator.GetFbxString();

        std::istringstream iss(fbxStr.data());
        std::string line;

        std::getline(iss, line);
        EXPECT_EQ(line, "FBXHeaderExtension:  {");

        std::getline(iss, line);
        EXPECT_EQ(line, "  FBXHeaderVersion: 1003");

        std::getline(iss, line);
        EXPECT_EQ(line, "  FBXVersion: 7500");

        std::getline(iss, line);
        EXPECT_EQ(line, "  CreationTimeStamp:  {");
    }

    TEST_F(FbxGeneratorTest, AddModelAndMaterial)
    {
        FbxGenerator generator;

        // Add material
        Color color(0.0, 0.0, 0.0);
        const auto materialId = generator.AddMaterial("black", color);

        // Add cube object with specific material
        const double cubeSize = 1.0; // m
        const auto cubeId = generator.AddCubeObject("cube", cubeSize, materialId);

        EXPECT_EQ(static_cast<int>(cubeId), 2);

        const auto fbxStr = generator.GetFbxString();
        PrintFbxContent(fbxStr);
    }

} // namespace UnitTest
