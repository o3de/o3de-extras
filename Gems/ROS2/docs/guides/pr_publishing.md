# Publishing a pull request

## Overview
Make sure to read this guide and adjust your code accordingly before publishing a pull request.

The repository performs 3 pull request checks:

- License header check
- Code formatting check
- Build check

## License Header check

Each file in the Code directory (and CMakeLists.txt located at root)

should contain the license header (in form of a comment) provided below (you can also find it in the *[.licenserc.yaml](https://github.com/RobotecAI/o3de-ros2-gem/blob/development/.licenserc.yaml)* file):

```
Copyright (c) Contributors to the Open 3D Engine Project.
For complete copyright and license terms please see the LICENSE at the root of this distribution.

SPDX-License-Identifier: Apache-2.0 OR MIT
```

## Code formatting check

All of the *C++* files located in the Code directory **should be formatted** using clang-format 

(version **13**) with the *[.clang-format](https://github.com/RobotecAI/o3de-ros2-gem/blob/development/.clang-format)* file located in the root of the repository.

💡 ***Note:*** If you plan on using Clion follow the Code Formatting section in the [Development in Clion](docs/static/development_in_clion.md) guide.

## Build check

New changes in the O3DE engine API may require the build workflow to be configured. This can be achieved by adjusting the ***O3DE_SHA*** environment variable in the [Build.yml](https://github.com/RobotecAI/o3de-ros2-gem/blob/development/.github/workflows/Build.yml) file (located in the *.github/workflows* directory) to a different o3de **long format** commit hash.

## Additional requirements

Remember to sign all of your commits (*-s flag*).