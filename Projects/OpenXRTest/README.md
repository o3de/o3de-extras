### OpenXRTest
This project was setup to function as test bed for openxr development. It is a simple level that can be used as is or even modified to run experiments related to feature development for XR work. These experiments could be related to cpu or gpu performance work or adding new features related to this work. 

# How to set up the environment

- Set up [o3de](https://github.com/o3de/o3de) from GitHub and sync to the `development` branch. For details refer to [Setting up O3DE from GitHub](https://o3de.org/docs/welcome-guide/setup/setup-from-github/).
- Set up [o3de-extras](https://github.com/o3de/o3de-extras) from GitHub and sync to the `development` branch. For details refer to [O3DE-Extras readme](https://github.com/o3de/o3de-extras#readme).
- Register `XR` and `OpenXRVk` gems within O3DE: 
  ````
  >scripts\o3de.bat register -gp <your path to o3de-extras>/Gems/XR
  >scripts\o3de.bat register -gp <your path to o3de-extras>/Gems/OpenXRVk
  ````

# How to enable XR gems

Once the gems are registered within O3DE you can enable `XR` and `OpenXRVk` gems in your project as usual with project manager or command line. For details refer to [Adding and Removing Gems](https://www.o3de.org/docs/user-guide/project-config/add-remove-gems/).

The project [OpenXRTest](https://github.com/o3de/o3de-extras/tree/development/Projects/OpenXRTest) is already setup with `XR` and `OpenXRVk` gems enabled and provides test levels (`DefaultLevel` and `XR_Office`).

# How to build and run OpenXR using tethered connection

## Build steps
- Build O3DE on Windows PC with your project as usual. For details refer to the [User Guide Build page](https://www.o3de.org/docs/user-guide/build/).
- Run Asset Processor and wait for all the assets to be processed.

## Running OpenXR with the Editor
- Connect Meta Quest 2 to the PC and [launch Quest Link](https://www.meta.com/en-gb/help/quest/articles/headsets-and-accessories/oculus-link/connect-link-with-quest-2/).
- From command line run the Editor with the following options: `<project-build-path>/bin/profile> Editor.exe -rhi=vulkan -openxr=enable`
- Open a level.
- Enter game mode pressing `Ctrl+G`.
- The level will be rendered in both PC and Meta Quest 2.
- Pressing `ESC` will exit game mode and stop rendering on Meta Quest 2.

## Running OpenXR with Game launcher
- Connect Meta Quest 2 to the PC and [launch Quest Link](https://www.meta.com/en-gb/help/quest/articles/headsets-and-accessories/oculus-link/connect-link-with-quest-2/).
- From command line run the Game Launcher with the following options: `<project-build-path>/bin/profile> <ProjectName>.GameLauncher.exe -rhi=vulkan -openxr=enable`
  > **Note**
  > Make sure to indicate an [initial level](https://docs.o3de.org/docs/user-guide/build/distributable-engine/#optional-create-and-load-an-initial-level) to load.
- The level will be rendered in both PC and Meta Quest 2.

# How to build and run OpenXR natively on Android (i.e Meta Quest 2)

## Build steps
- Download the [Oculus OpenXR Mobile SDK](https://developer.oculus.com/downloads/native-android/) and unzip it inside `OpenXRVk` gem in the following folder `OpenXRVk\External\OculusOpenXRMobileSDK`.
- Make sure to indicate an [initial level](https://docs.o3de.org/docs/user-guide/build/distributable-engine/#optional-create-and-load-an-initial-level) to load. For OpenXrTest project the initial level is defined in `<project-path>/Registry/autoexec.game.setreg` and is set to be `DefaultLevel`
- Make sure that Asset Processor is set to process Android assets. For openXrTest project we have created `<project-path>/Registry/AssetProcessor.setreg` file with following content to generate android assets in the cache:
````
{
    "Amazon": {
        "AssetProcessor": {
            "Settings": {
                "Platforms": {
                    "android": "enabled"
                }
            }
        }
    }
}
````
- In order to help with gpu performance we have created `<project-path>/Registry/OpenXR.setreg` within OpenXRTest project. You can also set the `ViewResolutionScale` setting per platform to scale down the resolution and gain performance at the cost of picture quality. We have currently set it to 75% of the native resolution. This can be bumped to 100% when foveated rendering is enabled.
````
{
    "O3DE": {
        "Atom": {
            "OpenXR": {
                "Enable": true,
                "android_ViewResolutionScale": 0.75
            }
        }
    }
}
````
- Run Asset Processor and wait for all the assets to be processed.
- Connect Meta Quest 2 to the PC.
- Build and deploy android following [these steps](https://www.o3de.org/docs/user-guide/platforms/android/generating_android_project_windows/) with the following alterations:
  - Use your project instead of `o3de-atom-sampleviewer`. So in this case it will be OpenXrTest project.
  - When running `generate_android_project.py` command use `LOOSE` for `%O3DE_ANDROID_ASSET_MODE%` and add the option `--oculus-project`.
    > Note:
    > When targeting OpenXR devices other than Meta Quest 2, do not include `--oculus-project` option.
  - When running `deploy_android.py` command use `APK` for `%O3DE_ANDROID_DEPLOY_TYPE%`.

## Running app
- Launch app from Meta Quest 2.

## Modify OpenXR rendering pipeline
- When openxr is enabled (-openxr=enable) the app will automatically try to use the XR pipeline. There are three cvars defined in \o3de\Gems\Atom\Bootstrap\Code\Source\BootstrapSystemComponent.cpp which contain names of the XR related pipelines. These cvars can be overriden via a settings registry file. 

    - r_default_openxr_pipeline_name -> Pipeline name used by PC when using Link mode and want to render the scene on PC as well as the device.
    - r_default_openxr_left_pipeline_name -> Pipeline name for the left eye when running the app natively or via link mode on a stereoscopic device
    - r_default_openxr_right_pipeline_name -> Pipeline name for the right eye when running the app natively or via link mode on a stereoscopic device

## License
For terms please see the LICENSE*.TXT files at the root of this distribution.
