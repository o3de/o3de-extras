# ONNX

This is an experimental gem implementing [ONNX Runtime](https://onnxruntime.ai/) in O3DE and demoing it using inference examples from the MNIST dataset with an Editor dashboard displaying inference statistics. Image decoding is done using a modified version of [uPNG](https://github.com/elanthis/upng).

![Image of ONNX Dashboard](https://user-images.githubusercontent.com/108667365/182392987-e39a38af-169e-47a8-b14d-c18c431386ee.png)

## Setup

1. Download the zip of the GPU version of ONNX Runtime v1.9.0 (Windows or Linux) from the [ONNX Runtime GitHub](https://github.com/microsoft/onnxruntime/releases/tag/v1.9.0), extract, and put the contents inside the *External/onnxruntime* folder in the gem. The path should look like *External/onnxruntime/\<include and lib folders in here\>*.
2. Put your *.onnx* file inside the *Assets* folder of the gem. By default the Model wrapper class looks for a file called *model.onnx*. To run the MNIST example you need an MNIST model, which you can get from the [ONNX GitHub](https://github.com/onnx/models/tree/main/vision/classification/mnist).
3. Download Johnathan Orsolini's [MNIST.png dataset](https://www.kaggle.com/datasets/playlist/mnistzip) from Kaggle, unzip (takes a while, there are a lot of pictures), and place inside the *Assets* folder of the gem (i.e. the path should look like *Assets/mnist_png/\<testing and training folders in here\>*).
4. Download the [uPNG source](https://github.com/elanthis/upng) from GitHub and copy into *Code/Source/Clients*. The source for uPNG has to be modified to work with the build, see [Modifying uPNG](#modifying-upng) below. You must have the following 2 files in these locations (the other uPNG files are unnecessary):
    - *Code/Source/Clients/upng/upng.c*
    - *Code/Source/Clients/upng/upng.h*
5. GPU inferencing using CUDA is ENABLED by default. Please see [Requirements to inference using GPU](#requirements-to-inference-using-gpu) below to make sure you have the correct dependencies installed. If you do not have a CUDA enabled NVIDIA GPU, or would like to run the gem without the CUDA example, then you must make sure that the definition `PUBLIC ENABLE_CUDA=true` on ***line 51*** in *Code/CMakeLists.txt* is either commented out or removed. When you do this, the Model class will inference using CPU regardless of params passed in.
6. Add the [ONNX](#onnx) gem to your project using the [Project Manager](https://docs.o3de.org/docs/user-guide/project-config/add-remove-gems/) or the [Command Line Interface (CLI)](https://docs.o3de.org/docs/user-guide/project-config/add-remove-gems/#using-the-command-line-interface-cli). See the documentation on  [Adding and Removing Gems in a Project](https://docs.o3de.org/docs/user-guide/project-config/add-remove-gems/).
7. Compile your project and run.
8. Once the editor starts, and you go to edit a level with an initialized ONNX Model, you should be able to press the **HOME** (or equivalent) button on your keyboard to see the inference dashboard containing runtime graphs for any initialised model run history.
9. The MNIST example is located in the **ONNX.Tests** project, and demos the ONNX Model class by loading in the MNIST ONNX model file and running several thousand inferences against different digits in the MNIST testing dataset.

## Modifying uPNG

1. Change the extension of *Code/Source/Clients/upng/upng.c* to *Code/Source/Clients/upng/upng.cpp*
2. Go into *Code/Source/Clients/upng/upng.cpp*
3. Modify ***lines 1172-1176*** as follows:

```diff
-  file = fopen(filename, "rb");
-  if (file == NULL) {
+  errno_t err = fopen_s(&file, filename, "rb");
+  if (err != NULL) {
       SET_ERROR(upng, UPNG_ENOTFOUND);
       return upng;
   }
```

## Requirements to inference using GPU

- [CUDA Toolkit](https://developer.nvidia.com/cuda-toolkit) v11.4 or greater.
- [CUDNN library](https://developer.nvidia.com/cudnn) v8.2.4 or greater.
- [zlib](https://zlib.net/) v1.2.3 or greater.

## Additional developer docs

### What is ONNX?

- [Open Neural Network Exchange](https://onnx.ai/)
- An open standard for machine learning models, defining an extensible computation graph model, operators and data types.
- Its key benefit is interoperability - the idea is that you can generate a model using different ML frameworks (such as Tensorflow, Keras, PyTorch), export it as an ONNX model, and in theory these models should behave the same.
- It has also recently become a member of the [Linux Foundation](https://www.linuxfoundation.org/).

### Why do we need an ONNX Gem in O3DE?

- We want to give users the ability to easily inference ONNX models within the engine and measure inference performance.
- There are many uses for ML within games, such as Terrain Generation, ML Agents, Computer Vision and Animation.
- The existing [ONNX Runtime C++ API](https://onnxruntime.ai/docs/api/c/namespace_ort.html) is not user friendly and requires a lot of additional steps for integration with the engine.
- Provide a plug-and-play experience, removing as much prerequisite knowledge required as possible, so that people with no ML knowledge can still implement ML model inferencing using ONNX Runtime within the engine.

### General steps to run an inference using the Ort::Api

1. Include and link [ONNX Runtime libs](https://github.com/microsoft/onnxruntime).
2. Create the [Ort::Env](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_env.html).
3. Specify [SessionOptions](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_session_options.html).
4. Create [Ort::Session](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_session.html).
5. Create [Ort::MemoryInfo](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_memory_info.html).
6. Create [Ort::Allocator](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_allocator_with_default_options.html).
7. Specify ONNX file location.
8. Specify input names.
9. Specify input shapes.
10. Specify input count.
11. Specify output names.
12. Specify output shapes.
13. Specify output count.
14. Create input vector.
15. Create [input tensor](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_value.html#a3898146b5fc35b838bd48db807dd6c8e) from input vector.
16. Create output vector.
17. Create [output tensor](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_value.html#a3898146b5fc35b838bd48db807dd6c8e) from output vector.
18. Configure [RunOptions](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_run_options.html).
19. [Run inference](https://onnxruntime.ai/docs/api/c/struct_ort_1_1_session.html#ae1149a662d2a2e218e5740672bbf0ebe).
20. Retrieve data from output tensor.

- These are broadly the steps that the ONNX Gem uses in order to run an inference, however as opposed to the Ort::Api, most of these are hidden from the end user.

### Help me understand the code?

The gem is fairly well commented, and should give you a general idea of how it works. It's recommended to start with [Model.h](https://github.com/o3de/o3de-extras/blob/onnx-experimental/Gems/ONNX/Code/Include/ONNX/Model.h) as that it where the Model class lives. Along with the general steps to run an inference using the Ort::Api above, as well as the [documentation for the Ort::Api](https://onnxruntime.ai/docs/api/c/namespace_ort.html), take a look at:

1. The class member variables.
2. The `InitSettings` struct.
3. The `Load()` function.
4. The `Run()` function.

In that order, looking through the implementation in the [Model.cpp](https://github.com/o3de/o3de-extras/blob/onnx-experimental/Gems/ONNX/Code/Source/Clients/Model.cpp) file for each of the functions.

The `Ort::Env`, use of eBuses, and all ImGui functionality is implemented in the [ONNXBus.h](https://github.com/o3de/o3de-extras/blob/onnx-experimental/Gems/ONNX/Code/Include/ONNX/ONNXBus.h), [ONNXSystemComponent.h](https://github.com/o3de/o3de-extras/blob/onnx-experimental/Gems/ONNX/Code/Source/Clients/ONNXSystemComponent.h) and [ONNXSystemComponent.cpp](https://github.com/o3de/o3de-extras/blob/onnx-experimental/Gems/ONNX/Code/Source/Clients/ONNXSystemComponent.cpp). The thing to note with the `Ort::Env` is that it is initialized only once, and only one instance of it exists inside the gem, and is retrieved using the `ONNXRequestBus`. All instances of the Model use the same `Ort::Env` instance. 

The ImGui dashboard provides basic debugging functionality for the gem, providing information about the time taken to inference for each time the `Run()` function is called. In that function, you'll see that an `AZ::Debug::Timer` being used to time its execution - this is then dispatched to a function `AddTimingSample` in the `ONNXRequestBus`, which adds that value to the ImGui `HistogramGroup` for that model instance.

### What does the MNIST bit do?

[Mnist.h](https://github.com/o3de/o3de-extras/blob/onnx-experimental/Gems/ONNX/Code/Source/Clients/Mnist.h) and [Mnist.cpp](https://github.com/o3de/o3de-extras/blob/onnx-experimental/Gems/ONNX/Code/Source/Clients/Mnist.cpp) implement the inferencing of [MNIST models](https://en.wikipedia.org/wiki/MNIST_database) in the ONNX.Tests project using the `Model` class. They define an `Mnist` struct which extends the `Model` class, as well as several functions which inference several thousand MNIST images using an MNIST ONNX model file. These are executed by running the tests project, which calculates an accuracy by measuring the number of correct inferences and ensures it is above 90% - a good MNIST model will easily exceed this level of accuracy. The Mnist components used to be part of the main ONNX gem during development, used for testing features which were being added. Once the gem reached a usable state, the Mnist part of the gem was moved to the Tests project as the typical user will not want to just inference Mnist models, but it's useful leaving it in as a test to make sure everything is set up properly and as a demo so users can see the inferencing process.

### How do I run a basic inference using the gem?

1. Include the ONNX gem as a build dependency in your CMakeLists.txt - the example here is integrating it into the MotionMatching gem.

```CMAKE
ly_add_target(
    NAME MotionMatching.Static STATIC
    NAMESPACE Gem
    FILES_CMAKE
        motionmatching_files.cmake
    INCLUDE_DIRECTORIES
        PUBLIC
            Include
        PRIVATE
            Source
    BUILD_DEPENDENCIES
        PUBLIC
            AZ::AzCore
            AZ::AzFramework
            Gem::EMotionFXStaticLib
            Gem::ImguiAtom.Static
            ONNX.Private.Object
)
```

2. Include ONNX/Model.h in your file.

```C++
#include <ONNX/Model.h>
```

3. Initialize an `ONNX::Model`.

```C++
ONNX::Model onnxModel;
```

4. Initialize your input vector.

```C++
AZStd::vector<AZStd::vector<float>> onnxInput;
//Fill input vector here
```

5. Initialize `InitSettings`.

```C++
ONNX::Model::InitSettings onnxSettings;
onnxSettings.m_modelFile = "D:/MyModel.onnx";
onnxSettings.m_modelName = "Best Model Ever";
onnxSettings.m_modelColor = AZ::Colors::Tomato;
onnxSettings.m_cudaEnable = true;

//////////////////////////////////////////////////////////////////
DEFAULTS:

m_modelFile = “@gemroot:ONNX@/Assets/model.onnx”;
m_modelName = <Model filename without extension>;
m_modelColor = AZ::Color::CreateFromRgba(229, 56, 59, 255)
m_cudaEnable = false;
//////////////////////////////////////////////////////////////////
```

6. Load the model (this only needs to happen once when first initializing the model).

```C++
onnxModel.Load(onnxSettings);
```

7. Run the model.

```C++
onnxModel.Run(onnxInput);
```

8. Retrieve the outputs of the inference from the `m_outputs` member.

```C++
AZStd::vector<AZStd::vector<float>> myAmazingResults = onnxModel.m_outputs;
```
