/*
 * Copyright (c) Contributors to the Open 3D Engine Project.
 * For complete copyright and license terms please see the LICENSE at the root of this distribution.
 *
 * SPDX-License-Identifier: Apache-2.0 OR MIT
 *
 */

#include <ONNX/Model.h>

namespace ONNX
{
    void Model::Load(const InitSettings& initSettings)
    {
        // Get the FileIOBase to resolve the path to the ONNX gem
        AZ::IO::FixedMaxPath onnxModelPath;

        // If no filepath provided for onnx model, set default to a model.onnx file in the Assets folder.
        if (initSettings.m_modelFile.empty())
        {
            AZ::IO::FileIOBase* fileIo = AZ::IO::FileIOBase::GetInstance();
            fileIo->ResolvePath(onnxModelPath, "@gemroot:ONNX@/Assets/model.onnx");
        }
        else
        {
            onnxModelPath = initSettings.m_modelFile;
        }

        // If no model name is provided, will default to the name of the onnx model file.
        if (initSettings.m_modelName.empty())
        {
            AZ::StringFunc::Path::GetFileName(onnxModelPath.c_str(), m_modelName);
        }
        else
        {
            m_modelName = initSettings.m_modelName;
        }

        m_modelColor = initSettings.m_modelColor;

        // Grabs environment created on init of system component.
        Ort::Env* env = nullptr;
        ONNXRequestBus::BroadcastResult(env, &ONNXRequestBus::Events::GetEnv);

#ifdef ENABLE_CUDA
        // OrtCudaProviderOptions must be added to the session options to specify execution on CUDA.
        // Can specify a number of parameters about the CUDA execution here - currently all left at default.
        Ort::SessionOptions sessionOptions;
        if (initSettings.m_cudaEnable)
        {
            OrtCUDAProviderOptions cuda_options;
            sessionOptions.AppendExecutionProvider_CUDA(cuda_options);
        }
        m_cudaEnable = initSettings.m_cudaEnable;
#endif

        // The model_path provided to Ort::Session needs to be const wchar_t*, even though the docs state const char* - doesn't work otherwise.
        AZStd::string onnxModelPathString = onnxModelPath.String();
        m_session = Ort::Session(*env, AZStd::wstring(onnxModelPathString.cbegin(), onnxModelPathString.cend()).c_str(), sessionOptions);
        m_memoryInfo = Ort::MemoryInfo::CreateCpu(OrtDeviceAllocator, OrtMemTypeCPU);

        // Grabs memory allocator created on init of system component.
        Ort::AllocatorWithDefaultOptions* m_allocator;
        ONNXRequestBus::BroadcastResult(m_allocator, &ONNXRequestBus::Events::GetAllocator);

        // Extract input names from model file and put into const char* vectors.
        // Extract input shapes from model file and put into AZStd::vector<int64_t>.
        m_inputCount = m_session.GetInputCount();
        for (size_t i = 0; i < m_inputCount; i++)
        {
            const char* in_name = m_session.GetInputName(i, *m_allocator);
            m_inputNames.push_back(in_name);

            std::vector<int64_t> inputShape = m_session.GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape();
            AZStd::vector<int64_t> azInputShape(inputShape.begin(), inputShape.end());
            for (int index = 0; index < azInputShape.size(); index++)
            {
                if (azInputShape[index] == -1)
                {
                    azInputShape[index] = 1;
                }
            }
            m_inputShapes.push_back(azInputShape);
        }

        // Extract output names from model file and put into const char* vectors.
        // Extract output shapes from model file and put into AZStd::vector<int64_t>.
        // Initialize m_outputs vector using output shape and count.
        m_outputCount = m_session.GetOutputCount();
        AZStd::vector<AZStd::vector<float>> outputs(m_outputCount);
        for (size_t i = 0; i < m_outputCount; i++)
        {
            const char* out_name = m_session.GetOutputName(i, *m_allocator);
            m_outputNames.push_back(out_name);

            std::vector<int64_t> outputShape = m_session.GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
            AZStd::vector<int64_t> azOutputShape(outputShape.begin(), outputShape.end());
            for (int index = 0; index < azOutputShape.size(); index++)
            {
                if (azOutputShape[index] == -1)
                {
                    azOutputShape[index] = 1;
                }
            }
            m_outputShapes.push_back(azOutputShape);

            int64_t outputSize = 1;
            for (int j = 0; j < m_outputShapes[i].size(); j++)
            {
                // The size of each output is simply all the magnitudes of the shape dimensions multiplied together.
                if (m_outputShapes[i][j] > 0)
                {
                    outputSize *= m_outputShapes[i][j];
                }
            }
            AZStd::vector<float> output(outputSize);
            outputs[i] = output;
        }
        m_outputs = outputs;
    }

    void Model::Run(AZStd::vector<AZStd::vector<float>>& inputs)
    {
        m_timer.Stamp(); // Start timing of inference.

        // Tensor creation is lightweight, and a tensor is just a wrapper around the memory owned by the vector passed in as data during creation.
        // As such, creating input and output tensors in each run call does not adversely affect performance.
        AZStd::vector<Ort::Value> inputTensors;
        for (int i = 0; i < m_inputCount; i++)
        {
            Ort::Value inputTensor =
                Ort::Value::CreateTensor<float>(m_memoryInfo, inputs[i].data(), inputs[i].size(), m_inputShapes[i].data(), m_inputShapes[i].size());
            inputTensors.push_back(std::move(inputTensor));
        }

        AZStd::vector<Ort::Value> outputTensors;
        for (int i = 0; i < m_outputCount; i++)
        {
            Ort::Value outputTensor =
                Ort::Value::CreateTensor<float>(m_memoryInfo, m_outputs[i].data(), m_outputs[i].size(), m_outputShapes[i].data(), m_outputShapes[i].size());
            outputTensors.push_back(std::move(outputTensor));
        }

        Ort::RunOptions runOptions;
        runOptions.SetRunLogVerbosityLevel(ORT_LOGGING_LEVEL_VERBOSE); // Gives more useful logging info if m_session.Run() fails.
        m_session.Run(runOptions, m_inputNames.data(), inputTensors.data(), m_inputCount, m_outputNames.data(), outputTensors.data(), m_outputCount);

        float delta = 1000 * m_timer.GetDeltaTimeInSeconds(); // Finish timing of inference and get time in milliseconds.
        m_delta = delta;

        ONNXRequestBus::Broadcast(&::ONNX::ONNXRequestBus::Events::AddTimingSample, m_modelName.c_str(), m_delta, m_modelColor);
    }
} // namespace ONNX
