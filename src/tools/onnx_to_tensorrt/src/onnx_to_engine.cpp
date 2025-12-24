#include <iostream>
#include <fstream>
#include <memory>
#include <string>

#include <NvInfer.h>
#include <NvOnnxParser.h>

#ifdef TENSORRT_AVAILABLE

namespace
{

class Logger : public nvinfer1::ILogger
{
public:
  void log(Severity severity, const char *msg) noexcept override
  {
    // 只打印 WARNING 以上级别的信息，避免太多 INFO
    if (severity <= Severity::kWARNING) {
      std::cout << "[TensorRT][" << static_cast<int>(severity) << "]: " << msg << std::endl;
    }
  }
};

bool fileExists(const std::string &path)
{
  std::ifstream f(path, std::ios::binary);
  return f.good();
}

bool writeToFile(const std::string &path, const void *data, size_t size)
{
  std::ofstream f(path, std::ios::binary);
  if (!f.good()) {
    return false;
  }
  f.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(size));
  return f.good();
}

void printUsage(const std::string &program_name)
{
  std::cout << "用法:\n"
            << "  " << program_name
            << " <onnx_model_path> <engine_output_path> [--fp16] [--max-batch N]\n\n"
            << "示例:\n"
            << "  " << program_name
            << " policy.onnx policy.engine --fp16 --max-batch 1\n";
}

}  // namespace

int main(int argc, char **argv)
{
  if (argc < 3) {
    printUsage(argv[0]);
    return 1;
  }

  const std::string onnx_path = argv[1];
  const std::string engine_path = argv[2];

  bool use_fp16 = false;
  int max_batch = 1;

  for (int i = 3; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--fp16") {
      use_fp16 = true;
    } else if (arg == "--max-batch" && i + 1 < argc) {
      max_batch = std::stoi(argv[++i]);
    } else {
      std::cerr << "未知参数: " << arg << std::endl;
      printUsage(argv[0]);
      return 1;
    }
  }

  if (!fileExists(onnx_path)) {
    std::cerr << "ONNX 文件不存在: " << onnx_path << std::endl;
    return 1;
  }

  Logger logger;

  // 创建 builder / network / config
  auto builder = std::unique_ptr<nvinfer1::IBuilder>(
    nvinfer1::createInferBuilder(logger));
  if (!builder) {
    std::cerr << "创建 TensorRT IBuilder 失败！" << std::endl;
    return 1;
  }

  // TensorRT 10+ 默认使用 explicit batch，直接使用 0 作为标志即可
  // 对于旧版本，使用 kEXPLICIT_BATCH 标志
#if NV_TENSORRT_MAJOR >= 10
  auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
    builder->createNetworkV2(0));
#else
  const auto explicitBatch =
    1U << static_cast<uint32_t>(nvinfer1::NetworkDefinitionCreationFlag::kEXPLICIT_BATCH);
  auto network = std::unique_ptr<nvinfer1::INetworkDefinition>(
    builder->createNetworkV2(explicitBatch));
#endif
  if (!network) {
    std::cerr << "创建 INetworkDefinition 失败！" << std::endl;
    return 1;
  }

  auto config = std::unique_ptr<nvinfer1::IBuilderConfig>(
    builder->createBuilderConfig());
  if (!config) {
    std::cerr << "创建 IBuilderConfig 失败！" << std::endl;
    return 1;
  }

  // 设置 workspace 大小（根据需要调整，这里默认 1GB）
  // TensorRT 10 之后不再使用 setMaxWorkspaceSize，而是使用 setMemoryPoolLimit
  const uint64_t workspace_size_bytes = 1ULL * 1024ULL * 1024ULL * 1024ULL;
#if NV_TENSORRT_MAJOR >= 10
  config->setMemoryPoolLimit(nvinfer1::MemoryPoolType::kWORKSPACE, workspace_size_bytes);
#else
  config->setMaxWorkspaceSize(workspace_size_bytes);
#endif

  if (use_fp16) {
#if NV_TENSORRT_MAJOR < 10
    if (builder->platformHasFastFp16()) {
      config->setFlag(nvinfer1::BuilderFlag::kFP16);
      std::cout << "启用 FP16 模式\n";
    } else {
      std::cout << "警告：平台不支持快速 FP16，将使用 FP32\n";
    }
#else
    // TensorRT 10+ 中 kFP16 虽然标记为 deprecated，但仍可使用
    // 使用 pragma 抑制警告
    #pragma GCC diagnostic push
    #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
    config->setFlag(nvinfer1::BuilderFlag::kFP16);
    #pragma GCC diagnostic pop
    std::cout << "启用 FP16 模式\n";
#endif
  }

  // 显式 batch 网络下，batch size 由网络本身的维度决定，TensorRT 10 API 已不再提供 setMaxBatchSize
  // 这里保留 max_batch 参数，仅作日志提示，方便用户确认配置
  if (max_batch > 0) {
    std::cout << "注意：使用显式 batch 网络，max-batch=" << max_batch
              << " 仅用于记录，不再调用 builder->setMaxBatchSize。" << std::endl;
  }

  // 创建 ONNX Parser
  auto parser = std::unique_ptr<nvonnxparser::IParser>(
    nvonnxparser::createParser(*network, logger));
  if (!parser) {
    std::cerr << "创建 nvonnxparser::IParser 失败！" << std::endl;
    return 1;
  }

  std::cout << "开始解析 ONNX 模型: " << onnx_path << std::endl;
  if (!parser->parseFromFile(onnx_path.c_str(),
      static_cast<int>(nvinfer1::ILogger::Severity::kWARNING)))
  {
    std::cerr << "解析 ONNX 失败，请检查模型是否兼容 TensorRT。" << std::endl;
    int nbErrors = parser->getNbErrors();
    for (int i = 0; i < nbErrors; ++i) {
      auto *error = parser->getError(i);
      if (error) {
        std::cerr << "Parser Error " << i << ": "
                  << error->desc() << std::endl;
      }
    }
    return 1;
  }

  std::cout << "ONNX 解析完成，开始构建 TensorRT engine ..." << std::endl;

  auto engine = std::unique_ptr<nvinfer1::ICudaEngine>(
    builder->buildEngineWithConfig(*network, *config));
  if (!engine) {
    std::cerr << "构建 TensorRT engine 失败！" << std::endl;
    return 1;
  }

  std::cout << "engine 构建完成，开始序列化并保存到文件: " << engine_path << std::endl;

  auto serialized = std::unique_ptr<nvinfer1::IHostMemory>(
    engine->serialize());
  if (!serialized) {
    std::cerr << "engine 序列化失败！" << std::endl;
    return 1;
  }

  if (!writeToFile(engine_path, serialized->data(), serialized->size())) {
    std::cerr << "写入 engine 文件失败: " << engine_path << std::endl;
    return 1;
  }

  std::cout << "转换成功！输出 engine 文件: " << engine_path << std::endl;
  return 0;
}

#else  // TENSORRT_AVAILABLE

int main(int, char **)
{
  std::cerr << "本工具在编译时未启用 TensorRT(TENSORRT_AVAILABLE 未定义)。" << std::endl;
  return 1;
}

#endif  // TENSORRT_AVAILABLE


