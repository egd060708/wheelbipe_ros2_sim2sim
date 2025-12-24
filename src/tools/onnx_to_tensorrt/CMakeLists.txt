cmake_minimum_required(VERSION 3.8)
project(onnx_to_tensorrt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

# ===================== TensorRT & CUDA 查找 =====================

# TensorRT 头文件（NvInfer.h / NvOnnxParser.h）
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS
    /usr/include/x86_64-linux-gnu
    /usr/local/cuda/include
    /usr/include
    /opt/tensorrt/include
)

find_path(TENSORRT_ONNX_INCLUDE_DIR NvOnnxParser.h
  PATHS
    /usr/include/x86_64-linux-gnu
    /usr/local/cuda/include
    /usr/include
    /opt/tensorrt/include
)

# TensorRT 库
find_library(TENSORRT_LIBRARY NAMES nvinfer
  PATHS
    /usr/lib/x86_64-linux-gnu
    /usr/local/cuda/lib64
    /usr/lib
    /opt/tensorrt/lib
)

find_library(TENSORRT_PARSER_LIBRARY NAMES nvonnxparser
  PATHS
    /usr/lib/x86_64-linux-gnu
    /usr/local/cuda/lib64
    /usr/lib
    /opt/tensorrt/lib
)

# CUDA 运行时
find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
  PATHS
    /usr/local/cuda/include
    /usr/local/cuda/targets/x86_64-linux/include
    /usr/include
)

find_library(CUDA_RUNTIME_LIBRARY NAMES cudart
  PATHS
    /usr/local/cuda/lib64
    /usr/local/cuda/targets/x86_64-linux/lib
    /usr/lib/x86_64-linux-gnu
    /usr/lib
)

if(NOT (TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY AND TENSORRT_PARSER_LIBRARY))
  message(FATAL_ERROR
    "TensorRT 或其 ONNX Parser 未找到，请检查是否已正确安装。\n"
    "  TENSORRT_INCLUDE_DIR=${TENSORRT_INCLUDE_DIR}\n"
    "  TENSORRT_LIBRARY=${TENSORRT_LIBRARY}\n"
    "  TENSORRT_PARSER_LIBRARY=${TENSORRT_PARSER_LIBRARY}")
endif()

if(NOT (CUDA_INCLUDE_DIR AND CUDA_RUNTIME_LIBRARY))
  message(FATAL_ERROR
    "CUDA 未找到，请检查 CUDA 是否已正确安装。\n"
    "  CUDA_INCLUDE_DIR=${CUDA_INCLUDE_DIR}\n"
    "  CUDA_RUNTIME_LIBRARY=${CUDA_RUNTIME_LIBRARY}")
endif()

# ===================== 可执行程序 =====================

add_executable(onnx_to_engine
  src/onnx_to_engine.cpp
)

target_include_directories(onnx_to_engine PRIVATE
  ${TENSORRT_INCLUDE_DIR}
  ${TENSORRT_ONNX_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIR}
)

target_link_libraries(onnx_to_engine PRIVATE
  ${TENSORRT_LIBRARY}
  ${TENSORRT_PARSER_LIBRARY}
  ${CUDA_RUNTIME_LIBRARY}
)

target_compile_definitions(onnx_to_engine PRIVATE TENSORRT_AVAILABLE)

install(TARGETS
  onnx_to_engine
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()


