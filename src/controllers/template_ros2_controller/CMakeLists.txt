cmake_minimum_required(VERSION 3.8)
project(template_ros2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Set linker flags to exclude /usr/local/lib/libfmt.a from being linked
# Use APPEND to ensure flags are added correctly
# Explicitly exclude /usr/local/lib from library search path and exclude libfmt.a
# Use -Wl,--exclude-libs=ALL to exclude all static libraries from dependencies
# Also use -Wl,--exclude-libs=libfmt.a to explicitly exclude libfmt.a
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--exclude-libs=ALL -Wl,--exclude-libs=libfmt.a -Wl,--as-needed -L/usr/lib/x86_64-linux-gnu -Wl,-rpath-link=/usr/lib/x86_64-linux-gnu" CACHE STRING "" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--exclude-libs=ALL -Wl,--exclude-libs=libfmt.a -Wl,--as-needed -L/usr/lib/x86_64-linux-gnu -Wl,-rpath-link=/usr/lib/x86_64-linux-gnu" CACHE STRING "" FORCE)
# Remove /usr/local/lib from library search paths
set(CMAKE_LIBRARY_PATH "${CMAKE_LIBRARY_PATH}" CACHE STRING "" FORCE)
list(REMOVE_ITEM CMAKE_LIBRARY_PATH "/usr/local/lib")
# Also remove /usr/local/lib from link directories
link_directories(/usr/lib/x86_64-linux-gnu)
# Find fmt package, but prefer system libraries
find_package(fmt REQUIRED PATHS /usr/lib/x86_64-linux-gnu/cmake /usr/lib/cmake NO_DEFAULT_PATH)
if(NOT fmt_FOUND)
  find_package(fmt REQUIRED)
endif()

include_directories()

# generate parameter listener hpp file
generate_parameter_library(template_ros2_controller_parameters
  config/template_ros2_controller_parameters.yaml
)


pluginlib_export_plugin_description_file(controller_interface template_ros2_controller_plugin.xml)

set(FSM_FILES
  src/fsm/state_machine.cpp
  src/fsm/state_base.cpp
  src/fsm/states/state_init.cpp
  src/fsm/states/state_idle.cpp
  src/fsm/states/state_rl.cpp
)

set(TENSORRT_CUDA_FILES
  src/tensorrt_cuda/tensorrt_inference.cpp
)

set(ROBOT_STATE_FILES
  src/robot_state/robot_state.cpp
)

set(CONTROLLER_MODULES_FILES
  src/controller_modules/PIDmethod.cpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${FSM_FILES}
  ${TENSORRT_CUDA_FILES}
  ${ROBOT_STATE_FILES}
  ${CONTROLLER_MODULES_FILES}
  src/template_ros2_controller.cpp
)

# Set linker flags on the target to exclude libfmt.a
# This must be done after ament_auto_add_library
target_link_options(${PROJECT_NAME} PRIVATE
  -Wl,--exclude-libs=libfmt.a
  -Wl,--exclude-libs=ALL
)

# Link fmt::fmt before template_ros2_controller_parameters to ensure correct library is used
# Also set linker flags to exclude /usr/local/lib/libfmt.a and miniconda paths
# Explicitly link to system libfmt.so.8 to avoid miniconda's libfmt.so.9
# Use -L to prioritize system library path and exclude /usr/local/lib
# Use --exclude-libs=ALL to prevent linking static libraries from all dependencies
# Explicitly exclude /usr/local/lib from library search
target_link_options(template_ros2_controller PRIVATE 
  -Wl,--as-needed
  -Wl,--exclude-libs=ALL
  -L/usr/lib/x86_64-linux-gnu
  -Wl,-rpath,/usr/lib/x86_64-linux-gnu
  -Wl,--rpath-link=/usr/lib/x86_64-linux-gnu
  -Wl,--exclude-libs=libfmt.a
  -Wl,--exclude-libs=ALL
)
# Find TensorRT and CUDA
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS
    /usr/include/x86_64-linux-gnu
    /usr/local/cuda/include
    /usr/include
    ${CUDA_TOOLKIT_ROOT_DIR}/include
    /opt/tensorrt/include
)

find_library(TENSORRT_LIBRARY NAMES nvinfer
  PATHS
    /usr/lib/x86_64-linux-gnu
    /usr/local/cuda/lib64
    /usr/lib
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64
)


# Suppress CMake policy warning for FindCUDA
cmake_policy(SET CMP0146 NEW)

# Find CUDA - try multiple methods
find_package(CUDA QUIET)
if(NOT CUDA_FOUND)
  # Try to find CUDA by checking common installation paths
  if(EXISTS "/usr/local/cuda")
    set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda")
  elseif(EXISTS "/usr/local/cuda-12.8")
    set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-12.8")
  elseif(DEFINED ENV{CUDA_PATH})
    set(CUDA_TOOLKIT_ROOT_DIR $ENV{CUDA_PATH})
  endif()
endif()

if(CUDA_TOOLKIT_ROOT_DIR)
  # Find CUDA include directory
  find_path(CUDA_INCLUDE_DIR cuda_runtime_api.h
    PATHS
      ${CUDA_TOOLKIT_ROOT_DIR}/targets/x86_64-linux/include
      ${CUDA_TOOLKIT_ROOT_DIR}/include
      /usr/local/cuda/targets/x86_64-linux/include
      /usr/local/cuda/include
    NO_DEFAULT_PATH
  )
  
  # Find CUDA runtime library
  find_library(CUDA_RUNTIME_LIBRARY cudart
    PATHS
      ${CUDA_TOOLKIT_ROOT_DIR}/targets/x86_64-linux/lib
      ${CUDA_TOOLKIT_ROOT_DIR}/lib64
      /usr/local/cuda/targets/x86_64-linux/lib
      /usr/local/cuda/lib64
      /usr/lib/x86_64-linux-gnu
    NO_DEFAULT_PATH
  )
  
  if(CUDA_INCLUDE_DIR)
    message(STATUS "CUDA include directory found: ${CUDA_INCLUDE_DIR}")
  endif()
  
  if(CUDA_RUNTIME_LIBRARY)
    message(STATUS "CUDA runtime library found: ${CUDA_RUNTIME_LIBRARY}")
  endif()
endif()

# Find fmt library dynamically (Ubuntu 24.04 uses libfmt.so.9)
# Link system fmt library first, then template_ros2_controller_parameters
# Collect all libraries to link in one call to avoid signature mismatch
# Note: Using plain signature to match ament_auto_add_library's internal usage
find_library(FMT_LIBRARY 
  NAMES fmt libfmt.so.9 libfmt.so.8 libfmt.so
  PATHS /usr/lib/x86_64-linux-gnu /usr/lib
  NO_DEFAULT_PATH
)
if(NOT FMT_LIBRARY)
  find_library(FMT_LIBRARY NAMES fmt)
endif()

set(TEMPLATE_ROS2_CONTROLLER_LIBS
  ${FMT_LIBRARY}
  template_ros2_controller_parameters
)

# Link TensorRT and CUDA libraries if found
if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY)
  target_include_directories(template_ros2_controller PRIVATE ${TENSORRT_INCLUDE_DIR})
  # Add CUDA include directory if found
  if(CUDA_INCLUDE_DIR)
    target_include_directories(template_ros2_controller PRIVATE ${CUDA_INCLUDE_DIR})
  endif()
  list(APPEND TEMPLATE_ROS2_CONTROLLER_LIBS ${TENSORRT_LIBRARY})
  if(CUDA_RUNTIME_LIBRARY)
    list(APPEND TEMPLATE_ROS2_CONTROLLER_LIBS ${CUDA_RUNTIME_LIBRARY})
  endif()
  target_compile_definitions(template_ros2_controller PRIVATE TENSORRT_AVAILABLE)
  message(STATUS "TensorRT found: ${TENSORRT_LIBRARY}")
  if(CUDA_INCLUDE_DIR)
    message(STATUS "CUDA include directory: ${CUDA_INCLUDE_DIR}")
  endif()
else()
  message(WARNING "TensorRT not found. RL inference will not be available.")
endif()

# Link all libraries at once using plain signature to match ament_auto_add_library
# Explicitly link to system libfmt.so.8 FIRST to override any fmt dependency from other libraries
# Use -Wl,--exclude-libs=ALL to prevent linking static libraries from dependencies
# Use target_link_options to ensure /usr/local/lib/libfmt.a is excluded
target_link_options(template_ros2_controller PRIVATE
  -Wl,--exclude-libs=libfmt.a
  -Wl,--exclude-libs=ALL
)
target_link_libraries(template_ros2_controller 
  ${TEMPLATE_ROS2_CONTROLLER_LIBS}
)

# Note: patchelf step removed as we now use the system fmt library directly

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
