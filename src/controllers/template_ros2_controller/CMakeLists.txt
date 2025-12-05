cmake_minimum_required(VERSION 3.8)
project(template_ros2_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -O3")

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Explicitly find fmt package to use system shared library instead of /usr/local/lib/libfmt.a
# Set CMAKE_PREFIX_PATH to prioritize system libraries over /usr/local and miniconda
if(CMAKE_PREFIX_PATH)
  list(REMOVE_ITEM CMAKE_PREFIX_PATH "/usr/local")
  # Remove miniconda paths to avoid using miniconda's libfmt.so.9
  string(REPLACE "/home/lu/miniconda3" "" CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
  string(REPLACE "//" "/" CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}")
  string(STRIP "${CMAKE_PREFIX_PATH}" CMAKE_PREFIX_PATH)
endif()
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


find_package(CUDA QUIET)
if(CUDA_FOUND)
  find_library(CUDA_RUNTIME_LIBRARY cudart
    PATHS
      ${CUDA_TOOLKIT_ROOT_DIR}/lib64
      /usr/local/cuda/lib64
      /usr/lib/x86_64-linux-gnu
  )
endif()

# Directly link to system libfmt.so.8 to avoid runtime dependency on miniconda's libfmt.so.9
# Link system fmt library first, then template_ros2_controller_parameters
# Collect all libraries to link in one call to avoid signature mismatch
# Note: Using plain signature to match ament_auto_add_library's internal usage
set(TEMPLATE_ROS2_CONTROLLER_LIBS
  /usr/lib/x86_64-linux-gnu/libfmt.so.8
  template_ros2_controller_parameters
)

# Link TensorRT and CUDA libraries if found
if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY)
  target_include_directories(template_ros2_controller PRIVATE ${TENSORRT_INCLUDE_DIR})
  list(APPEND TEMPLATE_ROS2_CONTROLLER_LIBS ${TENSORRT_LIBRARY})
  if(CUDA_RUNTIME_LIBRARY)
    list(APPEND TEMPLATE_ROS2_CONTROLLER_LIBS ${CUDA_RUNTIME_LIBRARY})
  endif()
  target_compile_definitions(template_ros2_controller PRIVATE TENSORRT_AVAILABLE)
  message(STATUS "TensorRT found: ${TENSORRT_LIBRARY}")
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
  /usr/lib/x86_64-linux-gnu/libfmt.so.8
  ${TEMPLATE_ROS2_CONTROLLER_LIBS}
)

# Use patchelf to fix library dependency from libfmt.so.9 to libfmt.so.8
# This is needed because template_ros2_controller_parameters may link to miniconda's libfmt.so.9
find_program(PATCHELF_EXECUTABLE patchelf)
if(PATCHELF_EXECUTABLE)
  add_custom_command(TARGET template_ros2_controller POST_BUILD
    COMMAND ${PATCHELF_EXECUTABLE} --replace-needed libfmt.so.9 libfmt.so.8 $<TARGET_FILE:template_ros2_controller>
    COMMENT "Fixing libfmt.so.9 dependency to libfmt.so.8"
  )
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
