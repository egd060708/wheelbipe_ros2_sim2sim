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
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,--exclude-libs=libfmt.a -Wl,--as-needed" CACHE STRING "" FORCE)
set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--exclude-libs=libfmt.a -Wl,--as-needed" CACHE STRING "" FORCE)
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

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/template_ros2_controller.cpp
  src/fsm/state_machine.cpp
  src/fsm/state_init.cpp
  src/fsm/state_idle.cpp
  src/fsm/state_standing.cpp
  src/fsm/state_walking.cpp
  src/fsm/state_running.cpp
  src/fsm/state_error.cpp
  src/fsm/state_emergency_stop.cpp
  src/robot_state/robot_state.cpp
)

# Link fmt::fmt before template_ros2_controller_parameters to ensure correct library is used
# Also set linker flags to exclude /usr/local/lib/libfmt.a and miniconda paths
# Explicitly link to system libfmt.so.8 to avoid miniconda's libfmt.so.9
target_link_options(template_ros2_controller PRIVATE 
  -Wl,--as-needed
  -Wl,--exclude-libs=libfmt.a
  -Wl,-rpath,/usr/lib/x86_64-linux-gnu
  # Remove miniconda from library search path
  -Wl,--rpath-link=/usr/lib/x86_64-linux-gnu
)
# Directly link to system libfmt.so.8 to avoid runtime dependency on miniconda's libfmt.so.9
# Link system fmt library first, then template_ros2_controller_parameters
# Use -Bstatic to force static linking of fmt, then -Bdynamic for others
target_link_libraries(template_ros2_controller 
  /usr/lib/x86_64-linux-gnu/libfmt.so.8
  template_ros2_controller_parameters
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
