# NVIDIA Flow for Android (build_physx.py --platform android --flow): libnvflow.so and libnvflowext.so, which the
# module opens at run time, from the same flow/ checkout the desktop build uses. Flow's premake build has no Android
# target; the file lists, include paths and options follow its premake5.lua. The shaders come precompiled from a
# desktop Flow build's _generated/<platform>/release folder (FLOW_GENERATED): the module only uses their SPIR-V
# (Vulkan) variants, which don't depend on the platform that compiled them.
#   cmake -S <this dir> -B <build> -DFLOW_ROOT=<checkout>/flow -DFLOW_GENERATED=<dir> + NDK toolchain
cmake_minimum_required(VERSION 3.20)
project(GodotPhysXFlowAndroid C CXX)

set(FLOW_ROOT "" CACHE PATH "the flow/ directory of the PhysX checkout")
set(FLOW_GENERATED "" CACHE PATH "a desktop Flow build's _generated/<platform>/release folder")
if(NOT EXISTS "${FLOW_ROOT}/source/nvflow")
	message(FATAL_ERROR "FLOW_ROOT must be the flow/ directory of the PhysX checkout")
endif()
if(NOT EXISTS "${FLOW_GENERATED}/nvflow/shaders")
	message(FATAL_ERROR "FLOW_GENERATED must hold nvflow/shaders (build Flow for the desktop first)")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(F "${FLOW_ROOT}")

function(flow_library name)
	file(GLOB srcs "${F}/shared/*.cpp" "${F}/source/${name}/*.cpp" "${F}/source/${name}/*.c" ${ARGN})
	add_library(${name} SHARED ${srcs})
	target_include_directories(${name} PRIVATE "${F}/shared" "${F}/include/nvflow" "${F}/include/nvflow/shaders"
		"${FLOW_GENERATED}" "${FLOW_GENERATED}/${name}")
	target_compile_definitions(${name} PRIVATE NDEBUG)
	target_compile_options(${name} PRIVATE -O2 -ffast-math -fno-exceptions -fno-rtti -fvisibility=hidden -Wno-everything)
	target_link_libraries(${name} PRIVATE dl log)
	install(TARGETS ${name} LIBRARY DESTINATION lib)
endfunction()

flow_library(nvflow)
# Not vulkan/ and cpu/: Flow's own devices (window + swapchain code that needs X11). The module runs Flow on Godot's
# RenderingDevice instead; patch 0011 gives NvFlowGetDeviceInterface an empty interface on Android.
flow_library(nvflowext "${F}/source/nvflowext/opt/*.cpp")
target_include_directories(nvflowext PRIVATE "${F}/include/nvflowext" "${F}/include/nvflowext/shaders"
	"${F}/external/vulkan")
