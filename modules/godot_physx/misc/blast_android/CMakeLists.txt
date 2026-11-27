# Blast for Android (build_physx.py --platform android --blast): the four libraries the module links (NvBlast,
# NvBlastGlobals, NvBlastExtShaders, NvBlastExtAuthoring) as static libraries, from the same blast/ checkout the
# desktop SDK builds. Blast's own premake build has no Android target; the file lists, include paths and defines
# follow its premake5.lua. Installs the same include/ + bin/ layout as its blast-sdk folder.
#   cmake -S <this dir> -B <build> -DBLAST_ROOT=<checkout>/blast -DBOOST_MULTIPRECISION=<dir> + NDK toolchain
cmake_minimum_required(VERSION 3.20)
project(GodotPhysXBlastAndroid CXX)

set(BLAST_ROOT "" CACHE PATH "the blast/ directory of the PhysX checkout")
set(BOOST_MULTIPRECISION "" CACHE PATH "Blast's BoostMultiprecision dependency (header only)")
if(NOT EXISTS "${BLAST_ROOT}/source/sdk/lowlevel")
	message(FATAL_ERROR "BLAST_ROOT must be the blast/ directory of the PhysX checkout")
endif()
if(NOT EXISTS "${BOOST_MULTIPRECISION}/boost")
	message(FATAL_ERROR "BOOST_MULTIPRECISION must hold boost/ (Blast's packman dependency)")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(S "${BLAST_ROOT}/source")
set(I "${BLAST_ROOT}/include")

function(blast_library name component)
	file(GLOB srcs "${S}/sdk/common/*.cpp" "${S}/sdk/${component}/*.cpp" ${ARGN})
	add_library(${name} STATIC ${srcs})
	target_include_directories(${name} PRIVATE "${S}/sdk/common" "${I}/${component}" "${S}/sdk/${component}")
	target_compile_definitions(${name} PRIVATE "LOG_COMPONENT=\"${name}\"" NDEBUG CARB_DEBUG=0)
	target_compile_options(${name} PRIVATE -O2 -fno-strict-aliasing -Wno-everything)
	install(TARGETS ${name} ARCHIVE DESTINATION bin)
endfunction()

blast_library(NvBlast lowlevel)
target_include_directories(NvBlast PRIVATE "${I}/shared/NvFoundation")

blast_library(NvBlastGlobals globals)
target_include_directories(NvBlastGlobals PRIVATE "${I}/lowlevel" "${S}/shared/NsFoundation/include" "${I}/shared/NvFoundation")

blast_library(NvBlastExtShaders extensions/shaders)
target_include_directories(NvBlastExtShaders PRIVATE "${I}/lowlevel" "${I}/globals" "${I}/shared/NvFoundation"
	"${S}/shared/NsFoundation/include")

blast_library(NvBlastExtAuthoring extensions/authoring
	"${S}/sdk/extensions/authoringCommon/*.cpp" "${S}/sdk/extensions/authoring/VHACD/src/*.cpp")
target_include_directories(NvBlastExtAuthoring PRIVATE "${I}/lowlevel" "${I}/globals" "${I}/extensions/assetutils"
	"${I}/extensions/authoringCommon" "${S}/sdk/extensions/authoring" "${S}/sdk/extensions/authoringCommon"
	"${S}/sdk/extensions/authoring/VHACD/inc" "${S}/sdk/extensions/authoring/VHACD/public"
	"${I}/shared/NvFoundation" "${S}/shared/NsFoundation/include" "${BOOST_MULTIPRECISION}")

# The headers, as the desktop blast-sdk folder has them (SCsub adds the subfolders it needs).
install(DIRECTORY "${I}/" DESTINATION include)
