# CMakeLists.txt based on NGL TODO This only works for Windows for now
cmake_minimum_required(VERSION 3.12)

# Try to find toolchain file of vcpkg
if(NOT DEFINED CMAKE_TOOLCHAIN_FILE AND DEFINED ENV{CMAKE_TOOLCHAIN_FILE})
  set(CMAKE_TOOLCHAIN_FILE $ENV{CMAKE_TOOLCHAIN_FILE})
endif()

# -----------------------------------------------------------------------------
# Project Details - Set up project, languages, and version number
# -----------------------------------------------------------------------------
project(
  FluidSimulationDemo
  VERSION 1.0
  DESCRIPTION "Fluid Simulation")

# Set the executable name
set(TARGET_NAME FluidSimulationDemo)
set(TESTS_NAME ${TARGET_NAME}Tests)
set(LIBRARY_OUTPUT_NAME fluidsimulation)
set(LIBRARY_NAME lib${LIBRARY_OUTPUT_NAME})

# Set C++ 17 standards
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Instruct CMake to run moc automatically when needed (Qt projects only)
set(CMAKE_AUTOMOC ON)

# Find all 3rd-party packages we are using
find_package(NGL CONFIG REQUIRED)
find_package(Qt5Widgets)
find_package(glm CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(OpenImageIO CONFIG REQUIRED)
find_package(freetype CONFIG REQUIRED)
find_package(IlmBase CONFIG REQUIRED)
find_package(OpenEXR CONFIG REQUIRED)

add_compile_definitions(ADDLARGEMODELS)
add_compile_definitions(USEOIIO)
add_compile_definitions(USEGLM)
add_compile_definitions(GLM_ENABLE_EXPERIMENTAL)
add_compile_definitions(_USE_MATH_DEFINES)
add_compile_definitions(NOMINMAX)
# Need to define this when building shared library or suffer dllimport errors
add_compile_definitions(BUILDING_DLL)

include_directories(include $ENV{HOMEDRIVE}/$ENV{HOMEPATH}/NGL/include)

# -----------------------------------------------------------------------------
# Library
# -----------------------------------------------------------------------------

# Files needed for this library
add_library(
  ${LIBRARY_NAME} STATIC
  ${CMAKE_SOURCE_DIR}/src/Fluid.cpp
  ${CMAKE_SOURCE_DIR}/include/Fluid.h
  ${CMAKE_SOURCE_DIR}/src/FluidGrid.cpp
  ${CMAKE_SOURCE_DIR}/include/FluidGrid.h
  )

set_target_properties(
  ${LIBRARY_NAME} PROPERTIES VERSION ${PROJECT_VERSION} OUTPUT_NAME
                                                        ${LIBRARY_OUTPUT_NAME})

# Libraries our library needs
target_link_libraries(
  ${LIBRARY_NAME}
  PRIVATE NGL
          OpenImageIO::OpenImageIO 
          OpenImageIO::OpenImageIO_Util 
          glm
          fmt::fmt-header-only 
          freetype)

target_include_directories(${LIBRARY_NAME} PRIVATE ${RAPIDXML_INCLUDE_DIRS}
                                                   ${RAPIDJSON_INCLUDE_DIRS})

# -----------------------------------------------------------------------------
# Build
# -----------------------------------------------------------------------------
add_executable(${TARGET_NAME})

set_target_properties(${TARGET_NAME} PROPERTIES VERSION ${PROJECT_VERSION})

# Files needed for the executable
target_sources(
  ${TARGET_NAME}
  PRIVATE 
          ${CMAKE_SOURCE_DIR}/src/NGLScene.cpp
          ${CMAKE_SOURCE_DIR}/include/NGLScene.h
          ${CMAKE_SOURCE_DIR}/src/main.cpp
          ${CMAKE_SOURCE_DIR}/shaders/PosDirVertex.glsl
          ${CMAKE_SOURCE_DIR}/shaders/PosDirFragment.glsl
          ${CMAKE_SOURCE_DIR}/shaders/PosDirGeo.glsl
          )

# Libraries needed for the executable, our library at the top
target_link_libraries(
  ${TARGET_NAME}
  PRIVATE ${LIBRARY_NAME}
          NGL
          Qt5::Widgets
          OpenImageIO::OpenImageIO
          OpenImageIO::OpenImageIO_Util
          glm
          fmt::fmt-header-only
          freetype)

# Copy the shaders from the root directory to the build directory
add_custom_command(
  TARGET ${TARGET_NAME}
  PRE_BUILD
  COMMAND
    ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/shaders
    $<TARGET_FILE_DIR:${TARGET_NAME}>/shaders)

# # Copy the images for testing from the root directory to the build directory
# add_custom_command(
#   TARGET ${TARGET_NAME}
#   PRE_BUILD
#   COMMAND
#     ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/img/tests
#     $<TARGET_FILE_DIR:${TARGET_NAME}>/img)

# Copy the font from the root directory to the build directory
add_custom_command(
  TARGET ${TARGET_NAME}
  PRE_BUILD
  COMMAND
    ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/fonts
    $<TARGET_FILE_DIR:${TARGET_NAME}>/fonts)

# -----------------------------------------------------------------------------
# Test
# -----------------------------------------------------------------------------
# add_executable(${TESTS_NAME})

# # Enable testing
# find_package(GTest CONFIG REQUIRED)
# include(GoogleTest)
# enable_testing()

# # Files needed for the test executable
# target_sources(
#   ${TESTS_NAME}
#   PRIVATE tests/TerrainTests.cpp tests/ClipmapLevelTests.cpp
#           tests/HeightmapTests.cpp tests/FootprintTests.cpp
#           tests/ManagerTests.cpp tests/CameraTests.cpp)
# gtest_discover_tests(${TESTS_NAME})

# # Libraries needed for the test executable, our library at the top
# target_link_libraries(
#   ${TESTS_NAME}
#   PRIVATE ${LIBRARY_NAME}
#           GTest::gtest
#           GTest::gtest_main
#           Qt5::Widgets
#           $ENV{HOMEDRIVE}/$ENV{HOMEPATH}/NGL/lib/NGL.lib
#           OpenImageIO::OpenImageIO
#           OpenImageIO::OpenImageIO_Util
#           glm
#           fmt::fmt-header-only
#           freetype)
