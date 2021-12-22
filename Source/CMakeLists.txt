cmake_minimum_required(VERSION 3.12)

######################################################

enable_language(CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
IF (MSVC)
  set(CMAKE_CXX_FLAGS "/W4")
ELSE ()
  set(CMAKE_CXX_FLAGS "-Wall -g")
ENDIF ()

####################################

project(Runtime_demo_by_YIFEI_CHEN)

add_definitions(-DPATH_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

# glfw
SET(GLFW_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/libglfw)
include(${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/FindGLFW.cmake)

# opengl
find_package(OpenGL REQUIRED)

# delfem2
add_definitions(-DDFM2_HEADER_ONLY=ON)
set(DELFEM2_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/delfem2/include")

# imgui_glfw
set(IMGUIGLFW_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/imguiglfw/include")
# file(SOURCES "${IMGUIGLFW_INCLUDE_DIR}/*.cpp" "${IMGUIGLFW_INCLUDE_DIR}/*.h")
file(GLOB IMGUI_SOURCES
     "${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/imguiglfw/include/*.h"
     "${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/imguiglfw/include/*.cpp"
)
add_library("ImGui" STATIC ${IMGUI_SOURCES})

# torch
include(${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/libtorch/share/cmake/Torch/TorchConfig.cmake)

####################################################

include_directories(
    ${OPENGL_INCLUDE_DIR}
    ${GLFW_INCLUDE_DIR}
    ${TORCH_INCLUDE_DIRS}
    ${DELFEM2_INCLUDE_DIR}
    ${IMGUIGLFW_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
    main.cpp
    )

target_link_libraries(${PROJECT_NAME}
    ${DFM2_LIBRARY}
    ${OPENGL_LIBRARY}
    ${GLFW_LIBRARIES}
    ${TORCH_LIBRARIES}
    ImGui
    )

if (MSVC)
  file(GLOB TORCH_DLLS "${TORCH_INSTALL_PREFIX}/lib/*.dll")
  add_custom_command(TARGET ${PROJECT_NAME}
                     POST_BUILD
                     COMMAND ${CMAKE_COMMAND} -E copy_if_different
                     ${TORCH_DLLS}
                     $<TARGET_FILE_DIR:${PROJECT_NAME}>)
endif (MSVC)