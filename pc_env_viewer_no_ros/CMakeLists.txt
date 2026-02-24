cmake_minimum_required(VERSION 3.16)
project(pc_env_viewer LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

include(CTest)

find_package(PCL REQUIRED COMPONENTS common io filters visualization)
find_package(VTK REQUIRED COMPONENTS
  CommonCore
  RenderingCore
  RenderingOpenGL2
  InteractionStyle
  GUISupportQt
)

if(DEFINED VTK_QT_VERSION AND VTK_QT_VERSION VERSION_GREATER_EQUAL 6)
  find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets Concurrent OpenGL OpenGLWidgets)
  set(QT_PACKAGE Qt6)
  set(QT_COMPONENTS Core Gui Widgets Concurrent OpenGL OpenGLWidgets)
else()
  find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets Concurrent OpenGL)
  set(QT_PACKAGE Qt5)
  set(QT_COMPONENTS Core Gui Widgets Concurrent OpenGL)
endif()

find_package(Catch2 3 REQUIRED)

add_library(pc_env_viewer_core STATIC
  src/core/CliOptions.cpp
  src/core/CloudLoader.cpp
  src/core/CloudProcessor.cpp
)

target_include_directories(pc_env_viewer_core
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

target_link_libraries(pc_env_viewer_core
  PUBLIC
    ${PCL_LIBRARIES}
)

add_library(pc_env_viewer_ui STATIC
  src/app/MainWindow.cpp
  src/app/ViewerWidget.cpp
  src/core/CloudRenderer.cpp
)

target_include_directories(pc_env_viewer_ui
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${PCL_INCLUDE_DIRS}
    ${VTK_INCLUDE_DIRS}
)

target_link_libraries(pc_env_viewer_ui
  PUBLIC
    pc_env_viewer_core
    ${PCL_LIBRARIES}
    ${VTK_LIBRARIES}
    ${QT_PACKAGE}::Core
    ${QT_PACKAGE}::Gui
    ${QT_PACKAGE}::Widgets
    ${QT_PACKAGE}::Concurrent
)

add_executable(pc_env_viewer
  src/main.cpp
)

target_link_libraries(pc_env_viewer
  PRIVATE
    pc_env_viewer_core
    pc_env_viewer_ui
    ${QT_PACKAGE}::Core
    ${QT_PACKAGE}::Gui
    ${QT_PACKAGE}::Widgets
)

if(COMMAND vtk_module_autoinit)
  vtk_module_autoinit(
    TARGETS pc_env_viewer_ui pc_env_viewer
    MODULES ${VTK_LIBRARIES}
  )
endif()

if(BUILD_TESTING)
  add_executable(test_loader tests/test_loader.cpp)
  target_link_libraries(test_loader PRIVATE pc_env_viewer_core Catch2::Catch2WithMain)
  target_compile_definitions(test_loader PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

  add_executable(test_cli_headless tests/test_cli_headless.cpp)
  target_link_libraries(test_cli_headless PRIVATE pc_env_viewer_core Catch2::Catch2WithMain)
  target_compile_definitions(test_cli_headless PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/tests/data")

  add_test(NAME test_loader COMMAND test_loader)
  add_test(NAME test_cli_headless COMMAND test_cli_headless)
endif()
