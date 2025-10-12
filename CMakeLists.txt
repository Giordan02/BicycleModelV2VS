cmake_minimum_required(VERSION 3.16)
project(BicycleModelV2 VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configurações básicas do Qt
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Configuração para MSVC estático
if(MSVC)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
endif()

# Configurar Qt 6.9.2 MSVC
set(CMAKE_PREFIX_PATH "C:/Qt/6.9.2/msvc2022_64")

# Encontrar Qt
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets PrintSupport)

# Forçar o caminho de todas as bibliotecas
set(Ceres_DIR "C:/vcpkg/installed/x64-windows-static/share/ceres")
set(glog_DIR "C:/vcpkg/installed/x64-windows-static/share/glog")
set(gflags_DIR "C:/vcpkg/installed/x64-windows-static/share/gflags")

# Encontrar as bibliotecas na ordem correta
find_package(gflags CONFIG REQUIRED)
find_package(glog CONFIG REQUIRED)
find_package(Ceres CONFIG REQUIRED)

# Comentar temporariamente o glog para isolar o problema
# find_package(glog CONFIG REQUIRED)

set(SOURCES
    main.cpp
    src/view/main_window.cpp
    resources.qrc
    resources/appicon.rc
    src/controller/input_manager.cpp
    src/controller/plot_tire_forces.cpp
    src/model/qcustomplot.cpp
    src/controller/simulation_inputs.cpp
    src/model/tire_model.cpp
    src/model/eqn_solver.cpp
    src/model/genetic_algorithm.cpp
    src/controller/tire_params_editor_dialog.cpp
)

set(HEADERS
    src/view/main_window.h
    src/controller/input_manager.h
    src/controller/plot_tire_forces.h
    src/model/qcustomplot.h
    src/controller/simulation_inputs.h
    src/model/tire_model.h
    src/model/eqn_solver.h
    src/model/genetic_algorithm.h
    src/controller/tire_params_editor_dialog.h
)

set(UIS
    src/view/mainwindow.ui
)

add_executable(${PROJECT_NAME} WIN32
    ${SOURCES}
    ${HEADERS}
    ${UIS}
)

target_include_directories(${PROJECT_NAME} PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/src/model 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/view
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
        Qt6::Core
        Qt6::Gui
        Qt6::Widgets
        Qt6::PrintSupport
        Ceres::ceres
        glog::glog
)

if(WIN32)
    target_sources(BicycleModelV2 PRIVATE resources/appicon.rc)
endif()