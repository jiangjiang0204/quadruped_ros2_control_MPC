cmake_minimum_required(VERSION 3.8)
project(unitree_guide_controller)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CONTROLLER_INCLUDE_DEPENDS
        pluginlib
        rcpputils
        controller_interface
        realtime_tools
        std_msgs
        control_input_msgs
        controller_common
        kdl_parser
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach (Dependency IN ITEMS ${CONTROLLER_INCLUDE_DEPENDS})
    find_package(${Dependency} REQUIRED)
endforeach ()

# --- Added: qpOASES dependencies ---
find_library(qpOASES_LIB qpOASES)
# 查找头文件路径，假设安装在标准路径 /usr/local/include/qpOASES 或类似位置
find_path(qpOASES_INCLUDE_DIR qpOASES.hpp PATH_SUFFIXES qpOASES)
if(NOT qpOASES_LIB OR NOT qpOASES_INCLUDE_DIR)
    message(FATAL_ERROR "qpOASES not found! Please install it first.")
endif()
# -----------------------------------

add_library(${PROJECT_NAME} SHARED

        src/UnitreeGuideController.cpp

        src/FSM/StateFixedStand.cpp
        src/FSM/StateSwingTest.cpp
        src/FSM/StateFreeStand.cpp
        src/FSM/StateBalanceTest.cpp
        src/FSM/StateTrotting.cpp

        src/robot/QuadrupedRobot.cpp
        src/robot/RobotLeg.cpp

        src/control/Estimator.cpp
        src/control/LowPassFilter.cpp
        src/control/BalanceCtrl.cpp
        src/control/MPC.cpp  # <--- Added MPC.cpp

        src/quadProgpp/Array.cc
        src/quadProgpp/QuadProg++.cc

        src/gait/WaveGenerator.cpp
        src/gait/FeetEndCalc.cpp
        src/gait/GaitGenerator.cpp

)

target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
        ${qpOASES_INCLUDE_DIR} # <--- Added qpOASES include
        PRIVATE
        src)

ament_target_dependencies(
        ${PROJECT_NAME} PUBLIC
        ${CONTROLLER_INCLUDE_DEPENDS}
)

# <--- Added Link qpOASES library
target_link_libraries(${PROJECT_NAME}
        PUBLIC 
        ${qpOASES_LIB}
)

pluginlib_export_plugin_description_file(controller_interface unitree_guide_controller.xml)

install(
        DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

install(
        TARGETS ${PROJECT_NAME}
        EXPORT export_${PROJECT_NAME}
        ARCHIVE DESTINATION lib/${PROJECT_NAME}
        LIBRARY DESTINATION lib/${PROJECT_NAME}
        RUNTIME DESTINATION bin
)

install(
        DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(${CONTROLLER_INCLUDE_DEPENDS})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_package()