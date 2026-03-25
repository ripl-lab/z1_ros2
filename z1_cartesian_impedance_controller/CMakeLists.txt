cmake_minimum_required(VERSION 3.8)
project(z1_cartesian_impedance_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Eigen3 REQUIRED)
set(ROS2_REQUIRED_COMPONENTS
    ament_cmake
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    geometry_msgs
    std_msgs
)
foreach(COMPONENT ${ROS2_REQUIRED_COMPONENTS})
    find_package(${COMPONENT} REQUIRED)
endforeach()

#  _   _       _ _                   ____  ____  _  __
# | | | |_ __ (_) |_ _ __ ___  ___  / ___||  _ \| |/ /
# | | | | '_ \| | __| '__/ _ \/ _ \ \___ \| | | | ' /
# | |_| | | | | | |_| | |  __/  __/  ___) | |_| | . \
#  \___/|_| |_|_|\__|_|  \___|\___| |____/|____/|_|\_\
#
include(FetchContent)
FetchContent_Declare(
    z1_sdk
    URL https://github.com/unitreerobotics/z1_sdk/archive/refs/heads/master.zip
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE
)
FetchContent_GetProperties(z1_sdk)
if(NOT z1_sdk_POPULATED)
    message(STATUS "Fetching z1_sdk for dynamics model...")
    FetchContent_Populate(z1_sdk)
endif()

set(Z1_SDK_PATH ${z1_sdk_SOURCE_DIR})
set(Z1_SDK_LIB  ${Z1_SDK_PATH}/lib/libZ1_SDK_${CMAKE_SYSTEM_PROCESSOR}.so)

add_library(unitree::z1_sdk INTERFACE IMPORTED)
target_include_directories(unitree::z1_sdk INTERFACE ${Z1_SDK_PATH}/include)

#   ____            _             _ _
#  / ___|___  _ __ | |_ _ __ ___ | | | ___ _ __
# | |   / _ \| '_ \| __| '__/ _ \| | |/ _ \ '__|
# | |__| (_) | | | | |_| | | (_) | | |  __/ |
#  \____\___/|_| |_|\__|_|  \___/|_|_|\___|_|
#
add_library(${PROJECT_NAME} SHARED
    src/cartesian_impedance_controller.cpp
)
target_include_directories(${PROJECT_NAME}
    PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(${PROJECT_NAME}
    PRIVATE
    Eigen3::Eigen
    unitree::z1_sdk
    ${Z1_SDK_LIB}
)
ament_target_dependencies(${PROJECT_NAME}
    PUBLIC
    ${ROS2_REQUIRED_COMPONENTS}
)
target_compile_options(${PROJECT_NAME}
    PRIVATE
    -Wall -Wextra -Wpedantic -Wno-comment
)

pluginlib_export_plugin_description_file(
    controller_interface z1_cartesian_impedance_controller.xml
)

#  ___           _        _ _
# |_ _|_ __  ___| |_ __ _| | |
#  | || '_ \/ __| __/ _` | | |
#  | || | | \__ \ || (_| | | |
# |___|_| |_|___/\__\__,_|_|_|
#
install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(PROGRAMS ${Z1_SDK_LIB} DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${ROS2_REQUIRED_COMPONENTS})

ament_package()
