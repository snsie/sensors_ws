cmake_minimum_required(VERSION 3.8)
project(orbbec_lidar_ros2)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG}  -fPIC -g3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -O3")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -fPIC -g3")
set(CMAKE_BUILD_TYPE "Release")

execute_process(COMMAND uname -m OUTPUT_VARIABLE MACHINES)
execute_process(COMMAND getconf LONG_BIT OUTPUT_VARIABLE MACHINES_BIT)
message(STATUS "ORRBEC Machine : ${MACHINES}")
message(STATUS "ORRBEC Machine Bits : ${MACHINES_BIT}")

if ((${MACHINES} MATCHES "x86_64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "x64")
elseif (${MACHINES} MATCHES "arm" OR (${MACHINES} MATCHES "aarch64" AND ${MACHINES_BIT} MATCHES "32"))
  set(HOST_PLATFORM "arm32")
elseif ((${MACHINES} MATCHES "aarch64") AND (${MACHINES_BIT} MATCHES "64"))
  set(HOST_PLATFORM "arm64")
endif ()

set(LIDAR_SDK_LIBS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib/${HOST_PLATFORM})
set(LIDAR_SDK_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sdk/include/)

set(CMAKE_BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/sdk/lib/${HOST_PLATFORM}")
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib/${HOST_PLATFORM}")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Check if ROS Jazzy or iron is installed
if ("$ENV{ROS_DISTRO}" STREQUAL "jazzy")
    add_compile_definitions(JAZZY)
    add_definitions(-DRCLCPP_HAS_OnSetParametersCallbackType)
elseif ("$ENV{ROS_DISTRO}" STREQUAL "iron")
    add_compile_definitions(IRON)
elseif ("$ENV{ROS_DISTRO}" STREQUAL "foxy")
    add_compile_definitions(FOXY)
elseif ("$ENV{ROS_DISTRO}" STREQUAL "humble")
    add_compile_definitions(HUMBLE)
endif ()

# find dependencies
set(dependencies
    ament_cmake
    ament_index_cpp
    rclcpp
    rclcpp_components
    sensor_msgs
    std_msgs
    orbbec_lidar_msgs
)
foreach (dep IN LISTS dependencies)
    find_package(${dep} REQUIRED)
endforeach ()

add_library(orbbec_lidar_ros2 SHARED
    src/dynamic_params.cpp
    src/ob_lidar_driver.cpp
    src/ob_lidar_node.cpp
    src/ros_param_backend.cpp
    src/utils.cpp
)

set(common_include_dirs
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/orbbec_lidar_sdk/include>
    $<INSTALL_INTERFACE:include>
    ${LIDAR_SDK_INCLUDE_DIR}
)

ament_target_dependencies(orbbec_lidar_ros2 ${dependencies})

target_include_directories(orbbec_lidar_ros2
    PRIVATE
    ${common_include_dirs}
)

ament_target_dependencies(orbbec_lidar_ros2 ${dependencies})

target_link_libraries(orbbec_lidar_ros2
    orbbec_lidar_sdk
    -L${LIDAR_SDK_LIBS_DIR}
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN "ob_lidar::OBLidarDriver"
    EXECUTABLE orbbec_lidar_node
)

add_executable(list_devices_node
    tools/list_device_node.cpp
)

target_include_directories(list_devices_node
    PRIVATE
    ${common_include_dirs}
)

ament_target_dependencies(list_devices_node ${dependencies})

target_link_libraries(list_devices_node
    orbbec_lidar_sdk
    -L${LIDAR_SDK_LIBS_DIR}
)

install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS
    list_devices_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)
install(DIRECTORY ${LIDAR_SDK_LIBS_DIR}/
    DESTINATION lib/
    FILES_MATCHING PATTERN "*.so*"
)


install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})
ament_package()
