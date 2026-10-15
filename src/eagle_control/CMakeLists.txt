cmake_minimum_required(VERSION 3.8)
project(eagle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(eagle_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(centerlineNode
    src/centerlineNode.cpp
)

add_executable(controlNode
    src/controlNode.cpp
    src/canInterface.cpp
    src/PIDcontroller.cpp
    src/generated/eagle_task.c
)

ament_target_dependencies(centerlineNode
    rclcpp
    nav_msgs
    geometry_msgs
    eagle_msgs
)

ament_target_dependencies(controlNode
    rclcpp
    nav_msgs
    geometry_msgs
    eagle_msgs
)

target_sources(controlNode PRIVATE
    src/generated/eagle_task.c
)

target_link_libraries(controlNode yaml-cpp)

target_include_directories(controlNode PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

install(TARGETS
    centerlineNode
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
    controlNode
    DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    set(ament_cmake_copyright_FOUND TRUE)
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()