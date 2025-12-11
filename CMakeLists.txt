cmake_minimum_required(VERSION 3.8)
project(stretch_ik)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in`
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(orocos_kdl_vendor REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(trac_ik_lib REQUIRED)
find_package(ompl 1.8 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(Boost COMPONENTS program_options REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2_ros REQUIRED)

# include(FetchContent)
# FetchContent_Declare(rerun_sdk URL
#         https://github.com/rerun-io/rerun/releases/latest/download/rerun_cpp_sdk.zip)
# FetchContent_MakeAvailable(rerun_sdk)

add_library(stretch_ik
        src/stretch_ik.cpp
        src/chainiksolverpos_tl_adapter.cpp
)
target_include_directories(stretch_ik
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(stretch_ik
        PUBLIC
        orocos-kdl
        kdl_parser::kdl_parser
        urdf::urdf
        trac_ik_lib::trac_ik
)

add_executable(ik_node src/ik_node.cpp)
target_include_directories(ik_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(ik_node
        PUBLIC
        rclcpp::rclcpp
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        stretch_ik
)

add_executable(planner_node src/planner_node.cpp)
target_include_directories(planner_node
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(planner_node
        PUBLIC
        rclcpp::rclcpp
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        ${trajectory_msgs_TARGETS}
        ${control_msgs_TARGETS}
        stretch_ik
        Boost::program_options
        ompl::ompl
        ${PCL_LIBRARIES}
        tf2_ros::tf2_ros
)
target_compile_options(planner_node PRIVATE -mavx2)


# add_executable(benchmark src/benchmark.cpp)
# target_include_directories(benchmark
#        PUBLIC
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        $<INSTALL_INTERFACE:include>)
# target_link_libraries(ik_node
#        PUBLIC
#        rclcpp::rclcpp
#        stretch_ik
# )


add_executable(planning_test src/planning_test.cpp)
target_include_directories(planning_test
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
target_link_libraries(planning_test
        PUBLIC
        rclcpp::rclcpp
        ${std_msgs_TARGETS}
        ${sensor_msgs_TARGETS}
        stretch_ik
        Boost::program_options
        ompl::ompl
        ${PCL_LIBRARIES}
)
target_compile_options(planning_test PRIVATE -mavx2)

install(TARGETS
        ik_node planning_test planner_node
        DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch include meshes urdf rviz DESTINATION share/${PROJECT_NAME})

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_dependencies(rclcpp orocos_kdl orocos_kdl_vendor kdl_parser trac_ik_lib ompl std_msgs sensor_msgs urdf)
ament_package()
