cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(Move_Obs)  # 替换为实际项目名
# 查找依赖库
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs imgproc)

# 设置编译选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

# 包含头文件目录
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# 链接库目录
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)

# 生成插件（动态链接库）
add_library(animated_models SHARED animated_models.cc)
add_library(competition_patrol SHARED competition_patrol.cc)
target_link_libraries(competition_patrol ${GAZEBO_LIBRARIES})
add_library(competition_camera_stream SHARED competition_camera_stream.cpp)
target_compile_features(competition_camera_stream PRIVATE cxx_std_17)
ament_target_dependencies(competition_camera_stream gazebo_ros rclcpp sensor_msgs)
target_include_directories(competition_camera_stream PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(competition_camera_stream ${GAZEBO_LIBRARIES} ${OpenCV_LIBRARIES})

# 链接依赖库
target_link_libraries(animated_models
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

# Standard RViz front end with project-local map shader resources and health reporting.
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
add_executable(competition_rviz competition_rviz.cpp)
target_compile_features(competition_rviz PRIVATE cxx_std_17)
ament_target_dependencies(competition_rviz rviz_common rviz_rendering rclcpp)
target_link_libraries(competition_rviz Qt5::Widgets)
