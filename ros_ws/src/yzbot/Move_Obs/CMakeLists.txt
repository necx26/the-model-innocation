cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(Move_Obs)  # 替换为实际项目名
# 查找依赖库
find_package(Boost REQUIRED COMPONENTS system)
find_package(Protobuf REQUIRED)
find_package(gazebo REQUIRED)

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

# 链接依赖库
target_link_libraries(animated_models
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)
