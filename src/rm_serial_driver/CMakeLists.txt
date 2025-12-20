cmake_minimum_required(VERSION 3.8)
project(rm_serial_driver)

# 編譯選項
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依賴
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(serial REQUIRED)

# 包含目錄
include_directories(
  include
)

# 依賴列表
set(dependencies
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  geometry_msgs
  auto_aim_interfaces
  tf2
  tf2_ros
  tf2_geometry_msgs
  serial
)

# ========================================
# 庫：協議層
# ========================================
add_library(${PROJECT_NAME}_protocol SHARED
  src/protocol.cpp
)
ament_target_dependencies(${PROJECT_NAME}_protocol ${dependencies})

# ========================================
# 庫：彈道解算
# ========================================
add_library(${PROJECT_NAME}_ballistic SHARED
  src/ballistic_solver.cpp
)
ament_target_dependencies(${PROJECT_NAME}_ballistic ${dependencies})

# ========================================
# 組件：主節點（可被composable容器加載）
# ========================================
add_library(${PROJECT_NAME}_component SHARED
  src/serial_driver_node.cpp
)
target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_protocol
  ${PROJECT_NAME}_ballistic
)
ament_target_dependencies(${PROJECT_NAME}_component ${dependencies})

# 註冊為ROS2組件
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rm_serial_driver::SerialDriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

# ========================================
# 安裝
# ========================================
# 安裝庫
install(TARGETS
  ${PROJECT_NAME}_protocol
  ${PROJECT_NAME}_ballistic
  ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 安裝頭文件
install(DIRECTORY include/
  DESTINATION include
)

# 安裝配置文件
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# 安裝啟動文件
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# ========================================
# 測試
# ========================================
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

