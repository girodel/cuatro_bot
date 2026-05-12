cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY launch assets urdf  worlds rviz  config models yaml maps
  DESTINATION share/${PROJECT_NAME}/
)

# 🔥 Scripts Python
install(
  PROGRAMS
    scripts/obstacle_avoidance
    scripts/bug0_node.py 
    scripts/bug1_node.py # <--- Agrégalo aquí
    scripts/bug2_node.py 
    scripts/nav_rooms.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
