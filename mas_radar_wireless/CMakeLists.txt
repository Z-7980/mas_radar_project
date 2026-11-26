cmake_minimum_required(VERSION 3.8)
project(radar_parse_em_wave)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

ament_python_install_package(rmuc)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RadarParseEmWaveDemodConfig.msg"
  "msg/RadarParseEmWave0A01RobotPosition.msg"
  "msg/RadarParseEmWave0A02RobotHp.msg"
  "msg/RadarParseEmWave0A03RobotAmmo.msg"
  "msg/RadarParseEmWave0A04FieldStatus.msg"
  "msg/RadarParseEmWave0A05RobotBuff.msg"
  "msg/RadarParseEmWave0A06InterferenceKey.msg"
  DEPENDENCIES std_msgs
)

install(
  PROGRAMS
    scripts/rmuc_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/rmuc_ros2_node.py
  RENAME radar_parse_em_wave_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY rmuc/web
  DESTINATION ${PYTHON_INSTALL_DIR}/rmuc
)

install(
  DIRECTORY grc
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
