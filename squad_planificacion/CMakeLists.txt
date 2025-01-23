cmake_minimum_required(VERSION 3.0.2)
project(squad_planificacion)

## Compila como C++11, soportado en ROS Kinetic y versiones más recientes
# add_compile_options(-std=c++11)

## Encuentra macros y librerías de catkin
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  message_generation
  std_msgs
  # NOTA: Se ha eliminado squad_exploracion de aquí para evitar la duplicación de targets
)

# Declara el archivo de mensaje de este paquete
add_message_files(
  FILES
  StationInfo.msg
  StationsInfo.msg
)

# Genera los mensajes
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

## Declaración del paquete catkin
## Si deseas usar mensajes de squad_exploracion, debes asegurarte que éste se compile antes
## y luego solo incluir su runtime (message_runtime) en las dependencias.
catkin_package(
  CATKIN_DEPENDS message_runtime geometry_msgs std_msgs
  # Si los mensajes de squad_exploracion se necesitan en tiempo de ejecución, 
  # y ya han sido generados por ese paquete previamente, puedes hacer:
  # CATKIN_DEPENDS message_runtime geometry_msgs std_msgs squad_exploracion
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

# Instala el script de Python
catkin_install_python(PROGRAMS src/squad_station_manager_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############

# Aquí puedes añadir tests si lo deseas
