# ************************************************
#               MENSAJE PERSONALIZADO              
# ************************************************

string type                # Tipo de objeto detectado
geometry_msgs/Pose  robot_pose  # Coordenadas del robot
geometry_msgs/Point pixel_coordinates  # Coordenadas del objeto en píxeles (cámara)
geometry_msgs/Point world_coordinates  # Coordenadas del objeto en el mundo (inicialmente null)
