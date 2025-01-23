Configuración del Entorno
=========================

1. **Crear el Workspace de ROS:**
   Si no tienes un workspace configurado, sigue estos pasos:

   .. code-block:: bash

      mkdir -p rob_mov_ws/src
      cd rob_mov_ws/src
      git clone https://github.com/ottocol/navigation_stage.git
      cd ..
      catkin_make
      source devel/setup.bash

2. **Dependencias:**
   Asegúrate de instalar las dependencias necesarias para SMACH y otros paquetes ROS:

   .. code-block:: bash

      sudo apt-get install ros-noetic-smach ros-noetic-smach-ros ros-noetic-executive-smach ros-noetic-smach-viewer

3. **Lanzar el Simulador:**
   Para iniciar el simulador con el stack de navegación:

   .. code-block:: bash

      roslaunch navigation_stage mi_navigation.launch
