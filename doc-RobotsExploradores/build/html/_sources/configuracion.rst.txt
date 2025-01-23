Configuración del Entorno
=========================

Para poner en marcha el proyecto de Robots Móviles, es necesario configurar un workspace en ROS y asegurar que todas las dependencias estén correctamente instaladas.

**Pasos de Configuración:**

1. **Crear el Workspace de ROS:**
   Si no tienes un workspace configurado, sigue estos pasos:

   .. code-block:: bash

      mkdir -p rob_mov_ws/src
      cd rob_mov_ws/src
      git clone https://github.com/ottocol/navigation_stage.git
      cd ..
      catkin_make
      source devel/setup.bash

2. **Instalar Dependencias:**
   Asegúrate de instalar las dependencias necesarias, como SMACH y otros paquetes de ROS:

   .. code-block:: bash

      sudo apt-get install ros-noetic-smach ros-noetic-smach-ros ros-noetic-executive-smach ros-noetic-smach-viewer

3. **Lanzar el Simulador:**
   Una vez configurado, lanza el simulador para verificar que todo funciona correctamente:

   .. code-block:: bash

      roslaunch squad_main main_gazebo.launch

**Configuración de Parámetros:**

Asegúrate de revisar y ajustar los parámetros del sistema según tus necesidades. Los archivos de configuración YAML incluyen opciones como:

- **Velocidades:** Velocidad lineal y angular.
- **Umbrales de distancia:** Mínimos para detección y navegación.
- **Tópicos:** Configuración de entradas y salidas.

Con esta configuración inicial, tu entorno estará listo para ejecutar los diferentes nodos y funcionalidades del proyecto.

