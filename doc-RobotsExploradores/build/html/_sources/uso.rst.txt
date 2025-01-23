Uso del Sistema
===============

Este sistema implementa una máquina de estados finitos (SMACH) para gestionar diferentes comportamientos del robot en el contexto del proyecto de robots móviles. Además, utiliza nodos especializados para detección de objetos, navegación autónoma y aproximación a objetivos, desarrollados en la carpeta `Trabajo_V3`.

Pasos para Lanzar el Sistema
----------------------------

1. **Preparar el Entorno:**

   Asegúrate de haber configurado correctamente el workspace, instalado las dependencias necesarias y haber ejecutado `catkin_make` en el directorio del proyecto.

2. **Probar los Nodos Individuales:**

   Antes de lanzar el sistema completo, puedes probar los nodos individualmente:

   - **Detección de Objetos:**

     .. code-block:: bash

        rosrun exploracion squad_object_detection_action.py

     Esto iniciará el nodo para detectar objetos en el entorno utilizando la cámara RGBD. Asegúrate de verificar los mensajes publicados en el tópico `/detected_objects`.


   - **Control Autónomo:**

     .. code-block:: bash

        rosrun exploracion squad_autonomous_control_action.py

     Este nodo activa la navegación autónoma. Puedes supervisar su funcionamiento revisando los mensajes publicados en los tópicos de diagnóstico.

3. **Lanzar el Sistema Completo:**

   Ejecuta el siguiente comando para iniciar todos los nodos del sistema:

   .. code-block:: bash

      roslaunch rob_mov_main main.launch

   Esto iniciará los nodos principales y configurará los parámetros necesarios para el funcionamiento del sistema.

Componentes Principales
------------------------

- **Máquina de Estados (squad_state_manager):**
  Gestiona las transiciones entre estados del robot, como reposo, exploración, aproximación y navegación. Publica el estado actual en el tópico `/current_state`.

- **Nodos Especializados:**
  - **`squad_object_detection_action`**: Procesa datos de la cámara RGBD para detectar objetos.
  - **`squad_autonomous_control_action`**: Realiza navegación autónoma evitando obstáculos.

- **Interfaz Gráfica:**
  Permite el control manual del robot, la visualización de datos de sensores y el monitoreo de estados.

Pruebas del Sistema
-------------------

1. **Prueba del Estado Reposo:**
   - Lanza el nodo principal y verifica que el robot se encuentre en el estado `Reposo` publicando en `/current_state`.

2. **Prueba del Estado Exploración:**
   - Cambia al estado de exploración enviando el comando `iniciar_exploracion` al tópico `/command`.
   - Verifica que el nodo `squad_object_detection_action` esté activo y publicando datos en `/detected_objects`.

3. **Prueba del Estado Navegación Autónoma:**
   - Envía el comando `ir_a_HOME` al tópico `/command`.
   - Verifica que el nodo `squad_autonomous_control_action` controle el movimiento hacia la posición predefinida.

Configuraciones Requeridas
---------------------------

Los parámetros del sistema se encuentran en los archivos YAML del proyecto:

- **`configGeneral.yaml`:** Incluye configuraciones globales como velocidades y frecuencias.
- **`detector_config.yaml`:** Define parámetros específicos para detección de objetos, como umbrales de color y distancia.

Tópicos Importantes
--------------------

- **`/current_state`:** Publica el estado actual del robot.
- **`/command`:** Recibe comandos para cambiar de estado.
- **`/detected_objects`:** Publica información sobre objetos detectados.
- **`/cmd_vel`:** Envía comandos de velocidad al robot.

Con esta guía y las pruebas recomendadas, deberías poder garantizar que todos los componentes del sistema funcionan correctamente. Si necesitas más detalles, consulta la documentación de cada nodo en las secciones correspondientes. 😊
