Paquete de Exploración (exploracion)
====================================

El paquete de exploración contiene los nodos necesarios para realizar tareas clave como detección de objetos, aproximación y navegación autónoma.

**Descripción General:**

Este paquete está diseñado para dotar al robot de capacidades básicas de exploración y navegación, utilizando múltiples nodos especializados que interactúan entre sí y con otros componentes del sistema.

**Componentes del Paquete:**

1. **Nodo de Detección de Objetos (`squad_object_detection_action`):**
   - Procesa datos de la cámara RGB y de profundidad para identificar objetos en el entorno.
   - Publica la información de los objetos detectados en el tópico `/detected_objects`.

   Para más detalles, consulta: `squad_object_detection_action.rst`.

2. **Nodo de Aproximación a Objetos (`squad_approach_control_action`):**
   - Mueve el robot hacia los objetos detectados de manera precisa y segura.
   - Utiliza coordenadas publicadas por el nodo de detección.

   Para más detalles, consulta: `squad_approach_control_action.rst`.

3. **Nodo de Control Autónomo (`squad_autonomous_control_action`):**
   - Permite que el robot navegue de manera autónoma evitando obstáculos.
   - Utiliza datos del sensor LIDAR para la planificación de rutas.

   Para más detalles, consulta: `squad_autonomous_control_action.rst`.

**Pruebas de los Nodos:**

Puedes probar cada nodo individualmente utilizando los comandos adecuados, como se describe en sus respectivos archivos `.rst`. Asegúrate de lanzar el entorno ROS y configurar los parámetros necesarios antes de iniciar las pruebas.

**Ejecución del Paquete:**

Para iniciar cualquier nodo de este paquete, utiliza el siguiente formato de comando:

.. code-block:: bash

   rosrun exploracion <nombre_del_nodo>.py

Ejemplo:

.. code-block:: bash

   rosrun exploracion squad_object_detection_action.py

**Tópicos Relevantes:**

- **`/detected_objects`**: Información sobre objetos detectados.
- **`/approach_target`**: Coordenadas de los objetivos para aproximación.
- **`/cmd_vel`**: Comandos de movimiento generados por los nodos.

**Configuraciones Adicionales:**

Asegúrate de ajustar los parámetros en los archivos YAML correspondientes según las necesidades del entorno:

- **`detector_config.yaml`**: Configuración para el nodo de detección de objetos.
- **`exploration_config.yaml`**: Parámetros generales de navegación y exploración.

Este paquete forma la base funcional del sistema, permitiendo al robot realizar tareas críticas de exploración y navegación de manera autónoma.
