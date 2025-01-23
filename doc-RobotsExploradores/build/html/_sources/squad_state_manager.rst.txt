Máquina de Estados (squad_state_manager)
========================================

El nodo `squad_state_manager` gestiona el flujo de trabajo del robot mediante una máquina de estados finitos, permitiendo coordinar las diferentes tareas de exploración, aproximación y navegación.

**Descripción General:**

Este nodo implementa una máquina de estados utilizando SMACH para garantizar la transición adecuada entre comportamientos. Los estados principales son:

1. **Reposo:**
   - Estado inicial donde el robot espera instrucciones.
   - Publica el estado actual en el tópico `/current_state`.

2. **Exploración:**
   - Activa el nodo de detección de objetos (`squad_object_detection_action`).
   - Supervisa el entorno para identificar objetos de interés.
   
3. **Navegación:**
   - Se desplaza hacia un punto predefinido utilizando el nodo de control autónomo (`squad_autonomous_control_action`).

**Transiciones entre Estados:**

.. graphviz::

   digraph state_machine {
       rankdir=LR;
       node [shape = circle];

       Reposo -> Exploración [label="comando: iniciar_exploracion"];
       Exploración -> Aproximación [label="objeto detectado"];
       Aproximación -> Reposo [label="comando: parar"];
       Aproximación -> Navegación [label="comando: ir_a_HOME"];
       Navegación -> Reposo [label="llegada completada"];
       Navegación -> Exploración [label="revisar nuevamente"];
   }

**Parámetros Configurables:**

- **`timeout`**: Tiempo máximo permitido en cada estado.
- **`retry_limit`**: Número de intentos antes de cambiar de estado.
- **`state_topic`**: Tópico donde se publica el estado actual.

**Prueba del Nodo:**

Ejecuta el nodo `squad_state_manager` con el siguiente comando:

.. code-block:: bash

   rosrun squad_main squad_state_manager.py

Publica comandos en el tópico `/command` para probar transiciones entre estados. Ejemplos:

- **Iniciar Exploración:**

   .. code-block:: bash

      rostopic pub /command std_msgs/String "iniciar_exploracion"

- **Ir a HOME:**

   .. code-block:: bash

      rostopic pub /command std_msgs/String "ir_a_HOME"

**Tópicos Relevantes:**

- **`/current_state`**: Publica el estado actual del robot.
- **`/command`**: Recibe comandos para cambiar el estado de la máquina de estados.
- **`/detected_objects`**: Recibe información de objetos detectados para decidir transiciones.

Este nodo es esencial para la coordinación del sistema, permitiendo que el robot gestione sus tareas de manera eficiente y adaptable según el entorno.

.. automodule:: squad_state_manager
    :members:
    :undoc-members:
    :show-inheritance: