Aproximación a Objetos (squad_approach_control_action)
======================================================

Este nodo permite que el robot se acerque a objetos detectados previamente en el entorno.

**Descripción del Nodo:**

- Utiliza una máquina de estados interna para gestionar el proceso de aproximación.
- Se basa en el nodo `move_base` para realizar movimientos precisos hacia los objetivos.
- Configurable con parámetros que determinan la distancia mínima y las velocidades de aproximación.

**Parámetros Importantes:**

- **`linear_speed`**: Velocidad lineal del robot durante la aproximación.
- **`angular_speed`**: Velocidad angular para ajustar la orientación del robot.
- **`min_distance`**: Distancia mínima para detener la aproximación al objetivo.

**Prueba del Nodo:**

Ejecuta este nodo con el siguiente comando:

.. code-block:: bash

   rosrun exploracion squad_approach_control_action.py

Envía un mensaje con las coordenadas de un objetivo al tópico `/approach_target` para probar su funcionalidad.

**Tópicos Relevantes:**

- **`/approach_target`**: Recibe las coordenadas del objetivo al que se debe aproximar.
- **`/cmd_vel`**: Publica comandos de movimiento al robot.

**Secuencia de Operación:**

1. Recibe las coordenadas del objetivo desde el tópico `/approach_target`.
2. Calcula la trayectoria óptima utilizando el nodo `move_base`.
3. Realiza movimientos hasta que la distancia mínima configurada sea alcanzada.
4. Publica el estado de finalización en el tópico `/current_state`.

Este nodo es fundamental para realizar interacciones precisas con objetos detectados, asegurando que el robot se acerque de manera segura y eficiente.

.. automodule:: squad_approach_control_action
    :members:
    :undoc-members:
    :show-inheritance: