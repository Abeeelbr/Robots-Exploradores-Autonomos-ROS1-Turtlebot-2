Aproximación a Objetos (squad_approach_control_action)
======================================================

Este nodo mueve el robot hacia un objeto detectado.

**Descripción del Nodo:**

- Maneja una máquina de estados interna para controlar la aproximación.
- Utiliza `move_base` para realizar movimientos precisos.
- Configurable con parámetros como velocidades y distancia mínima.

**Parámetros Importantes:**

- **`linear_speed`**: Velocidad lineal del robot.
- **`angular_speed`**: Velocidad angular del robot.
- **`min_distance`**: Distancia mínima al objetivo.

.. automodule:: squad_approach_control_action
    :members:
    :undoc-members:
    :show-inheritance:
