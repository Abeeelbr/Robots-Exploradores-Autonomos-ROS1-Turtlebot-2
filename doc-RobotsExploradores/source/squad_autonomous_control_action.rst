Control Autónomo (squad_autonomous_control_action)
==================================================

Este nodo permite la navegación autónoma evitando obstáculos.

**Descripción del Nodo:**

- Segmenta el entorno en regiones (frente, izquierda, derecha, etc.) usando datos de LIDAR.
- Toma decisiones de movimiento en función de distancias detectadas.
- Configurable con parámetros para velocidades y distancias mínimas.

**Parámetros Importantes:**

- **`front_distance_threshold`**: Distancia mínima para avanzar.
- **`side_distance_threshold`**: Distancia mínima lateral.
- **`linear_speed`**: Velocidad lineal del robot.

.. automodule:: squad_autonomous_control_action
    :members:
    :undoc-members:
    :show-inheritance:
