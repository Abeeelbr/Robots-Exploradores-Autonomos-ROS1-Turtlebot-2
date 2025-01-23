Máquina de Estados (squad_state_manager)
========================================

Descripción general del módulo `squad_state_manager`.

**Estados principales:**

- **Reposo:** Estado inicial de espera.
- **Exploración:** Activación del servidor de detección de objetos.
- **Aproximación:** Movimiento hacia un objeto detectado.
- **Navegación:** Desplazamiento a un punto específico.

**Transiciones:**
- De `Reposo` a `Exploración` tras recibir el comando correspondiente.
- De `Exploración` a `Aproximación` tras detectar un objeto.
- De `Aproximación` a `Reposo` si se alcanza el objetivo o tras cierto tiempo.

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

.. automodule:: squad_state_manager
    :members:
    :undoc-members:
    :show-inheritance:

.. automodule:: squad_interfaz
    :members:
    :undoc-members:
    :show-inheritance:
