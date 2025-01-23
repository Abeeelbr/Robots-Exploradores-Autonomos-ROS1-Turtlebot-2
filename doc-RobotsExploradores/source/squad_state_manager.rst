Máquina de Estados (`squad_state_manager`)
==========================================

El nodo **``squad_state_manager``** coordina el flujo de trabajo del robot mediante una 
**máquina de estados basada en SMACH**, facilitando la transición entre modos de operación 
como **reposo**, **exploración** y **navegación**, entre otros. De esta forma, se garantiza 
que el robot ejecute sus tareas de manera ordenada y con la lógica adecuada.

Descripción General
-------------------

Este nodo implementa una **máquina de estados** que orquesta los principales modos de operación:

1. **Reposo**  

   - Estado inicial (o de espera) en el que el robot no realiza movimientos autónomos.
   - Publica el estado actual en el tópico ``/current_state``.

2. **Exploración**  

   - Activa/Desactiva los módulos de **exploración autónoma** (por ejemplo, `explore_lite` modificado).
   - Permite la **detección de objetos** simultáneamente (nodo ``squad_object_detection_action``).
   - El robot recorre el entorno para mapear y descubrir nuevas zonas.

3. **Navegación**  

   - Se encarga de dirigir al robot a un objetivo específico, por ejemplo, una **estación** detectada (coordenadas publicadas por `squad_station_manager_node`).  
   - Utiliza la pila de **move_base** (u otro controlador) para planificar y ejecutar la ruta.

Transiciones entre Estados
--------------------------

La **lógica de transición** se basa en mensajes de comando (publicados en ``/command``) 
y/o eventos internos (como la finalización de la exploración). Un diagrama simplificado 
podría ser:

.. graphviz::

   digraph state_machine {
       rankdir=LR;
       node [shape = oval, style="filled", fillcolor="lightblue"];

       Reposo -> Exploracion [label="comando: modo_exploracion"];
       Exploracion -> Reposo     [label="comando: reposo"];
       Exploracion -> Navegacion [label="comando: ir_a_{nombre estación}"];
       Navegacion -> Reposo      [label="reposo"];
       Navegacion -> Exploracion [label="comando: modo_exploracion"];
   }

Por ejemplo:

- Al publicar `"modo_exploracion"` en ``/command``, se pasa de **Reposo** a **Exploración**.
- Si se publica `"ir_a_{nombre estación}"`` mientras está en **Exploración**, el robot cambia a **Navegación** y se dirige a la estación indicada.
- Al recibir `"reposo"`` en cualquier estado, el robot vuelve a **Reposo**.

Parámetros Configurables
------------------------

- **``timeout``**  
  Tiempo máximo que el robot puede permanecer en un estado antes de forzar un cambio.  


Prueba del Nodo
---------------

1. **Ejecución**  
   Para lanzar el nodo de la máquina de estados (normalmente se lanza junto con todo el sistema):

   .. code-block:: bash

      rosrun squad_main squad_state_manager.py

2. **Cambio de Estados**  
   Publica comandos en el tópico ``/command`` para probar las transiciones:

   - **Iniciar Exploración**:

     .. code-block:: bash

        rostopic pub /command std_msgs/String "modo_exploracion"

   - **Ir a una Estación** (si existe alguna activa):

     .. code-block:: bash

        rostopic pub /command std_msgs/String "ir_a_HOME"

   - **Detener**:

     .. code-block:: bash

        rostopic pub /command std_msgs/String "reposo"

Tópicos Relevantes
------------------

- ``/current_state``  
  Publica el estado actual del robot (Reposo, Exploración, Navegación, etc.).
- ``/command``  
  Recibe comandos (strings) para forzar transiciones (p. ej. `"start_exploration"`, `"stop"`, 
  `"go_to_station"`).
- ``/detected_objects`` (opcional)  
  Información de los objetos detectados 

Integración con Otros Nodos
---------------------------

- **squad_exploracion**: Durante el estado de **Exploración**, se activa la lógica de 
  explorar fronteras (`explore_lite`) y detectar objetos (`squad_object_detection_action`).
- **squad_planificacion**: Al cambiar a **Navegación**, el robot utiliza las coordenadas 
  generadas (posiciones de estaciones) para decidir hacia dónde moverse.
- **squad_interfaz**: Permite a un usuario (vía GUI) enviar comandos de estado 
  y visualizar el estado actual del robot.

Conclusiones
------------

Este nodo **orquesta** los principales modos de operación, garantizando que el robot:

1. Espere en **Reposo** hasta recibir una instrucción.  
2. Active la **Exploración** y la Detección de Objetos cuando sea necesario.  
3. Cambie a **Navegación** al recibir un objetivo específico (como una estación detectada).  

De este modo, se logra un sistema **modular** y **controlado**, que responde a eventos 
y a comandos de manera coherente con el resto de componentes de ROS.

Cóidgo Fuente
--------------

.. automodule:: squad_state_manager
    :members:
    :undoc-members:
    :show-inheritance:
