Uso del Sistema
===============

Este sistema implementa una máquina de estados finitos (SMACH) para gestionar diferentes comportamientos del robot. Además, proporciona una interfaz gráfica basada en Tkinter para facilitar la interacción con el robot.


Pasos para Lanzar el Sistema
----------------------------

Para lanzar el sistema completo, ejecuta el archivo `main.launch` con el siguiente comando:

.. code-block:: bash

   roslaunch squad_main main.launch

Este archivo lanzará todos los nodos necesarios, incluidos los siguientes:

- **Máquina de Estados (TurtleBot State Manager):**
  Nodo principal que gestiona los estados del robot y publica el estado actual en el tópico `/current_state`.

- **Servidores de Acción:**
  - `object_detection` (detección de objetos).
  - `approach_object` (aproximación a objetos).
  - `autonomous_control` (navegación autónoma).

- **Interfaz Gráfica:**
  Proporciona una ventana para el control manual del robot y la visualización de imágenes.


Flujo del Sistema
------------------------------

1. **Estado Inicial (Reposo):**
   El robot comienza en reposo esperando instrucciones.

2. **Exploración:**
   Activa el servidor de detección de objetos y supervisa el entorno en busca de objetos.

3. **Aproximación:**
   Si se detecta un objeto durante la exploración, el robot se aproxima a él utilizando el servidor de acción correspondiente. Este se activa automáticametne desde el estado de exploración si lo hemos activado en la configuración.

4. **Navegación:**
   El robot se desplaza a una posición predefinida (por ejemplo, "HOME"). Este comando se activa automaticamente al acabar el estado de exploración si lo hemos activado en la configuración.

Detalles de la Interfaz Gráfica
-------------------------------

La interfaz gráfica permite controlar y monitorizar el estado del robot de manera interactiva. Las funcionalidades principales incluyen:

- **Botones de Estado:**
  Cambia entre los diferentes estados del robot (reposo, exploración, navegación, aproximación).

- **Visualización de la Cámara:**
  Muestra en tiempo real las imágenes captadas por la cámara RGB.

- **Controles Manuales:**
  Permite mover el robot manualmente utilizando las teclas de flecha del teclado.

Configuración de la Máquina de Estados
--------------------------------------

La máquina de estados está compuesta por los siguientes estados:

1. **Reposo:**
   - Publica el estado actual en `/current_state`.
   - Permite las transiciones a `Exploración` o `Navegación` mediante comandos en el tópico `/command`.

2. **Exploración:**
   - Activa la detección de objetos mediante el servidor de acción `object_detection`.
   - Permite las transiciones a `Reposo`, `Aproximación`, o `Navegación` según los eventos detectados.

3. **Aproximación:**
   - Acerca el robot a un objeto detectado utilizando el servidor de acción `approach_object`.
   - Permite transiciones a `Reposo`, `Exploración`, o `Navegación`.

4. **Navegación:**
   - Mueve el robot a una posición predefinida utilizando el servidor de acción `move_base`.
   - Permite transiciones a `Reposo` o `Exploración`.

Comandos y Transiciones
-----------------------

Los comandos para cambiar de estado se publican en el tópico `/command`. Algunos ejemplos incluyen:

- **`modo exploracion`**: Cambia al estado de exploración.
- **`ir a HOME`**: Cambia al estado de navegación hacia la posición "HOME".
- **`acercarse_objetivo`**: Cambia al estado de aproximación hacia un objeto detectado.
- **`parar`**: Regresa al estado de reposo.

Parámetros Configurables
------------------------

Los parámetros del sistema se configuran en archivos YAML, que incluyen:

- **`configGeneral.yaml`**:
  Contiene configuraciones globales, como frecuencias de procesamiento, velocidades y distancias mínimas.

- **`config.yaml`**:
  Específico para cada estado, define parámetros como umbrales de detección y destinos predefinidos.

Tópicos Importantes
-------------------

- **`/current_state`**:
  Publica el estado actual del robot.

- **`/command`**:
  Recibe comandos para cambiar el estado de la máquina de estados.

- **`/detected_objects`**:
  Publica la información de los objetos detectados.

- **`/cmd_vel`**:
  Envia comandos de movimiento al robot.

- **`/camera/rgb/image_raw`**:
  Proporciona imágenes RGB de la cámara.

---

Con esta guía, deberías tener todo lo necesario para ejecutar, configurar y controlar el sistema. Si necesitas más información, consulta la sección de configuración o los detalles específicos de los nodos en la documentación. 😊
