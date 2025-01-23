Station Manager Node (`squad_station_manager_node`)
===================================================

Este nodo coordina la **creación y gestión** de “estaciones” en el mapa del robot, 
basándose en la **detección de objetos**. Constituye un pilar fundamental del sistema 
de **planificación** para el desplazamiento y la organización del entorno.

Descripción General
--------------------

- **Objetivo**:  
  Identificar grupos (clusters) de objetos detectados y **generar estaciones** cuando 
  se cumplen criterios de cercanía y tipología (por ejemplo, dos sillas juntas, 
  indicando una “zona de asientos”).

- **Funcionamiento**:  
  El nodo se suscribe al tópico ``/detected_objects`` y, por cada objeto detectado, 
  decide si se asocia a un cluster ya existente o crea uno nuevo. Cuando un cluster 
  reúne al menos **dos objetos del mismo tipo** y se confirma su posición promedio, 
  el nodo publica la estación resultante en ``/stations_info``.

- **Diccionario de estaciones**:  
  Almacena las coordenadas promedio de cada estación, con nombres generados 
  dinámicamente (por ejemplo, “baño1”, “baño2”) y evita duplicados en estaciones ya publicadas.

Características Clave
---------------------

1. **Agrupamiento por Radio (``radius_threshold``)**  
   Si un objeto detectado se encuentra dentro de un **radio máximo** respecto al centro 
   de un cluster existente, se añade a dicho cluster. De lo contrario, se crea uno nuevo.  

2. **Publicación de Estaciones Completadas**  
   Cuando el cluster cumple el criterio de ser una estación (p. ej., dos o más objetos), 
   se calcula la posición promedio y se emite un mensaje en ``/stations_info`` con la 
   nueva estación (nombre, tipo, coordenadas, etc.).

3. **Nombres Únicos**  
   Si ya existe “baño”, la siguiente estación del mismo tipo se llama “baño2”, y así 
   sucesivamente.

4. **Integración con Otros Módulos**  

   - Interactúa con el nodo de **detección de objetos** (publica en ``/detected_objects``).  
   - Provee estaciones a la **máquina de estados** y a la **interfaz**, facilitando 
     la navegación hacia dichas estaciones.

Funciones Principales
---------------------

1. **Inicialización del Nodo**  

   - Inicia el nodo ROS con el nombre ``station_manager_node``.  
   - Lee parámetros como ``radius_threshold`` (umbral de distancia para el agrupamiento).  
   - Crea suscriptores y publicadores:
     - Suscriptor: ``/detected_objects`` (para recibir datos de objetos).
     - Publicador: ``/stations_info`` (para difundir nuevas estaciones).

2. **``detected_object_callback``**  

   - Cada objeto detectado se clasifica con la función ``classify_object`` (por ejemplo, 
     “habitación”, “baño”, “comedor”), según tablas o listas predefinidas.  
   - Se busca si el objeto puede agregarse a un cluster existente (distancia al centro 
     menor que ``radius_threshold``).  
   - Si el cluster resultante contiene al menos **2 objetos** y aún no se había publicado 
     como estación, se crea la estación.

3. **Gestión de Clusters y Estaciones**  

   - Calcula la posición media de los objetos en el cluster mediante 
     ``calculate_average_position``.  
   - Publica la estación en el tópico ``/stations_info``.  
   - Actualiza el diccionario ``almacen_estaciones`` y lo muestra por consola.

4. **Bloqueo de Hilos**  

   - Para evitar condiciones de carrera, utiliza un **lock** al modificar estructuras 
     compartidas como ``almacen_estaciones``.

Estructura del Código
---------------------

.. code-block:: none

   station_manager_node.py
   ├── StationManagerNode (clase principal)
   │    ├── __init__()
   │    ├── detected_object_callback()
   │    ├── add_completed_station()
   │    ├── calculate_average_position()
   │    ├── distance_to_center()
   │    └── classify_object()

Parámetros Configurables
------------------------

- **``radius_threshold``**  
  Radio máximo para considerar que un objeto pertenece a un cluster existente.  

- **Diccionario ``almacen_estaciones``**  
  Estructura Python que mapea cada nombre de estación a sus coordenadas promedio.  

Tópicos Utilizados
------------------

- **Suscriptor**:
  - ``/detected_objects``: Mensajes con la información de cada objeto (tipo, posición, confianza).

- **Publicador**:
  - ``/stations_info``: Publica un mensaje (p. ej. ``StationsInfo``) con las estaciones completadas (nombre, coordenadas, tipo).

Uso del Nodo
------------

1. **Lanzar el Nodo**  
   Para iniciar el node manager de estaciones:

   .. code-block:: bash

      rosrun squad_planificacion station_manager_node.py

2. **Configurar Parámetros**  
   A través de un archivo YAML o parámetros en la línea de comando, define:
   - ``radius_threshold``
   - Listas de tipos de objetos (en caso de clasificar objetos como “baño”, “comedor”, etc.).

3. **Visualizar la Salida**  
   - Observa en consola la evolución de ``almacen_estaciones``.  
   - Suscríbete al tópico ``/stations_info`` para ver la información de las nuevas estaciones.

Documentación Adicional
-----------------------

Para más detalles sobre la definición de los mensajes, consulta los ficheros 
``StationsInfo.msg`` y ``StationInfo.msg`` en el paquete de mensajes.

.. automodule:: squad_station_manager_node
    :members:
    :undoc-members:
    :show-inheritance:
