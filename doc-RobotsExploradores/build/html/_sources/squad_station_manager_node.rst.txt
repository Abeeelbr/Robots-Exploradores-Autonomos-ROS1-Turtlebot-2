Station Manager Node
=====================

Este nodo gestiona estaciones en el mapa basándose en la detección de objetos. Es parte fundamental del sistema de planificación del robot.

Descripción General
-------------------

El nodo se encarga de:

- Publicar estaciones cuando se detecten al menos dos objetos del mismo tipo dentro de un radio definido.
- Evitar múltiples publicaciones para la misma estación.
- Mantener un diccionario ``almacen_estaciones`` con las coordenadas promedio de cada estación, por ejemplo:
  ``{"HOME": (5, 4), "ESTACION1": (1, 1), "ESTACION2": (2, 2)}``.
- Manejar nombres únicos para estaciones repetidas (por ejemplo, "baño", "baño2", etc.).

**Características Clave:**

- Utiliza suscriptores y publicadores de ROS para interactuar con otros nodos.
- Implementa bloqueo de hilos para la manipulación segura de datos compartidos.
- Proporciona una salida visual del diccionario de estaciones en la consola.

Funciones Principales
----------------------

**Inicialización del Nodo**

- Inicia el nodo ROS con el nombre ``station_manager_node``.
- Configura parámetros como el ``radius_threshold`` para definir el radio máximo para agrupar objetos.
- Crea suscriptores y publicadores:
  - Suscriptor al tópico ``/detected_objects`` para recibir detecciones.
  - Publicador al tópico ``/stations_info`` para transmitir la información de las estaciones.

**Callback de Detección de Objetos**

- Clasifica el objeto detectado mediante la función ``classify_object``.
- Agrupa objetos en clusters basándose en su distancia al centro del cluster.
- Publica estaciones completas cuando un cluster tiene al menos dos objetos.

**Gestión de Estaciones**

- Calcula la posición promedio de los objetos en un cluster usando ``calculate_average_position``.
- Publica las estaciones completadas en el tópico ``/stations_info``.
- Actualiza el diccionario ``almacen_estaciones`` y lo imprime en la consola.

Estructura del Código
---------------------

.. literalinclude:: station_manager_node.py
   :language: python
   :linenos:

**Clases y Métodos Principales**

1. **`StationManagerNode`**: Clase principal que contiene toda la lógica del nodo.
   - ``__init__``: Inicializa los parámetros, suscriptores y publicadores.
   - ``detected_object_callback``: Maneja los mensajes de objetos detectados.
   - ``add_completed_station``: Agrega estaciones completadas a las estructuras de datos y las publica.
   - ``calculate_average_position``: Calcula las coordenadas promedio de un cluster.
   - ``distance_to_center``: Calcula la distancia entre un objeto y el centro del cluster.
   - ``classify_object``: Clasifica objetos en categorías como ``habitacion``, ``baño``, etc.

**Parámetros Configurables**

- ``radius_threshold``: Radio máximo permitido para considerar un objeto en un cluster.
- Diccionario ``almacen_estaciones``: Contiene las coordenadas promedio de las estaciones.

Tópicos Utilizados
-------------------

- **Suscriptor:**
  - ``/detected_objects``: Recibe detecciones de objetos con tipo y posición.

- **Publicador:**
  - ``/stations_info``: Publica un mensaje de tipo ``StationsInfo`` con todas las estaciones completadas.

Uso del Nodo
------------

1. **Lanzar el Nodo:**

   Para ejecutar el nodo, usa el siguiente comando:

   .. code-block:: bash

      rosrun squad_planificacion station_manager_node.py

2. **Configurar Parámetros:**

   Asegúrate de definir los parámetros necesarios en un archivo YAML y cargarlos al iniciar el nodo.

3. **Visualizar Salida:**

   Los datos de estaciones se publican en ``/stations_info`` y se imprimen en la consola.

Documentación Adicional
-----------------------

Para más información, consulta los mensajes definidos en los archivos ``StationsInfo.msg`` y ``StationInfo.msg``.

.. automodule:: squad_station_manager_node
    :members:
    :undoc-members:
    :show-inheritance: