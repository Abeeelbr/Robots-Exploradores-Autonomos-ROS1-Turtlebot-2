Detección de Objetos (`squad_object_detection_action`)
=====================================================

Este nodo implementa la **detección de objetos** mediante **visión por computadora**, 
combina imágenes RGB-D (color y profundidad) para identificar elementos de interés 
y publica sus coordenadas en el marco global del robot.

Descripción General
------------------

- **Nodo**: ``squad_object_detection_action.py``  
- **Objetivo**: Detectar objetos relevantes (por ejemplo, sillas, personas, camas, etc.) 
  y proveer sus posiciones tanto en el marco de la cámara como en el marco global del mapa.  
- **Enfoque principal**:  
  - Uso de un **modelo YOLOv8** (versión `nano` o `small`) preentrenado, con un conjunto 
    de clases de interés reducido para optimizar el rendimiento.  
  - Cálculo de la **distancia** a cada objeto a través de la imagen de **profundidad**, 
    transformando las coordenadas al espacio 3D e incorporando la **odometría** o TF 
    para obtener posiciones absolutas (en el marco `map`).

Flujo de Operación
------------------

1. **Conversión de Imágenes**  
   Mediante `cv_bridge`, las imágenes de los tópicos ROS (RGB y profundidad) se convierten 
   a formato OpenCV para su procesamiento.  

2. **Detección YOLO**  
   - El nodo ejecuta un modelo YOLOv8 cargado previamente (por ejemplo, `yolov8n.pt`), 
     filtrando predicciones por:
     - **Umbral de confianza** (típ. >70%).  
     - **Clases de interés** (ej. personas, sillas, mesas, etc.).  

3. **Cálculo de Coordenadas en 3D**  
   - A partir del centro del bounding box y la imagen de profundidad, se estima la **distancia** real.  
   - Se aplican los parámetros intrínsecos de la cámara (Fx, Fy, cx, cy) para pasar de 
     coordenadas de píxel a coordenadas 3D en el marco de la cámara (`camera_link`).  
   - Luego se utiliza TF (Transform Frames) para convertir dichas coordenadas al marco `map`, 
     dando como resultado la posición global del objeto.  

4. **Publicación de Resultados**  
   - Los objetos detectados se publican en el tópico ``/detected_objects`` junto con:  
     - Clase (tipo de objeto).  
     - Confianza de la detección.  
     - Coordenadas en el marco `map`.  
     - Información del robot en el momento de la detección (opcional).  

5. **Opcional: Visualización**  
   - El nodo puede mostrar en pantalla las imágenes con el **bounding box** y la distancia de cada objeto.  
   - Se pueden activar/desactivar estas vistas en función de parámetros de depuración.

Parámetros Importantes
----------------------

- **``confidence_threshold``**  
  Umbral mínimo de confianza para considerar una detección (p. ej. 0.7).  
- **``classes_of_interest``**  
  Lista de IDs de clases que YOLO debe reportar (por ejemplo, persona=0, silla=56, etc.).  
- **``topic_rgb``**  
  Tópico de entrada para las imágenes RGB, típicamente ``/camera/rgb/image_raw``.  
- **``topic_depth``**  
  Tópico para la imagen de profundidad, típicamente ``/camera/depth/image_raw``.  
- **``use_cuda``**  
  Activa o desactiva la aceleración por GPU.  

Prueba del Nodo
---------------

1. **Ejecutar el Sistema**  
   Para asegurarte de que la cámara (real o simulada) está publicando correctamente, 
   en una terminal:  

   .. code-block:: bash

      roslaunch squad_main main_gazebo.launch

   (o el archivo correspondiente si se prueba en el robot real).

2. **Correr el Nodo de Detección**  
   Si deseas lanzar el nodo de manera independiente, por ejemplo:  

   .. code-block:: bash

      rosrun squad_exploracion squad_object_detection_action.py

   Observa la salida en la consola y los mensajes publicados en ``/detected_objects``.

3. **Visualizar Detecciones en RViz (Opcional)**  
   - Puedes mostrar un marcador (MarkerArray) con la posición de los objetos detectados 
     o superponer la imagen en RViz usando plugins de cámara.

Tópicos Relevantes
------------------

- **Suscriptores**:
  - ``/camera/rgb/image_raw``: Imágenes RGB utilizadas para la detección.  
  - ``/camera/depth/image_raw``: Imágenes de profundidad para calcular distancias.  
  - ``/odom`` o TF: Se utilizan para la conversión de coordenadas al marco global.  

- **Publicadores**:
  - ``/detected_objects``: Información de los objetos detectados, incluyendo:
    - Clase (string o ID)
    - Confianza (float)
    - Posición (x, y, z) en el marco `map`

Código Fuente
-------------

.. automodule:: squad_object_detection_action
    :members:
    :undoc-members:
    :show-inheritance:

Conclusiones
------------

Este nodo es **esencial** para la identificación de elementos de interés en el entorno, 
alimentando el módulo de **asignación dinámica de estaciones** (p. ej., ``squad_station_manager_node``) 
y facilitando la **búsqueda y rescate** en contextos donde la detección de personas u objetos 
es prioritaria. El uso de **YOLO** optimiza la detección multiclase en tiempo real, 
haciendo viable la exploración autónoma con información visual detallada. 

