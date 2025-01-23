Detección de Objetos (squad_object_detection_action)
====================================================

Este nodo procesa datos RGB y de profundidad para identificar objetos en el entorno del robot.

**Descripción del Nodo:**

- Convierte las imágenes de ROS a OpenCV usando `cv_bridge`.
- Detecta objetos basados en características específicas, como color o forma.
- Calcula las coordenadas globales de los objetos utilizando datos de la cámara y la odometría.
- Publica la información de los objetos detectados en el tópico `/detected_objects`.

**Parámetros Importantes:**

- **`color_threshold`**: Umbral de detección basado en color (por ejemplo, rojo o azul).
- **`topic_rgb`**: Tópico de entrada para las imágenes RGB.
- **`topic_depth`**: Tópico de entrada para las imágenes de profundidad.

**Prueba del Nodo:**

Para ejecutar este nodo, utiliza el siguiente comando:

.. code-block:: bash

   rosrun exploracion squad_object_detection_action.py

Verifica los mensajes publicados en el tópico `/detected_objects` para confirmar que los objetos se detectan correctamente.

**Tópicos Relevantes:**

- **`/detected_objects`**: Publica los datos de los objetos detectados (coordenadas y características).
- **`/camera/rgb/image_raw`**: Imágenes RGB utilizadas para la detección.
- **`/camera/depth/image_raw`**: Imágenes de profundidad utilizadas para calcular distancias.

Este nodo es esencial para la exploración y navegación del robot, proporcionando información crítica sobre el entorno inmediato.

.. automodule:: squad_object_detection_action
    :members:
    :undoc-members:
    :show-inheritance: