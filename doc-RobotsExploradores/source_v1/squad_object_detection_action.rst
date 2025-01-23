Detección de Objetos (squad_object_detection_action)
====================================================

Este nodo procesa imágenes RGB y de profundidad para detectar objetos en el entorno.

**Descripción del Nodo:**

- Convierte imágenes de ROS a OpenCV usando `cv_bridge`.
- Detecta objetos por color (ejemplo: objetos rojos).
- Calcula coordenadas globales utilizando datos de la cámara y la odometría.
- Publica información en `/detected_objects`.

**Parámetros Importantes:**

- **`color_threshold`**: Umbral de detección por color.
- **`topic_rgb`**: Tópico de entrada de imágenes RGB.
- **`topic_depth`**: Tópico de entrada de imágenes de profundidad.

.. automodule:: squad_object_detection_action
    :members:
    :undoc-members:
    :show-inheritance: