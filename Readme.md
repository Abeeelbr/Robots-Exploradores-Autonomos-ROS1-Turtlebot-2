# Proyecto de Exploración con Robots Móviles

Este proyecto tiene como objetivo la exploración de una zona desconocida utilizando robots móviles TurtleBot2. Se implementa un sistema de despliegue de los robots que les permite recorrer el entorno de manera autónoma, recopilando información a través de su cámara, que usa visión artificial para el reconocimiento de distintos objetos presentes en el área de exploración.

## Características Principales

- **Exploración Autónoma:** El TurtleBot2 navega por el entorno desconocido, generando un mapeo del espacio con un algoritmo de exploración específico para cubrir el mayor espacio posible de la manera más óptima.
- **Visión Artificial:** Se utiliza Yolov8 Nano para identificar y clasificar objetos en tiempo real.
- **Asignaciones Dinámicas:** Una vez detectados los objetos, se generan asignaciones dinámicas de salas correspondientes a los objetos.
- **Despliegue Flexible:** El sistema estaba pensando inicialmente para desplegar uno o varios robots.

---

Este proyecto está diseñado principalmente para aplicaciones de exploración, aunque otras potenciales actividades serían entornos de investigación o escenarios donde se requiera automatización con robots móviles.

## Manual de instalación

