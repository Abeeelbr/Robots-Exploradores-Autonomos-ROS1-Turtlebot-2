# Robots Móviles – Proyecto Final

¡Bienvenido/a al repositorio del **Proyecto de Robots Móviles**! Este proyecto presenta un sistema completo para la **exploración, mapeo y detección de objetos** en entornos interiores, utilizando **ROS** y un **TurtleBot 2** (u otro robot móvil similar).

> **Nota:** Para detalles más extensos, por favor revisa la documentación en Read the Docs _(reemplaza con el enlace real)_.

## Características Principales

- **Exploración Autónoma**  
  Utiliza un algoritmo basado en _fronteras_ (frontier exploration) para cubrir el entorno de manera eficiente.

- **SLAM con GMapping**  
  Genera mapas 2D de ocupación en tiempo real, permitiendo una navegación y localización precisa.

- **Detección de Objetos**  
  Implementada con **YOLOv8**, calcula coordenadas globales de los elementos detectados (personas, mobiliario, etc.).

- **Asignación Dinámica de Estaciones**  
  Asocia objetos a estancias específicas (baño, cocina, sala…), permitiendo la navegación autónoma a puntos de interés.

- **Interfaz Gráfica (GUI)**  
  Construida en _Tkinter_ para el control y la visualización (mapa, cámara, estados...).  
  - Muestra la posición del robot y las estaciones creadas.  
  - Permite cambiar entre modos de operación: **Reposo**, **Exploración** y **Navegación**.

## Vista Rápida

Aquí te dejamos algunos "vistazos" del proyecto en acción. **Reemplaza los enlaces y rutas de las imágenes/videos con los reales cuando los tengas.**

1. **Mapa y Exploración**  
   ![Mapa de Exploración](docs/img/mapa_placeholder.png)  
   _Figura: Ejemplo de mapeo y fronteras detectadas._

2. **Detección de Objetos**  
   ![Detección de Objetos](docs/img/deteccion_placeholder.png)  
   _Figura: Ventana de la cámara con boxes de detección YOLO._

3. **Asignación Dinámica de Estaciones**  
   ![Estaciones](docs/img/estaciones_placeholder.png)  
   _Figura: Se crean salas automáticamente y se visualizan en la GUI._

4. **Video Demostrativo**  
   [![Video Demostrativo](docs/img/video_thumbnail.png)](https://youtu.be/tu-video)  
   _Haz clic para ver el video._

## Requisitos e Instalación

### 1. Clona este Repositorio
```bash
git clone https://github.com/tu-usuario/tu-repo.git
cd tu-repo
```

### 2. Instala las Dependencias ROS y Librerías de Visión
```bash
sudo apt update
sudo apt install \
  ros-noetic-geometry-msgs \
  ros-noetic-cv-bridge \
  ros-noetic-nav-msgs \
  ros-noetic-sensor-msgs \
  ros-noetic-std-msgs \
  ros-noetic-actionlib \
  ros-noetic-actionlib-msgs \
  ros-noetic-smach \
  ros-noetic-smach-ros \
  python3-pip \
  python3-opencv \
  python3-tk \
  python3-numpy \
  python3-yaml \
  build-essential \
  cmake \
  git \
  libopencv-dev
```

### 3. Instala Pytorch y YOLOv8
```bash
pip3 install torch torchvision torchaudio
pip3 install ultralytics
```

### 4. Compila tu Workspace
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Uso Rápido

### En Simulación
Lanza el Mundo y el Robot en Gazebo:
```bash
roslaunch squad_main main_gazebo.launch
```

Inicia la Exploración:
1. Abre la interfaz gráfica.
2. Pulsa el botón EXPLORACIÓN para que el robot comience a mapear y detectar objetos.
3. Observa el Proceso:
   - Las estaciones se irán añadiendo dinámicamente en la interfaz.
   - Monitorea el mapeo y la detección en tiempo real.

### En el TurtleBot 2 Real
Configura las Variables de Entorno:
```bash
echo "export ROS_MASTER_URI=http://ip_del_turtlebot:11311" >> ~/.bashrc
echo "export ROS_HOSTNAME=tu_ip" >> ~/.bashrc
source ~/.bashrc
```

Configura el Robot:
- Asegúrate de que el TurtleBot 2 esté conectado a la misma red que tu PC.
- Verifica la conexión de los sensores (LIDAR, cámaras, etc.).

Lanza el Sistema en el Robot:
```bash
roslaunch turtlebot_bringup minimal.launch
```

Lanza el Programa en tu PC:
```bash
roslaunch squad_main main_robot.launch
```

Controla el Robot:
1. Abre la interfaz gráfica.
2. Selecciona el modo deseado y comienza a interactuar con el robot.

## Estructura General del Repositorio
```
├─ squad_main/          # Nodo principal y estados (SMACH), interfaz gráfica
├─ squad_exploracion/   # Algoritmos de exploración, integración con explore_lite
├─ squad_navegacion/    # Configuraciones de navegación (gmapping, move_base, etc.)
├─ squad_planificacion/ # Manejo dinámico de estaciones y detecciones
├─ squad_simulacion/    # Archivos y modelos 3D para Gazebo
├─ docs/                # Documentación y recursos visuales
└─ README.md            # Este archivo
```

## Documentación Detallada
Toda la documentación ampliada del proyecto está disponible en Read the Docs.

## Posibles Mejoras
- Integración de Más Robots
- Uso de Otro Hardware
- Optimización de la IA

## Contribución
¡Las PRs son bienvenidas! Por favor, sigue estos pasos:

1. Haz un fork del repositorio.
2. Crea tu rama de feature:
   ```bash
   git checkout -b feature/nueva-funcionalidad
   ```
3. Realiza tus cambios y asegúrate de que todo compila.
4. Envía un pull request cuando termines.

## Licencia
Este proyecto se distribuye bajo la MIT License.

## Notas Adicionales
- Reemplaza los enlaces y rutas de las imágenes/videos con los reales.
- Actualiza los enlaces de Read the Docs.
- Coloca las imágenes en la carpeta `docs/img/`.

¡Gracias por tu interés en Robots Móviles! Para dudas o sugerencias, puedes abrir un issue.