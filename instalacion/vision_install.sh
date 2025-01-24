#!/usr/bin/env bash

# Asegurémonos de que estamos en el directorio correcto
echo "Script de instalación para YOLOv8 en ROS"

# Actualizar el sistema
echo "Actualizando el sistema..."
sudo apt-get update -y
sudo apt-get upgrade -y

# Instalar dependencias comunes
echo "Instalando dependencias generales..."
sudo apt-get install -y python3-pip python3-dev python3-rosdep python3-opencv \
                        python3-tk python3-numpy python3-yaml \
                        build-essential cmake git libopencv-dev

# Instalar cv_bridge para convertir imágenes entre ROS y OpenCV
echo "Instalando cv_bridge..."
sudo apt-get install -y ros-noetic-cv-bridge  # Para ROS 1 Noetic

# Asegúrate de tener las herramientas de CUDA si tienes GPU
# Solo si tienes una GPU Nvidia y quieres aprovechar CUDA
echo "Instalando CUDA y cuDNN..."
sudo apt-get install -y nvidia-cuda-toolkit

# Instalar PyTorch
echo "Instalando PyTorch..."
pip3 install torch torchvision torchaudio

# Instalar la librería de YOLOv8 (Ultralytics)
echo "Instalando YOLOv8 (Ultralytics)..."
pip3 install ultralytics

# Verificar instalación de YOLOv8
echo "Verificando la instalación de YOLOv8..."
python3 -c "import ultralytics; print('YOLOv8 versión:', ultralytics.__version__)"


# Finalizar
echo "Instalación completada. Por favor, ejecuta 'source ~/catkin_ws/devel/setup.bash' antes de usar ROS."
echo "Puedes comenzar a desarrollar tu nodo con YOLOv8 en ROS."