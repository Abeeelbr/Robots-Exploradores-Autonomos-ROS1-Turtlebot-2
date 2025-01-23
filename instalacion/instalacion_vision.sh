#!/usr/bin/env bash

# Colores para los mensajes
GREEN="\e[32m"
YELLOW="\e[33m"
BLUE="\e[34m"
RED="\e[31m"
RESET="\e[0m"

echo -e "${BLUE}Iniciando la instalación automatizada de YOLOv5 y PyTorch (CPU) desde PyPI...${RESET}"

# Paso 1: Actualizar repositorios de paquetes del sistema
echo -e "${YELLOW}Actualizando repositorios...${RESET}"
sudo apt-get update -y

# Paso 2: Instalar dependencias del sistema
echo -e "${YELLOW}Instalando dependencias del sistema (git, python3, pip, etc.)...${RESET}"
sudo apt-get install -y build-essential cmake git pkg-config \
libopencv-dev libboost-all-dev python3-venv python3-pip

# Paso 3: Crear y activar un entorno virtual de Python
echo -e "${YELLOW}Creando entorno virtual 'yolov5_env'...${RESET}"
python3 -m venv yolov5_env

echo -e "${YELLOW}Activando entorno virtual...${RESET}"
source yolov5_env/bin/activate

# Asegurar la última versión de pip
echo -e "${YELLOW}Actualizando pip...${RESET}"
pip install --upgrade pip

# Paso 4: Instalar PyTorch (CPU) y torchvision, torchaudio desde el índice oficial
echo -e "${YELLOW}Instalando PyTorch (CPU), torchvision y torchaudio...${RESET}"
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Paso 5: Instalar YOLOv5 desde PyPI
echo -e "${YELLOW}Instalando YOLOv5 desde PyPI...${RESET}"
pip install yolov5

# Paso 6: Mensaje final
echo -e "${GREEN}Instalación completa.${RESET}"
echo -e "${GREEN}Se ha creado y activado el entorno virtual 'yolov5_env' en la carpeta actual.${RESET}"
echo -e "${GREEN}Para usar YOLOv5 en cualquier momento, ejecuta:${RESET}"
echo -e "${BLUE}    source yolov5_env/bin/activate${RESET}"
echo -e "${GREEN}Podrás importar YOLOv5 directamente en Python así:${RESET}"
echo -e "${BLUE}    python -c 'import yolov5; print(\"YOLOv5 importado correctamente\")'${RESET}"

echo -e "${YELLOW}Si más adelante cierras la terminal o cambias de carpeta, deberás reactivar el entorno virtual con:${RESET}"
echo -e "${BLUE}    source yolov5_env/bin/activate${RESET}"

echo -e "${GREEN}¡Listo! PyTorch, YOLOv5 y sus dependencias están instalados y preparados para su uso.${RESET}"
