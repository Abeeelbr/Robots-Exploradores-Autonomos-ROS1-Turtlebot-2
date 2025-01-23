#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Servidor de Acción para la Detección de Objetos en TurtleBot.

Este nodo implementa un servidor de acción que permite al TurtleBot detectar objetos en su entorno
utilizando imágenes RGB y de profundidad. Coordina la conversión de imágenes ROS a OpenCV, la detección
de objetos basados en color, el cálculo de coordenadas mundiales de los objetos detectados y la publicación
de información relevante. Además, proporciona herramientas de depuración para visualizar imágenes
procesadas y datos en tiempo real.

Funcionalidades Principales:
    - Conversión de imágenes ROS a formatos utilizables por OpenCV.
    - Detección de objetos basados en color (por ejemplo, objetos rojos).
    - Cálculo de las coordenadas mundiales de los objetos detectados utilizando datos de
      profundidad y odometría.
    - Publicación de información de objetos detectados en tópicos ROS.
    - Proporciona una interfaz de retroalimentación para la frecuencia de procesamiento.
    - Herramientas de depuración para visualizar imágenes procesadas y datos de profundidad.

Este servidor de acción está diseñado para ser parte integral de una máquina de estados finitos
(SMACH) que gestiona el comportamiento del robot en diferentes estados, como exploración,
aproximación a objetos y navegación.
"""

import rospy
import actionlib
from sensor_msgs.msg import Image, CameraInfo
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Point
from squad_exploracion.msg import ObjectDetectionAction, ObjectDetectionFeedback, ObjectDetectionResult
from squad_exploracion.msg import DetectedObject  # Mensaje personalizado
import message_filters
from cv_bridge import CvBridge, CvBridgeError
import cv2
import threading
import time
import numpy as np
from collections import deque
import tf

# Definir si el modo DEBUG está activo
DEBUG = False

class TurtleBotObjectDetectionAction:
    """
    Servidor de Acción para la Detección de Objetos en TurtleBot.

    Este servidor de acción procesa imágenes RGB y de profundidad para detectar objetos de interés
    en el entorno del TurtleBot. Coordina la conversión de imágenes, detección de objetos, cálculo
    de coordenadas mundiales y publicación de información detectada.

    Hereda de:
        objectlib.SimpleActionServer: Proporciona funcionalidades de servidor de acción.

    Atributos:
        server (SimpleActionServer): Servidor de acción para la detección de objetos.
        bridge (CvBridge): Instancia de CvBridge para convertir imágenes ROS a OpenCV.
        lock (threading.Lock): Bloqueo para controlar el acceso a datos compartidos.
        object_pub (Publisher): Publicador para la ubicación de objetos detectados.
        process_image_pub (Publisher): Publicador para imágenes procesadas.
        latest_image (Image): Última imagen RGB recibida.
        latest_depth (Image): Última imagen de profundidad recibida.
        latest_position (PoseStamped): Última posición del robot.
        latest_depth_cv (ndarray): Última imagen de profundidad convertida a OpenCV.
        image_count (int): Contador de imágenes procesadas.
        processing_times (deque): Cola para almacenar tiempos de procesamiento.
        fx (float): Parámetro intrínseco de la cámara (focal length x).
        fy (float): Parámetro intrínseco de la cámara (focal length y).
        cx (float): Parámetro intrínseco de la cámara (principal point x).
        cy (float): Parámetro intrínseco de la cámara (principal point y).
        camera_info_sub (Subscriber): Suscriptor al tópico de CameraInfo.
    """

    def __init__(self):
        """
        Inicializa el servidor de acción para la detección de objetos.

        Configura el nodo ROS, servidor de acción, parámetros, herramientas de procesamiento,
        suscriptores y publicadores necesarios para la detección de objetos.
        """
        # ************************************************
        #              INICIALIZACIÓN DEL NODO            
        # ************************************************
        rospy.init_node('turtlebot_object_detection_action', anonymous=True)
        
        # ************************************************
        #            CREACIÓN DEL SERVIDOR DE ACCIÓN       
        # ************************************************
        self.server = actionlib.SimpleActionServer('object_detection', ObjectDetectionAction, self.execute, False)
        self.server.start()
        
        # ************************************************
        #               GESTIÓN DE PARÁMETROS             
        # ************************************************
        self.load_parameters()
        
        # ************************************************
        #          INICIALIZACIÓN DE HERRAMIENTAS         
        # ************************************************
        # Instanciar CvBridge para convertir imágenes
        self.bridge = CvBridge()

        # Inicializar lock para controlar el acceso a la imagen y la posición
        self.lock = threading.Lock()
        
        # ************************************************
        #            SUSCRIPCIÓN A LOS TÓPICOS            
        # ************************************************
        self.subscribe_to_topics()
        
        # ************************************************
        #                PUBLICADORES                     
        # ************************************************
        # Publicador para la ubicación del objeto detectado (usando mensaje personalizado)
        self.object_pub = rospy.Publisher('/detected_objects', DetectedObject, queue_size=10)
        self.process_image_pub = rospy.Publisher(self.process_image_topic, Image, queue_size=10)
        
        # ************************************************
        #                VARIABLES INTERNAS               
        # ************************************************
        # Variables para almacenar la última imagen, profundidad y posición
        self.latest_image = None
        self.latest_depth = None
        self.latest_position = None
        self.latest_depth_cv = None  # Inicializar aquí

        # Variables para medir el tiempo de procesamiento
        self.image_count = 0
        self.processing_times = deque(maxlen=10)  # Guardar los últimos 10 tiempos de procesamiento

        # Parámetros intrínsecos de la cámara
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Suscripción al tópico de CameraInfo para obtener parámetros intrínsecos
        self.camera_info_sub = rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback)
        

    # ************************************************
    #               GESTIÓN DE PARÁMETROS             
    # ************************************************
    def load_parameters(self):
        """
        Carga los parámetros esenciales del nodo desde los archivos de configuración.

        Establece los tópicos de suscripción, frecuencia de procesamiento y otros parámetros clave.
        """
        # Cargar parámetros importantes del nodo
        self.image_topic =          rospy.get_param('image_topic', '/camera/rgb/image_raw')
        self.process_image_topic = rospy.get_param('process_image_topic', '/processed_image')
        self.depth_topic = rospy.get_param('depth_topic', '/camera/depth/image_raw')
        self.camera_info_topic = rospy.get_param('camera_info_topic', '/camera/rgb/camera_info')
        self.odometry_topic = rospy.get_param('odometry_topic', '/odom')
        self.process_frequency = rospy.get_param('process_frequency', 30.0)  # Frecuencia de procesamiento (Hz)

        rospy.loginfo('Frecuencia de procesamiento: %.2f Hz', self.process_frequency)
        rospy.loginfo('Tópico de odometría: %s', self.odometry_topic)
        rospy.loginfo('Tópico de profundidad: %s', self.depth_topic)
        rospy.loginfo('Tópico de imagen: %s', self.image_topic)
        rospy.loginfo('Tópico de CameraInfo: %s', self.camera_info_topic)
    
    # ************************************************
    #            SUSCRIPCIÓN A LOS TÓPICOS            
    # ************************************************
    def subscribe_to_topics(self):
        """
        Suscribe a los tópicos necesarios y sincroniza los mensajes recibidos.

        Utiliza message_filters para sincronizar las imágenes RGB, de profundidad y los datos de odometría.
        """
        # Suscribirse a los tópicos de interés utilizando message_filters para sincronización
        image_sub = message_filters.Subscriber(self.image_topic, Image, queue_size=10)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image, queue_size=10)
        odom_sub = message_filters.Subscriber(self.odometry_topic, Odometry, queue_size=10)
        
        # Sincronización de mensajes
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub, odom_sub],
            queue_size=10,
            slop=0.1  # Ajustar el 'slop' para sincronización
        )
        self.ts.registerCallback(self.callback)
    
    # ************************************************
    #          CALLBACK DE SINCRONIZACIÓN DE DATOS    
    # ************************************************
    def callback(self, image_msg, depth_msg, odom_msg):
        """
        Callback para procesar los mensajes sincronizados de imagen, profundidad y odometría.

        Convierte las imágenes de ROS a formatos utilizables por OpenCV y almacena los datos recibidos.

        Args:
            image_msg (Image): Mensaje de imagen RGB.
            depth_msg (Image): Mensaje de imagen de profundidad.
            odom_msg (Odometry): Mensaje de odometría.
        """
        try:
            with self.lock:
                self.latest_image = image_msg
                self.latest_depth = depth_msg
                self.latest_position = odom_msg.pose.pose

                # Convertir la imagen de profundidad a OpenCV
                self.latest_depth_cv = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(f"Error al convertir la imagen de profundidad: {e}")
        except Exception as e:
            rospy.logerr(f"Excepción en el callback: {e}")
    
    # ************************************************
    #            CALLBACK DE CameraInfo              
    # ************************************************
    def camera_info_callback(self, msg):
        """
        Callback para procesar el mensaje de CameraInfo y extraer parámetros intrínsecos.

        Almacena los parámetros intrínsecos de la cámara y desuscribe del tópico una vez recibidos.

        Args:
            msg (CameraInfo): Mensaje con información de la cámara.
        """
        with self.lock:
            self.fx = msg.K[0]
            self.fy = msg.K[4]
            self.cx = msg.K[2]
            self.cy = msg.K[5]
            # rospy.loginfo("Recibidos parámetros intrínsecos de la cámara: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", self.fx, self.fy, self.cx, self.cy)
        
        # Desregistrar el suscriptor ya que no se necesita más
        if hasattr(self, 'camera_info_sub'):
            self.camera_info_sub.unregister()
            rospy.loginfo("Desregistrado del tópico CameraInfo.")
    
    # ************************************************
    #               EJECUCIÓN DE LA ACCIÓN            
    # ************************************************
    def execute(self, goal):
        """
        Ejecuta la acción de detección de objetos.

        Este método maneja el flujo principal de la acción, incluyendo la espera de CameraInfo,
        procesamiento de imágenes y publicación de resultados.

        Args:
            goal (ObjectDetectionGoal): Objetivo de la acción recibido.
        """
        # Feedback y resultado de la acción
        feedback = ObjectDetectionFeedback()
        result = ObjectDetectionResult()

        rospy.loginfo("Objetivo recibido.")

        # Esperar a recibir CameraInfo con un timeout
        try:
            rospy.loginfo("Esperando a recibir CameraInfo...")
            camera_info_msg = rospy.wait_for_message(self.camera_info_topic, CameraInfo, timeout=5.0)
            self.camera_info_callback(camera_info_msg)
            rospy.loginfo("Parámetros intrínsecos de la cámara recibidos.")
        except rospy.ROSException:
            rospy.logerr("No se recibió CameraInfo dentro del tiempo de espera.")
            result.success = False
            self.server.set_aborted(result, "No se recibió CameraInfo.")
            return

        # Inicializar la frecuencia de procesamiento
        rate = rospy.Rate(self.process_frequency)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                rospy.loginfo('Objetivo preemptado')
                self.server.set_preempted()
                cv2.destroyAllWindows()
                return
            
            # Procesar la imagen y la posición
            self.process_latest_data(feedback)
            rate.sleep()
        
        # Definir el resultado de la acción
        result.success = True
        self.server.set_succeeded(result)
    
    # ************************************************
    #           PROCESAMIENTO DE LOS DATOS            
    # ************************************************
    def process_latest_data(self, feedback):
        """
        Procesa los datos más recientes de imagen, profundidad y posición.

        Detecta objetos en la imagen, calcula sus coordenadas mundiales y publica la información.

        Args:
            feedback (ObjectDetectionFeedback): Feedback para actualizar la frecuencia de procesamiento.
        """
        with self.lock:
            # Verificar que todos los datos necesarios estén disponibles
            if (self.latest_image is not None and 
                self.latest_position is not None and 
                self.latest_depth_cv is not None and 
                self.fx is not None and self.fy is not None and self.cx is not None and self.cy is not None):
                try:
                    start_processing_time = time.time()

                    # Convertir la imagen de ROS a OpenCV
                    cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")

                    # Procesar la imagen para detectar objetos
                    processed_image, detected_objects = self.detect_objects(cv_image)
                    rospy.logdebug(f"Objetos detectados: {len(detected_objects)}")

                    # Calcular coordenadas mundiales utilizando la imagen de profundidad
                    detected_objects_world, processed_image = self.calculate_world_coordinates(detected_objects, processed_image)
                    rospy.logdebug(f"Objetos en coordenadas mundiales: {len(detected_objects_world)}")

                    # Actualizar el contador de imágenes procesadas
                    self.image_count += 1
                    processing_time = time.time() - start_processing_time
                    self.processing_times.append(processing_time)

                    # Calcular la frecuencia promedio de procesamiento basada en los últimos tiempos de procesamiento
                    if len(self.processing_times) > 0:
                        avg_frequency = 1.0 / (sum(self.processing_times) / len(self.processing_times))
                    else:
                        avg_frequency = 0

                    # Actualizar el feedback con la frecuencia de procesamiento
                    feedback.average_processing_frequency = avg_frequency

                    # Agregar información de todos los objetos detectados
                    for obj_world in detected_objects_world:
                        detected_object_msg = DetectedObject()
                        detected_object_msg.type = obj_world['tipo']
                        detected_object_msg.robot_pose = self.latest_position
                        detected_object_msg.pixel_coordinates = obj_world['pixel_coordinates']
                        detected_object_msg.world_coordinates = obj_world['world_coordinates']

                        self.object_pub.publish(detected_object_msg)
                    
                    # Publicar el feedback actualizado
                    self.server.publish_feedback(feedback)
                    
                    # Mostrar imágenes procesadas
                    self.display_debug_info(cv_image, processed_image, self.latest_depth_cv)

                except Exception as e:
                    rospy.logerr(f"Error al procesar la imagen: {e}")
            else:
                # Esperando a tener todos los datos necesarios
                rospy.logdebug("Esperando a recibir todos los datos necesarios para el procesamiento.")
                pass
    
    # ************************************************
    #          DETECCIÓN DE OBJETOS EN IMÁGENES       
    # ************************************************
    def detect_objects(self, cv_image):
        """
        Detecta objetos de color específico en una imagen RGB.

        Utiliza la conversión a espacio de color HSV y segmentación por color para identificar objetos
        rojos, luego calcula sus coordenadas de píxeles.

        Args:
            cv_image (ndarray): Imagen RGB en formato OpenCV.

        Returns:
            tuple: Imagen procesada y lista de objetos detectados con sus coordenadas de píxel.
        """
        detected_objects = []

        # Procesar la imagen para detectar objetos de color (por ejemplo, rojo)
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 120, 70])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 70])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # Detectar contornos del objeto
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) > 500:  # Filtrar por tamaño mínimo del objeto
                # Obtener el centroide del contorno
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])
                    
                    # Dibujar el contorno y el centroide en la imagen
                    cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 2)
                    cv2.circle(cv_image, (cX, cY), 5, (255, 0, 0), -1)

                    # Crear la información del objeto detectado
                    object_info = {
                        'pixel_coordinates': Point(x=cX, y=cY, z=0.0),
                        'tipo': 'objeto_rojo',
                    }
                    detected_objects.append(object_info)

        return cv_image, detected_objects

    # ************************************************
    #          CÁLCULO DE COORDENADAS MUNDIALES        
    # ************************************************
    def calculate_world_coordinates(self, detected_objects, processed_image):
        """
        Calcula las coordenadas mundiales de los objetos detectados a partir de sus coordenadas de píxel.

        Utiliza la imagen de profundidad y los parámetros intrínsecos de la cámara para convertir
        las coordenadas de píxel a coordenadas mundiales.

        Args:
            detected_objects (list): Lista de objetos detectados con coordenadas de píxel.
            processed_image (ndarray): Imagen procesada para dibujar información adicional.

        Returns:
            tuple: Lista de objetos con coordenadas mundiales y la imagen procesada con anotaciones.
        """
        detected_objects_world = []
        for obj in detected_objects:
            pixel_x = int(obj['pixel_coordinates'].x)
            pixel_y = int(obj['pixel_coordinates'].y)

            rospy.logdebug(f"Procesando objeto en ({pixel_x}, {pixel_y})")

            # Obtener la profundidad en el pixel detectado
            depth = self.get_depth_at_pixel(pixel_x, pixel_y)

            rospy.logdebug(f"Profundidad en ({pixel_x}, {pixel_y}): {depth}")

            if depth is None:
                rospy.logwarn(f"No se pudo obtener profundidad para el objeto en ({pixel_x}, {pixel_y})")
                continue

            # Calcular coordenadas en el marco de la cámara
            X_cam = (pixel_x - self.cx) * depth / self.fx
            Y_cam = (pixel_y - self.cy) * depth / self.fy
            Z_cam = depth

            # Convertir a coordenadas globales
            X_global, Y_global = self.transform_camera_to_global(X_cam, Y_cam, Z_cam)

            rospy.logdebug(f"Coordenadas en el marco de la cámara: ({X_cam}, {Y_cam}, {Z_cam})")

            if X_global is None or Y_global is None:
                rospy.logwarn(f"No se pudo transformar las coordenadas para el objeto en ({pixel_x}, {pixel_y})")
                continue

            obj['pixel_coordinates'].z = depth  # Actualizar la coordenada Z con la profundidad

            # Crear punto en coordenadas globales
            world_coordinates = Point(x=X_global, y=Y_global, z=0.0)  # Asumiendo plano 2D
            detected_objects_world.append({
                'tipo': obj['tipo'],
                'pixel_coordinates': obj['pixel_coordinates'],
                'world_coordinates': world_coordinates
            })

            # ---------- DIBUJAR PROFUNDIDAD SOBRE OBJETOS ------------------        

            # Obtener las coordenadas del pixel
            cX = int(obj['pixel_coordinates'].x)
            cY = int(obj['pixel_coordinates'].y)
            depth = obj['pixel_coordinates'].z

            # Formatear el texto de profundidad
            depth_text = f"{depth:.2f}m"

            # Calcular la posición del texto (arriba del centro detectado)
            text_position = (cX, cY - 10)

            # Dibujar el texto en la imagen procesada
            cv2.putText(processed_image, depth_text, text_position, cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 1, cv2.LINE_AA)
            
        return detected_objects_world, processed_image
    
    # ************************************************
    #       OBTENER PROFUNDIDAD DEL PIXEL             
    # ************************************************
    def get_depth_at_pixel(self, x, y):
        """
        Obtiene la profundidad en un píxel específico de la imagen de profundidad.

        Verifica que las coordenadas estén dentro de los límites de la imagen y maneja valores inválidos.

        Args:
            x (int): Coordenada x del píxel.
            y (int): Coordenada y del píxel.

        Returns:
            float or None: Valor de profundidad en metros o None si es inválido.
        """
        try:
            x = int(x)
            y = int(y)
            # Asegurarse de que las coordenadas estén dentro de la imagen
            if x < 0 or y < 0 or x >= self.latest_depth_cv.shape[1] or y >= self.latest_depth_cv.shape[0]:
                return None

            depth = self.latest_depth_cv[y, x]

            # Manejar valores inválidos
            if np.isnan(depth) or np.isinf(depth) or depth <= 0.0:
                return None

            return depth
        except Exception as e:
            rospy.logerr(f"Error al obtener profundidad en el pixel ({x}, {y}): {e}")
            return None

    # ************************************************
    #       TRANSFORMAR CAMARA A GLOBAL              
    # ************************************************
    def transform_camera_to_global(self, X_cam, Y_cam, Z_cam):
        """
        Transforma las coordenadas desde el marco de la cámara al marco global.

        Utiliza la orientación y posición actual del robot para realizar la transformación.

        Args:
            X_cam (float): Coordenada X en el marco de la cámara.
            Y_cam (float): Coordenada Y en el marco de la cámara.
            Z_cam (float): Coordenada Z en el marco de la cámara.

        Returns:
            tuple: Coordenadas X y Y en el marco global o (None, None) si falla la transformación.
        """
        # Obtener la pose actual del robot
        rospy.logdebug("Transformando coordenadas de cámara a globales...")
        
        try:
            if self.latest_position is None:
                rospy.logerr("Pose del robot no disponible.")
                return None, None

            # Extracción de la orientación en yaw
            orientation_q = self.latest_position.orientation
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(orientation_list)

            # Transformar de cámara a robot local (asumiendo que cámara y robot están alineados)
            X_robot = X_cam
            Y_robot = Y_cam

            # Transformar de robot local a global
            X_global = self.latest_position.position.x + (X_robot * np.cos(yaw) - Y_robot * np.sin(yaw))
            Y_global = self.latest_position.position.y + (X_robot * np.sin(yaw) + Y_robot * np.cos(yaw))

            rospy.logdebug(f"Coordenadas globales: ({X_global}, {Y_global})")

            return X_global, Y_global
        except Exception as e:
            rospy.logerr(f"Error al transformar coordenadas de cámara a globales: {e}")
            return None, None

    # ************************************************
    #           MOSTRAR INFORMACIÓN EN DEBUG          
    # ************************************************
    def display_debug_info(self, raw_image, processed_image, depth_image):
        """
        Muestra información de depuración en las imágenes procesadas.

        Dibuja timestamps, posición del robot y profundidad sobre las imágenes.

        Args:
            raw_image (ndarray): Imagen RGB original.
            processed_image (ndarray): Imagen procesada con detecciones.
            depth_image (ndarray): Imagen de profundidad procesada.
        """
        try:
            # Reemplazar valores inválidos en la imagen de profundidad
            if depth_image is not None:
                depth_image = np.nan_to_num(depth_image, nan=0.0, posinf=0.0, neginf=0.0)
                depth_image_norm = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
                depth_image_norm = depth_image_norm.astype(np.uint8)
            else:
                depth_image_norm = np.zeros_like(raw_image[:, :, 0])

            # Mostrar los timestamps en la imagen para verificar actualización
            timestamp_text = f"Timestamp: {self.latest_image.header.stamp.to_sec()}"
            cv2.putText(raw_image, timestamp_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(processed_image, timestamp_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # Colocar la posición actual del robot en la imagen
            position = self.latest_position
            position_text = f"Robot: ({position.position.x:.2f}, {position.position.y:.2f})"
            cv2.putText(raw_image, position_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(processed_image, position_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(depth_image_norm, position_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            self.process_image_pub.publish(self.bridge.cv2_to_imgmsg(processed_image, "bgr8"))

            if DEBUG:
                # Mostrar las imágenes
                cv2.imshow('Imagen en bruto', raw_image)
                cv2.imshow('Imagen procesada', processed_image)
                cv2.imshow('Imagen de profundidad', depth_image_norm)
                cv2.waitKey(1)
        except Exception as e:
            rospy.logerr(f"Error al mostrar información en DEBUG: {e}")

if __name__ == '__main__':
    try:
        TurtleBotObjectDetectionAction()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
