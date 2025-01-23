#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TurtleBot State Manager

Este script gestiona el estado del TurtleBot utilizando SMACH, una biblioteca de Python
para construir máquinas de estados finitas. Define y maneja diferentes estados como
reposo, exploración, navegación y aproximación a objetos detectados. 

Dependencias:
    - rospy
    - smach
    - smach_ros
    - actionlib
    - std_msgs.msg
    - squad_exploracion.msg
    - sensor_msgs.msg
    - cv_bridge
    - PIL (Pillow)
    - cv2 (OpenCV)
    - move_base_msgs.msg
    - geometry_msgs.msg
    - threading
    - tkinter
"""

import rospy
import smach
import smach_ros
from std_msgs.msg import String, Bool
import actionlib
from squad_exploracion.msg import ObjectDetectionAction, ObjectDetectionGoal, ObjectDetectionFeedback
from squad_exploracion.msg import AutonomousControlAction, AutonomousControlGoal, AutonomousControlResult
from squad_exploracion.msg import ApproachControlAction, ApproachControlFeedback, ApproachControlResult,ApproachControlGoal
from squad_exploracion.msg import DetectedObject
from sensor_msgs.msg import Image as ROSImage
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image, ImageTk
import cv2
import signal
import sys
from squad_planificacion.msg import StationInfo, StationsInfo



from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import threading
import tkinter as tk



# ************************************************
#              VARIABLES GLOBALES
# ************************************************

destino = "HOME"
last_object_detected = DetectedObject()

almacen_estaciones = {"HOME": (0, 0)}


# ************************************************
#              DEFINICIÓN DE ESTADOS
# ************************************************

class BaseState(smach.State):
    """
    Clase base para todos los estados que publica el estado actual.

    Esta clase sirve como clase base para los diferentes estados de la máquina de estados finita (SMACH).
    Publica el estado actual y suscribe a comandos para gestionar transiciones entre estados.

    Hereda de:
        smach.State: Clase base proporcionada por SMACH para definir estados.

    Atributos:
        state_pub (Publisher): Publicador al tópico '/current_state' para publicar el estado actual.
        command_sub (Subscriber): Suscriptor al tópico '/command' para recibir comandos de transición.
        comando (str): Comando actual recibido desde el tópico '/command'.
        satate_name (str): Nombre del estado actual.
    """

    def __init__(self, **kwargs):
        """
        Inicializa la clase base para los estados, configurando el publicador y el suscriptor.

        Args:
            **kwargs: Argumentos adicionales para pasar a la clase base smach.State.
        """
        super(BaseState, self).__init__(**kwargs)
        self.state_pub = rospy.Publisher('current_state', String, queue_size=1)
        self.command_sub = rospy.Subscriber('command', String, self.command_callback)
        self.comando = None
        self.satate_name = "BASE"

    def execute(self, userdata):
        """
        Publica el nombre del estado actual.

        Este método debe ser implementado en las subclases para definir el comportamiento específico del estado.

        Args:
            userdata: Datos de usuario proporcionados por SMACH.

        Returns:
            str: Resultado del estado que determina la transición siguiente.
        """
        self.state_pub.publish(self.satate_name)
        # Implementar en las subclases
        pass

    def command_callback(self, msg):
        """
        Callback para recibir comandos desde el tópico '/command'.

        Actualiza el atributo 'comando' con el comando recibido en minúsculas.

        Args:
            msg (String): Mensaje con el comando recibido.
        """
        self.comando = msg.data.lower()


# Definir los estados posibles del sistema
class EstadoReposo(BaseState):
    def __init__(self):
        BaseState.__init__(self, outcomes=['modo_exploracion', 'ir_a'])
        rospy.loginfo("INICIALIZANDO ESTADO: Reposo")
        self.satate_name = "REPOSO"

    def execute(self, userdata):
        rospy.loginfo("Ejecutando estado: REPOSO")
        self.state_pub.publish(self.satate_name)
        while not rospy.is_shutdown():
            comando = rospy.wait_for_message('command', String).data.lower()
            if comando == "modo exploracion":
                return 'modo_exploracion'
            elif comando.startswith("ir a"):
                return 'ir_a'

class EstadoExploracion(BaseState):
    """
    Estado de Exploración del robot que gestiona la detección de objetos y el control autónomo.

    Este estado coordina los comportamientos del robot durante la fase de exploración, incluyendo la
    detección de objetos, la aproximación a objetos detectados y la navegación autónoma.

    Hereda de:
        BaseState: Clase base para estados en la máquina de estados finita (SMACH).

    Atributos:
        client_object_detection (SimpleActionClient): Cliente de acción para la detección de objetos.
        autonomous_control_pub (Publisher): Publicador para activar/desactivar el control autónomo.
        object_sub (Subscriber): Suscriptor al tópico de objetos detectados.
        state_sub (Subscriber): Suscriptor al tópico del estado actual.
        control_mode_sub (Subscriber): Suscriptor al tópico del modo de control.
        feedback_timeout (Duration): Tiempo máximo de espera para recibir feedback de detección.
        last_feedback_time (Time): Última vez que se recibió feedback.
        detectar_multiples_objetos (bool): Indicador para detectar múltiples objetos.
        acercarse_objetos (bool): Indicador para aproximarse a objetos detectados.
        volver_casa (bool): Indicador para volver a la posición "HOME".
        comando (str): Comando actual recibido.
        control_mode (str): Modo de control actual ('manual' o 'autonomous').
        autonomous_control_active (bool): Estado de la acción de control autónomo.
        objeto_detectado (bool): Indicador de detección de objeto.
        last_object_detected (DetectedObject): Último objeto detectado.
    """

    def __init__(self):
        """
        Inicializa el estado de Exploración, configurando clientes de acción, suscriptores,
        parámetros y variables internas.
        """
        # ************************************************
        #              INICIALIZACIÓN DEL ESTADO
        # ************************************************
        # BaseState.__init__(self, outcomes=['reposo', 'ir_a', 'acercarse_objetivo'])
        BaseState.__init__(self, outcomes=['reposo', 'ir_a'])

        rospy.loginfo("INICIALIZANDO ESTADO: Exploración")
        self.satate_name = "EXPLORACION"

        # ************************************************
        #     INICIALIZACIÓN DE SUSCRIPTORES Y PUBLICADORES
        # ************************************************
        self.client_object_detection = actionlib.SimpleActionClient('object_detection', ObjectDetectionAction)
        #Publicamos un booleano para activar el control autónomo
        self.autonomous_control_pub =  rospy.Publisher('explore/start_frontier_exploration', Bool, queue_size=1)
        self.control_mode_pub = rospy.Publisher('control_mode', String, queue_size=1)

        self.object_sub = rospy.Subscriber("detected_objects", DetectedObject, self.object_callback)
        # self.state_sub = rospy.Subscriber("/current_state", String, self.state_callback)
        self.control_mode_sub = rospy.Subscriber( "control_mode", String, self.control_mode_callback)

        # ************************************************
        #              CARGA DE PARÁMETROS
        # ************************************************
        # Cargar parámetros desde el archivo YAML
        feedback_timeout_param = rospy.get_param('~feedback_objectDetection_timeout', {'value': 2, 'unit': 'seconds'})
        timeout_value = feedback_timeout_param['value']
        timeout_unit = feedback_timeout_param['unit'].lower()

        if timeout_unit == 'seconds':
            self.feedback_timeout = rospy.Duration(timeout_value)
        elif timeout_unit == 'milliseconds':
            self.feedback_timeout = rospy.Duration(timeout_value / 1000.0)
        else:
            rospy.logwarn(
                "Unidad de tiempo desconocida, utilizando valor por defecto de 2 segundos.")
            self.feedback_timeout = rospy.Duration(2)

        self.last_feedback_time = rospy.Time.now()

        self.detectar_multiples_objetos = rospy.get_param(
            '/ESTADO_EXPLORACION/DETECTAR_MULTIPLE', False)
        self.acercarse_objetos = rospy.get_param(
            '/ESTADO_EXPLORACION/ACERCARSE_OBJETO', False)
        self.volver_casa = rospy.get_param(
            '/ESTADO_EXPLORACION/VOLVER_A_CASA', True)

        # ************************************************
        #              INICIALIZACIÓN DE VARIABLES
        # ************************************************
        self.comando = None
        self.control_mode = 'manual'
        self.autonomous_control_active = False  # Variable para controlar el estado de la acción
        self.objeto_detectado = False
        self.last_object_detected = None

    def execute(self, userdata):
        """
        Ejecuta el comportamiento principal del estado de Exploración.

        Este método gestiona la lógica de detección de objetos, control autónomo,
        y las transiciones entre diferentes estados basándose en eventos y comandos.

        Args:
            userdata: Datos de usuario proporcionados por SMACH.

        Returns:
            str: El resultado del estado, que determina la transición siguiente.
        """
        rospy.loginfo("Ejecutando estado: EXPLORACIÓN")
        self.state_pub.publish(self.satate_name)

        # Reiniciar la variable objeto_detectado
        self.objeto_detectado = False

        # ************************************************
        #     ESPERAR AL SERVIDOR DE ACCIÓN
        # ************************************************
        rospy.loginfo(
            'Esperando al servidor de acción de detección de objetos...')
        self.client_object_detection.wait_for_server()
        rospy.loginfo(
            'Servidor de acción de detección de objetos (ACTIVADO)')


        # ************************************************
        #          ENVIAR OBJETIVO DE DETECCIÓN
        # ************************************************
        goal = ObjectDetectionGoal()
        rospy.loginfo('Enviando objetivo de detección de objetos...')
        self.client_object_detection.send_goal(goal, feedback_cb=self.feedback_cb)

        # ************************************************
        #     BUCLE PRINCIPAL DE MONITORIZACIÓN
        # ************************************************
        self.comando = None
        rate = rospy.Rate(1)  # Frecuencia de comprobación
        while not rospy.is_shutdown():
            # Comprobar si ha pasado demasiado tiempo sin feedback
            if rospy.Time.now() - self.last_feedback_time > self.feedback_timeout:
                rospy.logwarn(
                    "No se ha recibido feedback en el tiempo esperado, posible problema con el nodo de detección de objetos.")

            # ------------------MODO CONTROL DEL ROBOT--------------------
            if self.control_mode == 'autonomous':
                if not self.autonomous_control_active:

                    rospy.loginfo('Enviando objetivo de control autónomo...')
                    self.autonomous_control_pub.publish(True)
                    self.autonomous_control_active = True
                else:
                    rospy.loginfo("Control autónomo ya está activo.")
            elif self.control_mode == 'manual':
                if self.autonomous_control_active:
                    rospy.loginfo('Cancelando objetivo de control autónomo...')
                    self.autonomous_control_pub.publish(False)
                    self.autonomous_control_active = False

            # ------------------TRANSICIÓN ENTRE ESTADOS--------------------
            if self.objeto_detectado:
                if self.acercarse_objetos:
                    global last_object_detected
                    last_object_detected = self.last_object_detected

                    rospy.loginfo("Objeto detectado, acercándose al objeto...")
                    self.comando = "acercarse_objetivo"

                elif self.detectar_multiples_objetos:
                    rospy.loginfo("Objeto detectado, detectando más objetos...")

                elif self.volver_casa:
                    global destino

                    rospy.loginfo(f"Objeto detectado, volviendo a HOME... {self.volver_casa}")
                    self.comando = "ir a HOME"
                    # self.state_pub.publish("ir a HOME")
                    destino = "HOME"
                else:
                    rospy.loginfo("Objeto detectado, sin acciones adicionales.")
                    

            # ------------------GESTIÓN DE COMANDOS DE TRANSICIÓN--------------------
            if self.comando is not None:
                if self.comando == "parar":
                    # ************************************************
                    #     CANCELAR OBJETIVOS Y REINICIAR ESTADO
                    # ************************************************
                    if self.client_object_detection.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                        self.client_object_detection.cancel_goal()

                    self.objeto_detectado = False

                    # Cancelar el objetivo de control autónomo si está activo
                    if self.autonomous_control_active:
                        self.autonomous_control_pub.publish(False)
                        self.autonomous_control_active = False

                    return 'reposo'
                elif self.comando.startswith("ir a") :
                    # ************************************************
                    #     TRANSICIÓN A UN DESTINO ESPECÍFICO
                    # ************************************************
                    # TODO: Asegurarse de gestionar que se puedan mandar diferentes destinos

                    if self.client_object_detection.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                        self.client_object_detection.cancel_goal()

                    self.objeto_detectado = False

                    # Cancelar el objetivo de control autónomo si está activo
                    if self.autonomous_control_active:
                        rospy.loginfo('Cancelando objetivo de control autónomo...')
                        self.autonomous_control_pub.publish(False)
                        self.autonomous_control_active = False

                    return 'ir_a'



            rate.sleep()

        #return 'reposo'


    def control_mode_callback(self, msg):
        """
        Callback para el suscriptor del modo de control.

        Cambia el modo de control del robot entre 'manual' y 'autonomous'.

        Args:
            msg (String): Mensaje con el modo de control.
        """
        self.control_mode = msg.data
        rospy.loginfo(f"Modo de control cambiado a: {self.control_mode}")

    def feedback_cb(self, feedback):
        """
        Callback para recibir feedback del servidor de detección de objetos.

        Actualiza el tiempo del último feedback recibido.

        Args:
            feedback: Feedback recibido del servidor de acción de detección de objetos.
        """
        # ************************************************
        #     CALLBACK DE FEEDBACK DEL OBJETIVO
        # ************************************************
        # Actualizar el tiempo del último feedback recibido
        self.last_feedback_time = rospy.Time.now()

    def object_callback(self, msg):
        """
        Callback para el suscriptor de objetos detectados.

        Actualiza el estado interno cuando se detecta un objeto.

        Args:
            msg (DetectedObject): Mensaje con la información del objeto detectado.
        """
        # ************************************************
        #     CALLBACK DE DETECCIÓN DE OBJETOS (ACTUALIZADO)
        # ************************************************
        # Mostrar la información del objeto detectado utilizando el nuevo mensaje personalizado

        self.last_object_detected = msg
        self.objeto_detectado = True
        self.last_feedback_time = rospy.Time.now()
        #rospy.loginfo(f"Objeto detectado: {msg}")





class EstadoNavegacion(BaseState):
    """
    Estado de Navegación del robot que maneja el desplazamiento a una posición predefinida.

    Este estado utiliza el servidor de acción `move_base` para mover el robot a una ubicación específica
    definida en un diccionario de estaciones.

    Hereda de:
        BaseState: Clase base para estados en la máquina de estados finita (SMACH).

    Atributos:
        client (SimpleActionClient): Cliente de acción para `move_base`.
        satate_name (str): Nombre del estado actual.
        comando (str): Comando actual recibido.
    """

    def __init__(self):
        """
        Inicializa el estado de Navegación, configurando el cliente de acción
        y estableciendo el nombre del estado.
        """
        # ************************************************
        #              INICIALIZACIÓN DEL ESTADO
        # ************************************************
        BaseState.__init__(self, outcomes=['reposo', 'modo_exploracion'])
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("INICIALIZANDO ESTADO: Navegación")
        self.state_name = "NAVEGACION"

        # Suscripción al tópico de comandos, "/command"
        self.comando = None
        self.command_sub = rospy.Subscriber('command', String, self.command_callback)
        # Suscriptor al tópico de estaciones
        self.stations_subscriber = rospy.Subscriber('stations_info', StationsInfo, self.stations_callback)



    def stations_callback(self, msg):
        """
        Callback para el suscriptor del tópico de estaciones.

        Actualiza la variable global 'almacen_estaciones' con los datos recibidos.

        Args:
            msg (Stations): Mensaje con los datos de estaciones.
        """
        global almacen_estaciones
        try:
            estaciones = almacen_estaciones
            for station in msg.stations:
                station_id = station.station_id
                coordinates = (station.coordinates.x, station.coordinates.y)
                # Puedes incluir el polígono si es necesario
                # polygon = [(point.x, point.y) for point in station.polygon]
                estaciones[station_id] = coordinates
            almacen_estaciones = estaciones
            rospy.loginfo(f"Actualizado almacen_estaciones: {almacen_estaciones}")
        except AttributeError as e:
            rospy.logerr(f"Error en el acceso a atributos del mensaje de estaciones: {e}")
        except Exception as e:
            rospy.logerr(f"Error inesperado en stations_callback: {e}")


    def command_callback(self, msg):
        """
        Callback para el tópico de comandos. Actualiza el último comando recibido.
        """
        self.comando = msg.data
        if self.comando.startswith("ir a"):
            global destino
            destino = self.comando.split(" ")[-1]

        rospy.loginfo(f"Comando recibido: {self.comando}")

    def execute(self, userdata):
        """
        Ejecuta el comportamiento principal del estado de Navegación.

        Este método envía un objetivo al servidor de acción `move_base` para mover el robot
        a las coordenadas especificadas en el destino y gestiona las transiciones basadas
        en el resultado de la navegación.

        Args:
            userdata: Datos de usuario proporcionados por SMACH.

        Returns:
            str: El resultado del estado, que determina la transición siguiente.
        """
        rospy.loginfo("Ejecutando estado: NAVEGACIÓN")
        self.state_pub.publish(self.state_name)

        # ************************************************
        #     ESPERAR AL SERVIDOR DE ACCIÓN
        # ************************************************
        rospy.loginfo('Esperando al servidor de acción MOVE ACTION...')
        self.client.wait_for_server()
        rospy.loginfo('Servidor de acción MOVE ACTION (ACTIVADO)')

        # ************************************************
        #          OBTENER Y ENVIAR OBJETIVO DE NAVEGACIÓN
        # ************************************************
        # Obtener las coordenadas del destino guardado
        if destino in almacen_estaciones:
            x = almacen_estaciones[destino][0]
            y = almacen_estaciones[destino][1]
            self.moveTo(x, y)
        else:
            rospy.logerr(f"El destino '{destino}' no se encuentra en el diccionario de estaciones.")
            return 'reposo'

        self.comando = None

        # ************************************************
        #     BUCLE PRINCIPAL DE MONITORIZACIÓN
        # ************************************************
        rate = rospy.Rate(2)  # Frecuencia de comprobación (2 Hz)
        while not rospy.is_shutdown():
            # Obtener el estado actual del cliente de acción
            state = self.client.get_state()
            state_str = actionlib.GoalStatus.to_string(state)
            rospy.loginfo(f"Estado actual del cliente de acción MoveBaseAction: {state_str}")

            # Verificar si la acción ha finalizado
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo(f"Llegando a '{destino}', pasando a estado REPOSO.")
                return 'reposo'
            elif state in [actionlib.GoalStatus.PREEMPTED, actionlib.GoalStatus.ABORTED]:
                rospy.logwarn("El objetivo fue cancelado o falló, pasando a estado REPOSO.")
                return 'reposo'

            # ************************************************
            #     GESTIÓN DE COMANDOS DE TRANSICIÓN
            # ************************************************
            if self.comando is not None:
                if self.comando == "parar":
                    # ************************************************
                    #     CANCELAR OBJETIVOS Y REINICIAR ESTADO
                    # ************************************************
                    if self.client.get_state() in [actionlib.GoalStatus.ACTIVE, 
                                                   actionlib.GoalStatus.PENDING]:
                        self.client.cancel_goal()
                    return 'reposo'
                elif self.comando == "modo exploracion":
                    # ************************************************
                    #     TRANSICIÓN A MODO EXPLORACIÓN
                    # ************************************************
                    if self.client.get_state() in [actionlib.GoalStatus.ACTIVE, 
                                                   actionlib.GoalStatus.PENDING]:
                        self.client.cancel_goal()
                    return 'modo_exploracion'
                else:
                    rospy.loginfo(f"Comando no reconocido: {self.comando}")

            rate.sleep()

        #return 'reposo'

    def moveTo(self, x, y):
        """
        Envía un objetivo al servidor de acción `move_base` para mover el robot a las coordenadas especificadas.

        Args:
            x (float): Coordenada X del destino.
            y (float): Coordenada Y del destino.
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.w = 1.0  # Orientación fija hacia adelante

        self.client.send_goal(goal)
        rospy.loginfo(f"Enviando goal a las coordenadas x: {x}, y: {y}")

    def control_mode_callback(self, msg):
        """
        Callback para el suscriptor del modo de control.

        Cambia el modo de control del robot entre 'manual' y 'autonomous'.

        Args:
            msg (String): Mensaje con el modo de control.
        """
        #self.control_mode = msg.data
        #rospy.loginfo(f"Modo de control cambiado a: {self.control_mode}")



class TurtleBotStateManager:
    """
    Gestor de Estados para el TurtleBot utilizando SMACH.

    Esta clase inicializa la máquina de estados finita (SMACH) para gestionar los diferentes
    comportamientos del TurtleBot, incluyendo reposo, exploración, navegación y aproximación a
    objetivos. También configura la introspección para visualizar la máquina de estados y
    lanza la interfaz gráfica para la interacción del usuario.

    Atributos:
        sm (StateMachine): Máquina de estados finita que define los diferentes estados y sus transiciones.
        state_machine_thread (Thread): Hilo que ejecuta la máquina de estados.
    """

    def __init__(self):
        """
        Inicializa el gestor de estados del TurtleBot, configurando la máquina de estados,
        la introspección y la interfaz gráfica.
        """
        # ************************************************
        #              INICIALIZACIÓN DEL NODO ROS
        # ************************************************
        rospy.init_node('turtlebot_state_manager', anonymous=True)

        # ************************************************
        #              INICIALIZACIÓN DE LA MÁQUINA DE ESTADOS
        # ************************************************
        self.sm = smach.StateMachine(outcomes=['sistema_finalizado'])
        with self.sm:
            # Agregar el estado REPOSO
            smach.StateMachine.add(
                'REPOSO',
                EstadoReposo(),
                transitions={
                    'modo_exploracion': 'EXPLORACION',
                    'ir_a': 'NAVEGACION',
                }
            )

            # Agregar el estado EXPLORACION
            smach.StateMachine.add(
                'EXPLORACION',
                EstadoExploracion(),
                transitions={
                    'reposo': 'REPOSO',
                    'ir_a': 'NAVEGACION'#,
                    # 'acercarse_objetivo': 'APPROACH'
                }
            )

            # Agregar el estado NAVEGACION
            smach.StateMachine.add(
                'NAVEGACION',
                EstadoNavegacion(),
                transitions={
                    'reposo': 'REPOSO',
                    'modo_exploracion': 'EXPLORACION'
                }
            )


        # ************************************************
        #      CONFIGURACIÓN DEL INTROSPECTOR SMACH
        # ************************************************
        sis = smach_ros.IntrospectionServer(
            'turtlebot_state_manager_introspection',
            self.sm,
            '/SM_ROOT'
        )
        sis.start()

        # ************************************************
        #     EJECUCIÓN DE LA MÁQUINA DE ESTADOS EN UN HILO
        # ************************************************
        self.state_machine_thread = threading.Thread(target=self.run_state_machine)
        self.state_machine_thread.start()

        # Mantener el Introspection Server activo
        rospy.spin()

        # ************************************************
        #              DETENER INTROSPECCIÓN
        # ************************************************
        sis.stop()

         # Esperar a que el hilo de la máquina de estados termine
        if self.state_machine_thread.is_alive():
            self.state_machine_thread.join()

            
    def run_state_machine(self):
        """
        Ejecuta la máquina de estados finita (SMACH).

        Este método es ejecutado en un hilo separado para no bloquear el hilo principal.
        """
        # outcome = self.sm.execute()
        # rospy.loginfo(f"MÁQUINA DE ESTADOS FINALIZADA con resultado: {outcome}")

        try:
            outcome = self.sm.execute()
            rospy.loginfo(f"MÁQUINA DE ESTADOS FINALIZADA con resultado: {outcome}")
        except rospy.ROSInterruptException:
            rospy.loginfo("Máquina de estados interrumpida.")




if __name__ == '__main__':
    try:
        manager = TurtleBotStateManager()
    except rospy.ROSInterruptException:
        pass
