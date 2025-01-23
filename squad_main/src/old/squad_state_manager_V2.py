#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TurtleBot State Manager

Este script gestiona el estado del TurtleBot utilizando SMACH, una biblioteca de Python
para construir máquinas de estados finitas. Define y maneja diferentes estados como
reposo, exploración, navegación y aproximación a objetos detectados. Además, integra
una interfaz gráfica de usuario (GUI) construida con Tkinter para permitir la interacción
manual y visualización del estado actual del robot y la imagen de la cámara.

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
from std_msgs.msg import String
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


from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Twist
import threading
import tkinter as tk



# ************************************************
#              VARIABLES GLOBALES
# ************************************************

destino = "HOME"
last_object_detected = DetectedObject()

almacen_estaciones = {"HOME": (5, 4), "ESTACION1": (1, 1), "ESTACION2": (2, 2), "ESTACION3": (3, 3)}


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
        self.state_pub = rospy.Publisher('/current_state', String, queue_size=1)
        self.command_sub = rospy.Subscriber('/command', String, self.state_callback)
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

    def state_callback(self, msg):
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
            comando = rospy.wait_for_message('/command', String).data.lower()
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
        client_autonomous_control (SimpleActionClient): Cliente de acción para el control autónomo.
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
        BaseState.__init__(self, outcomes=['reposo', 'ir_a', 'acercarse_objetivo'])
        rospy.loginfo("INICIALIZANDO ESTADO: Exploración")
        self.satate_name = "EXPLORACION"

        # ************************************************
        #     INICIALIZACIÓN DE SUSCRIPTORES Y PUBLICADORES
        # ************************************************
        self.client_object_detection = actionlib.SimpleActionClient(
            'object_detection', ObjectDetectionAction)
        self.client_autonomous_control = actionlib.SimpleActionClient(
            'autonomous_control', AutonomousControlAction)

        self.object_sub = rospy.Subscriber(
            "/detected_objects", DetectedObject, self.object_callback)
        self.state_sub = rospy.Subscriber(
            "/current_state", String, self.state_callback)
        self.control_mode_sub = rospy.Subscriber(
            "/control_mode", String, self.control_mode_callback)

        # ************************************************
        #              CARGA DE PARÁMETROS
        # ************************************************
        # Cargar parámetros desde el archivo YAML
        feedback_timeout_param = rospy.get_param(
            '/feedback_objectDetection_timeout', {'value': 2, 'unit': 'seconds'})
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

        rospy.loginfo(
            'Esperando al servidor de acción de control autónomo...')
        self.client_autonomous_control.wait_for_server()
        rospy.loginfo(
            'Servidor de acción de control autónomo (ACTIVADO)')

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
                    if not self.client_autonomous_control.wait_for_server(rospy.Duration(1)):
                        rospy.logwarn(
                            "Servidor de acción de control autónomo no disponible.")
                    else:
                        goal = AutonomousControlGoal()
                        rospy.loginfo('Enviando objetivo de control autónomo...')
                        self.client_autonomous_control.send_goal(goal)
                        self.autonomous_control_active = True
                else:
                    rospy.loginfo("Control autónomo ya está activo.")
            elif self.control_mode == 'manual':
                if self.autonomous_control_active:
                    if self.client_autonomous_control.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                        rospy.loginfo(
                            'Cancelando objetivo de control autónomo...')
                        self.client_autonomous_control.cancel_goal()
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
                        if self.client_autonomous_control.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                            rospy.loginfo(
                                'Cancelando objetivo de control autónomo...')
                            self.client_autonomous_control.cancel_goal()
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
                        if self.client_autonomous_control.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                            rospy.loginfo(
                                'Cancelando objetivo de control autónomo...')
                            self.client_autonomous_control.cancel_goal()
                            self.autonomous_control_active = False

                    return 'ir_a'

                elif self.comando == "acercarse_objetivo":
                    # ************************************************
                    #     TRANSICIÓN PARA ACERCARSE AL OBJETIVO DETECTADO
                    # ************************************************
                    if self.client_object_detection.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                        self.client_object_detection.cancel_goal()

                    self.objeto_detectado = False

                    # Cancelar el objetivo de control autónomo si está activo
                    if self.autonomous_control_active:
                        if self.client_autonomous_control.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
                            rospy.loginfo(
                                'Cancelando objetivo de control autónomo...')
                            self.client_autonomous_control.cancel_goal()
                            self.autonomous_control_active = False

                    return 'acercarse_objetivo'

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



class EstadoApproach(BaseState):
    """
    Estado de Aproximación del robot que maneja el acercamiento a objetos detectados.

    Este estado gestiona la lógica para que el robot se acerque a un objeto detectado utilizando
    un servidor de acción específico para el control de aproximación.

    Hereda de:
        BaseState: Clase base para estados en la máquina de estados finita (SMACH).

    Atributos:
        client_approach (SimpleActionClient): Cliente de acción para el control de aproximación.
        detectar_multiples_objetos (bool): Indicador para detectar múltiples objetos.
        acercarse_objetos (bool): Indicador para acercarse a objetos detectados.
        volver_casa (bool): Indicador para volver a la posición "HOME".
        last_object_detected (DetectedObject): Último objeto detectado.
        comando (str): Comando actual recibido.
    """

    def __init__(self):
        """
        Inicializa el estado de Aproximación, configurando el cliente de acción,
        parámetros y variables internas.
        """
        # ************************************************
        #              INICIALIZACIÓN DEL ESTADO
        # ************************************************
        BaseState.__init__(self, outcomes=['reposo', 'ir_a', 'modo_exploracion'])
        rospy.loginfo("INICIALIZANDO ESTADO: Approach") 
        self.satate_name = "APPROACH"

        # ************************************************
        #     INICIALIZACIÓN DE SUSCRIPTORES Y PUBLICADORES
        # ************************************************
        self.client_approach = actionlib.SimpleActionClient(
            'approach_object', ApproachControlAction)

        # ************************************************
        #              CARGA DE PARÁMETROS
        # ************************************************
        self.detectar_multiples_objetos = rospy.get_param(
            '/ESTADO_EXPLORACION/DETECTAR_MULTIPLE', False)
        self.acercarse_objetos = rospy.get_param(
            '/ESTADO_EXPLORACION/ACERCARSE_OBJETO', False)
        self.volver_casa = rospy.get_param(
            '/ESTADO_EXPLORACION/VOLVER_CASA', True)

        # ************************************************
        #              INICIALIZACIÓN DE VARIABLES
        # ************************************************
        self.last_object_detected = None
        self.comando = None  # Inicialización del comando

    def execute(self, userdata):
        """
        Ejecuta el comportamiento principal del estado de Aproximación.

        Este método gestiona la lógica para acercarse a un objeto detectado y las transiciones
        hacia otros estados basándose en el resultado de la aproximación.

        Args:
            userdata: Datos de usuario proporcionados por SMACH.

        Returns:
            str: El resultado del estado, que determina la transición siguiente.
        """
        rospy.loginfo("Ejecutando estado: APPROACH")
        self.state_pub.publish(self.satate_name)

        # ************************************************
        #     ESPERAR AL SERVIDOR DE ACCIÓN
        # ************************************************
        rospy.loginfo('Esperando al servidor de acción de approach...')
        self.client_approach.wait_for_server()
        rospy.loginfo('Servidor de acción de approach (ACTIVADO)')

        # ************************************************
        #          ENVIAR OBJETIVO DE APPROACH
        # ************************************************
        goal = ApproachControlGoal()
        rospy.loginfo('Enviando objetivo de approach...')

        # Asignar el último objeto detectado al objetivo
        goal.info_objetivo = last_object_detected
        goal.method = "vision"

        rospy.loginfo(
            f"Objeto detectado: Tipo: {last_object_detected.type}, "
            f"Coordenadas del Robot: "
            f"({last_object_detected.robot_pose.position.x:.2f}, "
            f"{last_object_detected.robot_pose.position.y:.2f}, "
            f"{last_object_detected.robot_pose.position.z:.2f})"
        )

        # Enviar el objetivo al servidor de acción con callback de feedback
        self.client_approach.send_goal(goal, feedback_cb=self.feedback_cb)
        rospy.loginfo('Objetivo enviado...')

        self.comando = None

        # ************************************************
        #     BUCLE PRINCIPAL DE MONITORIZACIÓN
        # ************************************************
        rate = rospy.Rate(1)  # Frecuencia de comprobación (1 Hz)
        while not rospy.is_shutdown():
            # Obtener el estado actual del cliente de acción
            state = self.client_approach.get_state()
            state_str = actionlib.GoalStatus.to_string(state)

            rospy.loginfo(f"Estado actual del cliente de acción ApproachControlAction: {state_str}")
            rospy.loginfo(f"Comando actual: {self.comando}")

            # Verificar si la acción ha finalizado
            if state in [actionlib.GoalStatus.SUCCEEDED, 
                        actionlib.GoalStatus.ABORTED, 
                        actionlib.GoalStatus.PREEMPTED]:
                if self.detectar_multiples_objetos:
                    rospy.loginfo("Fin acercando a objetos, detectando más objetos...")
                    self.comando = 'modo exploracion'

                elif self.volver_casa:
                    global destino

                    rospy.loginfo("Fin acercando a objetos, volviendo a HOME...")
                    self.comando = "ir a HOME"
                    # self.state_pub.publish("ir a HOME")
                    destino = "HOME"
                    self.objeto_detectado = False

            # ************************************************
            #     GESTIÓN DE COMANDOS DE TRANSICIÓN
            # ************************************************
            if self.comando is not None:
                if self.comando == "parar":
                    # ************************************************
                    #     CANCELAR OBJETIVOS Y REINICIAR ESTADO
                    # ************************************************
                    if self.client_approach.get_state() in [actionlib.GoalStatus.ACTIVE, 
                                                           actionlib.GoalStatus.PENDING]:
                        self.client_approach.cancel_goal()

                    return 'reposo'
                
                elif self.comando == "modo exploracion":
                    # ************************************************
                    #     TRANSICIÓN A MODO EXPLORACIÓN
                    # ************************************************
                    if self.client_approach.get_state() in [actionlib.GoalStatus.ACTIVE, 
                                                           actionlib.GoalStatus.PENDING]:
                        self.client_approach.cancel_goal()
                    return 'modo_exploracion'
                
                elif self.comando.startswith("ir a"):
                    # ************************************************
                    #     TRANSICIÓN A UN DESTINO ESPECÍFICO
                    # ************************************************
                    # TODO: Asegurarse de gestionar que se puedan mandar diferentes destinos
                    if self.client_approach.get_state() in [actionlib.GoalStatus.ACTIVE, 
                                                           actionlib.GoalStatus.PENDING]:
                        self.client_approach.cancel_goal()

                    return 'ir_a'

            rate.sleep()

        return 'reposo'

    def control_mode_callback(self, msg):
        """
        Callback para el suscriptor del modo de control.

        Cambia el modo de control del robot entre 'manual' y 'autonomous'.

        Args:
            msg (String): Mensaje con el modo de control.
        """
        self.control_mode = msg.data
        #rospy.loginfo(f"Modo de control cambiado a: {self.control_mode}")

    def feedback_cb(self, feedback):
        """
        Callback para recibir feedback del servidor de aproximación.

        Actualmente, este callback no realiza ninguna acción adicional.

        Args:
            feedback: Feedback recibido del servidor de acción de aproximación.
        """
        # ************************************************
        #     CALLBACK DE FEEDBACK DEL OBJETIVO
        # ************************************************
        pass




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

        # Suscripción al tópico de comandos, por ejemplo: "/robot_command"
        # Ajusta el nombre del tópico y el tipo de mensaje según tus necesidades
        self.comando = None
        self.command_sub = rospy.Subscriber('/robot_command', String, self.command_callback)

    def command_callback(self, msg):
        """
        Callback para el tópico de comandos. Actualiza el último comando recibido.
        """
        self.comando = msg.data
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
        gui (InterfazManager): Gestor de la interfaz gráfica de usuario.
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
                    'ir_a': 'NAVEGACION',
                    'acercarse_objetivo': 'APPROACH'
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

            # Agregar el estado APPROACH
            smach.StateMachine.add(
                'APPROACH',
                EstadoApproach(),
                transitions={
                    'reposo': 'REPOSO',
                    'ir_a': 'NAVEGACION',
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

        # ************************************************
        #              INICIALIZACIÓN DE LA GUI
        # ************************************************
        self.gui = InterfazManager(self.sm)


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





class InterfazManager:
    """
    Gestor de la Interfaz Gráfica de Usuario (GUI) para la Máquina de Estados del TurtleBot.

    Esta clase maneja la creación y actualización de la interfaz gráfica, la interacción con
    los tópicos de ROS para enviar comandos y recibir actualizaciones del estado del robot.
    También gestiona la visualización de imágenes de la cámara en tiempo real.

    Atributos:
        state_machine (StateMachine): Máquina de estados finita (SMACH) para gestionar los estados del robot.
        state_names (list): Lista de nombres de los estados definidos en la máquina de estados.
        current_state (str): Nombre del estado actual del robot.
        command_state_mapping (dict): Mapeo de comandos a nombres de estados.
        cmd_vel_pub (Publisher): Publicador al tópico '/cmd_vel' para enviar comandos de movimiento.
        state_pub (Publisher): Publicador al tópico '/command' para enviar comandos de estado.
        control_mode_pub (Publisher): Publicador al tópico '/control_mode' para cambiar el modo de control.
        control_mode_sub (Subscriber): Suscriptor al tópico '/control_mode' para recibir actualizaciones del modo de control.
        bridge (CvBridge): Objeto para convertir mensajes de ROS Image a OpenCV.
        camera_sub (Subscriber): Suscriptor al tópico de la cámara.
        normal_camera_topic (str): Tópico de la cámara normal.
        action_camera_topic (str): Tópico de la cámara procesada para acciones.
        current_camera_topic (str): Tópico de la cámara actualmente suscrito.
        camera_image (ImageTk.PhotoImage): Imagen actual de la cámara para visualizar en la GUI.
        control_mode (str): Modo de control actual ('manual' o 'autonomous').
        window (Tk): Ventana principal de la GUI.
        state_buttons (dict): Diccionario de botones de estado en la GUI.
        exploration_controls_frame (Frame): Marco para los controles de exploración en la GUI.
    """

    def __init__(self, state_machine):
        """
        Inicializa el gestor de la interfaz gráfica, configurando publicadores, suscriptores,
        la GUI y los controles de exploración.

        Args:
            state_machine (StateMachine): Máquina de estados finita (SMACH) para gestionar los estados del robot.
        """
        self.state_machine = state_machine
        self.state_names = list(state_machine._states.keys())
        self.current_state = None

        # ************************************************
        #          MAPEADO DE COMANDOS A ESTADOS
        # ************************************************
        self.command_state_mapping = {
            'parar': 'REPOSO',
            'modo exploracion': 'EXPLORACION',
            'ir a': 'NAVEGACION',
            'acercarse_objetivo': 'APPROACH'
        }

        # ************************************************
        #       INICIALIZACIÓN DE PUBLICADORES Y SUSCRIPTORES
        # ************************************************
        # Publicador para comandos de movimiento
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Publicador para enviar comandos de estado
        self.state_pub = rospy.Publisher('/command', String, queue_size=1)

        # Publicador y suscriptor para el modo de control
        self.control_mode_pub = rospy.Publisher('/control_mode', String, queue_size=1)
        self.control_mode_sub = rospy.Subscriber('/control_mode', String, self.control_mode_callback)

        # Suscriptor para obtener el estado actual del robot
        rospy.Subscriber('/current_state', String, self.current_state_callback)

        # Suscriptor para la cámara
        self.bridge = CvBridge()
        self.camera_sub = None
        self.normal_camera_topic = rospy.get_param('/image_topic', '/camera')
        self.action_camera_topic = rospy.get_param('/processed_image_topic', '/processed_image')
        self.current_camera_topic = self.normal_camera_topic
        self.camera_image = None

        # ************************************************
        #              MODOS DE EXPLORACIÓN
        # ************************************************
        self.control_mode = 'manual'

        # ************************************************
        #           INICIALIZACIÓN DE LA INTERFAZ
        # ************************************************
        # Configurar la interfaz gráfica
        self.setup_gui()

        # Iniciar la suscripción a la cámara
        self.subscribe_to_camera(self.current_camera_topic)

        # Iniciar el spinning de ROS en un hilo separado
        threading.Thread(target=rospy.spin).start()

       # Manejar el evento de cierre de la ventana
        self.window.protocol("WM_DELETE_WINDOW", self.on_closing)

        # Manejar la señal Ctrl+C
        signal.signal(signal.SIGINT, self.signal_handler)

        # Iniciar el spinning de ROS en un hilo separado
        self.ros_thread = threading.Thread(target=rospy.spin)
        self.ros_thread.start()


        # Iniciar el bucle principal de la GUI
        self.window.mainloop()

    def current_state_callback(self, msg):
        """
        Callback para el suscriptor del estado actual.

        Actualiza el estado actual y resalta el botón correspondiente en la GUI.

        Args:
            msg (String): Mensaje con el nombre del estado actual.
        """
        self.current_state = msg.data
        # Actualizar la GUI para resaltar el estado actual
        self.update_state_highlight()

    def setup_gui(self):
        """
        Configura la interfaz gráfica de usuario utilizando Tkinter.

        Crea la ventana principal, los botones de control, la visualización de la cámara
        y los controles de exploración.
        """
        self.window = tk.Tk()
        self.window.title("Interfaz de la Máquina de Estados")
        self.window.configure(bg='#f0f0f0')
        self.window.geometry("1200x800")  # Tamaño ajustable según necesidad

        # Título
        self.title_label = tk.Label(
            self.window,
            text="Control de la Máquina de Estados",
            font=('Arial', 16, 'bold'),
            bg='#f0f0f0'
        )
        self.title_label.pack(pady=10)

        # Crear un marco principal para las tres columnas
        self.main_frame = tk.Frame(self.window, bg='#f0f0f0')
        self.main_frame.pack(fill=tk.BOTH, expand=True, padx=20, pady=20)

        # Columna Izquierda: Botones de Control
        self.left_frame = tk.Frame(self.main_frame, bg='#e0e0e0', width=200)
        self.left_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10)

        # Columna Central: Imagen de la Cámara
        self.center_frame = tk.Frame(self.main_frame, bg='#d0d0d0')
        self.center_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10)

        # Columna Derecha: Secciones Futuras
        self.right_frame = tk.Frame(self.main_frame, bg='#c0c0c0', width=200)
        self.right_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10)

        # ************************************************
        #              BOTONES DE CONTROL
        # ************************************************
        self.state_buttons = {}
        for state_name in self.state_names:
            command = self.get_command_for_state(state_name)
            initial_color = '#2196F3' if state_name == 'REPOSO' else '#4CAF50'
            button = tk.Button(
                self.left_frame,
                text=state_name,
                command=lambda c=command: self.on_state_button_click(c),
                width=20,
                height=2,
                bg=initial_color,
                fg='white',
                font=('Arial', 12)
            )
            button.pack(pady=5)
            self.state_buttons[state_name] = button

        # ************************************************
        #        CONTROLES DE EXPLORACIÓN
        # ************************************************
        self.exploration_controls_frame = tk.Frame(
            self.right_frame,
            bg='#d0d0d0',
            padx=20,
            pady=20
        )
        self.exploration_controls_frame.pack(pady=10)

        # Crear controles de exploración
        self.create_exploration_controls()

        # ************************************************
        #              VISUALIZACIÓN DE LA CÁMARA
        # ************************************************
        self.camera_label = tk.Label(self.center_frame, bg='#d0d0d0')
        self.camera_label.pack(fill=tk.BOTH, expand=True)

    def create_exploration_controls(self):
        """
        Crea los controles de exploración en la interfaz gráfica.

        Incluye un botón para cambiar el modo de exploración y las instrucciones para el
        control manual mediante el teclado.
        """
        self.exploration_label = tk.Label(
            self.exploration_controls_frame,
            text="Controles de Exploración",
            font=('Arial', 14),
            bg='#d0d0d0'
        )
        self.exploration_label.pack(pady=10)

        # Botón para cambiar el modo de exploración
        self.change_exploration_button = tk.Button(
            self.exploration_controls_frame,
            text="Cambiar Modo de Exploración",
            command=self.change_exploration_mode,
            width=25,
            height=2,
            bg='#2196F3',
            fg='white',
            font=('Arial', 12)
        )
        self.change_exploration_button.pack(pady=5)

        # Instrucciones para el control manual
        self.instructions_label = tk.Label(
            self.exploration_controls_frame,
            text="Usa las flechas para mover el robot",
            font=('Arial', 12),
            bg='#d0d0d0'
        )
        self.instructions_label.pack(pady=10)

        # Vincular teclas de flecha para control manual
        self.window.bind('<Up>', self.on_arrow_up)
        self.window.bind('<Down>', self.on_arrow_down)
        self.window.bind('<Left>', self.on_arrow_left)
        self.window.bind('<Right>', self.on_arrow_right)
        self.window.bind('<KeyRelease>', self.on_key_release)

    def get_command_for_state(self, state_name):
        """
        Obtiene el comando correspondiente para un nombre de estado dado.

        Args:
            state_name (str): Nombre del estado.

        Returns:
            str or None: Comando asociado al estado o None si no hay mapeo.
        """
        for command, state in self.command_state_mapping.items():
            if state == state_name:
                return command
        return None

    def on_state_button_click(self, command):
        """
        Maneja el evento de clic en un botón de estado.

        Publica el comando correspondiente al tópico '/command' para cambiar el estado
        de la máquina de estados.

        Args:
            command (str): Comando a publicar.
        """
        if command:
            self.state_pub.publish(command)
            rospy.loginfo("Comando publicado: %s", command)
        else:
            rospy.logwarn("No hay comando mapeado para este estado.")

    def update_state_highlight(self):
        """
        Actualiza la interfaz gráfica para resaltar el botón correspondiente al estado actual
        y mostrar u ocultar los controles de exploración según sea necesario.
        """
        print(f"Estado actual: {self.current_state}")
        for state_name, button in self.state_buttons.items():
            if state_name == self.current_state:
                button.config(bg='#2196F3')
            else:
                button.config(bg='#4CAF50')

        #TODO: Aplicar una nueva lgoca para el itnercambio de topics ya uqe hay la posibilidad de que
        #       en ele stado approach, mientras está en navegación, no esté el servidor de accion de la camara 
        #       activado y estemos unr ato con la imagen congelada 

        # Mostrar u ocultar controles de exploración según el estado actual
        if self.current_state in ["EXPLORACION", "APPROACH"]:
            self.exploration_controls_frame.pack(padx=20, pady=20)
            # Cambiar la suscripción de la cámara si es necesario
            self.switch_camera_topic(self.action_camera_topic)
        else:
            self.exploration_controls_frame.pack_forget()
            # Cambiar la suscripción de la cámara al tópico normal
            self.switch_camera_topic(self.normal_camera_topic)

    def on_arrow_up(self, event):
        """
        Maneja la pulsación de la tecla de flecha hacia arriba para mover el robot hacia adelante.

        Args:
            event: Evento de la tecla pulsada.
        """
        twist = Twist()
        twist.linear.x = 0.5  # Ajusta la velocidad según sea necesario
        self.cmd_vel_pub.publish(twist)

    def on_arrow_down(self, event):
        """
        Maneja la pulsación de la tecla de flecha hacia abajo para mover el robot hacia atrás.

        Args:
            event: Evento de la tecla pulsada.
        """
        twist = Twist()
        twist.linear.x = -0.5
        self.cmd_vel_pub.publish(twist)

    def on_arrow_left(self, event):
        """
        Maneja la pulsación de la tecla de flecha hacia la izquierda para girar el robot a la izquierda.

        Args:
            event: Evento de la tecla pulsada.
        """
        twist = Twist()
        twist.angular.z = 0.5
        self.cmd_vel_pub.publish(twist)

    def on_arrow_right(self, event):
        """
        Maneja la pulsación de la tecla de flecha hacia la derecha para girar el robot a la derecha.

        Args:
            event: Evento de la tecla pulsada.
        """
        twist = Twist()
        twist.angular.z = -0.5
        self.cmd_vel_pub.publish(twist)

    def on_key_release(self, event):
        """
        Maneja el evento de liberación de cualquier tecla para detener el movimiento del robot.

        Args:
            event: Evento de la tecla liberada.
        """
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def control_mode_callback(self, msg):
        """
        Callback para el suscriptor del modo de control.

        Actualiza el modo de control actual y realiza acciones adicionales si es necesario.

        Args:
            msg (String): Mensaje con el modo de control ('manual' o 'autonomous').
        """
        self.control_mode = msg.data

    def change_exploration_mode(self):
        """
        Cambia el modo de exploración entre 'manual' y 'autonomous'.

        Actualiza la GUI y publica el nuevo modo de control al tópico '/control_mode'.
        """
        # Lógica para cambiar el modo de exploración
        if self.control_mode == 'manual':
            self.control_mode = 'autonomous'
            self.change_exploration_button.config(text="Cambiar a Modo Manual", bg='#2196F3')
            self.instructions_label.config(text="Control Autónomo Activado")
            self.control_mode_pub.publish('autonomous')
        else:
            self.control_mode = 'manual'
            self.change_exploration_button.config(text="Cambiar a Modo Autónomo", bg='#4CAF50')
            self.instructions_label.config(text="Usa las flechas para mover el robot")
            self.control_mode_pub.publish('manual')

    # ************************************************
    #                SUSCRIPCIÓN A LA CÁMARA
    # ************************************************

    def subscribe_to_camera(self, topic):
        """
        Suscribe al tópico de la cámara especificado.

        Desuscribe del tópico de cámara anterior si existe y suscribe al nuevo tópico.

        Args:
            topic (str): Tópico de la cámara al que suscribirse.
        """
        if self.camera_sub:
            self.camera_sub.unregister()

        self.current_camera_topic = topic
        self.camera_sub = rospy.Subscriber(topic, ROSImage, self.camera_callback)

    def switch_camera_topic(self, new_topic):
        """
        Cambia el tópico de la cámara a uno nuevo si es diferente al actual.

        Args:
            new_topic (str): Nuevo tópico de la cámara al que suscribirse.
        """
        if self.current_camera_topic != new_topic:
            self.subscribe_to_camera(new_topic)

    def camera_callback(self, msg):
        """
        Callback para el suscriptor de la cámara.

        Convierte la imagen recibida de ROS a un formato compatible con Tkinter y la
        muestra en la GUI.

        Args:
            msg (ROSImage): Mensaje de imagen recibido del tópico de la cámara.
        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Convertir la imagen de BGR a RGB
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            # Convertir la imagen a PIL
            pil_image = Image.fromarray(cv_image)
            # Redimensionar la imagen para ajustarla al tamaño del label
            pil_image = pil_image.resize((600, 400), Image.LANCZOS)
            # Convertir la imagen a ImageTk
            tk_image = ImageTk.PhotoImage(image=pil_image)
            # Actualizar la imagen en el label
            self.camera_label.configure(image=tk_image)
            self.camera_label.image = tk_image
        except CvBridgeError as e:
            rospy.logerr(f"Error al convertir la imagen de la cámara: {e}")
        except Exception as e:
            rospy.logerr(f"Error inesperado en camera_callback: {e}")

    # ************************************************
    #            GESTION CIERRE PROGRAMA
    # ************************************************

    def on_closing(self):
        rospy.loginfo("Cerrando la interfaz gráfica.")
        rospy.signal_shutdown("Interfaz cerrada por el usuario.")
        self.window.destroy()
        # Asegurarse de que los hilos se detengan
        if self.ros_thread.is_alive():
            self.ros_thread.join()
        sys.exit()

    def signal_handler(self, sig, frame):
        rospy.loginfo("Señal de interrupción recibida. Cerrando la interfaz.")
        self.window.quit()
        rospy.signal_shutdown("Interfaz cerrada por señal de interrupción.")
        # Asegurarse de que los hilos se detengan
        if self.ros_thread.is_alive():
            self.ros_thread.join()
        sys.exit()

if __name__ == '__main__':
    try:
        manager = TurtleBotStateManager()
    except rospy.ROSInterruptException:
        pass