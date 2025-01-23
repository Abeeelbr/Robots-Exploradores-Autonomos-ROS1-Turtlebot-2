#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Twist, Pose
from nav_msgs.msg import Odometry
from squad_exploracion.msg import (
    ApproachControlAction,
    ApproachControlFeedback,
    ApproachControlResult,
    DetectedObject,
    ObjectDetectionAction,
    ObjectDetectionGoal
)
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import math
import threading
import tf
from geometry_msgs.msg import Pose, Point, Quaternion


class ApproachObjectActionServer(object):
    """
    Servidor de acción para la aproximación al objeto detectado.

    Esta clase implementa un servidor de acción que coordina la aproximación del robot hacia
    un objeto detectado en el entorno. Utiliza una máquina de estados finitos (SMACH) para
    gestionar las diferentes etapas de la aproximación, incluyendo la verificación de la posición,
    movimiento hacia la posición objetivo, detección y centrado del objeto, aproximación final
    y espera después de la aproximación. Además, maneja la preempción de la acción para permitir
    la cancelación o interrupción del proceso en cualquier momento.
    """

    def __init__(self, name):
        """
        Constructor de la clase ApproachObjectActionServer.

        Inicializa el servidor de acción, suscriptores, publicadores y clientes de acción necesarios.
        Configura los parámetros de velocidad y distancia mínima para la aproximación, así como
        mecanismos de sincronización para manejar preempción de manera segura.

        Args:
            name (str): Nombre del servidor de acción.
        """
        self._action_name = name

        # ************************************************
        #          INICIALIZACIÓN DEL SERVIDOR DE ACCIÓN
        # ************************************************

        # Iniciar el servidor de acción
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            ApproachControlAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()
        rospy.loginfo("Servidor de acción '%s' iniciado.", self._action_name)

        # ************************************************
        #            SUSCRIPTORES Y PUBLICADORES
        # ************************************************

        

        # Suscribirse al tópico de detección de objetos
        self.detected_object = None
        rospy.Subscriber('/detected_objects', DetectedObject, self.object_callback)

        # Publicador para mover el robot
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # ************************************************
        #          CLIENTES DE ACCIÓN
        # ************************************************

        # Cliente de acción para move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Esperando al servidor de acción move_base...')
        self.move_base_client.wait_for_server()
        rospy.loginfo('Servidor de acción move_base disponible.')

        # Cliente de acción para object_detection
        self.object_detection_client = actionlib.SimpleActionClient('object_detection', ObjectDetectionAction)
        rospy.loginfo('Esperando al servidor de acción de detección de objetos...')
        if not self.object_detection_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logerr("El servidor de detección de objetos no está disponible.")
            # Puedes decidir cómo manejar esta situación, por ahora continuamos
        else:
            rospy.loginfo('Servidor de acción de detección de objetos disponible.')

        # ************************************************
        #               PARÁMETROS DE VELOCIDAD
        # ************************************************

        self.linear_speed = rospy.get_param('/approach_object/linear_speed', 0.1)
        self.angular_speed = rospy.get_param('/approach_object/angular_speed', 0.2)
        self.min_distance = rospy.get_param('/approach_object/min_distance', 0.5)
        self.wait_time = rospy.get_param('/approach_object/wait_time', 5)

        # ************************************************
        #           INICIALIZACIÓN DE TF LISTENER
        # ************************************************
        
        #Iniciailizar tf listener
        self.tf_listener = tf.TransformListener()
        self.current_pose = None

        # ************************************************
        #              MANEJO DE PREEMPCIÓN
        # ************************************************

        # Lock para manejar acceso concurrente
        self.lock = threading.Lock()

        # Crear una bandera de preempción compartida
        self.preempt_event = threading.Event()

        # Registrar un callback para manejar la preempción
        self._as.register_preempt_callback(self.preempt_cb)

    def preempt_cb(self):
        """
        Callback para manejar la preempción de la acción.

        Establece la bandera de preempción para indicar que se ha solicitado la interrupción
        de la acción en curso.
        """
        rospy.loginfo("Preempción solicitada!")
        self.preempt_event.set()

    # ************************************************
    #           OBTENER POSE DEL ROBOT
    # ************************************************
    def get_robot_pose(self):
        """
        Obtener la pose actual del robot en el marco de referencia 'map'.

        Returns:
            Pose: Pose actual del robot en el marco de referencia 'map'.

        Raises:
            rospy.ROSException: Si no se puede obtener la transformación de 'map' a 'base_link'.
        """


        try:
            # Esperar a que la transformación esté disponible
            self.tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(4.0))
            (trans, rot) = self.tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
            position = Point(*trans)
            orientation = Quaternion(*rot)
            pose = Pose(position=position, orientation=orientation)
            return pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("No se pudo obtener la transformación de 'map' a 'base_link'")
            return None

    def object_callback(self, msg):
        """
        Callback para almacenar la información del objeto detectado.

        Actualiza el objeto detectado con los datos recibidos del tópico de detección de objetos.

        Args:
            msg (DetectedObject): Mensaje con la información del objeto detectado.
        """
        self.detected_object = msg

    def execute_cb(self, goal):
        """
        Callback de ejecución de la acción.

        Coordina la máquina de estados para aproximarse al objeto detectado. Gestiona las transiciones
        entre estados como la verificación de la posición, movimiento hacia la posición objetivo,
        detección y centrado del objeto, aproximación final y espera después de la aproximación.

        Args:
            goal (ApproachControlGoal): Objetivo de la acción (no utilizado en este caso).
        """
        rospy.loginfo("Objetivo recibido: %s", goal)
        feedback = ApproachControlFeedback()
        result = ApproachControlResult()

        # ************************************************
        #          CREACIÓN DE LA MÁQUINA DE ESTADOS
        # ************************************************

        # Crear una máquina de estados interna
        sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
        sm.userdata.goal = goal
        sm.userdata.current_pose = self.get_robot_pose()

        with sm:
            smach.StateMachine.add(
                'CHECK_POSITION',
                CheckPositionState(self.preempt_event),
                transitions={
                    'at_position': 'START_OBJECT_DETECTION',
                    'not_at_position': 'MOVE_TO_POSITION',
                    'aborted': 'aborted',
                    'preempted': 'preempted'
                },
                remapping={
                    'goal': 'goal',
                    'current_pose': 'current_pose'
                }
            )

            smach.StateMachine.add(
                'MOVE_TO_POSITION',
                MoveToPositionState(self.move_base_client, self.preempt_event),
                transitions={
                    'succeeded': 'START_OBJECT_DETECTION',
                    'aborted': 'aborted',
                    'preempted': 'preempted'
                },
                remapping={
                    'goal': 'goal'
                }
            )

            smach.StateMachine.add(
                'START_OBJECT_DETECTION',
                StartObjectDetectionState(self.object_detection_client, self.preempt_event),
                transitions={
                    'succeeded': 'CENTER_OBJECT',
                    'aborted': 'aborted',
                    'preempted': 'preempted'
                }
            )

            smach.StateMachine.add(
                'CENTER_OBJECT',
                CenterObjectState(self.preempt_event),
                transitions={
                    'succeeded': 'APPROACH_OBJECT',
                    'aborted': 'aborted',
                    'preempted': 'preempted'
                }
            )

            smach.StateMachine.add(
                'APPROACH_OBJECT',
                ApproachObjectState(self.preempt_event),
                transitions={
                    'succeeded': 'STOP_OBJECT_DETECTION',
                    'aborted': 'aborted',
                    'preempted': 'preempted'
                }
            )

            smach.StateMachine.add(
                'STOP_OBJECT_DETECTION',
                StopObjectDetectionState(self.object_detection_client, self.preempt_event),
                transitions={
                    'succeeded': 'WAIT',
                    'aborted': 'aborted',
                    'preempted': 'preempted'
                }
            )

            smach.StateMachine.add(
                'WAIT',
                WaitState(self.preempt_event),
                transitions={
                    'succeeded': 'succeeded',
                    'preempted': 'preempted'
                }
            )

        # ************************************************
        #      SERVIDOR DE INTROSPECCIÓN DE SMACH
        # ************************************************

        # Añadir el servidor de introspección para visualizar la máquina de estados
        introspection_server = smach_ros.IntrospectionServer(
            'approach_object_introspection',
            sm,
            '/approach_object_sm'
        )
        introspection_server.start()

        # ************************************************
        #            EJECUCIÓN DE LA MÁQUINA DE ESTADOS
        # ************************************************

        # Ejecutar la máquina de estados
        outcome = sm.execute()

        # ************************************************
        #          MANEJO DEL RESULTADO DE LA ACCIÓN
        # ************************************************

        if outcome == 'succeeded':
            result.success = 1
            self._as.set_succeeded(result)
        elif outcome == 'preempted':
            result.success = 0
            self._as.set_preempted(result, "Acción preemptada")
        else:
            result.success = 0
            self._as.set_aborted(result, "Acción abortada")

        # Limpiar la bandera de preempción
        self.preempt_event.clear()

        # ************************************************
        #    CANCELACIÓN DEL SERVIDOR DE DETECCIÓN DE OBJETOS
        # ************************************************

        # Asegurarse de que el object_detection action se cancele al finalizar
        if self.object_detection_client.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
            self.object_detection_client.cancel_goal()
            rospy.loginfo('Detección de objetos cancelada al finalizar la acción.')


class CheckPositionState(smach.State):
    """
    Estado para verificar la posición actual del robot respecto al objetivo.

    Este estado calcula la distancia y la diferencia de orientación entre la posición actual
    del robot y la posición objetivo. Determina si el robot ha alcanzado la posición deseada
    dentro de una tolerancia definida.
    """

    def __init__(self, preempt_event):
        """
        Constructor del estado CheckPositionState.

        Inicializa el estado con los outcomes posibles y las claves de entrada necesarias.

        Args:
            preempt_event (threading.Event): Evento para manejar la preempción de la acción.
        """
        smach.State.__init__(
            self,
            outcomes=['at_position', 'not_at_position', 'aborted', 'preempted'],
            input_keys=['goal', 'current_pose']
        )
        self.preempt_event = preempt_event

    def execute(self, userdata):
        """
        Ejecuta la lógica del estado para verificar la posición.

        Calcula la distancia y la diferencia de orientación entre la posición actual y el objetivo.
        Determina si el robot está dentro de las tolerancias definidas para considerar que ha
        alcanzado la posición objetivo.

        Args:
            userdata (smach.UserData): Datos de usuario proporcionados por SMACH.

        Returns:
            str: Outcome del estado.
        """
        rospy.loginfo('Ejecutando estado CHECK_POSITION')

        if rospy.is_shutdown():
            return 'aborted'

        if self.preempt_requested() or self.preempt_event.is_set():
            self.service_preempt()
            return 'preempted'

        goal_pose = userdata.goal.info_objetivo.robot_pose
        current_pose = userdata.current_pose

        if current_pose is None:
            rospy.logwarn("Posición actual del robot desconocida.")
            return 'aborted'

        # Calcular la distancia y diferencia de orientación
        position_diff = self.calculate_distance(goal_pose.position, current_pose.position)
        orientation_diff = self.calculate_orientation_diff(goal_pose.orientation, current_pose.orientation)

        position_tolerance = 0.1  # metros
        orientation_tolerance = 0.1  # radianes

        rospy.loginfo(f"Diferencia de posición: {position_diff:.2f} m, Diferencia de orientación: {orientation_diff:.2f} rad")

        if position_diff <= position_tolerance and orientation_diff <= orientation_tolerance:
            return 'at_position'
        else:
            return 'not_at_position'

    def calculate_distance(self, pos1, pos2):
        """
        Calcula la distancia euclidiana entre dos posiciones.

        Args:
            pos1 (geometry_msgs/Point): Primera posición.
            pos2 (geometry_msgs/Point): Segunda posición.

        Returns:
            float: Distancia en metros.
        """
        return math.sqrt((pos1.x - pos2.x)**2 + (pos1.y - pos2.y)**2)

    def calculate_orientation_diff(self, ori1, ori2):
        """
        Calcula la diferencia de orientación en yaw entre dos cuaterniones.

        Args:
            ori1 (geometry_msgs/Quaternion): Primera orientación.
            ori2 (geometry_msgs/Quaternion): Segunda orientación.

        Returns:
            float: Diferencia de orientación en radianes.
        """
        # Convertir cuaterniones a ángulos
        yaw1 = self.quaternion_to_yaw(ori1)
        yaw2 = self.quaternion_to_yaw(ori2)
        return abs(yaw1 - yaw2)

    def quaternion_to_yaw(self, quat):
        """
        Convierte un cuaternión a un ángulo yaw.

        Args:
            quat (geometry_msgs/Quaternion): Cuaternión.

        Returns:
            float: Ángulo yaw en radianes.
        """
        # Utilizar tf.transformations para convertir de cuaternión a yaw
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        return euler[2]


class MoveToPositionState(smach.State):
    """
    Estado para mover el robot hacia la posición objetivo utilizando move_base.

    Este estado envía un objetivo al servidor move_base para que el robot se desplace
    hacia la posición deseada. Monitorea el estado de la acción y maneja posibles
    resultados como éxito, aborto o preempción.
    """

    def __init__(self, move_base_client, preempt_event):
        """
        Constructor del estado MoveToPositionState.

        Inicializa el estado con los outcomes posibles y las claves de entrada necesarias.

        Args:
            move_base_client (actionlib.SimpleActionClient): Cliente de acción para move_base.
            preempt_event (threading.Event): Evento para manejar la preempción de la acción.
        """
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted'],
            input_keys=['goal']
        )
        self.move_base_client = move_base_client
        self.preempt_event = preempt_event

    def execute(self, userdata):
        """
        Ejecuta la lógica del estado para mover el robot hacia la posición objetivo.

        Envía un objetivo al servidor move_base y monitorea su estado hasta que se
        complete la acción o se solicite una preempción.

        Args:
            userdata (smach.UserData): Datos de usuario proporcionados por SMACH.

        Returns:
            str: Outcome del estado.
        """
        rospy.loginfo('Ejecutando estado MOVE_TO_POSITION')

        if rospy.is_shutdown():
            return 'aborted'

        if self.preempt_requested() or self.preempt_event.is_set():
            self.move_base_client.cancel_goal()
            self.service_preempt()
            return 'preempted'

        goal_pose = userdata.goal.info_objetivo.robot_pose

        # Crear el objetivo para move_base
        move_base_goal = MoveBaseGoal()
        move_base_goal.target_pose.header.frame_id = "map"
        move_base_goal.target_pose.header.stamp = rospy.Time.now()
        move_base_goal.target_pose.pose = goal_pose

        # Enviar el objetivo a move_base
        self.move_base_client.send_goal(move_base_goal)

        while not rospy.is_shutdown():
            state = self.move_base_client.get_state()
            if state == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Robot ha llegado a la posición objetivo.")
                return 'succeeded'
            elif state in [actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                rospy.logwarn("No se pudo alcanzar la posición objetivo.")
                return 'aborted'
            elif self.preempt_requested():
                self.move_base_client.cancel_goal()
                self.service_preempt()
                return 'preempted'
            rospy.sleep(0.1)


class StartObjectDetectionState(smach.State):
    """
    Estado para iniciar la detección de objetos mediante el servidor de acción de detección.

    Este estado envía un objetivo al servidor de acción de detección de objetos para comenzar
    la detección. Monitorea el estado de la acción y maneja posibles resultados como éxito,
    aborto o preempción.
    """

    def __init__(self, object_detection_client, preempt_event):
        """
        Constructor del estado StartObjectDetectionState.

        Inicializa el estado con los outcomes posibles y las dependencias necesarias.

        Args:
            object_detection_client (actionlib.SimpleActionClient): Cliente de acción para la detección de objetos.
            preempt_event (threading.Event): Evento para manejar la preempción de la acción.
        """
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )
        self.object_detection_client = object_detection_client
        self.preempt_event = preempt_event

    def execute(self, userdata):
        """
        Ejecuta la lógica del estado para iniciar la detección de objetos.

        Envía un objetivo vacío al servidor de acción de detección de objetos para comenzar
        la detección. Verifica si el servidor está disponible y maneja posibles preempciones.

        Args:
            userdata (smach.UserData): Datos de usuario proporcionados por SMACH.

        Returns:
            str: Outcome del estado.
        """
        rospy.loginfo('Ejecutando estado START_OBJECT_DETECTION')

        if rospy.is_shutdown():
            return 'aborted'

        if self.preempt_requested() or self.preempt_event.is_set():
            self.service_preempt()
            return 'preempted'

        if not self.object_detection_client.wait_for_server(timeout=rospy.Duration(5.0)):
            rospy.logerr("El servidor de detección de objetos no está disponible.")
            return 'aborted'

        # Enviar goal para iniciar la detección
        detection_goal = ObjectDetectionGoal()
        self.object_detection_client.send_goal(detection_goal)

        rospy.loginfo('Detección de objetos iniciada.')
        return 'succeeded'


class StopObjectDetectionState(smach.State):
    """
    Estado para detener la detección de objetos mediante el servidor de acción de detección.

    Este estado cancela cualquier objetivo pendiente o en curso en el servidor de acción
    de detección de objetos para detener la detección.
    """

    def __init__(self, object_detection_client, preempt_event):
        """
        Constructor del estado StopObjectDetectionState.

        Inicializa el estado con los outcomes posibles y las dependencias necesarias.

        Args:
            object_detection_client (actionlib.SimpleActionClient): Cliente de acción para la detección de objetos.
            preempt_event (threading.Event): Evento para manejar la preempción de la acción.
        """
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )
        self.object_detection_client = object_detection_client
        self.preempt_event = preempt_event

    def execute(self, userdata):
        """
        Ejecuta la lógica del estado para detener la detección de objetos.

        Cancela cualquier objetivo activo o pendiente en el servidor de acción de detección
        de objetos y maneja posibles preempciones.

        Args:
            userdata (smach.UserData): Datos de usuario proporcionados por SMACH.

        Returns:
            str: Outcome del estado.
        """
        rospy.loginfo('Ejecutando estado STOP_OBJECT_DETECTION')

        if rospy.is_shutdown():
            return 'aborted'

        if self.preempt_requested() or self.preempt_event.is_set():
            self.service_preempt()
            return 'preempted'

        # Cancelar el goal de detección de objetos
        if self.object_detection_client.get_state() in [actionlib.GoalStatus.ACTIVE, actionlib.GoalStatus.PENDING]:
            self.object_detection_client.cancel_goal()
            rospy.loginfo('Detección de objetos cancelada.')

        return 'succeeded'


class CenterObjectState(smach.State):
    """
    Estado para centrar el objeto detectado en el campo de visión de la cámara.

    Este estado ajusta la orientación del robot girando hacia el objeto detectado hasta que
    esté centrado en la imagen de la cámara. Utiliza las coordenadas del objeto para determinar
    la dirección del giro.
    """

    def __init__(self, preempt_event):
        """
        Constructor del estado CenterObjectState.

        Inicializa el estado con los outcomes posibles y configura los publicadores y suscriptores necesarios.

        Args:
            preempt_event (threading.Event): Evento para manejar la preempción de la acción.
        """
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.detected_object = None
        rospy.Subscriber('/detected_objects', DetectedObject, self.object_callback)

        self.angular_speed = rospy.get_param('/approach_object/angular_speed', 0.2)

        self.preempt_event = preempt_event

    def object_callback(self, msg):
        """
        Callback para almacenar la información del objeto detectado.

        Actualiza el objeto detectado con los datos recibidos del tópico de detección de objetos.

        Args:
            msg (DetectedObject): Mensaje con la información del objeto detectado.
        """
        self.detected_object = msg

    def execute(self, userdata):
        """
        Ejecuta la lógica del estado para centrar el objeto en la cámara.

        Ajusta la orientación del robot girando hacia el objeto detectado hasta que esté
        centrado en el campo de visión de la cámara. Si el objeto no está centrado dentro
        de una tolerancia definida, continúa girando en la dirección apropiada.

        Args:
            userdata (smach.UserData): Datos de usuario proporcionados por SMACH.

        Returns:
            str: Outcome del estado.
        """
        rospy.loginfo('Ejecutando estado CENTER_OBJECT')

        if rospy.is_shutdown():
            return 'aborted'

        rate = rospy.Rate(10)
        twist_msg = Twist()

        while not rospy.is_shutdown():
            if self.preempt_requested() or self.preempt_event.is_set():
                twist_msg.angular.z = 0.0
                self.cmd_pub.publish(twist_msg)
                self.service_preempt()
                return 'preempted'

            if self.detected_object is not None:
                cX = int(self.detected_object.pixel_coordinates.x)
                # Ajustar velocidad angular hasta que el objeto esté centrado
                if abs(cX - 300) > 10:  # Suponiendo ancho de imagen de 600px
                    twist_msg.angular.z = -self.angular_speed if cX > 300 else self.angular_speed
                    self.cmd_pub.publish(twist_msg)
                    rospy.loginfo("Centrando objeto en la cámara: Coordenada X = %d", cX)
                else:
                    twist_msg.angular.z = 0.0
                    self.cmd_pub.publish(twist_msg)
                    rospy.loginfo("Objeto centrado.")
                    return 'succeeded'
            else:
                rospy.loginfo("Esperando detección del objeto...")

            rate.sleep()

        return 'aborted'


class ApproachObjectState(smach.State):
    """
    Estado para aproximarse al objeto detectado.

    Este estado mueve el robot hacia adelante en línea recta hasta que alcanza una distancia
    mínima especificada del objeto detectado. Monitorea continuamente la distancia al objeto
    para determinar cuándo detenerse.
    """

    def __init__(self, preempt_event):
        """
        Constructor del estado ApproachObjectState.

        Inicializa el estado con los outcomes posibles y configura los publicadores y suscriptores necesarios.

        Args:
            preempt_event (threading.Event): Evento para manejar la preempción de la acción.
        """
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'aborted', 'preempted']
        )

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.detected_object = None
        rospy.Subscriber('/detected_objects', DetectedObject, self.object_callback)

        self.linear_speed = rospy.get_param('/approach_object/linear_speed', 0.1)
        self.min_distance = rospy.get_param('/approach_object/min_distance', 0.5)

        self.preempt_event = preempt_event

    def object_callback(self, msg):
        """
        Callback para almacenar la información del objeto detectado.

        Actualiza el objeto detectado con los datos recibidos del tópico de detección de objetos.

        Args:
            msg (DetectedObject): Mensaje con la información del objeto detectado.
        """
        self.detected_object = msg

    def execute(self, userdata):
        """
        Ejecuta la lógica del estado para aproximarse al objeto detectado.

        Mueve el robot hacia adelante hasta que la distancia al objeto detectado sea menor o
        igual a la distancia mínima especificada. Si la distancia es adecuada, detiene el robot.

        Args:
            userdata (smach.UserData): Datos de usuario proporcionados por SMACH.

        Returns:
            str: Outcome del estado.
        """
        rospy.loginfo('Ejecutando estado APPROACH_OBJECT')

        if rospy.is_shutdown():
            return 'aborted'

        rate = rospy.Rate(10)
        twist_msg = Twist()

        while not rospy.is_shutdown():
            if self.preempt_requested() or self.preempt_event.is_set():
                twist_msg.linear.x = 0.0
                self.cmd_pub.publish(twist_msg)
                self.service_preempt()
                return 'preempted'

            if self.detected_object is not None:
                # La distancia al objeto se encuentra en la coordenada z pixel_coordinates.z
                distance_to_object = self.detected_object.pixel_coordinates.z
                if distance_to_object is None or distance_to_object == 0.0:
                    distance_to_object = 1.0  # Distancia predeterminada si no hay datos

                if distance_to_object > self.min_distance:
                    twist_msg.linear.x = self.linear_speed
                    self.cmd_pub.publish(twist_msg)
                    rospy.loginfo("Acercándose al objeto, distancia actual: %.2f m", distance_to_object)
                else:
                    twist_msg.linear.x = 0.0
                    self.cmd_pub.publish(twist_msg)
                    rospy.loginfo("Objeto alcanzado.")
                    return 'succeeded'
            else:
                rospy.loginfo("Esperando detección del objeto...")

            rate.sleep()

        return 'aborted'


class WaitState(smach.State):
    """
    Estado de espera tras aproximarse al objeto.

    Este estado mantiene el robot detenido durante un tiempo definido, permitiendo que
    se realicen acciones posteriores o simplemente esperando antes de finalizar la acción.
    """

    def __init__(self, preempt_event):
        """
        Constructor del estado WaitState.

        Inicializa el estado con los outcomes posibles y configura los parámetros de espera.

        Args:
            preempt_event (threading.Event): Evento para manejar la preempción de la acción.
        """
        smach.State.__init__(
            self,
            outcomes=['succeeded', 'preempted']
        )
        self.wait_time = rospy.get_param('/approach_object/wait_time', 5)
        self.preempt_event = preempt_event

    def execute(self, userdata):
        """
        Ejecuta la lógica del estado de espera.

        Mantiene el robot detenido durante el tiempo especificado. Monitorea continuamente
        si se ha solicitado una preempción para poder interrumpir la espera si es necesario.

        Args:
            userdata (smach.UserData): Datos de usuario proporcionados por SMACH.

        Returns:
            str: Outcome del estado.
        """
        rospy.loginfo('Ejecutando estado WAIT')

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.preempt_requested() or self.preempt_event.is_set():
                self.service_preempt()
                return 'preempted'

            elapsed_time = rospy.Time.now() - start_time
            if elapsed_time.to_sec() >= self.wait_time:
                rospy.loginfo("Tiempo de espera completado.")
                return 'succeeded'

            rate.sleep()

        return 'preempted'


if __name__ == '__main__':
    """
    Punto de entrada principal para el servidor de acción de aproximación al objeto.

    Inicializa el nodo ROS y el servidor de acción, luego mantiene el nodo en ejecución
    esperando solicitudes de acción.
    """
    try:
        rospy.init_node('approach_object_server')
        server = ApproachObjectActionServer('approach_object')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
