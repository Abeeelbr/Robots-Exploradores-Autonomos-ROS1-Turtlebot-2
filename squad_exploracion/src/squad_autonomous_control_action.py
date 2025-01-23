#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Servidor de acción para el control autónomo del robot.

Este nodo implementa una acción que permite controlar el robot de manera autónoma,
evitando obstáculos y colisiones con las paredes. La acción puede ser activada o
cancelada mientras se cambia de modo. Está diseñada para ser extendida con algoritmos
de control más avanzados en el futuro.
"""

# ************************************************
#                 IMPORTACIÓN DE MÓDULOS
# ************************************************

import rospy
import actionlib
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from squad_exploracion.msg import AutonomousControlAction, AutonomousControlFeedback, AutonomousControlResult

# ************************************************
#               CLASE DEL SERVIDOR DE ACCIÓN
# ************************************************

class AutonomousControlActionServer:
    """
    Servidor de acción para el control autónomo del robot.

    Esta clase implementa la lógica para controlar el robot de manera autónoma,
    evitando colisiones con obstáculos detectados por el sensor LIDAR.
    """

    def __init__(self, name):
        """
        Constructor de la clase AutonomousControlActionServer.

        Inicializa el nodo, los suscriptores, publicadores y el servidor de acción.

        :param name: Nombre del servidor de acción.
        """
        # ********************************************
        #          INICIALIZACIÓN DEL NODO
        # ********************************************

        rospy.init_node('autonomous_control_server')
        self._action_name = name

        # ********************************************
        #          VARIABLES Y PARÁMETROS
        # ********************************************

        # Velocidades lineal y angular
        self.linear_speed = rospy.get_param('/autonomous_control/linear_speed', 0.2)
        self.angular_speed = rospy.get_param('/autonomous_control/angular_speed', 0.5)

        # Distancia mínima para evitar obstáculos
        self.min_distance = rospy.get_param('/autonomous_control/min_distance', 0.5)

        # Tasa de procesamiento
        self.rate = rospy.Rate(rospy.get_param('/autonomous_control/processing_rate', 10))

        self.ejecucion_mode = rospy.get_param( '/MODO_EJECUCION', 'REAL' )

        self.control_teleop_topic = rospy.get_param( f'/{self.ejecucion_mode}/control_teleop_topic', '/cmd_vel_mux/input/teleop')

        # Variable para almacenar los datos del LIDAR
        self.laser_data = None

        # ********************************************
        #      SUSCRIPTORES Y PUBLICADORES
        # ********************************************

        # Suscriptor al tópico del LIDAR
        self.laser_sub = rospy.Subscriber('/base_scan', LaserScan, self.laser_callback)

        # Publicador al tópico de velocidades
        self.cmd_pub = rospy.Publisher(self.control_teleop_topic, Twist, queue_size=1)

        # ********************************************
        #        INICIALIZACIÓN DEL SERVIDOR
        # ********************************************

        # Servidor de acción
        self._as = actionlib.SimpleActionServer(
            self._action_name,
            AutonomousControlAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._as.start()

        rospy.loginfo("Servidor de acción '%s' iniciado.", self._action_name)

    # ************************************************
    #          CALLBACK DEL SENSOR LIDAR
    # ************************************************

    def laser_callback(self, data):
        """
        Callback para almacenar los datos del LIDAR.

        :param data: Mensaje de tipo LaserScan con los datos del sensor.
        """
        self.laser_data = data

    # ************************************************
    #          CALLBACK DE EJECUCIÓN DE LA ACCIÓN
    # ************************************************

    def execute_cb(self, goal):
        """
        Método de ejecución de la acción.

        Controla el robot de manera autónoma, evitando obstáculos basados en los
        datos del LIDAR. La acción puede ser preempted si se recibe una solicitud
        de cancelación.

        :param goal: Objetivo de la acción (no utilizado en este caso).
        """
        rospy.loginfo("Control autónomo iniciado.")

        # Inicializar feedback y resultado
        feedback = AutonomousControlFeedback()
        result = AutonomousControlResult()

        while not rospy.is_shutdown():
            # Verificar si se solicitó la preempción de la acción
            if self._as.is_preempt_requested():
                rospy.loginfo("Control autónomo preempted.")
                self.stop_robot()
                self._as.set_preempted()
                return

            # Verificar si hay datos del LIDAR disponibles
            if self.laser_data is None:
                rospy.logwarn("Esperando datos del LIDAR...")
                self.rate.sleep()
                continue

            # Obtener las regiones de interés del LIDAR
            regions = self.get_laser_regions()

            # Tomar decisiones basadas en las lecturas del LIDAR
            twist_msg = self.decide_motion(regions)

            # Publicar el mensaje de velocidad
            self.cmd_pub.publish(twist_msg)

            # Actualizar el feedback
            feedback.status = "Control autónomo en ejecución."
            self._as.publish_feedback(feedback)

            # Esperar al siguiente ciclo
            self.rate.sleep()

        # Finalizar la acción
        rospy.loginfo("Control autónomo completado.")
        self.stop_robot()
        self._as.set_succeeded(result)

    # ************************************************
    #            MÉTODOS DE CONTROL DEL ROBOT
    # ************************************************

    def get_laser_regions(self):
        """
        Divide las lecturas del LIDAR en regiones.

        :return: Diccionario con las distancias mínimas en cada región.
        """
        ranges = self.laser_data.ranges
        regions = {
            'right':  min(min(ranges[0:143]), 10),
            'fright': min(min(ranges[144:287]), 10),
            'front':  min(min(ranges[288:431]), 10),
            'fleft':  min(min(ranges[432:575]), 10),
            'left':   min(min(ranges[576:719]), 10),
        }
        return regions

    def decide_motion(self, regions):
        """
        Decide el movimiento del robot basado en las regiones del LIDAR.

        :param regions: Diccionario con las distancias mínimas en cada región.
        :return: Mensaje Twist con las velocidades lineal y angular.
        """
        msg = Twist()
        linear_x = 0.0
        angular_z = 0.0

        state_description = ''

        if regions['front'] > self.min_distance and regions['fleft'] > self.min_distance and regions['fright'] > self.min_distance:
            state_description = 'case 1 - sin obstáculos'
            linear_x = self.linear_speed
            angular_z = 0.0
        elif regions['front'] < self.min_distance and regions['fleft'] > self.min_distance and regions['fright'] > self.min_distance:
            state_description = 'case 2 - obstáculo al frente'
            linear_x = 0.0
            angular_z = self.angular_speed
        elif regions['front'] > self.min_distance and regions['fleft'] > self.min_distance and regions['fright'] < self.min_distance:
            state_description = 'case 3 - obstáculo al frente-derecha'
            linear_x = 0.0
            angular_z = self.angular_speed
        elif regions['front'] > self.min_distance and regions['fleft'] < self.min_distance and regions['fright'] > self.min_distance:
            state_description = 'case 4 - obstáculo al frente-izquierda'
            linear_x = 0.0
            angular_z = -self.angular_speed
        elif regions['front'] < self.min_distance and regions['fleft'] > self.min_distance and regions['fright'] < self.min_distance:
            state_description = 'case 5 - obstáculo al frente y derecha'
            linear_x = 0.0
            angular_z = self.angular_speed
        elif regions['front'] < self.min_distance and regions['fleft'] < self.min_distance and regions['fright'] > self.min_distance:
            state_description = 'case 6 - obstáculo al frente y izquierda'
            linear_x = 0.0
            angular_z = -self.angular_speed
        elif regions['front'] < self.min_distance and regions['fleft'] < self.min_distance and regions['fright'] < self.min_distance:
            state_description = 'case 7 - obstáculo en todas las direcciones'
            linear_x = 0.0
            angular_z = self.angular_speed
        elif regions['front'] > self.min_distance and regions['fleft'] < self.min_distance and regions['fright'] < self.min_distance:
            state_description = 'case 8 - obstáculos a izquierda y derecha'
            linear_x = self.linear_speed
            angular_z = 0.0
        else:
            state_description = 'unknown case'
            rospy.loginfo(regions)

        rospy.logdebug(state_description)

        msg.linear.x = linear_x
        msg.angular.z = angular_z
        return msg

    def stop_robot(self):
        """
        Detiene el robot publicando velocidades cero.
        """
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_pub.publish(msg)


# ************************************************
#                   FUNCIÓN MAIN
# ************************************************

if __name__ == '__main__':
    try:
        server = AutonomousControlActionServer('autonomous_control')
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
