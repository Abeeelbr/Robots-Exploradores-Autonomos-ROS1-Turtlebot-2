#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from squad_planificacion.msg import StationInfo, StationsInfo
from squad_exploracion.msg import DetectedObject
from geometry_msgs.msg import Point
import threading
import math

class StationManagerNode:
    """
    Nodo para gestionar las estaciones en el mapa.

    - Publicará la estación cuando se detecten al menos dos objetos del mismo tipo dentro de un radio definido desde el centro.
    - No hay un máximo de objetos por estación; se pueden agregar más objetos siempre que estén dentro del radio.
    - Evita múltiples publicaciones para la misma estación.
    - Mantiene un diccionario 'almacen_estaciones' con el formato:
      {"HOME": (5, 4), "ESTACION1": (1, 1), "ESTACION2": (2, 2), ...}
      Pudiendo haber estaciones repetidas con nombre distinto (baño, baño2, etc.).
    """

    def __init__(self):
        rospy.init_node('station_manager_node')

        # Diccionario de objetos requeridos por tipo de estación (puede ser útil para futuras extensiones)
        self.required_objects = {
            'habitacion': {"bed", "clock", "book"},
            'baño': {"toilet", "toothbrush"},
            'comedor': {"dining table", "chair"},
            'garaje': {"car", "bicycle"},
            'calle': {"traffic light"},
            'cocina': {"fork", "spoon", "bowl"},
            'especial': {"person"},
        }

        self.lock = threading.Lock()

        # Lista con las estaciones completadas
        self.completed_stations = []

        # Contador para numerar estaciones repetidas
        self.station_counts = {}

        # Diccionario con las estaciones y sus coordenadas promedio
        self.almacen_estaciones = {}

        # Umbral de distancia máximo permitido desde el centro
        self.radius_threshold = rospy.get_param('~radius_threshold', 5.0)

        # Diccionario para almacenar clusters por tipo de estación
        # Estructura: { 'station_type': [ { 'positions': [pos1, pos2, ...], 'published': bool }, ... ], ... }
        self.clusters = {}

        rospy.Subscriber('/detected_objects', DetectedObject, self.detected_object_callback)

        # Publicador del array de estaciones completadas
        self.stations_info_pub = rospy.Publisher('/stations_info', StationsInfo, queue_size=10)

    def detected_object_callback(self, msg):
        with self.lock:
            station_type = self.classify_object(msg.type)
            
            position = msg.world_coordinates

            position = msg.robot_pose.position


            # Inicializar la lista de clusters para este tipo de estación si no existe
            if station_type not in self.clusters:
                self.clusters[station_type] = []

            # Crear una nueva posición dict
            new_pos = {'x': position.x, 'y': position.y, 'z': position.z}

            # Buscar un cluster existente al que se pueda agregar este objeto
            cluster_found = False
            for cluster in self.clusters[station_type]:
                center = self.calculate_average_position(cluster['positions'])
                dist = self.distance_to_center(new_pos, center)
                if dist <= self.radius_threshold:
                    # Agregar al cluster
                    cluster['positions'].append(new_pos)
                    cluster_found = True

                    # Si el cluster tiene al menos dos objetos y no ha sido publicado
                    if len(cluster['positions']) >= 2 and not cluster['published']:
                        self.add_completed_station(station_type, cluster)
                        cluster['published'] = True
                    break  # Asumimos que un objeto pertenece a un solo cluster

            if not cluster_found:
                # Crear un nuevo cluster con este objeto
                new_cluster = {
                    'positions': [new_pos],
                    'published': False
                }
                self.clusters[station_type].append(new_cluster)

    def add_completed_station(self, station_type, cluster):
        positions = cluster['positions']
        avg_position = self.calculate_average_position(positions)

        # Contar cuántas veces ha aparecido esta estación
        count = self.station_counts.get(station_type, 0) + 1
        self.station_counts[station_type] = count

        # Generar nombre único
        if count == 1:
            station_id = station_type
        else:
            station_id = f"{station_type}{count}"

        # Definir un polígono simple alrededor del centro (puedes ajustar según tus necesidades)
        polygon_points = [
            Point(x=avg_position['x'] - 1, y=avg_position['y'] - 1, z=0),
            Point(x=avg_position['x'] - 1, y=avg_position['y'] + 1, z=0),
            Point(x=avg_position['x'] + 1, y=avg_position['y'] + 1, z=0),
            Point(x=avg_position['x'] + 1, y=avg_position['y'] - 1, z=0)
        ]

        station_info = StationInfo()
        station_info.station_id = station_id
        station_info.coordinates = Point(
            x=avg_position['x'],
            y=avg_position['y'],
            z=avg_position['z']
        )
        station_info.polygon = polygon_points

        # Añadir a la lista global de completadas
        self.completed_stations.append(station_info)

        # Actualizar el diccionario almacen_estaciones con (x,y)
        self.almacen_estaciones[station_id] = (avg_position['x'], avg_position['y'])

        # Publicar el array completo de estaciones
        self.publish_all_stations()

    def publish_all_stations(self):
        stations_msg = StationsInfo()
        stations_msg.stations = self.completed_stations
        self.stations_info_pub.publish(stations_msg)

        # Imprimir el diccionario almacen_estaciones en consola
        print(self.almacen_estaciones)

    def calculate_average_position(self, positions):
        avg_x = sum(pos['x'] for pos in positions) / len(positions)
        avg_y = sum(pos['y'] for pos in positions) / len(positions)
        avg_z = sum(pos['z'] for pos in positions) / len(positions)
        return {'x': avg_x, 'y': avg_y, 'z': avg_z}

    def distance_to_center(self, pos, center):
        dx = pos['x'] - center['x']
        dy = pos['y'] - center['y']
        dz = pos['z'] - center['z']
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def classify_object(self, object_type):
        habitaciones = ["airplane", "bus", "train", "boat", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "skateboard", "bed", "cell phone", "book", "clock", "teddy bear"]
        bano = ["toilet", "hair drier", "toothbrush"]
        comedor = ["bench","bird", "cat", "dog", "umbrella", "handbag", "potted plant", "dining table", "tv", "laptop", "mouse", "remote", "keyboard", "couch", "chair", "vase"]
        garaje = ["bicycle", "car", "motorcycle", "truck", "surfboard", "tennis racket"]
        calle = ["traffic light", "fire hydrant", "stop sign", "parking meter"]
        cocina = ["bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "microwave", "oven", "toaster", "sink", "refrigerator", "scissors"]
        persona = ["person"]

        if object_type in habitaciones:
            return 'habitacion'
        elif object_type in bano:
            return 'baño'
        elif object_type in comedor:
            return 'comedor'
        elif object_type in garaje:
            return 'garaje'
        elif object_type in calle:
            return 'calle'
        elif object_type in cocina:
            return 'cocina'
        elif object_type in persona:
            return 'Persona encontrada'
        else:
            return 'general_station'

if __name__ == '__main__':
    try:
        node = StationManagerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
