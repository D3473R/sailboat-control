#!/usr/bin/env python
# coding: utf-8

import json
import logging

import geojson
import math
import numpy as np
import paho.mqtt.client as mqtt
from geographiclib.geodesic import Geodesic

logging.basicConfig(level=logging.INFO, format='%(asctime)s %(message)s')

MS_KN = 1.944

wind_direction = 0
wind_speed = 7.71604938271605
wind_data = False


def on_connect(client, userdata, flags, rc):
    logging.info("Connected with result code " + str(rc))
    client.subscribe("weather")


def on_disconnect(client, userdata, rc=0):
    logging.info("Disconnected with result code " + str(rc))
    client.loop_stop()


def on_message(client, userdata, msg):
    global wind_direction, wind_speed, wind_data
    msg = json.loads(msg.payload)
    wind_direction = msg['direction']
    wind_speed = msg['speed']
    wind_data = True
    logging.info(msg)


def unit_vector(angle):
    return np.array([round(np.cos(angle), 2), round(np.sin(angle), 2)])


def angle_between_vectors(v1, v2):
    return np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))


def angle_between_angles(a1, a2):
    return 180 - abs(abs(a1 - a2) - 180)


def get_nearest_value(dictionary, value):
    return min(dictionary, key=lambda x: abs(float(x) - value))


def median(l1, l2, l3):
    return math.sqrt((2 * (l1 ** 2 + l2 ** 2) - l3 ** 2) / 4)


def kn_to_ms(kn):
    return kn / MS_KN


def ms_to_kn(ms):
    return ms * MS_KN


class Vector:
    __vector = None

    def __init__(self, angle, length):
        self.__vector = np.array([round(np.cos(angle), 2), round(np.sin(angle), 2)]) * length

    def get_angle(self):
        v1 = self.__vector
        v2 = np.array([1, 0])
        return np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2)))

    def set_angle(self, angle):
        self.__init__(angle, self.get_length())

    def get_angle_degrees(self):
        return math.degrees(self.get_angle())

    def get_length(self):
        return np.linalg.norm(self.__vector)

    def set_length(self, length):
        self.__init__(self.get_angle(), length)

    def get_adjacent_len(self):
        return math.fabs(math.cos(self.get_angle()) * self.get_length())

    def get_adjacent_angle(self, cathetus):
        return math.acos(math.radians(cathetus.get_length() / self.get_length()))

    def get_opposite_len(self):
        return math.fabs(math.sin(self.get_angle()) * self.get_length())

    def get_opposite_angle(self, cathetus):
        return math.asin(math.radians(cathetus.get_length() / self.get_length()))

    def __repr__(self):
        return json.dumps({'vector': '[{:.2f}, {:.2f}]'.format(round(self.__vector[0], 2), round(self.__vector[1], 2))})


class Path:
    __length = 0
    __time = 0

    def __init__(self, *vectors):
        self.__vectors = list(vectors)

    def get_vectors(self):
        return self.__vectors

    def get_length(self):
        return self.__length

    def add_length(self, length):
        self.__length += length

    def get_time(self):
        return self.__time

    def add_time(self, time):
        self.__time += time

    def __repr__(self):
        return json.dumps(
            {'path': {'length': self.__length, 'time': self.__time, 'vectors': json.loads(str(self.__vectors))}})


def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    client.connect("localhost", 1883, 60)

    client.loop_start()

    # while not wind_data:
    # pass

    gps = [19.953566, 60.10242]

    with open('waypoints.json') as f:
        waypoints = geojson.load(f)

    with open('wind.json') as g:
        wind = json.load(g)

    cyclic = waypoints['properties']['cyclic']
    logging.info('Waypoints: %s' % waypoints)
    logging.info('Number of coordinates: %d, Cyclic: %s' % (len(waypoints['geometry']['coordinates']), cyclic))

    for i, waypoint in enumerate(waypoints['geometry']['coordinates']):
        logging.info('Using waypoint %i: %s' % (i, waypoint))
        geodesic = Geodesic.WGS84.Inverse(gps[1], gps[0], waypoint[1], waypoint[0])
        geodesic['s12'] = 10.816653826392
        geodesic['azi1'] = 56.3099324740202

        v1 = Vector(math.radians(angle_between_angles(90, geodesic['azi1'])), geodesic['s12'])
        v2_a = Vector(math.radians(90), v1.get_opposite_len())
        v2_b = Vector(v1.get_opposite_angle(v2_a), v1.get_adjacent_len())
        v3_a = Vector(math.radians(90), v2_a.get_length() / 2)
        v3_b = Vector(math.radians(angle_between_angles(90, math.degrees(math.atan(v2_b.get_length() / v3_a.get_length())))), median(v1.get_length(), v2_b.get_length(), v2_a.get_length()))

        paths = [Path(v1), Path(v2_a, v2_b), Path(v3_a, v3_b)]
        times = {}

        for j, path in enumerate(paths):
            for k, vector in enumerate(path.get_vectors()):
                logging.info('Vector {}: {}'.format(k, vector))
                apparent_wind_angle = angle_between_angles(wind_direction, vector.get_angle_degrees())
                logging.info('Real wind speed: {:.2f} m/s / {:.2f} kn'.format(wind_speed, ms_to_kn(wind_speed)))
                logging.info('Real wind angle: {:.2f}°'.format(wind_direction))
                logging.info('Boat angle: {:.2f}°'.format(vector.get_angle_degrees()))
                logging.info('Apparent wind angle: {:.2f}°'.format(apparent_wind_angle))
                nearest_wind_speed = float(get_nearest_value(wind['wind'], ms_to_kn(wind_speed)))
                boat_angle = get_nearest_value(wind['wind'][str(nearest_wind_speed)], apparent_wind_angle)
                boat_speed = wind['wind'][str(nearest_wind_speed)][boat_angle]
                boat_angle = float(boat_angle)
                logging.info('Boat speed {:.2f} m/s / {:.2f} kn'
                             .format(kn_to_ms(boat_speed), boat_speed))
                logging.info('Boat angle: {:.2f}°'.format(boat_angle))
                path.add_length(vector.get_length())
                path.add_time(vector.get_length() / boat_speed)
                times[j] = path.get_time()

            logging.info('Path {}: {}'.format(j, path))

        shortest_path_index = min(times, key=times.get)
        logging.info('Shortest path by time is Path {}: {}'.format(shortest_path_index, paths[shortest_path_index]))

        break

    # while True:
        # pass


if __name__ == '__main__':
    main()
