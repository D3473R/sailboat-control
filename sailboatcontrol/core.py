#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import logging
import time
from timeit import default_timer as timer

import geojson
import math
import numpy as np
import paho.mqtt.client as mqtt
from geographiclib.geodesic import Geodesic

from sailboatcontrol import helpers

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)-5.5s]  %(message)s',
    handlers=[
        logging.FileHandler('{0}/{1}.log'.format('../logs', 'sailboat-control'), 'a', 'utf-8'),
        logging.StreamHandler()
    ])

MS_KN = 1.944
WIND_ANGLE_THRESHOLD_DEGREE = 15
TARGET_RADIUS = 5
PATH_CALCULATION_ITERATIONS = 4
PATH_CALCULATION_TIMEOUT = 0.5

wind_direction = 90
wind_speed = 7.71604938271605
wind_data = False

boat_heading = 0
boat_speed = 0

with open('../json/wind.json') as g:
    wind = json.load(g)


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
    return np.array([np.cos(angle), np.sin(angle)])


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


def get_opposite_angle(hypotenuse, cathetus):
    return math.asin(cathetus / hypotenuse)


def get_adjacent_angle(hypotenuse, cathetus):
    return math.acos(cathetus / hypotenuse)


class Vector:
    __vector = None

    def __init__(self, angle, length):
        self.__vector = np.array([np.cos(angle), np.sin(angle)]) * length

    @staticmethod
    def from_vector(vector):
        vector_length = np.linalg.norm(vector)
        v2 = np.array([1, 0])
        angle = vector_length * np.linalg.norm(v2)
        return Vector(np.arccos(np.dot(vector, v2) / angle), vector_length)

    def get_angle(self):
        v1 = self.__vector
        v2 = np.array([1, 0])
        angle = np.linalg.norm(v1) * np.linalg.norm(v2)
        return np.arccos(np.dot(v1, v2) / angle) if angle > 0 else 0

    def get_vector(self):
        return self.__vector

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

    def get_opposite_len(self):
        return math.fabs(math.sin(self.get_angle()) * self.get_length())

    def x(self):
        return self.__vector.item(0)

    def y(self):
        return self.__vector.item(1)

    def __repr__(self):
        return json.dumps({'vector': '[{:.2f}, {:.2f}]'.format(self.__vector[0], self.__vector[1])})


class Path:
    __length = 0
    __time = 0
    __index = -1

    def __init__(self, index, *vectors):
        self.__index = index
        self.__vectors = list(vectors)

    def calculate_vectors(self):
        impossible_path = False

        for k, vector in enumerate(self.__vectors):
            if not impossible_path:
                logging.info('Vector {}: {}'.format(k, vector))
                apparent_wind_angle = angle_between_angles(wind_direction, vector.get_angle_degrees())
                if apparent_wind_angle < WIND_ANGLE_THRESHOLD_DEGREE:
                    logging.info('Path {}: {} has an impossible angle in Vector: {} {} with {:.2f}°'
                                 .format(self.__index, self, k, vector, apparent_wind_angle))
                    impossible_path = True
                else:
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
                    self.add_length(vector.get_length())
                    self.add_time(vector.get_length() / boat_speed)
        return impossible_path

    def get_heading(self):
        return self.__vectors[0].get_angle_degrees()

    def get_vectors(self):
        return self.__vectors

    def set_vector(self, index, vector):
        self.__vectors[index] = vector

    def add_vector_prefix(self, vector_prefix):
        self.__vectors.insert(0, vector_prefix)

    def get_length(self):
        return self.__length

    def add_length(self, length):
        self.__length += length

    def get_time(self):
        return self.__time

    def add_time(self, time):
        self.__time += time

    def get_index(self):
        return self.__index

    def __repr__(self):
        return json.dumps(
            {'path': {'length': '{:.2f}'.format(self.__length), 'time': '{:.2f}'.format(self.__time),
                      'vectors': json.loads(str(self.__vectors))}})


def calculate_initial_paths(target_angle, target_distance):
    v1 = Vector(math.radians(target_angle), target_distance)
    v2_a = Vector(math.radians(90), v1.get_opposite_len())
    v2_b = Vector(math.radians(0), v1.get_adjacent_len())
    v3_a = Vector(math.radians(90), v2_a.get_length() / 2)
    angle = math.degrees(math.atan(v2_b.get_length() / v3_a.get_length())) if v3_a.get_length() > 0 else 0
    v3_b = Vector(
        math.radians(angle_between_angles(90, angle)),
        median(v1.get_length(), v2_b.get_length(), v2_a.get_length()))

    return [Path(0, v1), Path(1, v2_a, v2_b), Path(2, v3_a, v3_b)]


def calculate_paths(v1, v2, v3):
    v2_a = v2
    v2_b = v3

    v3_a = Vector(math.radians(90), v2_a.get_length() / 2)
    v3_b_l = median(v1.get_length(), v2_b.get_length(), v2_a.get_length())
    v3_b_a = math.degrees(angle_between_vectors(v2.get_vector(), np.array([v1.x(), np.subtract(v1.y(), v2.y() / 2.0)])))
    v3_b = Vector(math.radians(angle_between_angles(90, v3_b_a)), v3_b_l)

    return [Path(0, v1), Path(1, v2_a, v2_b), Path(2, v3_a, v3_b)]


def calculate_best_path(paths, increments):
    vector_prefix = np.array([0, 0])

    for x in range(increments):
        logging.info('Starting iteration: {}'.format(x + 1))
        impossible_paths = []

        for j, path in enumerate(paths):
            if path.calculate_vectors():
                impossible_paths.append(j)

            logging.info('Path {}: {}'.format(j, path))

        for path in sorted(impossible_paths, reverse=True):
            del paths[path]

        paths.sort(key=lambda p: p.get_time())

        if len(paths) == 0:
            logging.info('No suitable path found. Target is directly in the wind')
            paths.append(Path(0, Vector(math.radians(45), 1)))
        if len(paths) == 1:
            return paths[0]
        logging.info('Sorted paths by time: {}'.format(paths))

        if x + 1 < increments:
            if paths[0].get_index() == 0 or paths[1].get_index() == 0:
                p1, p2 = (paths[0], paths[1]) if paths[0].get_index() == 0 else (paths[1], paths[0])
                v1 = p1.get_vectors()[0]
                v2 = p2.get_vectors()[0]
                v3 = p2.get_vectors()[1]

                paths = calculate_paths(v1, v2, v3)
            elif paths[0].get_index() == 1 or paths[1].get_index() == 1:
                p1, p2 = (paths[0], paths[1]) if paths[0].get_index() == 0 else (paths[1], paths[0])
                v1 = p1.get_vectors()[1]
                v2 = p1.get_vectors()[0]
                v3 = p2.get_vectors()[1]

                vector_prefix = np.add(vector_prefix, v2.get_vector())

                paths = calculate_paths(v1, v2, v3)

    best_path = paths[0]

    if np.any(vector_prefix):
        first_vector = best_path.get_vectors()[0].get_vector()
        if angle_between_vectors(first_vector, vector_prefix) == 0.0:
            best_path.set_vector(0, Vector(math.radians(90), np.linalg.norm(np.add(first_vector, vector_prefix))))
        else:
            best_path.add_vector_prefix(Vector(math.radians(90), np.linalg.norm(vector_prefix)))
        logging.info('Updating the best path with the prefix vector')
        best_path.calculate_vectors()

    return best_path


def main():
    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    # client.connect("localhost", 1883, 60)

    # client.loop_start()

    # while not wind_data:
    # pass

    gps = [19.953566, 60.10242]

    with open('../json/waypoints.json') as f:
        waypoints = geojson.load(f)

    cyclic = waypoints['properties']['cyclic']
    logging.info('Waypoints: %s' % waypoints)
    logging.info('Number of coordinates: %d, Cyclic: %s' % (len(waypoints['geometry']['coordinates']), cyclic))

    while True:
        for i, waypoint in enumerate(waypoints['geometry']['coordinates']):
            logging.info('Using waypoint %i: %s' % (i, waypoint))
            geodesic = Geodesic.WGS84.Inverse(gps[1], gps[0], waypoint[1], waypoint[0])

            while geodesic['s12'] > TARGET_RADIUS:
                geodesic = Geodesic.WGS84.Inverse(gps[1], gps[0], waypoint[1], waypoint[0])

                paths = calculate_initial_paths(angle_between_angles(90, geodesic['azi1']), geodesic['s12'])
                best_path = calculate_best_path(paths, PATH_CALCULATION_ITERATIONS)
                logging.info('Best path is {}'.format(best_path))

                heading_delta = angle_between_angles(boat_heading, best_path.get_heading())
                logging.info('Heading delta is: {:.2f}°'.format(heading_delta))

                rudder_servo_value = helpers.map_rudder_servo(heading_delta)

                logging.info('Rudder servo value is: {:.2f}'.format(rudder_servo_value))

                real_wind_vector = Vector(math.radians(wind_direction), wind_speed)
                boat_vector = Vector(math.radians(boat_heading), boat_speed)
                apparent_wind_vector = Vector.from_vector(
                    np.subtract(real_wind_vector.get_vector(), boat_vector.get_vector()))

                sail_angle = helpers.get_sail_angle(math.degrees(apparent_wind_vector.get_angle()))

                logging.info('Sail angle is: {:.2f}°'.format(sail_angle))

                sail_servo_value = helpers.map_sail_servo(sail_angle)

                logging.info('Sail servo value is: {:.2f}'.format(sail_servo_value))

                time.sleep(PATH_CALCULATION_TIMEOUT)
        if cyclic:
            logging.info('Start new cycle')
        else:
            break


if __name__ == '__main__':
    start = timer()
    main()
    end = timer()
    logging.info('Execution time: {:.2f} sec'.format(end - start))
