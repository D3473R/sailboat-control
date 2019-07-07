#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
from threading import Event

import geojson
import math
import numpy as np
from Adafruit_PCA9685 import PCA9685
from geographiclib.geodesic.Geodesic.WGS84 import Inverse

from .compass import Compass
from .gps import Gps
from .wind import Wind
from .setup_logger import logging

DEBUG = True

TARGET_RADIUS = 5
WAYPOINT_FILE = '../json/waypoints.json'
LINE_DISTANCE_MAX_METERS = 20
DIRECT_WIND_OFFSET = 45


class Vector:
    __vector = None

    def __init__(self, angle, length):
        self.__vector = np.array([np.cos(angle), np.sin(angle)]) * length

    def get_vector(self):
        return self.__vector

    def x(self):
        return self.__vector.item(0)

    def y(self):
        return self.__vector.item(1)


class Control:
    def __init__(self):
        """ The main control class. """

        self.gps_thread_stop = Event()
        self.wind_thread_stop = Event()
        self.compass_thread_stop = Event()

        self.gps_thread = Gps(self.gps_thread_stop)
        self.wind_thread = Wind(self.wind_thread_stop)
        self.compass_thread = Compass(self.compass_thread_stop)

        self.servo_pwm = PCA9685()
        self.servo_pwm.set_pwm_freq(60)

        with open(WAYPOINT_FILE) as f:
            self.waypoints = geojson.load(f)

        self.cyclic = self.waypoints['properties']['cyclic']

        while self.gps_thread.gps['status'] != 'A':
            logging.info('Waiting for gps...')
            time.sleep(0.2)

        self.main()

    def main(self):
        while True:
            for i, waypoint in enumerate(self.waypoints['geometry']['coordinates']):
                logging.info('Using waypoint %i: %s' % (i, waypoint))

                waypoint_geodesic = Inverse(self.gps_thread.gps['lat'], self.gps_thread.gps['lon'],
                                            waypoint[1], waypoint[0])
                boat_geodesic = waypoint_geodesic

                while boat_geodesic['s12'] > TARGET_RADIUS:
                    logging.info('Boat gps: {}, {}, timestamp={}, speed={}'.format(
                        self.gps_thread.gps['lat'], self.gps_thread.gps['lon'],
                        self.gps_thread.gps['timestamp'], self.gps_thread.gps['speed']))
                    waypoint_vector = Vector(math.radians(waypoint_geodesic['azi1']), waypoint_geodesic['s12'])
                    boat_vector = Vector(math.radians(boat_geodesic['azi1']), boat_geodesic['s12'])

                    distance_to_line = np.cross(waypoint_vector.get_vector(), boat_vector.get_vector()) / \
                                       np.linalg.norm(waypoint_vector.get_vector())

                    if self.wind_thread.wind < DIRECT_WIND_OFFSET:
                        # Waypoint is upwind.
                        pass

            if self.cyclic:
                logging.info('Start new cycle')
            else:
                logging.info('Finished course!')
                break
                # hold_position()
        self.shutdown_routine()

    def start_threads(self):
        self.gps_thread.start()
        self.wind_thread.start()
        self.compass_thread.start()

    def shutdown_routine(self):
        self.gps_thread_stop.set()
        self.wind_thread_stop.set()
        self.compass_thread_stop.set()
        logging.info('Goodbye :)')
        sys.exit(0)


if __name__ == '__main__':
    control = None
    try:
        control = Control()
    except KeyboardInterrupt:
        control.gps_thread_stop.set()
        control.wind_thread_stop.set()
        control.compass_thread_stop.set()
    except Exception as e:
        logging.error('ERROR IN MAIN THREAD: {}'.format(e))
