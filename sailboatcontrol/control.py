#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import time
from threading import Event

import geojson
import math
import numpy as np
from os import path
from Adafruit_PCA9685 import PCA9685
from geographiclib.geodesic import Geodesic

from store import Store
from compass import Compass
from gps import Gps
from wind import Wind
from mavlink import Mavlink
from setup_logger import logging

DEBUG = True

TARGET_RADIUS = 5
WAYPOINT_FILE = path.join(path.dirname(path.dirname(path.abspath(__file__))), path.join('json', 'waypoints.json'))
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

        self.store = Store()
        self.gps_thread_stop = Event()
        self.wind_thread_stop = Event()
        self.compass_thread_stop = Event()
        self.mavlink_thread_stop = Event()

        self.gps_thread = Gps(self.gps_thread_stop, self.store)
        self.wind_thread = Wind(self.wind_thread_stop, self.store)
        self.compass_thread = Compass(self.compass_thread_stop, self.store)
        self.mavlink_thread = Mavlink(self.mavlink_thread_stop, self.store)

        try:
            self.servo_pwm = PCA9685()
            self.servo_pwm.set_pwm_freq(60)
        except IOError as e:
            logging.error('ERROR WITH I2C: {}'.format(e))
            self.shutdown_routine(exit_code=1)

        with open(WAYPOINT_FILE) as f:
            self.waypoints = geojson.load(f)

        self.cyclic = self.waypoints['properties']['cyclic']
        self.start_threads()

    def run(self):
        while False and self.store.__getitem__('gps')['status'] != 'A':
            logging.info('Waiting for gps...')
            time.sleep(0.2)

        while True:
            for i, waypoint in enumerate(self.waypoints['geometry']['coordinates']):
                logging.info('Using waypoint %i: %s' % (i, waypoint))

                gps = self.store.__getitem__('gps')
                waypoint_geodesic = Geodesic.WGS84.Inverse(gps['lat'], gps['lon'], waypoint[1], waypoint[0])
                boat_geodesic = waypoint_geodesic

                while boat_geodesic['s12'] > TARGET_RADIUS:
                    waypoint_vector = Vector(math.radians(waypoint_geodesic['azi1']), waypoint_geodesic['s12'])
                    boat_vector = Vector(math.radians(boat_geodesic['azi1']), boat_geodesic['s12'])

                    distance_to_line = np.cross(waypoint_vector.get_vector(), boat_vector.get_vector()) / \
                                       np.linalg.norm(waypoint_vector.get_vector())

                    if self.store.__getitem__('wind') < DIRECT_WIND_OFFSET:
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
        self.mavlink_thread.start()

    def shutdown_routine(self, exit_code=0):
        self.gps_thread_stop.set()
        self.wind_thread_stop.set()
        self.compass_thread_stop.set()
        self.mavlink_thread_stop.set()
        if exit_code == 0:
            logging.info('Goodbye :)')
        else:
            logging.info('Goodbye :/')
        sys.exit(exit_code)


if __name__ == '__main__':
    control = None
    try:
        control = Control()
        control.run()
    except KeyboardInterrupt:
        control.shutdown_routine()
    except Exception as e:
        logging.error('ERROR IN MAIN THREAD: {}'.format(e))
        control.shutdown_routine(exit_code=1)
