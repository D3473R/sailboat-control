#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import time
import serial
import pynmea2

from os import path
from threading import Thread
from datetime import datetime
from helpers import kn_to_ms
from setup_logger import logging

GPS_ERROR_TIMEOUT = 2
GPS_SERIAL_PORT = '/dev/ttyACM0'
GPS_LOG_FILE = 'gps.csv'


class Gps(Thread):
    def __init__(self, stop_event, store):
        """ The gps sensor class. """

        Thread.__init__(self, name='Gps')
        self.stop_event = stop_event
        self.store = store
        self.gps_serial = serial.Serial(GPS_SERIAL_PORT, timeout=5.0)
        self.store.__setitem__('gps',
                               {'lon': 0.0, 'lat': 0.0, 'speed': 0.0, 'status': 'V', 'timestamp': datetime.now()})

    def run(self):
        try:
            with open(path.join(path.dirname(path.dirname(path.abspath(__file__))), 'logs', GPS_LOG_FILE),
                      'a') as gps_log_file:
                csv_writer = csv.writer(gps_log_file)
                while not self.stop_event.is_set():
                    try:
                        line = self.gps_serial.readline().decode('ASCII')
                        msg = pynmea2.parse(line)
                        if msg.sentence_type == 'RMC' and msg.status == 'A':
                            now = datetime.now()
                            speed = kn_to_ms(msg.spd_over_grnd)
                            self.store.__setitem__('gps', {'lon': msg.longitude, 'lat': msg.latitude, 'speed': speed,
                                                           'status': msg.status, 'timestamp': now})
                            csv_writer.writerow([datetime.utcnow().isoformat(), msg.latitude, msg.longitude])
                            logging.info(
                                'Boat gps: {}, {}, timestamp={}, speed={}'.format(msg.latitude, msg.longitude, now,
                                                                                  speed))
                    except Exception as e:
                        logging.warning('ERROR IN GPS THREAD, WAITING {} SECONDS...: {}'.format(GPS_ERROR_TIMEOUT, e))
                        time.sleep(GPS_ERROR_TIMEOUT)
        except Exception as e:
            logging.error('ERROR IN GPS THREAD: {}'.format(e))
