#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import time
import serial
import pynmea2
from threading import Thread
from datetime import datetime
from .helpers import kn_to_ms
from .setup_logger import logging

GPS_ERROR_TIMEOUT = 2
GPS_SERIAL_PORT = '/dev/ttyACM0'
GPS_LOG_FILE = '../logs/gps.csv'


class Gps(Thread):
    def __init__(self, stop_event):
        """ The gps sensor class. """

        Thread.__init__(self)
        self.stop_event = stop_event
        self.gps_serial = serial.Serial(GPS_SERIAL_PORT, timeout=5.0)
        self.gps = {'lon': 0.0, 'lat': 0.0, 'speed': 0.0, 'status': 'V', 'timestamp': datetime.now()}

    def run(self):
        try:
            with open(GPS_LOG_FILE, 'a') as gps_log_file:
                csv_writer = csv.writer(gps_log_file)
                while not self.stop_event.is_set():
                    try:
                        line = self.gps_serial.readline().decode('ASCII')
                        msg = pynmea2.parse(line)
                        if msg.sentence_type == 'RMC' and msg.status == 'A':
                            self.gps['status'] = msg.status
                            self.gps['lon'] = msg.longitude
                            self.gps['lat'] = msg.latitude
                            self.gps['speed'] = kn_to_ms(msg.spd_over_grnd)
                            self.gps['timestamp'] = datetime.now()
                            csv_writer.writerow([datetime.utcnow().isoformat(), msg.latitude, msg.longitude])
                    except Exception as e:
                        logging.warning('ERROR IN GPS THREAD, WAITING {} SECONDS...: {}'.format(GPS_ERROR_TIMEOUT, e))
                        time.sleep(GPS_ERROR_TIMEOUT)
        except Exception as e:
            logging.error('ERROR IN GPS THREAD: {}'.format(e))
