#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import math
import time

from setup_logger import logging
from threading import Thread, Event

os.environ['MAVLINK20'] = '1'
os.environ['MAVLINK_DIALECT'] = 'sailboat'

from pymavlink import mavutil
import pymavlink.dialects.v20.sailboat as mavlink

MAVLINK_SERIAL_PORT = '/dev/ttyUSB0'
MAVLINK_SERIAL_BAUD = 57600  # This should be set to 57600 in order to work with the Holybro Telemetry Radio V3
UPDATE_SLEEP = 1


class Mavlink(Thread):
    def __init__(self, stop_event, store):
        ''' The mavlink telemetry class. '''

        Thread.__init__(self, name='Mavlink')
        self.stop_event = stop_event
        self.store = store

        self.send_thread_stop = Event()
        self.receive_thread_stop = Event()
        self.heartbeat_thread_stop = Event()
        self.connection = mavutil.mavlink_connection(MAVLINK_SERIAL_PORT, dialect='sailboat', baud=MAVLINK_SERIAL_BAUD)
        self.send_thread = Thread(target=self.send, name='Mavlink-send')
        self.receive_thread = Thread(target=self.receive, name='Mavlink-receive')
        self.heartbeat_thread = Thread(target=self.heartbeat, name='Mavlink-heartbeat')

    def run(self):
        logging.info('Starting send thread')
        self.send_thread.start()

        logging.info('Starting receive thread')
        self.receive_thread.start()

        logging.info('Starting heartbeat thread')
        self.heartbeat_thread.start()

        while not self.stop_event.is_set():
            pass

        self.shutdown()

    def send(self):
        while not self.send_thread_stop.is_set():
            gps = self.store.__getitem__('gps')
            compass = self.store.__getitem__('compass')
            message = self.connection.mav.boat_status_encode(
                float(compass['roll']),
                float(compass['pitch']),
                float(compass['heading']),
                float(gps['lat']) * 10000000,
                float(gps['lon']) * 10000000,
                float(gps['speed'])
            )
            self.connection.write(message.pack(self.connection.mav))
            logging.info('sending: {}'.format(message))
            logging.debug(
                'sending hex: {}'.format(' '.join(hex(i) for i in bytearray(message.pack(self.connection.mav)))))
            time.sleep(UPDATE_SLEEP)

    def receive(self):
        while not self.receive_thread_stop.is_set():
            m = self.connection.recv_match(blocking=False)
            if m is not None:
                logging.debug('receiving: {}'.format(m))

    def heartbeat(self):
        last_heartbeat_send = time.time()
        while not self.heartbeat_thread_stop.is_set():
            now = time.time()
            if now - last_heartbeat_send > 1.0:
                heartbeat = self.connection.mav.heartbeat_encode(
                    mavlink.MAV_TYPE_GENERIC,
                    mavlink.MAV_AUTOPILOT_GENERIC,
                    mavlink.MAV_MODE_FLAG_AUTO_ENABLED,
                    0,
                    mavlink.MAV_STATE_ACTIVE
                )
                self.connection.write(heartbeat.pack(self.connection.mav))
                last_heartbeat_send = now
                logging.debug('sending: {}'.format(heartbeat))
                logging.debug(
                    'sending hex: {}'.format(' '.join(hex(i) for i in bytearray(heartbeat.pack(self.connection.mav)))))

    def shutdown(self):
        self.send_thread_stop.set()
        self.receive_thread_stop.set()
        self.heartbeat_thread_stop.set()
