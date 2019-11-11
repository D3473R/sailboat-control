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

from mission import Mission
from repeatedTimer import RepeatedTimer

MAVLINK_SERIAL_PORT = '/dev/ttyUSB0'
MAVLINK_SERIAL_BAUD = 57600  # This should be set to 57600 in order to work with the Holybro Telemetry Radio V3
STATUS_SLEEP = 1
HEARTBEAT_SLEEP = 1


class Mavlink():
    def __init__(self, store):
        ''' The mavlink telemetry class. '''
        self.store = store

        self.connection = mavutil.mavlink_connection(MAVLINK_SERIAL_PORT, dialect='sailboat', baud=MAVLINK_SERIAL_BAUD)

        self.receive_thread_stop = Event()

        self.status_timer = RepeatedTimer(STATUS_SLEEP, self.sendStatus)
        self.heartbeat_timer = RepeatedTimer(HEARTBEAT_SLEEP, self.sendHeartbeat)
        self.receive_thread = Thread(target=self.receive, name='Mavlink-receive')

        logging.info('Starting receive thread')
        self.receive_thread.start()

    def send(self, message):
        self.connection.write(message.pack(self.connection.mav))
        logging.info('sending: {}'.format(message))
        logging.debug('sending hex: {}'.format(' '.join(hex(i) for i in bytearray(message.pack(self.connection.mav)))))

    def sendStatus(self):
        gps = self.store.__getitem__('gps')
        compass = self.store.__getitem__('compass')
        self.send(self.connection.mav.boat_status_encode(
            float(compass['roll']),
            float(compass['pitch']),
            float(compass['heading']),
            float(gps['lat']) * 10000000,
            float(gps['lon']) * 10000000,
            float(gps['speed'])
        ))

    def receive(self):
        while not self.receive_thread_stop.is_set():
            m = self.connection.recv_msg()
            if m is not None:
                logging.info('receiving: {}'.format(m))
                if type(m) is mavlink.MAVLink_mission_count_message:
                    self.mission = Mission(self, m.count)
                if type(m) is mavlink.MAVLink_mission_item_int_message:
                    self.mission.update(m)
            # Sleep a little bit to free the CPU
            time.sleep(0.01)

    def sendHeartbeat(self):
        self.send(self.connection.mav.heartbeat_encode(
            mavlink.MAV_TYPE_GENERIC,
            mavlink.MAV_AUTOPILOT_GENERIC,
            mavlink.MAV_MODE_FLAG_AUTO_ENABLED,
            0,
            mavlink.MAV_STATE_ACTIVE
        ))

    def shutdown(self):
        self.status_timer.stop()
        self.heartbeat_timer.stop()
        self.receive_thread_stop.set()
