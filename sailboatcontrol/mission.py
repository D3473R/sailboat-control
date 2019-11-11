#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

from setup_logger import logging
from threading import Timer

MAX_REPEAT = 5
TIMEOUT_REPEAT = 2


class Mission():
    def __init__(self, mavlink, count):
        ''' The mavlink mission class. '''
        self.mavlink = mavlink
        self.count = count
        self.currentItem = 0
        self.repeatCounter = 0
        self.items = [None] * count
        self.send()
        self.timer = Timer(TIMEOUT_REPEAT, self.repeat)
        self.timer.start()

    def send(self):
        # No idea why seq has to be 1000
        self.mavlink.send(self.mavlink.connection.mav.mission_request_int_encode(1000, self.currentItem))

    def repeat(self):
        self.send()
        logging.info('repeating mission request {}/{} with {} tries'.format(self.currentItem, self.count, self.repeatCounter + 1))
        if self.repeatCounter < MAX_REPEAT:
            self.timer = Timer(TIMEOUT_REPEAT, self.repeat)
            self.timer.start()
        else:
            self.sendAck(self.mavlink.sailboat.MAV_MISSION_ERROR)
        self.repeatCounter += 1

    def update(self, item):
        if item.count == self.currentItem:
            self.timer.cancel()
            self.items[item.count] = item
            logging.info(item)
            if item.count < self.count - 1:
                self.currentItem += 1
                self.send()
            else:
                self.sendAck(self.mavlink.sailboat.MAV_MISSION_ACCEPTED)
                self.logMission()

    def sendAck(self, type):
        self.mavlink.send(self.mavlink.connection.mav.mission_ack_encode(type))

    def logMission(self):
        s = "Mission: "
        for item in self.items:
            s += "count: {} (lat: {}, lng: {}), ".format(item.count, float(item.lat) / 10000000, float(item.lon) / 10000000)
        logging.info(s)