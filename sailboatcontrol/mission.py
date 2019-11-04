#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time

from setup_logger import logging
from threading import Timer

MAX_REPEAT = 5
TIMEOUT_REPEAT = 10

class Mission():
    def __init__(self, mavlink, count):
        ''' The mavlink mission class. '''
        self.mavlink = mavlink
        self.count = count
        self.currentItem = 0
        self.repeatCounter = 0
        self.items = [None] * count

        self.mavlink.send(self.mavlink.connection.mav.mission_request_int_encode(self.currentItem))
        self.timer = Timer(TIMEOUT_REPEAT, self.repeat)
        self.timer.start()

    def repeat(self):
        self.mavlink.send(self.mavlink.connection.mav.mission_request_int_encode(self.currentItem))
        logging.info('repeating mission request {}/{} with {} tries'.format(self.currentItem, self.count, self.repeatCounter + 1))
        if self.repeatCounter < MAX_REPEAT:
            self.timer = Timer(TIMEOUT_REPEAT, self.repeat)
            self.timer.start()
        self.repeatCounter += 1

    def update(self, item):
        if item.seq == self.currentItem:
            self.timer.cancel()
            self.items[item.seq] = item
            logging.info(item)
            if item.seq < self.count:
                self.currentItem += 1
                self.mavlink.send(self.mavlink.connection.mav.mission_request_int_encode(self.currentItem))
