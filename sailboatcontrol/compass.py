#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import smbus
from .setup_logger import logging
from threading import Thread


class Compass(Thread):
    def __init__(self, stop_event):
        """ The compass sensor class. """

        Thread.__init__(self)
        self.calibration_sleep = 0.2
        self.update_sleep = 0.1
        self.stop_event = stop_event
        self.bus = smbus.SMBus(1)
        self.compass = {'heading': 0, 'pitch': 0, 'roll': 0}

    def run(self):
        while True:
            sys, gyro, accel, mag = self.read_calibration_state()
            logging.info('Calibration data: sys={}, gyro={}, accel={}, mag={}'.format(sys, gyro, accel, mag))

            if sys >= 1 and gyro >= 1 and accel >= 0 and mag >= 1:
                logging.info('Calibration complete!')
                break
            time.sleep(self.calibration_sleep)

        try:
            while not self.stop_event.is_set():
                self.compass['heading'] = self.read_heading()
                self.compass['pitch'] = self.read_pitch()
                self.compass['roll'] = self.read_roll()
                time.sleep(self.update_sleep)
        except Exception as e:
            logging.error('ERROR IN COMPASS THREAD: {}'.format(e))

    def read_calibration_state(self):
        value = self.bus.read_byte_data(0x60, 0x1E)
        mask = 0b00000011
        calibration = [0, 0, 0, 0]
        for i in range(4):
            calibration[3 - i] = (value & mask) >> (2 * i)
            mask = mask << 2
        return calibration

    def read_heading(self):
        return ((self.bus.read_byte_data(0x60, 0x02) << 8) + self.bus.read_byte_data(0x60, 0x03)) / 10

    def read_pitch(self):
        return self.bus.read_byte_data(0x60, 0x04)

    def read_roll(self):
        return self.bus.read_byte_data(0x60, 0x05)
