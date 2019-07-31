#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time
import smbus
from setup_logger import logging
from threading import Thread

CALIBRATION_SLEEP = 0.2
UPDATE_SLEEP = 0.1


class Compass(Thread):
    def __init__(self, stop_event, store):
        """ The compass sensor class. """

        Thread.__init__(self, name='Compass')
        self.stop_event = stop_event
        self.store = store
        self.bus = None
        self.store.__setitem__('compass', {'heading': 0, 'pitch': 0, 'roll': 0})

    def run(self):
        try:
            self.bus = smbus.SMBus(1)
            while True:
                sys, gyro, accel, mag = self.read_calibration_state()
                logging.info('Calibration data: sys={}, gyro={}, accel={}, mag={}'.format(sys, gyro, accel, mag))

                if sys >= 1 and gyro >= 1 and accel >= 0 and mag >= 1:
                    logging.info('Calibration complete!')
                    break
                time.sleep(CALIBRATION_SLEEP)

            while not self.stop_event.is_set():
                self.store.__setitem__('compass', {'heading': self.read_heading(), 'pitch': self.read_pitch(),
                                                   'roll': self.read_roll()})
                time.sleep(UPDATE_SLEEP)
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
        return self.twos_comp(self.bus.read_byte_data(0x60, 0x04))

    def read_roll(self):
        return self.twos_comp(self.bus.read_byte_data(0x60, 0x05))

    @staticmethod
    def twos_comp(val):
        if (val & (1 << 7)) != 0:
            val = val - (1 << 8)
        return val
