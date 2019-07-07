#!/usr/bin/env python
# -*- coding: utf-8 -*-

from threading import Thread


class Wind(Thread):
    def __init__(self, stop_event):
        """ The wind sensor class. """

        Thread.__init__(self)
        self.stop_event = stop_event
        self.wind = 0

    def run(self):
        pass
