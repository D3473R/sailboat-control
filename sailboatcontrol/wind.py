#!/usr/bin/env python
# -*- coding: utf-8 -*-

from threading import Thread


class Wind(Thread):
    def __init__(self, stop_event, store):
        """ The wind sensor class. """

        Thread.__init__(self, name='Wind')
        self.stop_event = stop_event
        self.store = store
        self.store.__setitem__('wind', 0)

    def run(self):
        pass
