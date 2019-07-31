#!/usr/bin/env python
# -*- coding: utf-8 -*-

from threading import Lock


class Store:
    def __init__(self):
        """ The store class. """

        self.dict = dict()
        self.lock = Lock()

    def __getitem__(self, key):
        with self.lock:
            return self.dict[key]

    def __setitem__(self, key, value):
        with self.lock:
            self.dict[key] = value

    def __delitem__(self, key):
        with self.lock:
            del self.dict[key]