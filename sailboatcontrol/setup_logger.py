#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging

from os import path
from logging import Formatter, StreamHandler
from logging.handlers import TimedRotatingFileHandler

logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)

formatter = Formatter('%(asctime)s [%(levelname)-5.5s] %(message)s')

file_handler = TimedRotatingFileHandler(path.join(path.dirname(path.dirname(path.abspath(__file__))), 'logs', 'sailboat-control'), encoding='utf-8', when="m", interval=30)
file_handler.setFormatter(formatter)
logger.addHandler(file_handler)

stream_handler = StreamHandler()
stream_handler.setFormatter(formatter)
logger.addHandler(stream_handler)

logging = logger
