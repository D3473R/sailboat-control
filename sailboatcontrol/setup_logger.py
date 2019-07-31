#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging

from os import path
from logging.handlers import TimedRotatingFileHandler

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)-5.5s]  %(message)s',
    handlers=[
        TimedRotatingFileHandler(
            path.join(path.dirname(path.dirname(path.abspath(__file__))), 'logs', 'sailboat-control'),
            encoding='utf-8', when="m", interval=30),
        logging.StreamHandler()
    ]
)
