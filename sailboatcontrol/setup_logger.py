#!/usr/bin/env python
# -*- coding: utf-8 -*-

import logging
from logging.handlers import TimedRotatingFileHandler

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)-5.5s]  %(message)s',
    handlers=[
        TimedRotatingFileHandler('{0}/{1}'.format('../logs', 'sailboat-control'), encoding='utf-8', when="m",
                                 interval=30),
        logging.StreamHandler()
    ]
)