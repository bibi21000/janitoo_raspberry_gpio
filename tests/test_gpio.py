# -*- coding: utf-8 -*-

"""Unittests for Janitoo-Roomba Server.
"""
__license__ = """
    This file is part of Janitoo.

    Janitoo is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    Janitoo is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Janitoo. If not, see <http://www.gnu.org/licenses/>.

"""
__author__ = 'Sébastien GALLET aka bibi21000'
__email__ = 'bibi21000@gmail.com'
__copyright__ = "Copyright © 2013-2014-2015-2016 Sébastien GALLET aka bibi21000"

import warnings
warnings.filterwarnings("ignore")

import sys, os
import time, datetime
import unittest
import threading
import logging
from pkg_resources import iter_entry_points

from janitoo_nosetests.server import JNTTServer, JNTTServerCommon
from janitoo_nosetests.thread import JNTTThread, JNTTThreadCommon
from janitoo_nosetests.component import JNTTComponent, JNTTComponentCommon

from janitoo.utils import json_dumps, json_loads
from janitoo.utils import HADD_SEP, HADD
from janitoo.utils import TOPIC_HEARTBEAT
from janitoo.utils import TOPIC_NODES, TOPIC_NODES_REPLY, TOPIC_NODES_REQUEST
from janitoo.utils import TOPIC_BROADCAST_REPLY, TOPIC_BROADCAST_REQUEST
from janitoo.utils import TOPIC_VALUES_USER, TOPIC_VALUES_CONFIG, TOPIC_VALUES_SYSTEM, TOPIC_VALUES_BASIC

#~ from janitoo_raspberry_gpio.thread_gpio import GpioThread
#~ import janitoo_raspberry_gpio.gpio

class TestGpioInput(JNTTComponent, JNTTComponentCommon):
    """Test the component
    """
    component_name = "rpigpio.input"

class TestGpioOutput(JNTTComponent, JNTTComponentCommon):
    """Test the component
    """
    component_name = "rpigpio.output"

class TestGpioPwm(JNTTComponent, JNTTComponentCommon):
    """Test the component
    """
    component_name = "rpigpio.pwm"

class TestGpioRGB(JNTTComponent, JNTTComponentCommon):
    """Test the component
    """
    component_name = "rpigpio.rgb"

class TestGpioServo(JNTTComponent, JNTTComponentCommon):
    """Test the component
    """
    component_name = "rpigpio.servo"

class TestGpioPir(JNTTComponent, JNTTComponentCommon):
    """Test the component
    """
    component_name = "rpigpio.pir"

    def test_101_detect(self):
        self.onlyRasperryTest()
        import Adafruit_GPIO as GPIO
        comp = self.factory[self.component_name]()
        gpio = GPIO.get_platform_gpio()
        comp.setup_pir(gpio, 21, GPIO.RISING, comp.callback_pir, 200)
        time.sleep(5)
        dist = comp.values['status'].data
        print("status", dist)
        self.assertNotEqual(dist, None)
        self.assertTrue(comp.check_heartbeat())
        gpio.cleanup()

class TestGpioSonic(JNTTComponent, JNTTComponentCommon):
    """Test the component
    """
    component_name = "rpigpio.sonic"

    def test_102_read_distance(self):
        self.onlyRasperryTest()
        import Adafruit_GPIO as GPIO
        comp = self.factory[self.component_name]()
        gpio = GPIO.get_platform_gpio()
        comp.setup_sonic(gpio, 20, 21, GPIO.RISING, comp.callback_echo, 200)
        comp.trigger_sonic(gpio, 20)
        time.sleep(1)
        dist = comp.values['status'].data
        print("distance", dist)
        self.assertNotEqual(dist, None)
        self.assertTrue(comp.check_heartbeat())
        gpio.cleanup()
