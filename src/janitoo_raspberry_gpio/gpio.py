# -*- coding: utf-8 -*-
"""The Raspberry GPIO components

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
__copyright__ = "Copyright © 2013-2014-2015 Sébastien GALLET aka bibi21000"

import logging
logger = logging.getLogger(__name__)
import os, sys
import time
import threading

from janitoo.thread import JNTBusThread, BaseThread
from janitoo.options import get_option_autostart
from janitoo.utils import HADD
from janitoo.node import JNTNode
from janitoo.value import JNTValue
from janitoo.component import JNTComponent
from janitoo.bus import JNTBus
try:
    import Adafruit_GPIO as GPIO
except RuntimeError:
    logger.exception('Can"t import GPIO')

##############################################################
#Check that we are in sync with the official command classes
#Must be implemented for non-regression
from janitoo.classes import COMMAND_DESC

COMMAND_WEB_CONTROLLER = 0x1030
COMMAND_WEB_RESOURCE = 0x1031
COMMAND_DOC_RESOURCE = 0x1032

assert(COMMAND_DESC[COMMAND_WEB_CONTROLLER] == 'COMMAND_WEB_CONTROLLER')
assert(COMMAND_DESC[COMMAND_WEB_RESOURCE] == 'COMMAND_WEB_RESOURCE')
assert(COMMAND_DESC[COMMAND_DOC_RESOURCE] == 'COMMAND_DOC_RESOURCE')
##############################################################

def make_input(**kwargs):
    return InputComponent(**kwargs)

def make_output(**kwargs):
    return OutputComponent(**kwargs)

def make_pwm(**kwargs):
    return PwmComponent(**kwargs)

def make_pir(**kwargs):
    return PirComponent(**kwargs)

def make_sonic(**kwargs):
    return SonicComponent(**kwargs)

class GpioBus(JNTBus):
    """A bus to manage GPIO
    """
    def __init__(self, **kwargs):
        """
        :param kwargs: parameters transmitted to :py:class:`smbus.SMBus` initializer
        """
        JNTBus.__init__(self, **kwargs)
        self._lock =  threading.Lock()
	self.gpio = GPIO.get_platform_gpio()
        uuid="boardmode"
        self.values[uuid] = self.value_factory['config_list'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The board mode to use',
            label='Boardmode',
            default='BOARD',
            list_items=['BCM', 'BOARD'],
        )

    def check_heartbeat(self):
        """Check that the component is 'available'

        """
        #~ print "it's me %s : %s" % (self.values['upsname'].data, self._ups_stats_last)
        if self.gpio.RPI_INFO['P1_REVISION']>0:
            return True
        return False

    def start(self, mqttc, trigger_thread_reload_cb=None):
        """Start the bus
        """
        JNTBus.start(self, mqttc, trigger_thread_reload_cb)
        try:
            #if self.values["boardmode"].data == "BCM":
            #    self.gpio.setmode(self.gpio.BCM)
            #else:
            #    self.gpio.setmode(self.gpio.BOARD)
	    pass
        except:
            logger.exception("Exception when starting GPIO bus")

    def stop(self):
        """Stop the bus
        """
        try:
            self.gpio.cleanup()
        except:
            logger.exception("Exception when stopping GPIO bus")
        JNTBus.stop(self)

class GpioComponent(JNTComponent):
    """ A generic component for gpio """

    def __init__(self, **kwargs):
        """
        """
        oid = kwargs.pop('oid', 'rpigpio.generic')
        name = kwargs.pop('name', "Input")
        product_name = kwargs.pop('product_name', "GPIO")
        product_type = kwargs.pop('product_type', "Software")
        product_manufacturer = kwargs.pop('product_manufacturer', "Janitoo")
        JNTComponent.__init__(self, oid=oid, name=name, product_name=product_name, product_type=product_type, product_manufacturer=product_manufacturer, **kwargs)
        logger.debug("[%s] - __init__ node uuid:%s", self.__class__.__name__, self.uuid)

        uuid="pin"
        self.values[uuid] = self.value_factory['config_integer'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The pin number on the board',
            label='Pin',
            default=kwargs.pop('pin', 1),
        )

class InputComponent(GpioComponent):
    """ An input gpio """

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rgpio.input')
        product_name = kwargs.pop('product_name', "Input GPIO")
        name = kwargs.pop('name', "Input GPIO")
        GpioComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)
        uuid="pullupdown"
        self.values[uuid] = self.value_factory['config_list'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='Use a pull up or a pull down',
            label='Pull Up/Down',
            default=kwargs.pop('pulldown', 'PUD_UP'),
            list_items=['PUD_UP', 'PUD_DOWN', 'NONE'],
        )
        uuid="edge"
        self.values[uuid] = self.value_factory['config_list'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='Edge to use (rising or falling)',
            label='Edge',
            default=kwargs.pop('edge','BOTH'),
            list_items=['BOTH', 'RISING', 'FALLING'],
        )
        uuid="bouncetime"
        self.values[uuid] = self.value_factory['config_integer'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='Bouncetime should be specified in milliseconds',
            label='bouncetime',
            default=kwargs.pop('bouncetime', 200),
        )
        uuid="trigger"
        self.values[uuid] = self.value_factory['config_boolean'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help="Should we trigger the state's change",
            label='trigger',
            default=True,
        )
        uuid="status"
        self.values[uuid] = self.value_factory['sensor_byte'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The status of the GPIO',
            label='Status',
            get_data_cb=self.get_status,
        )
        poll_value = self.values[uuid].create_poll_value(default=60)
        self.values[poll_value.uuid] = poll_value

    def get_status(self, node_uuid, index):
        """
        """
        if index in self._inputs:
            return self._inputs[index]['value']
        return None

    def trigger_status(self, channel):
        """
        """
	self._inputs[index]['value'] = self._bus.gpio.input(self.values["pin"].instances[config]['data'])
        self.node.publish_poll(None, self.values['status'])

    def start(self, mqttc):
        """Start the component.

        """
        GpioComponent.start(self, mqttc)
        configs = len(self.values["pin"].get_index_configs())
        for config in range(configs):
            try:
                if self.values['pullupdown'].instances[config]['data'] == "PUD_DOWN":
                    pull_up_down = GPIO.PUD_DOWN
                elif self.values['pullupdown'].instances[config]['data'] == "PUD_UP":
                    pull_up_down = GPIO.PUD_UP
                else :
                    pull_up_down = None
                self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.IN, pull_up_down=pull_up_down)
                sedge = self.values['edge'].instances[config]['data']
                if sedge == "RISING":
                    edge = GPIO.RISING
                elif sedge == "FALLING":
                    edge = GPIO.FALLING
                else:
                    edge = GPIO.BOTH
                self._bus.gpio.add_event_detect(self.values["pin"].instances[config]['data'], edge, callback=self.trigger_status, bouncetime=self.values["bouncetime"].instances[config]['data'])
            except:
                logger.exception("Exception when starting GPIO component")
        return True

    def stop(self):
        """Stop the component.

        """
        configs = len(self.values["pin"].get_index_configs())
        for config in range(configs):
            try:
                GPIO.remove_event_detect(self.values["pin"].instances[config]['data'])
            except:
                logger.exception("Exception when stopping GPIO component")
        GpioComponent.stop(self)
        return True

class PirComponent(InputComponent):
    """ A PIR motion sensor """

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rgpio.pir')
        product_name = kwargs.pop('product_name', "PIR sensor")
        name = kwargs.pop('name', "PIR sensor")
        InputComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)

    def start(self, mqttc):
        """Start the component.

        """
        InputComponent.start(self, mqttc)
        configs = len(self.values["pin"].get_index_configs())
        for config in range(configs):
            try:
                while self._bus.gpio.input(self.values["pin"].instances[config]['data'])==1:
                    self.values["status"].instances[config]['data'] = 0
            except:
                logger.exception("Exception when starting PIR component")
        return True

class SonicComponent(InputComponent):
    """ A Sonic sensor (HC-SR04)"""

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rgpio.sonic')
        product_name = kwargs.pop('product_name', "Sonic sensor")
        name = kwargs.pop('name', "Sonic sensori (HC-SR04)")
        InputComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)
	del self.values['pin']
        uuid="pin_trig"
        self.values[uuid] = self.value_factory['config_integer'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The pin number on the board for the trigger',
            label='Trig',
            default=kwargs.pop('pin_trig', 20),
        )

        uuid="pin_echo"
        self.values[uuid] = self.value_factory['config_integer'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The pin number on the board for the echo',
            label='Echo',
            default=kwargs.pop('pin_echo', 21),
        )
        self.check_timer = None
        uuid="timer_delay"
        self.values[uuid] = self.value_factory['config_integer'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The delay between 2 checks',
            label='Timer.',
            default=kwargs.pop('timer_delay', 5),
        )
	self.echo_start = None
	self.echo_stop = None

    def stop_check(self):
        """Check the sonic component

        """
        if self.check_timer is not None:
            self.check_timer.cancel()
            self.check_timer = None

    def on_check(self):
        """Make a check using a timer.

        """
        self.stop_check()
        if self.check_timer is None:
            self.check_timer = threading.Timer(self.values['timer_delay'].data, self.on_check)
            self.check_timer.start()
        state = True
        #if self._bus.nodeman.is_started:
	self.send_trigger()

    def send_trigger(self):
        """Make a check using a timer.

        """
        configs = len(self.values["pin_echo"].get_index_configs())
	if configs == 0:
            try:
		# Send 10us pulse to trigger
		self._bus.gpio.output(self.values["pin_trig"].data, GPIO.HIGH)
        	time.sleep(0.00001)
	        self._bus.gpio.output(self.values["pin_trig"].data, GPIO.LOW)
	        self.echo_start = time.time()
	        logger.debug("[%s] - Send trigger", self.__class__.__name__)
            except:
                logger.exception("Exception when starting sonic component")
	else:
	    for config in range(configs):
                try:
		    # Send 10us pulse to trigger
		    self._bus.gpio.output(self.values["pin_trig"].instances[config]['data'], GPIO.HIGH)
        	    time.sleep(0.00001)
	            self._bus.gpio.output(self.values["pin_trig"].instances[config]['data'], GPIO.LOW)
	            self.echo_start = time.time()
	            logger.debug("[%s] - Send trigger", self.__class__.__name__)
                except:
                    logger.exception("Exception when starting sonic component")

    def start(self, mqttc):
        """Start the component.

        """
        GpioComponent.start(self, mqttc)
        configs = len(self.values["pin_echo"].get_index_configs())
	if configs == 0:
	    try:
                self._bus.gpio.setup(self.values["pin_trig"].data, GPIO.OUT)
                self._bus.gpio.setup(self.values["pin_echo"].data, GPIO.IN)
                self._bus.gpio.output(self.values["pin_trig"].data, GPIO.LOW)
                self._bus.gpio.add_event_detect(self.values["pin_echo"].data, GPIO.RISING, callback=self.callback_echo, bouncetime=self.values["bouncetime"].data)
		logger.debug("[%s] - start sonic component on trigger pin %s and echo pin %s", self.__class__.__name__, self.values["pin_trig"].data, self.values["pin_echo"].data)
            except:
                logger.exception("Exception when starting sonic component")
	else:
            for config in range(configs):
	        print "config :", config
                try:
                    self._bus.gpio.setup(self.values["pin_trig"].instances[config]['data'], GPIO.OUT)
                    self._bus.gpio.setup(self.values["pin_echo"].instances[config]['data'], GPIO.IN)
                    self._bus.gpio.output(self.values["pin_trig"].instances[config]['data'], GPIO.LOW)
                    self._bus.gpio.add_event_detect(self.values["pin_echo"].instances[config]['data'], GPIO.RISING, callback=self.callback_echo, bouncetime=self.values["bouncetime"].instances[config]['data'])
                except:
                    logger.exception("Exception when starting sonic component")
	self.on_check()
        return True

    def callback_echo(self, channel):
        """
        """
	self.echo_stop = time.time()
	elapsed = self.echo_stop - self.echo_start
	dist = elapsed * 34000.0 / 2
	self._inputs[0]['value'] = dist
	logger.debug("[%s] - callback_echo distance : %s cm on channel %s", self.__class__.__name__, dist, channel)

    def stop(self):
        """Stop the component.

        """
	self.stop_check()
        configs = len(self.values["pin_echo"].get_index_configs())
        for config in range(configs):
            try:
                self._bus.gpio.remove_event_detect(self.values["pin_echo"].instances[config]['data'])
            except:
                logger.exception("Exception when stopping sonic component")
        GpioComponent.stop(self)
        return True

class OutputComponent(GpioComponent):
    """ A resource ie /rrd """

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rgpio.output')
        product_name = kwargs.pop('product_name', "Output GPIO")
        name = kwargs.pop('name', "Output GPIO")
        GpioComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)
        uuid="state"
        self.values[uuid] = self.value_factory['action_boolean'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The state of the GPIO',
            label='State',
            set_data_cb=self.set_state,
        )
        poll_value = self.values[uuid].create_poll_value(default=60)
        self.values[poll_value.uuid] = poll_value

    def start(self, mqttc):
        """Start the component.

        """
        GpioComponent.start(self, mqttc)
        configs = len(self.values["pin"].get_index_configs())
        for config in range(configs):
            try:
                self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.OUT)
            except:
                logger.exception("Exception when starting GPIO component")
        return True

    def stop(self):
        """Stop the component.

        """
        GpioComponent.stop(self)
        return True

    def set_state(self, node_uuid, index, data):
        """
        """
        if index in self._inputs:
            try:
                if data == True or data == 1:
                    self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.HIGH)
                else:
                    self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.LOW)
            except:
                logger.exception("Exception when updating GPIO component")

class PwmComponent(GpioComponent):
    """ A resource ie /rrd """
