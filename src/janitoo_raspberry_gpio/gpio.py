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
__copyright__ = "Copyright © 2013-2014-2015-2016 Sébastien GALLET aka bibi21000"

import logging
logger = logging.getLogger(__name__)
import os, sys
import time
import datetime
import threading

from janitoo.thread import JNTBusThread, BaseThread
from janitoo.options import get_option_autostart
from janitoo.utils import HADD
from janitoo.node import JNTNode
from janitoo.value import JNTValue
from janitoo.component import JNTComponent
from janitoo.bus import JNTBus
from janitoo_raspberry_gpio.thread_gpio import OID

try:
    import Adafruit_GPIO as GPIO
except:
    logger.exception('Can"t import GPIO')

##############################################################
#Check that we are in sync with the official command classes
#Must be implemented for non-regression
from janitoo.classes import COMMAND_DESC

COMMAND_SWITCH_BINARY = 0x0025
COMMAND_WEB_RESOURCE = 0x1031
COMMAND_DOC_RESOURCE = 0x1032

assert(COMMAND_DESC[COMMAND_SWITCH_BINARY] == 'COMMAND_SWITCH_BINARY')
assert(COMMAND_DESC[COMMAND_WEB_RESOURCE] == 'COMMAND_WEB_RESOURCE')
assert(COMMAND_DESC[COMMAND_DOC_RESOURCE] == 'COMMAND_DOC_RESOURCE')
##############################################################

def make_input(**kwargs):
    return InputComponent(**kwargs)

def make_output(**kwargs):
    return OutputComponent(**kwargs)

def make_pwm(**kwargs):
    return PwmComponent(**kwargs)

def make_rgb(**kwargs):
    return RGBComponent(**kwargs)

def make_led(**kwargs):
    return LedComponent(**kwargs)

def make_pir(**kwargs):
    return PirComponent(**kwargs)

def make_sonic(**kwargs):
    return SonicComponent(**kwargs)

def make_servo(**kwargs):
    return ServoComponent(**kwargs)

class GpioBus(JNTBus):
    """A bus to manage GPIO
    """
    def __init__(self, **kwargs):
        """
        :param kwargs: parameters transmitted to :py:class:`smbus.SMBus` initializer
        """
        JNTBus.__init__(self, **kwargs)
        self._lock =  threading.Lock()
        self.gpio = None
        uuid="%s_boardmode"%OID
        self.values[uuid] = self.value_factory['config_list'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The board mode to use',
            label='Boardmode',
            default='BOARD',
            list_items=['BCM', 'BOARD'],
        )
        self.export_attrs('gpio', self.gpio)

    def check_heartbeat(self):
        """Check that the component is 'available'

        """
        return self.gpio is not None

    def start(self, mqttc, trigger_thread_reload_cb=None):
        """Start the bus
        """
        JNTBus.start(self, mqttc, trigger_thread_reload_cb)
        try:
            self.gpio = GPIO.get_platform_gpio()
        except:
            logger.exception("[%s] - Exception when starting GPIO bus", self.__class__.__name__)
        self.update_attrs('gpio', self.gpio)

    def stop(self):
        """Stop the bus
        """
        JNTBus.stop(self)
        try:
            self.gpio.cleanup()
        except:
            logger.exception("[%s] - Exception when stopping GPIO bus", self.__class__.__name__)
        self.gpio = None
        self.update_attrs('gpio', self.gpio)

class GpioComponent(JNTComponent):
    """ A generic component for gpio """

    def __init__(self, **kwargs):
        """
        """
        oid = kwargs.pop('oid', 'rpigpio.generic')
        name = kwargs.pop('name', "Input")
        product_name = kwargs.pop('product_name', "GPIO")
        product_type = kwargs.pop('product_type', "Software")
        JNTComponent.__init__(self, oid=oid, name=name, product_name=product_name, product_type=product_type, **kwargs)
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
        oid = kwargs.pop('oid', 'rpigpio.input')
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
            default=kwargs.pop('bouncetime', 300),
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
                logger.debug("[%s] - start GPIO Input component on pin %s", self.__class__.__name__, self.values["pin"].instances[config]['data'])
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
                logger.exception("[%s] - Exception when starting GPIO component", self.__class__.__name__)
        return True

    def stop(self):
        """Stop the component.

        """
        configs = len(self.values["pin"].get_index_configs())
        for config in range(configs):
            try:
                logger.debug("[%s] - stop GPIO Input component on pin %s", self.__class__.__name__, self.values["pin"].instances[config]['data'])
                GPIO.remove_event_detect(self.values["pin"].instances[config]['data'])
            except:
                logger.exception("[%s] - Exception when stopping GPIO component", self.__class__.__name__)
        GpioComponent.stop(self)
        return True

class PirComponent(InputComponent):
    """ A PIR motion sensor """

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rpigpio.pir')
        product_name = kwargs.pop('product_name', "PIR sensor")
        name = kwargs.pop('name', "PIR sensor")
        InputComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)

    def start(self, mqttc):
        """Start the component.

        """
        GpioComponent.start(self, mqttc)
        configs = len(self.values["pin"].get_index_configs())
        if configs == 0:
            try:
                self.setup_pir( self._bus.gpio, \
                                  self.values["pin"].data, \
                                  GPIO.BOTH, \
                                  self.callback_pir, \
                                  self.values["bouncetime"].data)
            except:
                logger.exception("[%s] - Exception when starting PIR component", self.__class__.__name__)
        else:
            for config in range(configs):
                print "config :", config
                try:
                    self.setup_pir( self._bus.gpio, \
                                      self.values["pin"].instances[config]['data'], \
                                      GPIO.BOTH, \
                                      self.callback_pir, \
                                      self.values["bouncetime"].instances[config]['data'])
                except:
                    logger.exception("[%s] - Exception when starting PIR component", self.__class__.__name__)
        return True

    def setup_pir(self, gpio, pin, edge, callback, bouncetime ):
        try:
            logger.debug("[%s] - start PIR component on pin %s", self.__class__.__name__, pin)
            gpio.setup(pin, GPIO.IN)
            stop_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
            while gpio.input(pin) == GPIO.HIGH:
                if datetime.datetime.now() > stop_at:
                    logger.warning("[%s] - Timeout when starting PIR component", self.__class__.__name__)
                    break
                time.sleep(0.001)
            gpio.add_event_detect(pin, edge, callback=callback, bouncetime=bouncetime)
        except:
            logger.exception("[%s] - Exception when starting PIR component", self.__class__.__name__)

    def callback_pir(self, channel):
        """
        """
        if 0 not in self._inputs:
            self._inputs[0] = {}
        self._inputs[0]['value'] = None
        logger.debug("[%s] - callback_pir : %s on channel %s", self.__class__.__name__, self._inputs[0]['value'], channel)

    def stop(self):
        """Stop the component.

        """
        if self._bus.gpio is None:
            return False
        configs = len(self.values["pin"].get_index_configs())
        logger.warning("[%s] - Stop PIR : configs = %s", self.__class__.__name__, configs)
        if configs == 0:
            try:
                self._bus.gpio.remove_event_detect(self.values["pin"].data)
            except:
                logger.exception("[%s] - Exception when stopping PIR component", self.__class__.__name__)
        else:
            for config in range(configs):
                try:
                    self._bus.gpio.remove_event_detect(self.values["pin"].instances[config]['data'])
                except:
                    logger.exception("[%s] - Exception when stopping PIR component", self.__class__.__name__)
        GpioComponent.stop(self)
        return True

class SonicComponent(InputComponent):
    """ A Sonic sensor (HC-SR04)"""

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rpigpio.sonic')
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
        if self._bus is None or self._bus.gpio is None:
            return
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
                self.trigger_sonic(self._bus.gpio, self.values["pin_trig"].data)
            except:
                logger.exception("[%s] - Exception when starting sonic component", self.__class__.__name__)
        else:
            for config in range(configs):
                try:
                    # Send 10us pulse to trigger
                    self.trigger_sonic(self._bus.gpio, self.values["pin_trig"].instances[config]['data'])
                except:
                    logger.exception("[%s] - Exception when starting sonic component", self.__class__.__name__)

    def start(self, mqttc):
        """Start the component.

        """
        GpioComponent.start(self, mqttc)
        configs = len(self.values["pin_echo"].get_index_configs())
        if configs == 0:
            try:
                self.setup_sonic( self._bus.gpio, \
                                  self.values["pin_trig"].data, \
                                  self.values["pin_echo"].data, \
                                  GPIO.RISING, \
                                  self.callback_echo, \
                                  self.values["bouncetime"].data)
            except:
                logger.exception("[%s] - Exception when starting sonic component", self.__class__.__name__)
        else:
            for config in range(configs):
                print "config :", config
                try:
                    self.setup_sonic( self._bus.gpio, \
                                      self.values["pin_trig"].instances[config]['data'], \
                                      self.values["pin_echo"].instances[config]['data'], \
                                      GPIO.RISING, \
                                      self.callback_echo, \
                                      self.values["bouncetime"].instances[config]['data'])
                except:
                    logger.exception("[%s] - Exception when starting sonic component", self.__class__.__name__)
        self.on_check()
        return True

    def setup_sonic(self, gpio, pin_trig, pin_echo, edge, callback, bouncetime ):
        try:
            gpio.setup(pin_trig, GPIO.OUT)
            gpio.setup(pin_echo, GPIO.IN)
            gpio.output(pin_trig, GPIO.LOW)
            stop_at = datetime.datetime.now() + datetime.timedelta(seconds=5)
            logger.debug("[%s] - start sonic component on trigger pin %s and echo pin %s", self.__class__.__name__, pin_trig, pin_echo)
            while gpio.input(pin_echo) == GPIO.HIGH:
                if datetime.datetime.now() > stop_at:
                    logger.warning("[%s] - Timeout when starting sonic component", self.__class__.__name__)
                    break
                time.sleep(0.1)
            gpio.add_event_detect(pin_echo, edge, callback=callback, bouncetime=bouncetime)
        except:
            logger.exception("[%s] - Exception when setup_sonic component", self.__class__.__name__)

    def callback_echo(self, channel):
        """
        """
        if self.echo_start is None:
            return
        self.echo_stop = time.time()
        elapsed = self.echo_stop - self.echo_start
        dist = elapsed * 34000.0 / 2
        if 0 not in self._inputs:
            self._inputs[0] = {}
        self._inputs[0]['value'] = dist
        logger.debug("[%s] - callback_echo distance : %s cm on channel %s", self.__class__.__name__, dist, channel)

    def trigger_sonic(self, gpio, pin_trig ):
        try:
            # Send 10us pulse to trigger
            gpio.output(pin_trig, GPIO.HIGH)
            time.sleep(0.00001)
            gpio.output(pin_trig, GPIO.LOW)
            self.echo_start = time.time()
            logger.debug("[%s] - Send trigger", self.__class__.__name__)
        except:
            logger.exception("[%s] - Exception when trigger_sonic component", self.__class__.__name__)

    def stop(self):
        """Stop the component.

        """
        self.stop_check()
        if self._bus.gpio is None:
            return False
        configs = len(self.values["pin_echo"].get_index_configs())
        logger.warning("[%s] - Stop SONIC : configs = %s", self.__class__.__name__, configs)
        if configs == 0:
            try:
                self._bus.gpio.remove_event_detect(self.values["pin_echo"].data)
            except:
                logger.exception("[%s] - Exception when stopping sonic component", self.__class__.__name__)
        else:
            for config in range(configs):
                try:
                    self._bus.gpio.remove_event_detect(self.values["pin_echo"].instances[config]['data'])
                except:
                    logger.exception("[%s] - Exception when stopping sonic component", self.__class__.__name__)
        GpioComponent.stop(self)
        return True

class OutputComponent(GpioComponent):
    """ An output component"""

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rpigpio.output')
        product_name = kwargs.pop('product_name', "Output GPIO")
        name = kwargs.pop('name', "Output GPIO")
        GpioComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)
        uuid="switch"
        self.values[uuid] = self.value_factory['action_switch_binary'](
            options=self.options,
            uuid=uuid,
            node_uuid=self.uuid,
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
                logger.debug("[%s] - start GPIO Output component on pin %s", self.__class__.__name__, self.values["pin"].instances[config]['data'])
                self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.OUT)
            except:
                logger.exception("[%s] - Exception when starting GPIO component", self.__class__.__name__)
        return True

    def stop(self):
        """Stop the component.

        """
        logger.debug("[%s] - stop GPIO Output component", self.__class__.__name__)
        configs = len(self.values["pin"].get_index_configs())
        for config in range(configs):
            try:
                logger.debug("[%s] - stop GPIO Output component on pin %s", self.__class__.__name__, self.values["pin"].instances[config]['data'])
                self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.OUT)
            except:
                logger.exception("[%s] - Exception when stopping GPIO component", self.__class__.__name__)
        GpioComponent.stop(self)
        return True

    def set_state(self, node_uuid, index, data):
        """
        """
        if index in self._inputs:
            try:
                if data == True or data == 1 or data.lower() == 'on':
                    self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.HIGH)
                else:
                    self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.LOW)
            except:
                logger.exception("[%s] - Exception when updating GPIO component", self.__class__.__name__)

class PwmComponent(GpioComponent):
    """ A PWM component """

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rpigpio.pwm')
        product_name = kwargs.pop('product_name', "Output PWM")
        name = kwargs.pop('name', "Output PWM")
        GpioComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)
        uuid="level"
        self.values[uuid] = self.value_factory['action_switch_multilevel'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The level of the LED. A byte from 0 to 100',
            label='Level',
            default=0,
            set_data_cb=self.set_level,
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.values[poll_value.uuid] = poll_value
        uuid="max_level"
        self.values[uuid] = self.value_factory['config_byte'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help="The max level supported by the LED. Some LED doesn't seems support 100% PWM. A byte from 0 to 100",
            label='Max level',
            default=100,
        )
        uuid="switch"
        self.values[uuid] = self.value_factory['action_switch_binary'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            list_items=['on', 'off'],
            default='off',
            set_data_cb=self.set_switch,
            genre=0x01,
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.values[poll_value.uuid] = poll_value

    def set_level(self, node_uuid, index, data):
        """Set the level ot the LED
        """
        logger.warning("[%s] - set_level unknown data : %s", self.__class__.__name__, data)

    def set_switch(self, node_uuid, index, data):
        """Switch On/Off the led
        """
        logger.warning("[%s] - set_switch unknown data : %s", self.__class__.__name__, data)

class ServoComponent(GpioComponent):
    """ A servo component for GPIO """

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rpigpio.servo')
        product_name = kwargs.pop('product_name', "Servo")
        name = kwargs.pop('name', "Servo")
        GpioComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)
        uuid="angle"
        self.values[uuid] = self.value_factory['action_switch_multilevel'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The angle',
            label='angle',
            default=0,
            set_data_cb=self.set_angle,
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.values[poll_value.uuid] = poll_value
        uuid="pulse"
        self.values[uuid] = self.value_factory['action_switch_multilevel'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The pulse',
            label='pulse',
            default=0,
            set_data_cb=self.set_pulse,
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.values[poll_value.uuid] = poll_value

    def set_pulse(self, node_uuid, index, data):
        """Set the pulse
        """
        logger.warning("[%s] - set_pulse unknown data : %s", self.__class__.__name__, data)

    def set_angle(self, node_uuid, index, data):
        """Set the angle
        """
        logger.warning("[%s] - set_angle unknown data : %s", self.__class__.__name__, data)

class RGBComponent(GpioComponent):
    """ A PWM RGB component for GPIO """

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rpigpio.rgb')
        product_name = kwargs.pop('product_name', "RGB PWM")
        name = kwargs.pop('name', "RGB PWM")
        GpioComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)
        logger.debug("[%s] - __init__ node uuid:%s", self.__class__.__name__, self.uuid)
        del self.values['pin']

        uuid="pinr"
        self.values[uuid] = self.value_factory['config_integer'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The pin number on the board for Red',
            label='PinR',
            default=kwargs.pop('pinr', 1),
        )
        uuid="ping"
        self.values[uuid] = self.value_factory['config_integer'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The pin number on the board for Green',
            label='PinG',
            default=kwargs.pop('ping', 1),
        )
        uuid="pinb"
        self.values[uuid] = self.value_factory['config_integer'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The pin number on the board for Blue',
            label='PinB',
            default=kwargs.pop('pinb', 1),
        )
        uuid="color"
        self.values[uuid] = self.value_factory['action_switch_multilevel'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            help='The color',
            label='color',
            default='00#00#00',
            set_data_cb=self.set_color,
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.values[poll_value.uuid] = poll_value
        uuid="switch"
        self.values[uuid] = self.value_factory['action_switch_binary'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            list_items=['on', 'off'],
            default='off',
            set_data_cb=self.set_switch,
            genre=0x01,
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.values[poll_value.uuid] = poll_value

    def start(self, mqttc):
        """Start the component.

        """
        GpioComponent.start(self, mqttc)
        #~ configs = len(self.values["pin"].get_index_configs())
        #~ for config in range(configs):
            #~ try:
                #~ logger.debug("[%s] - start GPIO Output component on pin %s", self.__class__.__name__, self.values["pin"].instances[config]['data'])
                #~ self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.OUT)
            #~ except:
                #~ logger.exception("[%s] - Exception when starting GPIO component", self.__class__.__name__)
        return True

    def stop(self):
        """Stop the component.

        """
        logger.debug("[%s] - stop GPIO Output component", self.__class__.__name__)
        #~ configs = len(self.values["pin"].get_index_configs())
        #~ for config in range(configs):
            #~ try:
                #~ logger.debug("[%s] - stop GPIO Output component on pin %s", self.__class__.__name__, self.values["pin"].instances[config]['data'])
                #~ self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.OUT)
            #~ except:
                #~ logger.exception("[%s] - Exception when stopping GPIO component", self.__class__.__name__)
        GpioComponent.stop(self)
        return True

    def set_switch(self, node_uuid, index, data):
        """Switch On/Off the led
        """
        logger.warning("[%s] - set_switch unknown data : %s", self.__class__.__name__, data)

    def set_color(self, node_uuid, index, data):
        """Set the color
        """
        logger.warning("[%s] - set_color unknown data : %s", self.__class__.__name__, data)

class LedComponent(GpioComponent):
    """ A simple LED component for GPIO """

    def __init__(self, **kwargs):
        """
        """
        self._inputs = {}
        oid = kwargs.pop('oid', 'rpigpio.rgb')
        product_name = kwargs.pop('product_name', "RGB PWM")
        name = kwargs.pop('name', "RGB PWM")
        GpioComponent.__init__(self, oid=oid, name=name, product_name=product_name, **kwargs)

        uuid="switch"
        self.values[uuid] = self.value_factory['action_switch_binary'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            list_items=['on', 'off'],
            default='off',
            set_data_cb=self.set_switch,
            genre=0x01,
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.values[poll_value.uuid] = poll_value

        uuid="blink"
        self.values[uuid] = self.value_factory['blink'](options=self.options, uuid=uuid,
            node_uuid=self.uuid,
            blink_off_cb=self.blink_off_cb,
            blink_on_cb=self.blink_on_cb,
        )
        poll_value = self.values[uuid].create_poll_value(default=300)
        self.values[poll_value.uuid] = poll_value

    def blink_off_cb(self, node_uuid=None, index=None):
        """Callback for blink"""
        pass

    def blink_on_cb(self, node_uuid=None, index=None):
        """Callback for blink"""
        pass

    def start(self, mqttc):
        """Start the component.
        """
        GpioComponent.start(self, mqttc)
        configs = len(self.values["pin"].get_index_configs())
        for config in range(configs):
            try:
                logger.debug("[%s] - start GPIO Output component on pin %s", self.__class__.__name__, self.values["pin"].instances[config]['data'])
                self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.OUT)
            except:
                logger.exception("[%s] - Exception when starting GPIO component", self.__class__.__name__)
        return True

    def stop(self):
        """Stop the component.

        """
        logger.debug("[%s] - stop GPIO Output component", self.__class__.__name__)
        configs = len(self.values["pin"].get_index_configs())
        for config in range(configs):
            try:
                logger.debug("[%s] - stop GPIO Output component on pin %s", self.__class__.__name__, self.values["pin"].instances[config]['data'])
                self._bus.gpio.setup(self.values["pin"].instances[config]['data'], GPIO.OUT)
            except:
                logger.exception("[%s] - Exception when stopping GPIO component", self.__class__.__name__)
        GpioComponent.stop(self)
        return True

    def set_switch(self, node_uuid, index, data):
        """Switch On/Off the led
        """
        logger.warning("[%s] - set_switch unknown data : %s", self.__class__.__name__, data)

