#-*- coding: utf-8 -*-
"""
Laser management.

Qudi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

Qudi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with Qudi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import time
import numpy as np
from qtpy import QtCore

from core.connector import Connector
from core.configoption import ConfigOption
from logic.generic_logic import GenericLogic
from interface.simple_laser_interface import ControlMode, ShutterState, LaserState
from core.statusvariable import StatusVar

class LaserLogic(GenericLogic):
    """ Logic module agreggating multiple hardware switches.
    """

    # waiting time between queries in milliseconds
    laser = Connector(interface='SimpleLaserInterface')
    queryInterval = ConfigOption('query_interval', 200)

    # Need this to set the amplitude.
    scanner = Connector(interface='ConfocalScannerInterface')

    # Need this to signal the change in the amplitude.
    scannerlogic = Connector(interface='ConfocalLogic')

    # Default to 0 V, which is zero attenuation.
    # https://www.thorlabs.com/drawings/51658de5420550b4-9D54B893-9B1F-818E-91293FC428044917/V450A-SpecSheet.pdf
    voa_voltage = StatusVar(name='voa_voltage', default=3.5)
    #voa_voltage = ConfigOption('voa_voltage', 3.5)
    # TODO: why doesn't this value survive closing and re-opening?

    sigUpdate = QtCore.Signal()

    def on_activate(self):
        """ Prepare logic module for work.
        """
        self._laser = self.laser()
        self.stopRequest = False
        self.bufferLength = 100
        self.data = {}

        # set up VOA
        self._scanning_logic = self.scannerlogic()
        self._scanning_device = self.scanner()
        self.voa_voltage_range = self._scanning_device.get_position_range()[3] # TODO: check the bounds on this

        # delay timer for querying laser
        self.queryTimer = QtCore.QTimer()
        self.queryTimer.setInterval(self.queryInterval)
        self.queryTimer.setSingleShot(True)
        self.queryTimer.timeout.connect(self.check_laser_loop, QtCore.Qt.QueuedConnection)

        # get laser capabilities
        self.laser_state = self._laser.get_laser_state()
        self.laser_shutter = self._laser.get_shutter_state()
        self.laser_can_turn_on = self.laser_state.value <= LaserState.ON.value
        self.laser_current_unit = self._laser.get_current_unit()
        self.laser_power_range = self._laser.get_power_range()
        self.laser_current_range = self._laser.get_current_range()
        self.laser_power_setpoint = self._laser.get_power_setpoint()
        self.laser_current_setpoint = self._laser.get_current_setpoint()
        self.laser_extra = self._laser.get_extra_info()
        self.laser_can_power = ControlMode.POWER in self._laser.allowed_control_modes()
        self.laser_can_current = ControlMode.CURRENT in self._laser.allowed_control_modes()
        if ControlMode.MIXED in self._laser.allowed_control_modes():
            self.laser_can_power = True
            self.laser_can_current = True

        self.has_shutter = self._laser.get_shutter_state() != ShutterState.NOSHUTTER
        self.init_data_logging()
        self.start_query_loop()

    def on_deactivate(self):
        """ Deactivate module.
        """
        self.stop_query_loop()
        for i in range(5):
            time.sleep(self.queryInterval / 1000)
            QtCore.QCoreApplication.processEvents()

    @QtCore.Slot()
    def check_laser_loop(self):
        """ Get power, current, shutter state and temperatures from laser. """
        if self.stopRequest:
            if self.module_state.can('stop'):
                self.module_state.stop()
            self.stopRequest = False
            return
        qi = self.queryInterval
        try:
            #print('laserloop', QtCore.QThread.currentThreadId())
            self.laser_state = self._laser.get_laser_state()
            self.laser_shutter = self._laser.get_shutter_state()
            self.laser_power = self._laser.get_power()
            self.laser_power_setpoint = self._laser.get_power_setpoint()
            self.laser_current = self._laser.get_current()
            self.laser_current_setpoint = self._laser.get_current_setpoint()
            self.laser_temps = self._laser.get_temperatures()
            self.voa_voltage = self.get_voa_voltage()

            for k in self.data:
                self.data[k] = np.roll(self.data[k], -1)

            self.data['power'][-1] = self.laser_power
            self.data['current'][-1] = self.laser_current
            self.data['time'][-1] = time.time()

            for k, v in self.laser_temps.items():
                self.data[k][-1] = v
        except ValueError as e:
            import traceback
            self.log.warning(traceback.format_exc())
            # Traceback is better than just self.log.warning(e)
            qi = 3000
            self.log.warning("ValueError in laser status loop, throttling refresh rate.")
        except pyvisa.errors.VisaIOError as e:
            import traceback
            self.log.warning(traceback.format_exc())
            qi = 3000
            self.log.warning("pyvisa.errors.VisaIOError in laser status loop, throttling refresh rate.")

        self.queryTimer.start(qi)
        self.sigUpdate.emit()

    @QtCore.Slot()
    def start_query_loop(self):
        """ Start the readout loop. """
        self.module_state.run()
        self.queryTimer.start(self.queryInterval)

    @QtCore.Slot()
    def stop_query_loop(self):
        """ Stop the readout loop. """
        self.stopRequest = True
        for i in range(10):
            if not self.stopRequest:
                return
            QtCore.QCoreApplication.processEvents()
            time.sleep(self.queryInterval/1000)

    def init_data_logging(self):
        """ Zero all log buffers. """
        self.data['current'] = np.zeros(self.bufferLength)
        self.data['power'] = np.zeros(self.bufferLength)
        self.data['time'] = np.ones(self.bufferLength) * time.time()
        temps = self._laser.get_temperatures()
        for name in temps:
            self.data[name] = np.zeros(self.bufferLength)

    @QtCore.Slot(ControlMode)
    def set_control_mode(self, mode):
        """ Change whether the laser is controlled by diode current or output power. """
        #print('set_control_mode', QtCore.QThread.currentThreadId())
        if mode in self._laser.allowed_control_modes():
            ctrl_mode = ControlMode.MIXED
            if mode == ControlMode.POWER:
                self.laser_power = self._laser.get_power()
                self._laser.set_power(self.laser_power)
                ctrl_mode = self._laser.set_control_mode(mode)
            elif mode == ControlMode.CURRENT:
                self.laser_current = self._laser.get_current()
                self._laser.set_current(self.laser_current)
                ctrl_mode = self._laser.set_control_mode(mode)
            self.log.info('Changed control mode to {0}'.format(ctrl_mode))

    @QtCore.Slot(bool)
    def set_laser_state(self, state):
        """ Turn laser on or off. """
        if state and self.laser_state == LaserState.OFF:
            self._laser.on()
        if not state and self.laser_state == LaserState.ON:
            self._laser.off()
        self.sigUpdate.emit()

    @QtCore.Slot(bool)
    def set_shutter_state(self, state):
        """ Open or close the laser output shutter. """
        if state and self.laser_shutter == ShutterState.CLOSED:
            self._laser.set_shutter_state(ShutterState.OPEN)
        if not state and self.laser_shutter == ShutterState.OPEN:
            self._laser.set_shutter_state(ShutterState.CLOSED)

    @QtCore.Slot(float)
    def set_power(self, power):
        """ Set laser output power. """
        self._laser.set_power(power)

    @QtCore.Slot(float)
    def set_voa_voltage(self, voltage):
        """ Set voltage of variable optical attenuator (VOA). """
        self._scanning_device.scanner_set_position(a=voltage)
        self.voa_voltage = voltage
        # Send the signal so e.g. confocal can update its value.
        self.log.debug("set_voa_voltage(): emitting new voltage to signal_voa_voltage_changed: '{}".format(voltage))
        self._scanning_logic.signal_voa_voltage_changed.emit(self.voa_voltage)

    def get_voa_voltage(self):
        """ Get voltage of variable optical attenuator (VOA). """
        x,y,z,a = self._scanning_device.get_scanner_position()
        return a

    @QtCore.Slot(float)
    def set_current(self, current):
        """ Set laser diode current. """
        self._laser.set_current(current)

