# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware file to control Anritsu Microwave Device.

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

Parts of this file were developed from a PI3diamond module which is
Copyright (C) 2009 Helmut Rathgen <helmut.rathgen@gmail.com>

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

import visa
import time
import numpy as np

from core.base import Base
from interface.microwave_interface import MicrowaveInterface
from interface.microwave_interface import MicrowaveLimits
from interface.microwave_interface import MicrowaveMode
from interface.microwave_interface import TriggerEdge


class MicrowaveAnritsu(Base, MicrowaveInterface):
    """  Hardware controlfile for Anritsu Devices.  """

    _modclass = 'MicrowaveAnritsu'
    _modtype = 'hardware'

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """

        # checking for the right configuration
        config = self.getConfiguration()
        if 'gpib_address' in config.keys():
            self._gpib_address = config['gpib_address']
        else:
            self.log.error('This is MWanritsu: did not find >>gpib_address<< '
                    'in configration.')

        if 'gpib_timeout' in config.keys():
            self._gpib_timeout = int(config['gpib_timeout'])
        else:
            self._gpib_timeout = 10
            self.log.error('This is MWanritsu: did not find >>gpib_timeout<< '
                    'in configration. I will set it to 10 seconds.')

        # trying to load the visa connection to the module
        self.rm = visa.ResourceManager()
        try:
            self._gpib_connection = self.rm.open_resource(self._gpib_address,
                                                          timeout=self._gpib_timeout*1000)
        except:
            self.log.error('This is MWanritsu: could not connect to the GPIB '
                        'address >>{}<<.'.format(self._gpib_address))
            raise
        self.model = self._gpib_connection.query('*IDN?').split(',')[1]
        self.log.info('MicrowaveAnritsu initialised and connected to '
                'hardware.')

    def on_deactivate(self):
        """ Deinitialisation performed during deactivation of the module.
        """
        self._gpib_connection.close()
        self.rm.close()

    def get_limits(self):
        """ Right now, this is for Anritsu MG37022A with Option 4 only."""
        limits = MicrowaveLimits()
        limits.supported_modes = (MicrowaveMode.CW, MicrowaveMode.LIST, MicrowaveMode.SWEEP)

        limits.min_frequency = 10e6
        limits.max_frequency = 20e9

        limits.min_power = -105
        limits.max_power = 30

        limits.list_minstep = 0.001
        limits.list_maxstep = 20e9
        limits.list_maxentries = 10001

        limits.sweep_minstep = 0.001
        limits.sweep_maxstep = 20e9
        limits.sweep_maxentries = 10001
        return limits

    def off(self):
        """ 
        Switches off any microwave output.
        Must return AFTER the device is actually stopped.

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write('OUTP:STAT OFF')
        while int(float(self._gpib_connection.query('OUTP:STAT?'))) != 0:
            time.sleep(0.2)
        return 0

    def get_status(self):
        """ 
        Gets the current status of the MW source, i.e. the mode (cw, list or sweep) and 
        the output state (stopped, running)

        @return str, bool: mode ['cw', 'list', 'sweep'], is_running [True, False] 
        """
        is_running = bool(int(float(self._gpib_connection.query('OUTP:STAT?'))))
        mode = self._gpib_connection.query(':FREQ:MODE?').strip('\n').lower()
        return mode, is_running

    def get_power(self):
        """ 
        Gets the microwave output power. 

        @return float: the power set at the device in dBm
        """
        return float(self._gpib_connection.query(':POW?'))

    def set_power(self, power):
        """ 
        Sets the microwave output power. 
        
        @param float power: The power to set in dBm

        @return float: the power set at the device in dBm
        """
        self._gpib_connection.write(':POW {0:f}'.format(power))
        while int(float(self._gpib_connection.query('*OPC?'))) != 1:
            time.sleep(0.2)
        return self.get_power()

    def get_frequency(self):
        """ 
        Gets the frequency of the microwave output.
        Returns single float value if the device is in cw mode. 
        Returns list like [start, stop, step] if the device is in sweep mode.
        Returns list of frequencies if the device is in list mode.

        @return [float, list]: frequency(s) currently set for this device in Hz
        """
        mode, is_running = self.get_status()
        if 'cw' in mode:
            return_val = float(self._gpib_connection.query(':FREQ?'))
        elif 'sweep' in mode:
            start = float(self._gpib_connection.query(':FREQ:STAR?'))
            stop = float(self._gpib_connection.query(':FREQ:STOP?'))
            step = float(self._gpib_connection.query(':SWE:FREQ:STEP?'))
            return_val = [start, stop, step]
        elif 'list' in mode:
            stop_index = int(float(self._gpib_connection.query(':LIST:STOP?')))
            self._gpib_connection.write(':LIST:IND {0:d}'.format(stop_index))
            stop = float(self._gpib_connection.query(':LIST:FREQ?'))
            self._gpib_connection.write(':LIST:IND 0')
            start = float(self._gpib_connection.query(':LIST:FREQ?'))
            step = (stop - start) / stop_index
            return_val = np.arange(start, stop + step, step)
        return return_val

    def cw_on(self):
        """ Switches on any preconfigured microwave output.

        @return int: error code (0:OK, -1:error)
        """

        self._gpib_connection.write(':OUTP:STAT ON')
        dummy, is_running = self.get_status()
        while not is_running:
            time.sleep(0.2)
            dummy, is_running = self.get_status()
        return 0

    def set_cw(self, frequency=None, power=None):
        """ 
        Configures the device for cw-mode and optionally sets frequency and/or power

        @param float frequency: frequency to set in Hz
        @param float power: power to set in dBm
        @param bool useinterleave: If this mode exists you can choose it.

        @return float, float, str: current frequency in Hz, current power in dBm, current mode

        Interleave option is used for arbitrary waveform generator devices.
        """
        self._gpib_connection.write(':FREQ:MODE CW')
        self._gpib_connection.write('*WAI')

        if frequency is not None:
            self._gpib_connection.write(':FREQ {0:f}'.format(frequency))
            self._gpib_connection.write('*OPC')
            while int(float(self._gpib_connection.query('*OPC?'))) != 1:
                time.sleep(0.2)

        if power is not None:
            actual_power = self.set_power(power)

        mode, dummy = self.get_status()
        actual_freq = self.get_frequency()
        return actual_freq, actual_power, mode

    def list_on(self):
        """
        Switches on the list mode microwave output.
        Must return AFTER the device is actually running.

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write(':FREQ:MODE LIST')
        self._gpib_connection.write('*WAI')
        self._gpib_connection.write(':OUTP:STAT ON')
        dummy, is_running = self.get_status()
        while not is_running:
            time.sleep(0.2)
            dummy, is_running = self.get_status()
        return 0

    def set_list(self, frequency=None, power=None):
        """ 
        Configures the device for list-mode and optionally sets frequencies and/or power

        @param list frequency: list of frequencies in Hz
        @param float power: MW power of the frequency list in dBm

        @return list, float, str: current frequencies in Hz, current power in dBm, current mode
        """
        self._gpib_connection.write(':FREQ:MODE LIST')
        self._gpib_connection.write('*WAI')
        self._gpib_connection.write(':LIST:TYPE FREQ')
        self._gpib_connection.write(':LIST:IND 0')

        if frequency is not None:
            s = ' {0:f},'.format(frequency[0])
            for f in frequency[:-1]:
                s += ' {0:f},'.format(f)
            s += ' {0:f}'.format(frequency[-1])
            self._gpib_connection.write(':LIST:FREQ' + s)
            self._gpib_connection.write(':LIST:STAR 0')
            self._gpib_connection.write(':LIST:STOP {0:d}'.format(len(frequency)))
            self._gpib_connection.write(':LIST:MODE MAN')
            self._gpib_connection.write(':LIST:IND 0')
            self._gpib_connection.write('*WAI')
        actual_freq = self.get_frequency()

        if power is not None:
            actual_power = self.set_power(power)
        else:
            actual_power = self.get_power()

        mode, dummy = self.get_status()
        return actual_freq, actual_power, mode

    def reset_listpos(self):
        """ 
        Reset of MW list mode position to start (first frequency step)

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write(':LIST:IND 0')
        self._gpib_connection.write('*WAI')
        return 0

    def sweep_on(self):
        """ Switches on the sweep mode.

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write(':FREQ:MODE SWEEP')
        self._gpib_connection.write('*WAI')
        self._gpib_connection.write(':OUTP:STAT ON')
        dummy, is_running = self.get_status()
        while not is_running:
            time.sleep(0.2)
            dummy, is_running = self.get_status()
        return 0

    def set_sweep(self, start=None, stop=None, step=None, power=None):
        """ 
        Configures the device for sweep-mode and optionally sets frequency start/stop/step 
        and/or power
        
        @return float, float, float, float, str: current start frequency in Hz, 
                                                 current stop frequency in Hz,
                                                 current frequency step in Hz,
                                                 current power in dBm, 
                                                 current mode
        """
        self._gpib_connection.write(':FREQ:MODE SWEEP')
        self._gpib_connection.write('*WAI')
        self._gpib_connection.write(':SWE:GEN STEP')
        self._gpib_connection.write('*WAI')

        if start is not None and stop is not None and step is not None:
            self._gpib_connection.write(':FREQ:START {0}'.format(start - step))
            self._gpib_connection.write(':FREQ:STOP {0}'.format(stop))
            self._gpib_connection.write(':SWE:FREQ:STEP {0}'.format(step))
            self._gpib_connection.write('*WAI')

        if power is not None:
            actual_power = self.set_power(power)
        else:
            actual_power = self.get_power()

        freq_list = self.get_frequency()
        mode, dummy = self.get_status()
        return freq_list[0], freq_list[1], freq_list[2], actual_power, mode

    def reset_sweeppos(self):
        """ 
        Reset of MW sweep mode position to start (start frequency)

        @return int: error code (0:OK, -1:error)
        """
        self._gpib_connection.write(':ABORT')
        self._gpib_connection.write('*WAI')
        return 0

    def set_ext_trigger(self, pol=TriggerEdge.RISING):
        """ Set the external trigger for this device with proper polarization.

        @param TriggerEdge pol: polarisation of the trigger (basically rising edge or falling edge)

        @return object: current trigger polarity [TriggerEdge.RISING, TriggerEdge.FALLING]
        """
        if pol == TriggerEdge.RISING:
            edge = 'POS'
        elif pol == TriggerEdge.FALLING:
            edge = 'NEG'
        else:
            self.log.warning('No valid trigger polarity passed to microwave hardware module.')
            edge = None

        if edge is not None:
            self._gpib_connection.write(':TRIG:SOUR EXT')
            self._gpib_connection.write(':TRIG:SLOP {0}'.format(edge))
            self._gpib_connection.write('*WAI')

        polarity = self._gpib_connection.query(':TRIG:SEQ3:SLOPE?')
        if 'NEG' in polarity:
            return TriggerEdge.FALLING
        else:
            return TriggerEdge.RISING
