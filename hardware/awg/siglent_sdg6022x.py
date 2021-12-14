# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware module for the SIGLENT SDG6022X.

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

import visa

from core.module import Base
from core.configoption import ConfigOption
from interface.pulser_interface import PulserInterface, PulserConstraints, SequenceOption

class SDG6022X(Base, PulserInterface):
    """
    A hardware module for the SIGLENT SDG6022X for generating waveforms and
    sequences thereof.

    Based on keysight_m819x.py

    Nathaniel Beaver, 2021-12-13
    """

    _visa_address = ConfigOption(name='awg_visa_address', default='TCPIP0::129.10.130.63::INSTR', missing='error')

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        """When the module is activated, connect to the AWG.
        """
        # connect to AWG using PyVISA
        self._rm = visa.ResourceManager()
        try:
            self.awg = self._rm.open_resource(self._visa_address)
            # set timeout by default to 30 sec
            self.awg.timeout = self._awg_timeout * 1000
        except VisaIOError as err:
            self.awg = None
            self.log.error('VISA address "{0}" not found by the pyVISA resource manager.\n'
                           'Check the IP  address on the AWG with Utility -> Interface -> LAN Setup\n'
                           'or try connecting with EasyWaveX'
                           ''.format(self._visa_address))
            return err
        except BaseException as err:
            self.awg = None
            print("Unexpected error {}, {}".format(err, type(err) ) )
            return err

        # SCPI command return the company name, model number, serial
        # number, and firmware version number.
        # IDN = "identify"
        instrument_identifier = self.query('*IDN?')
        self._BRAND, self._MODEL, self._SERIALNUMBER, self._FIRMWARE_VERSION = instrument_identifier.split(',')

    def on_deactive(selfs):
        try:
            self.awg.close()
        except BaseException as err:
            print("Unexpected error {}, {}".format(err, type(err)))
            self.log.error('Closing SDG6022X AWG connection using pyvisa failed.')
        self.log.info('Closed connection to SDG6022X AWG')

    def query(self, question):
        """ Asks the device a 'question' and returns an answer from it.

        @param string question: string containing the command

        @return string: the answer of the device to the 'question' in a string
        """
        answer = self.awg.query(question)
        return answer