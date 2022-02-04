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

    _visa_address = ConfigOption(name='awg_visa_address', default='TCPIP0::129.10.128.231::INSTR', missing='error')

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

    def on_deactivate(self):
        try:
            self.awg.close()
        except BaseException as err:
            print("Unexpected error {}, {}".format(err, type(err)))
            self.log.error('Closing SDG6022X AWG connection using pyvisa failed.')
        self.log.info('Closed connection to SDG6022X AWG')

    def get_constraints(self):
        constraints = PulserConstraints()

        # 1 μHz to 50 MHz in DDS mode (from manual, p.11)
        # 1 μSa/s to 300 MSa/s in TrueArb mode (from manual, p.11)
        constraints.sample_rate.min = 1e-6  # Hz
        constraints.sample_rate.max = 300e6 # Hz
        constraints.a_ch_amplitude.default = 300e6
        # TODO: why does it seem the minimum is 2 mHz in TrueArb mode?
        # TODO: what should be the step size?

        # 20 Vpp maximum output (from manual)
        constraints.a_ch_amplitude.min = 0      # Vpp
        constraints.a_ch_amplitude.max = 20     # Vpp
        constraints.a_ch_amplitude.step = 0.001 # Vpp
        constraints.a_ch_amplitude.default = 4  # Vpp

        """
        The offset setting range is limited by the "Load" and "Amplitude/HighLevel"
        settings.
        """
        # 10 V maximum offset magnitude (from manual testing)
        constraints.a_ch_offset.min = -10 # V
        constraints.a_ch_offset.min =  10 # V
        # Note that the total voltage is limited to 20 V, apparently:
        # 2*offset + amplitude < 20 V

        # TODO: Does this even have digital outputs?
        # If not, what should the constraints for these be set to?

        """
        TrueArb output mode allows creation of arbitrary
        waveforms that contain from 2 to 20 Mpts.
        p.37
        """
        # In practice, 20e6 samples will crash the AWG.
        # TODO: what is the practical limit?
        constraints.waveform_length.min = 2
        constraints.waveform_length.max = 20e6
        constraints.waveform_length.step = 1

        # No sequence mode, only waveforms can be used
        constraints.sequence_option = SequenceOption.NON

    def get_status(self):
        """ Retrieves the status of the pulsing hardware

        @return (int, dict): tuple with an integer value of the current status and a corresponding
                             dictionary containing status description for all the possible status
                             variables of the pulse generator hardware.
        """
        current_status = instr.last_status.value

        status_dic = {0: 'success'}
        # TODO: add Status Codes:
        # https://pyvisa.readthedocs.io/en/latest/api/constants.html?highlight=status%20code%20success#pyvisa.constants.StatusCode
        # https://pyvisa.readthedocs.io/en/latest/_modules/pyvisa/constants.html#StatusCode

        return current_status, status_dic

    def get_sample_rate(self, channel=None):
        """ Get the sample rate of the pulse generator hardware

        @return float: The current sample rate of the device (in Hz)

        Do not return a saved sample rate from an attribute, but instead retrieve the current
        sample rate directly from the device.
        """
        if channel is None:
            # If channel is unspecified, use channel #1.
            # TODO: is there a better way?
            response = self.query("C1:SRATE?")
        elif channel == 1:
            response = self.query("C1:SRATE?")
        elif channel == 2:
            response = self.query("C2:SRATE?")
        else:
            self.log.error("Invalid channel number: {}".format(channel))
            return None
        channel_label, sample_rate_info = response.split(':')
        mode, truearb_or_dds, value_name, value, inter, hold = sample_rate_info.split(',')
        def remove_suffix(input):
            if input.endswith("Sa/s"):
                return input[0:-len("Sa/s")]
        value_number = remove_suffix(value)
        sample_rate = float(value_number)
        return sample_rate

    def set_sample_rate(self, sample_rate, channel=None):
        """ Set the sample rate of the pulse generator hardware.

        @param float sample_rate: The sampling rate to be set (in Hz)

        @return float: the sample rate returned from the device (in Hz).

        Note: After setting the sampling rate of the device, use the actually set return value for
              further processing.
        """
        if channel is None:
            self.write('C1:SRATE VALUE,{}'.format(sample_rate))
            self.write('C2:SRATE VALUE,{}'.format(sample_rate))
        elif channel == 1:
            self.write('C1:SRATE VALUE,{}'.format(sample_rate))
        elif channel === 2:
            self.write('C2:SRATE VALUE,{}'.format(sample_rate))
        else:
            self.log.error("Invalid channel number: {}".format(channel))
        return self.get_sample_rate()

    def query(self, question):
        """ Asks the device a 'question' and returns an answer from it.

        @param string question: string containing the command

        @return string: the answer of the device to the 'question' in a string
        """
        answer = self.awg.query(question)
        return answer


    def reset(self):
        """ Reset the device.

        @return int: error code (0:OK, -1:error)
        """
        self.write('*RST')
        # TODO: is this necessary?
        # self.write('*WAI')
        # https://na.support.keysight.com/vna/help/latest/Programming/Learning_about_GPIB/Understanding_Command_Synchronization.htm#wai

        return 0