# -*- coding: utf-8 -*-
"""
A hardware module for using an NI X-series DAQ card (e.g. PXIe-6738) as a fast gated counter.

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

from interface.fast_counter_interface import FastCounterInterface
from core.module import Base
from core.configoption import ConfigOption
import time # for time.sleep()
from .national_instruments_x_series import NationalInstrumentsXSeries

class NI_X_FastCounter(Base, FastCounterInterface):
    """ Hardware class to use an NI X-series DAQ card as a fast gated counter.

    Example config for copy-paste:

    fastcounter_ni_x_series:
        module.Class: 'ni_x_series_gated_fast_counter.NI_X_FastCounter'
        photon_source: /Dev1/PFI1
        signal_counter_channel: /Dev1/Ctr1
        reference_counter_channel: /Dev1/Ctr0
        signal_gating_source: /Dev1/PFI0
        reference_gating_source: /Dev1/PFI2
        timeout: -1 # wait indefinitely

    TODO: make reference counter optional

    """
    _photon_source = ConfigOption('photon_source', missing='error')
    _signal_counter_channel = ConfigOption('signal_counter_channel', missing='error')
    _reference_counter_channel = ConfigOption('reference_counter_channel', missing='error')
    _signal_gating_source = ConfigOption('signal_gating_source', missing='error')
    _reference_gating_source = ConfigOption('reference_gating_source', missing='error')
    _timeout = ConfigOption('timeout', missing='error')

    def get_constraints(self):
        raise NotImplementedError

    def configure(self, bin_width_s, record_length_s, number_of_gates=2):
        # TODO: what should be done with bin_width_s and record_length_s?
        if number_of_gates != 2:
            raise ValueError("number_of_gates != 2: {}".format(number_of_gates))

        config_signal = NationalInstrumentsXSeries.GatedCounterConfig()
        config_signal.counter_channel = self._signal_counter_channel
        config_signal.photon_source = self._photon_source
        config_signal.gating_source = self._signal_gating_source
        config_signal.channel_name = ''
        config_signal.task_name = "SignalCounter"
        config_signal.timeout = self._timeout

        config_reference = self.GatedCounterConfig()
        config_reference.counter_channel = self._reference_counter_channel
        config_reference.photon_source = self._photon_source
        config_reference.gating_source = self._reference_gating_source
        config_reference.channel_name = ''
        config_reference.task_name = "ReferenceCounter"
        config_reference.timeout = self._timeout

        self.counter_signal = NationalInstrumentsXSeries.GatedCounter(config_signal)
        self.counter_reference = NationalInstrumentsXSeries.GatedCounter(config_reference)

    def get_status(self):
        raise NotImplementedError

    def start_measure(self):
        raise NotImplementedError

    def measure_for(self, duration):
        """ Duration is in seconds"""
        self.counter_signal.start()
        self.counter_reference.start()
        time.sleep(duration)
        signal = self.counter_signal.get_gated_count()
        reference = self.counter_reference.get_gated_count()
        self.counter_signal.stop()
        self.counter_reference.stop()
        self.counter_signal.clear()
        self.counter_reference.clear()
        return signal, reference

    def stop_measure(self):
        self.counter_signal.stop()
        self.counter_reference.stop()
        self.counter_signal.clear()
        self.counter_reference.clear()

    def pause_measure(self):
        raise NotImplementedError

    def continue_measure(self):
        raise NotImplementedError

    def is_gated(self):
        raise NotImplementedError

    def get_binwidth(self):
        raise NotImplementedError

    def get_data_trace(self):
        raise NotImplementedError
