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

class NI_X_FastCounter(Base, FastCounterInterface):
    """ Hardware class to use an NI X-series DAQ card as a fast gated counter.

    Example config for copy-paste:

    TODO: add example config

    """

    def get_constraints(self):
        raise NotImplementedError

    def configure(self, bin_width_s, record_length_s, number_of_gates=0):
        raise NotImplementedError

    def get_status(self):
        raise NotImplementedError

    def start_measure(self):
        raise NotImplementedError

    def measure_for(self,duration):
        """ Duration is in seconds"""
        raise NotImplementedError

    def stop_measure(self):
        raise NotImplementedError

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
