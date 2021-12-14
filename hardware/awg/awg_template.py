# -*- coding: utf-8 -*-

"""
This file contains the Qudi hardware module for the <INSERT MODEL NAME HERE>.

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

# See interface/pulser_interface.py

class INSERT_MODEL_NAME_HERE(Base, PulserInterface):
    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)

    def on_activate(self):
        return

    def on_deactivate(self):
        return

    def get_constraints(self):
        # Used by sequence_generator_logic.py : on_activate() : _read_settings_from_device()
        constraints = PulserConstraints()
        return constraints

    def pulser_on(self):
        # Used by pulsed_measurement_logic.py : pulse_generator_on()
        return self.get_status()[0]

    def pulser_off(self):
        # Used by pulsed_measurement_logic.py : pulse_generator_off()
        return self.get_status()[0]

    def load_waveform(self, load_dict):
        return self.get_loaded_assets()[0]

    def load_sequence(self, sequence_name):
        return 0

    def get_loaded_assets(self):
        loaded_assets = {}
        type_per_ch = []
        type_per_ch.append('waveform')
        return loaded_assets, type_per_ch[0]

    def clear_all(self):
        return

    def get_status(self):
        status_dic = {-1: 'TEMPLATE DESCRIBE THIS',
                       0: 'TEMPLATE DESCRIBE THIS',
                       1: 'TEMPLATE DESCRIBE THIS'}
        current_status = -1
        return current_status, status_dic

    def get_sample_rate(self):
        # Used by sequence_generator_logic.py : on_activate() : _read_settings_from_device()
        return 0.0 # Hz

    def set_sample_rate(self, sample_rate):
        return self.get_sample_rate()

    def get_analog_level(self, amplitude=None, offset=None):
        # Used by sequence_generator_logic.py : on_activate() : _read_settings_from_device()
        amp = {}
        off = {}
        return amp, off

    def set_analog_level(self, amplitude=None, offset=None):
        return self.get_analog_level()

    def get_digital_level(self, low=None, high=None):
        # Used by sequence_generator_logic.py : on_activate() : _read_settings_from_device()
        low_val = {}
        high_val = {}
        return low_val, high_val

    def set_digital_level(self, low=None, high=None):
        return self.get_digital_level()

    def get_active_channels(self, ch=None):
        active_ch = {}
        return active_ch

    def set_active_channels(self, ch=None):
        ch = {}
        return self.get_active_channels(ch=list(ch))

    def write_waveform(self, name, analog_samples, digital_samples, is_first_chunk, is_last_chunk,
                       total_number_of_samples):
        waveforms = []
        return total_number_of_samples, waveforms

    def write_sequence(self, name, sequence_parameters):
        return -1

    def get_waveform_names(self):
        names = []
        return names

    def get_sequence_names(self):
        sequence_list = []
        return sequence_list

    def delete_waveform(self, waveform_name):
        deleted_waveforms = []
        return list(set(deleted_waveforms))

    def delete_sequence(self, sequence_name):
        deleted_sequences = []
        return deleted_sequences

    def get_interleave(self):
        # Used by sequence_generator_logic.py : on_activate() : _read_settings_from_device()
        return False

    def set_interleave(self, state=False):
        return self.get_interleave()

    def reset(self):
        return 0
