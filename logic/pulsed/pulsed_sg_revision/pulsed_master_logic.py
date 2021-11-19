# -*- coding: utf-8 -*-
"""
Master logic to combine sequence_generator_logic and pulsed_measurement_logic to be
used with a single GUI.

QuDi is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

QuDi is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with QuDi. If not, see <http://www.gnu.org/licenses/>.

Copyright (c) the Qudi Developers. See the COPYRIGHT.txt file at the
top-level directory of this distribution and at <https://github.com/Ulm-IQO/qudi/>
"""

from core.connector import Connector
from logic.generic_logic import GenericLogic
from qtpy import QtCore
import numpy as np
from core.configoption import ConfigOption
from core.statusvariable import StatusVar


class PulsedMasterLogic(GenericLogic):
    """
    This logic module combines the functionality of two modules.

    It can be used to generate pulse sequences/waveforms and to control the settings for the pulse
    generator via SequenceGeneratorLogic. Essentially this part controls what is played on the
    pulse generator.
    Furthermore it can be used to set up a pulsed measurement with an already set-up pulse generator
    together with a fast counting device via PulsedMeasurementLogic.

    The main purpose for this module is to provide a single interface while maintaining a modular
    structure for complex pulsed measurements. Each of the sub-modules can be used without this
    module but more care has to be taken in that case.
    Automatic transfer of information from one sub-module to the other for convenience is also
    handled here.
    Another important aspect is the use of this module in scripts (e.g. jupyter notebooks).
    All calls to sub-module setter functions (PulsedMeasurementLogic and SequenceGeneratorLogic)
    are decoupled from the calling thread via Qt queued connections.
    This ensures a more intuitive and less error prone use of scripting.
    """

    # Declare connectors
    pulsegenerator = Connector(interface='PulserInterface')
    odmrlogic1 = Connector(interface='ODMRLogic')

    do_channel_states = StatusVar('DO channel states', [0,0,0,0,0,0,0,0])

    # Define signals
    sigParameterUpdated = QtCore.Signal(dict)
    sigMWStatusChanged = QtCore.Signal(bool)


    def __init__(self, config, **kwargs):
        """ Create PulsedMasterLogic object with connectors.

          @param dict kwargs: optional parameters
        """
        super().__init__(config=config, **kwargs)

        # Dictionary servings as status register
        self.status_dict = dict()
        self.streamer_status = dict()
        return

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """

        # check status of connected instruments
        self.uw_frequency = self.odmrlogic1().cw_mw_frequency
        self.uw_power = self.odmrlogic1().cw_mw_power

        self.odmrlogic1().sigParameterUpdated.connect(self.update_from_odmr,
                                                 QtCore.Qt.QueuedConnection)
        self.odmrlogic1().sigOutputStateUpdated.connect(self.update_mw_status,
                                                       QtCore.Qt.QueuedConnection)

        return

    def on_deactivate(self):
        """

        @return:
        """
        # Disconnect all signals
        # Disconnect signals controlling PulsedMeasurementLogic

        return

    #######################################################################
    ###             Pulsed properties                       ###
    #######################################################################


    #######################################################################
    ###             Pulsed methods                          ###
    #######################################################################
    def get_streamer_status(self):
        self.streamer_status = self.pulsegenerator().get_connection_status()
        return self.streamer_status

    def set_continuous(self, channel_states=None):
        if channel_states is not None:
            self.do_channel_states = channel_states
        self.pulsegenerator().set_continuous_out(self.do_channel_states)
        update_dict = {'DO_states': self.do_channel_states}
        self.sigParameterUpdated.emit(update_dict)
        return

    #######################################################################
    ###             Microwave methods                          ###
    #######################################################################

    def update_from_odmr(self,dict_to_pass):
        self.sigParameterUpdated.emit(dict_to_pass)
        return

    def update_mw_status(self, _, is_running):
        if is_running:
            self.sigMWStatusChanged.emit(True)
        else:
            self.sigMWStatusChanged.emit(False)
        return

    def mw_cw_on(self):
        self.odmrlogic1().mw_cw_on()
        return

    def mw_off(self):
        self.odmrlogic1().mw_off()
        return

