# -*- coding: utf-8 -*-

"""
This file contains the QuDi main GUI for pulsed measurements.

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

import numpy as np
import os
import pyqtgraph as pg
import datetime

from core.connector import Connector
from core.statusvariable import StatusVar
from core.util import units
from core.util.helpers import natural_sort
from gui.colordefs import QudiPalettePale as palette
from gui.fitsettings import FitSettingsDialog
from gui.guibase import GUIBase
from qtpy import QtCore, QtWidgets, uic
from qtwidgets.scientific_spinbox import ScienDSpinBox, ScienSpinBox
from qtwidgets.loading_indicator import CircleLoadingIndicator
from enum import Enum


# TODO: Display the Pulse graphically (similar to AWG application)


class PulsedMeasurementMainWindow(QtWidgets.QMainWindow):
    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_pulsed_sg.ui')

        # Load it
        super(PulsedMeasurementMainWindow, self).__init__()

        uic.loadUi(ui_file, self)

        self.show()


class PulseStreamerTab(QtWidgets.QWidget):
    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_do_control.ui')
        # Load it
        super().__init__()
        uic.loadUi(ui_file, self)

class StandardExperimentsTab(QtWidgets.QWidget):
    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_standard_experiments.ui')
        # Load it
        super().__init__()
        uic.loadUi(ui_file, self)

class PulsedMeasurementGui(GUIBase):
    """ This is the main GUI Class for pulsed measurements. """
    ## declare connectors
    pulsedmasterlogic = Connector(interface='PulsedMasterLogic')

    sigCwMwOn = QtCore.Signal()
    sigMwOff = QtCore.Signal()

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)
        self.channel_states = [1, 1, 1, 0, 0, 0, 0, 0]

    def on_activate(self):
        """ Initialize, connect and configure the pulsed measurement GUI.

        Establish general connectivity and activate the different tabs of the
        GUI.
        """
        self._mw = PulsedMeasurementMainWindow()
        self._pst = PulseStreamerTab()
        self._st_expt = StandardExperimentsTab()


        self._mw.tabWidget.addTab(self._pst, 'PulseStreamer Controls')
        self._mw.tabWidget.addTab(self._st_expt, 'Standard Experiments')

        self._activate_main_window_ui()

        #######################################
        ###      Connecting signals         ###
        #######################################
        # Set up internal signals
        self._connect_pulsestreamer_signals()

        # Link to signals from the logic layer
        self.pulsedmasterlogic().sigParameterUpdated.connect(self.update_parameter,
                                                     QtCore.Qt.QueuedConnection)
        self.pulsedmasterlogic().sigMWStatusChanged.connect(self._update_mw_status,
                                                            QtCore.Qt.QueuedConnection)


        # Set up signals which use logic layer functions
        self.sigCwMwOn.connect(self.pulsedmasterlogic().mw_cw_on, QtCore.Qt.QueuedConnection)
        self.sigMwOff.connect(self.pulsedmasterlogic().mw_off, QtCore.Qt.QueuedConnection)




        # checking instrument status
        self.streamer_status = self.pulsedmasterlogic().get_streamer_status()
        if self.streamer_status['Connected'] != -1:
            self.status_text = 'Connected to Pulse Streamer at ' + str(self.streamer_status['Connected IP'])
        else:
            self.status_text = 'Disconnected'
        self._pst.connection_status_tb.setText(self.status_text)
        self.show()

        self._pst.connection_status_tb.append('Microwave Source: ' + self.pulsedmasterlogic().odmrlogic1().microwave1().microwave().model)

        # setting constant channels
        self.channel_states = self.pulsedmasterlogic().do_channel_states
        self._update_do_checkboxes()
        self.CW_clicked()

        self.uw_freq = self.pulsedmasterlogic().uw_frequency
        self._update_mw_freq()

        self.uw_power = self.pulsedmasterlogic().uw_power
        self._update_mw_power()

        self.uw_state = True # Start by declaring the MW on so the GUI immediately turns it off on startup
        self.toggle_mw_cw()


        return

    def on_deactivate(self):
        """ Undo the Definition, configuration and initialisation of the pulsed
            measurement GUI.

        This deactivation disconnects all the graphic modules, which were
        connected in the initUI method.
        """

        self._mw.close()
        return

    def show(self):
        """Make main window visible and put it above all other windows. """
        QtWidgets.QMainWindow.show(self._mw)
        self._mw.activateWindow()
        self._mw.raise_()
        return

## Signal connections
    def _connect_pulsestreamer_signals(self):
        # Connect main window actions and toolbar
        self._pst.set_cw_states_button.clicked.connect(self.CW_clicked)
        self._pst.mw_on_button.clicked.connect(self.toggle_mw_cw)


## activate deactivate functions
    def _activate_main_window_ui(self):
        # self._setup_toolbar()
        # self.loaded_asset_updated(*self.pulsedmasterlogic().loaded_asset)
        return

    def _deactivate_main_window_ui(self):
        pass

## GUI functions
    @QtCore.Slot(bool)
    def CW_clicked(self):
        """ Manually switch the pulser output on/off. """
        self._update_channel_states()
        self.pulsedmasterlogic().set_continuous(self.channel_states)
        return

    @QtCore.Slot(bool)
    def toggle_mw_cw(self):
        """ Manually switch the pulser output on/off. """
        if self.uw_state:
            print('State is '+str(self.uw_state)+': Turning off')
            self.sigMwOff.emit()
        else:
            print('State is ' + str(self.uw_state) + ': Turning on')
            self.sigCwMwOn.emit()
        return

## GUI update
    def _update_do_checkboxes(self):
        self._pst.do0_cb.setChecked(self.channel_states[0])
        self._pst.do1_cb.setChecked(self.channel_states[1])
        self._pst.do2_cb.setChecked(self.channel_states[2])
        self._pst.do3_cb.setChecked(self.channel_states[3])
        self._pst.do4_cb.setChecked(self.channel_states[4])
        self._pst.do5_cb.setChecked(self.channel_states[5])
        self._pst.do6_cb.setChecked(self.channel_states[6])
        self._pst.do7_cb.setChecked(self.channel_states[7])
        return

    def _update_channel_states(self):
        self.channel_states[0] = self._pst.do0_cb.isChecked()
        self.channel_states[1] = self._pst.do1_cb.isChecked()
        self.channel_states[2] = self._pst.do2_cb.isChecked()
        self.channel_states[3] = self._pst.do3_cb.isChecked()
        self.channel_states[4] = self._pst.do4_cb.isChecked()
        self.channel_states[5] = self._pst.do5_cb.isChecked()
        self.channel_states[6] = self._pst.do6_cb.isChecked()
        self.channel_states[7] = self._pst.do7_cb.isChecked()


    def _update_do_indicators(self):
        self._pst.do0_status.display(int(self.channel_states[0]))
        self._pst.do1_status.display(int(self.channel_states[1]))
        self._pst.do2_status.display(int(self.channel_states[2]))
        self._pst.do3_status.display(int(self.channel_states[3]))
        self._pst.do4_status.display(int(self.channel_states[4]))
        self._pst.do5_status.display(int(self.channel_states[5]))
        self._pst.do6_status.display(int(self.channel_states[6]))
        self._pst.do7_status.display(int(self.channel_states[7]))
        return

    def update_parameter(self, param_dict):
        """ Update the parameter display in the GUI.

        @param param_dict:
        @return:

        Any change event from the logic should call this update function.
        The update will block the GUI signals from emitting a change back to the
        logic.
        """


        param = param_dict.get('cw_mw_frequency')
        if param is not None:
            self._pst.mw_freq_spinbox.blockSignals(True)
            self._pst.mw_freq_spinbox.setValue(param)
            self._pst.mw_freq_spinbox.blockSignals(False)

            self._st_expt.mw_freq_spinbox.blockSignals(True)
            self._st_expt.mw_freq_spinbox.setValue(param)
            self._st_expt.mw_freq_spinbox.blockSignals(False)

        param = param_dict.get('cw_mw_power')
        if param is not None:
            self._pst.mw_power_spinbox.blockSignals(True)
            self._pst.mw_power_spinbox.setValue(param)
            self._pst.mw_power_spinbox.blockSignals(False)

            self._st_expt.mw_power_spinbox.blockSignals(True)
            self._st_expt.mw_power_spinbox.setValue(param)
            self._st_expt.mw_power_spinbox.blockSignals(False)

        param = param_dict.get('DO_states')
        if param is not None:
            self.channel_states = param
            self._update_do_indicators()

        return

    def _update_mw_status(self,status):
        if status:
            self._pst.mw_freq_spinbox.setEnabled(False)
            self._pst.mw_power_spinbox.setEnabled(False)
            self._st_expt.mw_freq_spinbox.setEnabled(False)
            self._st_expt.mw_power_spinbox.setEnabled(False)
            self._pst.mw_on_button.setChecked(True)
            self._pst.mw_on_button.setText('Push to Turn Microwaves Off')
            self.uw_state = True
        else:
            self._pst.mw_freq_spinbox.setEnabled(True)
            self._pst.mw_power_spinbox.setEnabled(True)
            self._st_expt.mw_freq_spinbox.setEnabled(True)
            self._st_expt.mw_power_spinbox.setEnabled(True)
            self._pst.mw_on_button.setChecked(False)
            self._pst.mw_on_button.setText('Push to Turn Microwaves On')
            self.uw_state = False

        return

    def _update_mw_freq(self):
        self._pst.mw_freq_spinbox.setValue(self.uw_freq)
        self._st_expt.mw_freq_spinbox.setValue(self.uw_freq)
        return

    def _update_mw_power(self):
        self._pst.mw_power_spinbox.setValue(self.uw_power)
        self._st_expt.mw_power_spinbox.setValue(self.uw_power)
        return
