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
from gui.guiutils import ColorBar
from gui.colordefs import QudiPalettePale as palette
from gui.colordefs import ColorScaleInferno
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

class CWODMRTab(QtWidgets.QWidget):
    def __init__(self):
        # Get the path to the *.ui file
        this_dir = os.path.dirname(__file__)
        ui_file = os.path.join(this_dir, 'ui_cwodmr.ui')
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
    sigStartPulsed = QtCore.Signal()
    sigStopPulsed = QtCore.Signal()

    sigMwCwParamsChanged = QtCore.Signal(float, float)
    sigPiPulseChanged = QtCore.Signal(float)
    sigExptChanged = QtCore.Signal(str)
    sigSaveChanged = QtCore.Signal(bool)
    sigTrackChanged = QtCore.Signal(bool,int)
    sigNameChanged = QtCore.Signal(str)
    sigSaveMeasurement = QtCore.Signal(str, list, list)

    def __init__(self, config, **kwargs):
        super().__init__(config=config, **kwargs)
        self.channel_states = [1, 1, 1, 0, 0, 0, 0, 0]
        self.expttorun = 'Rabi'
        self.rabiparams = [10.,1000.,10.,0.1,1] # start, stop, step, dwell per point, average
        self.ramseyparams = [10., 1000., 10., 0.1, 1]  # start, stop, step, dwell per point, average
        self.hahnparams = [100., 3000., 100., 0.1, 1]  # start, stop, step, dwell per point, average
        self.t1params = [10., 1000., 10., 0.1, 1]  # start, stop, step, dwell per point, average
        self.odmrparams = [2.80e9,2.90e9,1.e6,0.1,1]
        self.pulselengths = [700,10,3000,300]
        self.pulseconfigs = [0,2,1]
        self.cwparams = [2.80e9,2.90e9,1.e6,0.1,1,1.e-5]

    def on_activate(self):
        """ Initialize, connect and configure the pulsed measurement GUI.

        Establish general connectivity and activate the different tabs of the
        GUI.
        """
        self._pulsed_logic = self.pulsedmasterlogic()

        self._mw = PulsedMeasurementMainWindow()
        self._pst = PulseStreamerTab()
        self._cwodmr = CWODMRTab()
        self._st_expt = StandardExperimentsTab()


        self._mw.tabWidget.addTab(self._pst, 'PulseStreamer Controls')
        self._mw.tabWidget.addTab(self._cwodmr,'CW ODMR')
        self._mw.tabWidget.addTab(self._st_expt, 'Standard Pulsed Experiments')

        self._activate_main_window_ui()

        ## generate list of experiments in the combobox
        self.exptdict = {}
        for i in range(self._st_expt.combo_exptchoice.count()):
            keyname = self._st_expt.combo_exptchoice.itemText(i)
            self.exptdict[keyname] = i

        self._mw.save_tag_LineEdit = QtWidgets.QLineEdit(self._mw)
        self._mw.save_tag_LineEdit.setMaximumWidth(500)
        self._mw.save_tag_LineEdit.setMinimumWidth(200)
        self._mw.save_tag_LineEdit.setToolTip('Enter a nametag which will be\n'
                                              'added to the filename.')
        self._mw.save_ToolBar.addWidget(self._mw.save_tag_LineEdit)

        #######################################
        ###      Connecting signals         ###
        #######################################
        # Set up internal signals
        self._connect_pulsestreamer_signals()
        self._connect_standardexpt_signals()

        # Link to signals coming from the logic layer
        self._pulsed_logic.sigParameterUpdated.connect(self.update_parameter,
                                                     QtCore.Qt.QueuedConnection)
        self._pulsed_logic.sigMWStatusChanged.connect(self._update_mw_status,
                                                            QtCore.Qt.QueuedConnection)
        self._pulsed_logic.sigExptRunningUpdated.connect(self.update_status,
                                                         QtCore.Qt.QueuedConnection)
        self.sigSaveMeasurement.connect(self._pulsed_logic.save_pulsed_data, QtCore.Qt.QueuedConnection)
        self._mw.action_run_stop.triggered.connect(self.run_stop_pulsed)
        self._mw.action_save.triggered.connect(self.save_data)
        self._pulsed_logic.sigPulsedPlotsUpdated.connect(self.update_plots, QtCore.Qt.QueuedConnection)


        # Set up signals which use logic layer functions
        self.sigCwMwOn.connect(self._pulsed_logic.mw_cw_on, QtCore.Qt.QueuedConnection)
        self.sigMwOff.connect(self._pulsed_logic.mw_off, QtCore.Qt.QueuedConnection)
        self.sigStartPulsed.connect(self._pulsed_logic.start_pulsed_scan, QtCore.Qt.QueuedConnection)
        self.sigStopPulsed.connect(self._pulsed_logic.stop_pulsed_scan, QtCore.Qt.QueuedConnection)
        self.sigNameChanged.connect(self._pulsed_logic.change_fname, QtCore.Qt.QueuedConnection)
        self.sigMwCwParamsChanged.connect(self._pulsed_logic.odmrlogic1().set_cw_parameters,
                                          QtCore.Qt.QueuedConnection)
        self.sigPiPulseChanged.connect(self._pulsed_logic.set_pi_pulse,
                                       QtCore.Qt.QueuedConnection)
        self.sigExptChanged.connect(self._pulsed_logic.set_expt,
                                          QtCore.Qt.QueuedConnection)
        self.sigSaveChanged.connect(self._pulsed_logic.set_autosave,
                                    QtCore.Qt.QueuedConnection)
        self.sigTrackChanged.connect(self._pulsed_logic.set_autotrack,
                                    QtCore.Qt.QueuedConnection)


        # user input signals
        self._mw.tabWidget.currentChanged.connect(self.change_tab)
        self._mw.mw_freq_spinbox.editingFinished.connect(self.change_cw_params)
        self._mw.mw_power_spinbox.editingFinished.connect(self.change_cw_params)
        self._mw.mw_pi_pulse.editingFinished.connect(self.set_pi_pulse)
        self._mw.save_tag_LineEdit.editingFinished.connect(self.set_fname)

        self._st_expt.combo_exptchoice.currentIndexChanged.connect(self.change_exptchoice)
        self._mw.autosave_checkBox.stateChanged.connect(self.change_autosave)
        self._mw.autotrack_checkBox.stateChanged.connect(self.change_autotrack)
        self._mw.autotrack_spinBox.valueChanged.connect(self.change_autotrack)


        # checking instrument status
        self.streamer_status = self._pulsed_logic.get_streamer_status()
        if self.streamer_status['Connected'] != -1:
            self.status_text = 'Connected to Pulse Streamer at ' + str(self.streamer_status['Connected IP'])
        else:
            self.status_text = 'Disconnected'
        self._pst.connection_status_tb.setText(self.status_text)
        self.show()

        self._pst.connection_status_tb.append('Microwave Source: ' + self._pulsed_logic.odmrlogic1().microwave1().microwave().model)

        # setting constant channels
        self.tabstate = self._mw.tabWidget.currentIndex()
        self.change_tab()

        self.autosave = self._pulsed_logic.autosave
        self._update_save()

        self.autotrack = self._pulsed_logic._autotrack
        self.trackevery = self._pulsed_logic._trackevery
        self._update_track()

        self.channel_states = self._pulsed_logic.do_channel_states
        self._update_do_checkboxes()
        self.CW_clicked()

        self.expttorun = self._pulsed_logic.expt_current
        self._update_exptchoice()

        self.cwparams = self._pulsed_logic.cwparams
        self._update_cwbox()

        self.rabiparams = self._pulsed_logic.rabiparams
        self._update_rabibox()

        self.ramseyparams = self._pulsed_logic.ramseyparams
        self._update_ramseybox()

        self.hahnparams = self._pulsed_logic.hahnparams
        self._update_hahnbox()

        self.t1params = self._pulsed_logic.t1params
        self._update_t1box()

        self.odmrparams = self._pulsed_logic.odmrparams
        self._update_odmrbox()

        self.pulselengths = self._pulsed_logic.pulselengths
        self.pulseconfigs = self._pulsed_logic.pulseconfigs
        self._update_timingbox()

        self.uw_freq = self._pulsed_logic.uw_frequency
        self._update_mw_freq()

        self.uw_power = self._pulsed_logic.uw_power
        self._update_mw_power()

        self.pi_pulse = self._pulsed_logic.pi_pulse
        self._update_pi_pulse()


        # Start up all the elements
        self.uw_state = True # Start by declaring the MW on so the GUI immediately turns it off on startup
        self.toggle_mw_cw()

        # Initializing graphs
        self.pulsed_image = pg.PlotDataItem(self._pulsed_logic.pulsed_plot_x,
                                          self._pulsed_logic.pulsed_plot_y,
                                          pen=pg.mkPen(palette.c1, style=QtCore.Qt.DotLine),
                                          symbol='o',
                                          symbolPen=palette.c1,
                                          symbolBrush=palette.c1,
                                          symbolSize=7)

        self.pulsed_image2 = pg.PlotDataItem(self._pulsed_logic.pulsed_plot_x,
                                            self._pulsed_logic.pulsed_plot_y,
                                            pen=pg.mkPen(palette.c1, style=QtCore.Qt.DotLine),
                                            symbol='o',
                                            symbolPen=palette.c1,
                                            symbolBrush=palette.c1,
                                            symbolSize=7)

        self._st_expt.pulsed_PlotWidget.addItem(self.pulsed_image)
        self._st_expt.pulsed_PlotWidget.setLabel(axis='left',text='Normalized Counts',units='AU')
        self._st_expt.pulsed_PlotWidget.setLabel(axis='bottom', text='Pulse Duration', units='ns')

        self._cwodmr.pulsed_PlotWidget.addItem(self.pulsed_image2)
        self._cwodmr.pulsed_PlotWidget.setLabel(axis='left', text='Normalized Counts', units='AU')
        self._cwodmr.pulsed_PlotWidget.setLabel(axis='bottom', text='Pulse Duration', units='ns')

        ## Now do the 2D Plot
        self.pulsed_matrix_image = pg.ImageItem(
            self._pulsed_logic.pulsed_plot_xy.T,
            axisOrder = 'row-major')
        self.pulsed_matrix_image.setRect(QtCore.QRectF(
            self._pulsed_logic.rabiparams[0],
            0,
            self._pulsed_logic.rabiparams[1] - self._pulsed_logic.rabiparams[0],
            self._pulsed_logic.number_of_lines
        ))

        self.pulsed_matrix_image2 = pg.ImageItem(
            self._pulsed_logic.pulsed_plot_xy.T,
            axisOrder='row-major')
        self.pulsed_matrix_image.setRect(QtCore.QRectF(
            self._pulsed_logic.rabiparams[0],
            0,
            self._pulsed_logic.rabiparams[1] - self._pulsed_logic.rabiparams[0],
            self._pulsed_logic.number_of_lines
        ))

        self._st_expt.pulsed_matrix_PlotWidget.addItem(self.pulsed_matrix_image)
        self._st_expt.pulsed_matrix_PlotWidget.setLabel(axis='left',text='Sweep',units='#')
        self._st_expt.pulsed_matrix_PlotWidget.setLabel(axis='bottom', text='Pulse Duration', units='ns')

        self._cwodmr.pulsed_matrix_PlotWidget.addItem(self.pulsed_matrix_image2)
        self._cwodmr.pulsed_matrix_PlotWidget.setLabel(axis='left', text='Sweep', units='#')
        self._cwodmr.pulsed_matrix_PlotWidget.setLabel(axis='bottom', text='Pulse Duration', units='ns')

        mycolors = ColorScaleInferno()
        self.pulsed_matrix_image.setLookupTable(mycolors.lut)
        self.pulsed_matrix_image2.setLookupTable(mycolors.lut)

        self.pulsed_cb = ColorBar(mycolors.cmap_normed, 100, 0, 1)

        # adding colorbar to ViewWidget
        self._st_expt.cb_plot_widget.addItem(self.pulsed_cb)
        self._st_expt.cb_plot_widget.hideAxis('bottom')
        self._st_expt.cb_plot_widget.hideAxis('left')
        self._st_expt.cb_plot_widget.setLabel('right', 'Normalized Signal', units='AU')

        # adding colorbar to ViewWidget
        self._cwodmr.cb_plot_widget.addItem(self.pulsed_cb)
        self._cwodmr.cb_plot_widget.hideAxis('bottom')
        self._cwodmr.cb_plot_widget.hideAxis('left')
        self._cwodmr.cb_plot_widget.setLabel('right', 'Normalized Signal', units='AU')
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
        #
        self._pst.combo_aom.currentIndexChanged.connect(self._update_timingvals)
        self._pst.combo_gate.currentIndexChanged.connect(self._update_timingvals)
        self._pst.combo_mw.currentIndexChanged.connect(self._update_timingvals)
        #
        self._pst.aom_delay_spinbox.editingFinished.connect(self._update_timingvals)
        self._pst.green_pulse_spinbox.editingFinished.connect(self._update_timingvals)
        self._pst.integration_spinbox.editingFinished.connect(self._update_timingvals)

        return

    def _connect_standardexpt_signals(self):
        # CW
        self._cwodmr.spinbox_odmr_start.editingFinished.connect(self._update_cwvals)
        self._cwodmr.spinbox_odmr_stop.editingFinished.connect(self._update_cwvals)
        self._cwodmr.spinbox_odmr_step.editingFinished.connect(self._update_cwvals)
        self._cwodmr.spinbox_odmr_dwell.editingFinished.connect(self._update_cwvals)
        self._cwodmr.spinbox_odmr_rep.editingFinished.connect(self._update_cwvals)
        self._cwodmr.spinbox_odmr_ref.editingFinished.connect(self._update_cwvals)
        # Rabi
        self._st_expt.spinbox_rabi_start.editingFinished.connect(self._update_rabivals)
        self._st_expt.spinbox_rabi_stop.editingFinished.connect(self._update_rabivals)
        self._st_expt.spinbox_rabi_step.editingFinished.connect(self._update_rabivals)
        self._st_expt.spinbox_rabi_dwell.editingFinished.connect(self._update_rabivals)
        self._st_expt.spinbox_rabi_rep.editingFinished.connect(self._update_rabivals)
        # Ramsey
        self._st_expt.spinbox_ramsey_start.editingFinished.connect(self._update_ramseyvals)
        self._st_expt.spinbox_ramsey_stop.editingFinished.connect(self._update_ramseyvals)
        self._st_expt.spinbox_ramsey_step.editingFinished.connect(self._update_ramseyvals)
        self._st_expt.spinbox_ramsey_dwell.editingFinished.connect(self._update_ramseyvals)
        self._st_expt.spinbox_ramsey_rep.editingFinished.connect(self._update_ramseyvals)
        # Hahn
        self._st_expt.spinbox_hahn_start.editingFinished.connect(self._update_hahnvals)
        self._st_expt.spinbox_hahn_stop.editingFinished.connect(self._update_hahnvals)
        self._st_expt.spinbox_hahn_step.editingFinished.connect(self._update_hahnvals)
        self._st_expt.spinbox_hahn_dwell.editingFinished.connect(self._update_hahnvals)
        self._st_expt.spinbox_hahn_rep.editingFinished.connect(self._update_hahnvals)
        # T1
        self._st_expt.spinbox_t1_start.editingFinished.connect(self._update_t1vals)
        self._st_expt.spinbox_t1_stop.editingFinished.connect(self._update_t1vals)
        self._st_expt.spinbox_t1_step.editingFinished.connect(self._update_t1vals)
        self._st_expt.spinbox_t1_dwell.editingFinished.connect(self._update_t1vals)
        self._st_expt.spinbox_t1_rep.editingFinished.connect(self._update_t1vals)

        self._st_expt.spinbox_odmr_start.editingFinished.connect(self._update_odmrvals)
        self._st_expt.spinbox_odmr_stop.editingFinished.connect(self._update_odmrvals)
        self._st_expt.spinbox_odmr_step.editingFinished.connect(self._update_odmrvals)
        self._st_expt.spinbox_odmr_dwell.editingFinished.connect(self._update_odmrvals)
        self._st_expt.spinbox_odmr_rep.editingFinished.connect(self._update_odmrvals)
        return


## activate deactivate functions
    def _activate_main_window_ui(self):
        # self._setup_toolbar()
        return

    def _deactivate_main_window_ui(self):
        pass

    def run_stop_pulsed(self,is_checked):
        # manages what happens when the button to start is clicked
        if is_checked:
            # disable a bunch of things
            ##### Toolbar
            self._mw.action_run_stop.setEnabled(False)
            self._mw.action_continue_pause.setEnabled(False)

            ##### Microwave Control
            self._mw.mw_freq_spinbox.blockSignals(True)
            self._mw.mw_freq_spinbox.setEnabled(False)
            self._mw.mw_freq_spinbox.blockSignals(False)
            self._mw.mw_power_spinbox.blockSignals(True)
            self._mw.mw_power_spinbox.setEnabled(False)
            self._mw.mw_power_spinbox.blockSignals(False)
            self._mw.mw_pi_pulse.blockSignals(True)
            self._mw.mw_pi_pulse.setEnabled(False)
            self._mw.mw_pi_pulse.blockSignals(False)

            #### On/off buttons on pulsestreamer panel
            self._pst.set_cw_states_button.setEnabled(False)
            self._pst.mw_on_button.setEnabled(False)

            ##### Pulse configuration settings
            self._pst.aom_delay_spinbox.setEnabled(False)
            self._pst.green_pulse_spinbox.setEnabled(False)
            self._pst.integration_spinbox.setEnabled(False)
            self._pst.mw_delay_spinbox.setEnabled(False)

            self._pst.combo_aom.setEnabled(False)
            self._pst.combo_gate.setEnabled(False)
            self._pst.combo_mw.setEnabled(False)

            ###### Pulsed boxes ####
            self._st_expt.combo_exptchoice.setEnabled(False)
            ## Rabi ##
            self._st_expt.spinbox_rabi_start.setEnabled(False)
            self._st_expt.spinbox_rabi_stop.setEnabled(False)
            self._st_expt.spinbox_rabi_step.setEnabled(False)
            self._st_expt.spinbox_rabi_dwell.setEnabled(False)
            self._st_expt.spinbox_rabi_rep.setEnabled(False)
            ## Ramsey ##
            self._st_expt.spinbox_ramsey_start.setEnabled(False)
            self._st_expt.spinbox_ramsey_stop.setEnabled(False)
            self._st_expt.spinbox_ramsey_step.setEnabled(False)
            self._st_expt.spinbox_ramsey_dwell.setEnabled(False)
            self._st_expt.spinbox_ramsey_rep.setEnabled(False)
            ## Pulsed ODMR ##
            self._st_expt.spinbox_odmr_start.setEnabled(False)
            self._st_expt.spinbox_odmr_stop.setEnabled(False)
            self._st_expt.spinbox_odmr_step.setEnabled(False)
            self._st_expt.spinbox_odmr_dwell.setEnabled(False)
            self._st_expt.spinbox_odmr_rep.setEnabled(False)
            ## Hahn Echo ##
            self._st_expt.spinbox_hahn_start.setEnabled(False)
            self._st_expt.spinbox_hahn_stop.setEnabled(False)
            self._st_expt.spinbox_hahn_step.setEnabled(False)
            self._st_expt.spinbox_hahn_dwell.setEnabled(False)
            self._st_expt.spinbox_hahn_rep.setEnabled(False)
            ## T1 ##
            self._st_expt.spinbox_t1_start.setEnabled(False)
            self._st_expt.spinbox_t1_stop.setEnabled(False)
            self._st_expt.spinbox_t1_step.setEnabled(False)
            self._st_expt.spinbox_t1_dwell.setEnabled(False)
            self._st_expt.spinbox_t1_rep.setEnabled(False)

            ## CW ODMR ##
            self._cwodmr.spinbox_odmr_start.setEnabled(False)
            self._cwodmr.spinbox_odmr_stop.setEnabled(False)
            self._cwodmr.spinbox_odmr_step.setEnabled(False)
            self._cwodmr.spinbox_odmr_dwell.setEnabled(False)
            self._cwodmr.spinbox_odmr_rep.setEnabled(False)
            self._cwodmr.spinbox_odmr_ref.setEnabled(False)

            # now start to run
            self.expttorun = self._st_expt.combo_exptchoice.currentText()
            if self.expttorun == 'Rabi':
                self.sigStartPulsed.emit()
            elif self.expttorun == 'Ramsey':
                self.sigStartPulsed.emit()
            elif self.expttorun == 'Hahn Echo':
                self.sigStartPulsed.emit()
            elif self.expttorun == 'Pulsed ODMR':
                self.sigStartPulsed.emit()
            elif self.expttorun == 'T1':
                self.sigStartPulsed.emit()
            elif self.iscw:
                self.expttorun = 'CW ODMR'
                self.sigStartPulsed.emit()
            else:
                print("I don't know what experiment to run")
            return
        else:
            self._mw.action_run_stop.setEnabled(False)
            self.sigStopPulsed.emit()



## GUI functions
    @QtCore.Slot(bool)
    def CW_clicked(self):
        """ Manually switch the pulser output on/off. """
        self._update_channel_states()
        self._pulsed_logic.set_continuous(self.channel_states)
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

## Main Window Related Methods
    def update_plots(self,pulsed_data_x,pulsed_data_y,pulsed_data_xy):

        self.pulsed_image.setData(pulsed_data_x,pulsed_data_y[2,:])
        self.pulsed_image2.setData(pulsed_data_x, pulsed_data_y[2, :])

        self.pulsed_matrix_image.setImage(image=pulsed_data_xy[2, :, :].T,
                                          axisOrder='row-major')
        self.pulsed_matrix_image2.setImage(image=pulsed_data_xy[2, :, :].T,
                                          axisOrder='row-major')
        cb_range = self.get_matrix_cb_range()
        # cb_range=[0,1]
        self.update_colorbar(cb_range)

        self.pulsed_matrix_image.setImage(image=pulsed_data_xy[2,:,:].T,
                                          axisOrder='row-major',
                                          levels=(cb_range[0], cb_range[1]))
        self.pulsed_matrix_image2.setImage(image=pulsed_data_xy[2, :, :].T,
                                          axisOrder='row-major',
                                          levels=(cb_range[0], cb_range[1]))
        return

    def get_matrix_cb_range(self):
        matrix_image = self.pulsed_matrix_image.image
        matrix_image_nonzero = matrix_image[np.nonzero(matrix_image)]

        low_centile = self._st_expt.low_perc_doubleSpinBox.value()
        high_centile = self._st_expt.hi_perc_doubleSpinBox.value()

        cb_min = np.percentile(matrix_image_nonzero, low_centile)
        cb_max = np.percentile(matrix_image_nonzero, high_centile)

        cb_range = [cb_min, cb_max]
        return cb_range

    def update_colorbar(self, cb_range):
        """
        Update the colorbar to a new range.

        @param list cb_range: List or tuple containing the min and max values for the cb range
        """
        self.pulsed_cb.refresh_colorbar(cb_range[0], cb_range[1])
        return



## GUI update
    def update_status(self,is_running):
        self._mw.action_run_stop.blockSignals(True)

        if is_running:
            self._mw.action_run_stop.setEnabled(True)
            self._mw.action_run_stop.setChecked(True)
        else:
            ##### Toolbar
            self._mw.action_run_stop.setEnabled(True)
            self._mw.action_run_stop.setChecked(False)

            self._mw.action_continue_pause.setEnabled(True)

            ##### Microwave Control
            self._mw.mw_freq_spinbox.setEnabled(True)
            self._mw.mw_power_spinbox.setEnabled(True)
            self._mw.mw_pi_pulse.setEnabled(True)

            #### On/off buttons on pulsestreamer panel
            self._pst.set_cw_states_button.setEnabled(True)
            self._pst.mw_on_button.setEnabled(True)

            ##### Pulse configuration settings
            self._pst.aom_delay_spinbox.setEnabled(True)
            self._pst.green_pulse_spinbox.setEnabled(True)
            self._pst.integration_spinbox.setEnabled(True)
            self._pst.mw_delay_spinbox.setEnabled(True)

            self._pst.combo_aom.setEnabled(True)
            self._pst.combo_gate.setEnabled(True)
            self._pst.combo_mw.setEnabled(True)

            ###### Pulsed boxes ####
            self._st_expt.combo_exptchoice.setEnabled(True)
            ## Rabi ##
            self._st_expt.spinbox_rabi_start.setEnabled(True)
            self._st_expt.spinbox_rabi_stop.setEnabled(True)
            self._st_expt.spinbox_rabi_step.setEnabled(True)
            self._st_expt.spinbox_rabi_dwell.setEnabled(True)
            self._st_expt.spinbox_rabi_rep.setEnabled(True)
            ## Ramsey ##
            self._st_expt.spinbox_ramsey_start.setEnabled(True)
            self._st_expt.spinbox_ramsey_stop.setEnabled(True)
            self._st_expt.spinbox_ramsey_step.setEnabled(True)
            self._st_expt.spinbox_ramsey_dwell.setEnabled(True)
            self._st_expt.spinbox_ramsey_rep.setEnabled(True)
            ## Pulsed ODMR ##
            self._st_expt.spinbox_odmr_start.setEnabled(True)
            self._st_expt.spinbox_odmr_stop.setEnabled(True)
            self._st_expt.spinbox_odmr_dwell.setEnabled(True)
            self._st_expt.spinbox_odmr_step.setEnabled(True)
            self._st_expt.spinbox_odmr_rep.setEnabled(True)
            ## Hahn Echo ##
            self._st_expt.spinbox_hahn_start.setEnabled(True)
            self._st_expt.spinbox_hahn_stop.setEnabled(True)
            self._st_expt.spinbox_hahn_dwell.setEnabled(True)
            self._st_expt.spinbox_hahn_step.setEnabled(True)
            self._st_expt.spinbox_hahn_rep.setEnabled(True)
            ## T1 ##
            self._st_expt.spinbox_t1_start.setEnabled(True)
            self._st_expt.spinbox_t1_stop.setEnabled(True)
            self._st_expt.spinbox_t1_dwell.setEnabled(True)
            self._st_expt.spinbox_t1_step.setEnabled(True)
            self._st_expt.spinbox_t1_rep.setEnabled(True)
            ## CW ODMR ##
            self._cwodmr.spinbox_odmr_start.setEnabled(True)
            self._cwodmr.spinbox_odmr_stop.setEnabled(True)
            self._cwodmr.spinbox_odmr_step.setEnabled(True)
            self._cwodmr.spinbox_odmr_dwell.setEnabled(True)
            self._cwodmr.spinbox_odmr_rep.setEnabled(True)
            self._cwodmr.spinbox_odmr_ref.setEnabled(True)

        self._mw.action_run_stop.blockSignals(False)
        return

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

    def change_cw_params(self):
        """ Change CW frequency and power of microwave source """
        frequency = self._mw.mw_freq_spinbox.value()
        power = self._mw.mw_power_spinbox.value()
        self.sigMwCwParamsChanged.emit(frequency, power)
        return

    def set_fname(self):
        self.fname = self._mw.save_tag_LineEdit.text()
        self.sigNameChanged.emit(self.fname)

    def set_pi_pulse(self):
        """ Set the pulse length for a pi and pi/2 pulse"""
        self.pi_pulse = self._mw.mw_pi_pulse.value()
        self.sigPiPulseChanged.emit(self.pi_pulse)

    def change_exptchoice(self):
        exptind = self._st_expt.combo_exptchoice.currentIndex()
        self.expttorun = list(self.exptdict.keys())[list(self.exptdict.values()).index(exptind)]
        self.sigExptChanged.emit(self.expttorun)
        return

    def change_autosave(self):
        self.autosave = self._mw.autosave_checkBox.isChecked()
        self.sigSaveChanged.emit(self.autosave)
        return

    def change_tab(self):
        self.tabstate = self._mw.tabWidget.currentIndex()
        if self.tabstate == 1:
            self.iscw = True
            self.expttorun = 'CW ODMR'
            self.sigExptChanged.emit(self.expttorun)
        else:
            self.iscw = False
            self.change_exptchoice()
        return


    def change_autotrack(self):
        self.autotrack = self._mw.autotrack_checkBox.isChecked()
        self.trackevery = self._mw.autotrack_spinBox.value()
        self.sigTrackChanged.emit(self.autotrack,int(self.trackevery))
        return

    def _update_timingvals(self):
        self.pulselengths[0] = self._pst.aom_delay_spinbox.value()
        self.pulselengths[1] = self._pst.mw_delay_spinbox.value()
        self.pulselengths[2] = self._pst.green_pulse_spinbox.value()
        self.pulselengths[3] = self._pst.integration_spinbox.value()
        #
        self.pulseconfigs[0] = self._pst.combo_aom.currentIndex()
        self.pulseconfigs[1] = self._pst.combo_mw.currentIndex()
        self.pulseconfigs[2] = self._pst.combo_gate.currentIndex()
        self._pulsed_logic.set_timing(pulselengths=self.pulselengths,pulseconfigs=self.pulseconfigs)
        return

    def _update_timingbox(self):
        self._pst.aom_delay_spinbox.blockSignals(True)
        self._pst.aom_delay_spinbox.setValue(self.pulselengths[0])
        self._pst.aom_delay_spinbox.blockSignals(False)

        self._pst.mw_delay_spinbox.blockSignals(True)
        self._pst.mw_delay_spinbox.setValue(self.pulselengths[1])
        self._pst.mw_delay_spinbox.blockSignals(False)

        self._pst.green_pulse_spinbox.blockSignals(True)
        self._pst.green_pulse_spinbox.setValue(self.pulselengths[2])
        self._pst.green_pulse_spinbox.blockSignals(False)

        self._pst.integration_spinbox.blockSignals(True)
        self._pst.integration_spinbox.setValue(self.pulselengths[3])
        self._pst.integration_spinbox.blockSignals(False)
        #
        self._pst.combo_aom.blockSignals(True)
        self._pst.combo_aom.setCurrentIndex(self.pulseconfigs[0])
        self._pst.combo_aom.blockSignals(False)
        self._pst.combo_mw.blockSignals(True)
        self._pst.combo_mw.setCurrentIndex(self.pulseconfigs[1])
        self._pst.combo_mw.blockSignals(False)
        self._pst.combo_gate.blockSignals(True)
        self._pst.combo_gate.setCurrentIndex(self.pulseconfigs[2])
        self._pst.combo_gate.blockSignals(False)
        return

    def _update_exptchoice(self):
        getind = self.exptdict[self.expttorun]
        self._st_expt.combo_exptchoice.setCurrentIndex(getind)
        return

    def _update_save(self):
        self._mw.autosave_checkBox.setChecked(self.autosave)
        return

    def _update_track(self):
        self._mw.autotrack_checkBox.setChecked(self.autotrack)
        self._mw.autotrack_spinBox.setValue(self.trackevery)
        return

    #########################################
    #####   Experiment-specific methods #####
    #########################################

    def _update_cwvals(self):
        self.cwparams[0] = self._cwodmr.spinbox_odmr_start.value()
        self.cwparams[1] = self._cwodmr.spinbox_odmr_stop.value()
        self.cwparams[2] = self._cwodmr.spinbox_odmr_step.value()
        self.cwparams[3] = self._cwodmr.spinbox_odmr_dwell.value()
        self.cwparams[4] = self._cwodmr.spinbox_odmr_rep.value()
        self.cwparams[5] = self._cwodmr.spinbox_odmr_ref.value()
        self.log.info("_update_cwvals(): self.cwparams[4] = {}".format(self.cwparams[4]))
        self._pulsed_logic.set_cw(self.cwparams)
        return

    def _update_cwbox(self):
        self._cwodmr.spinbox_odmr_start.blockSignals(True)
        self._cwodmr.spinbox_odmr_start.setValue(self.cwparams[0])
        self._cwodmr.spinbox_odmr_start.blockSignals(False)

        self._cwodmr.spinbox_odmr_stop.blockSignals(True)
        self._cwodmr.spinbox_odmr_stop.setValue(self.cwparams[1])
        self._cwodmr.spinbox_odmr_stop.blockSignals(False)

        self._cwodmr.spinbox_odmr_step.blockSignals(True)
        self._cwodmr.spinbox_odmr_step.setValue(self.cwparams[2])
        self._cwodmr.spinbox_odmr_step.blockSignals(False)

        self._cwodmr.spinbox_odmr_dwell.blockSignals(True)
        self._cwodmr.spinbox_odmr_dwell.setValue(self.cwparams[3])
        self._cwodmr.spinbox_odmr_dwell.blockSignals(False)

        self._cwodmr.spinbox_odmr_rep.blockSignals(True)
        self._cwodmr.spinbox_odmr_rep.setValue(self.cwparams[4])
        self._cwodmr.spinbox_odmr_rep.blockSignals(False)

        self._cwodmr.spinbox_odmr_ref.blockSignals(True)
        self._cwodmr.spinbox_odmr_ref.setValue(self.cwparams[5])
        self._cwodmr.spinbox_odmr_ref.blockSignals(False)
        return

    def _update_rabivals(self):
        self.rabiparams[0] = self._st_expt.spinbox_rabi_start.value()
        self.rabiparams[1] = self._st_expt.spinbox_rabi_stop.value()
        self.rabiparams[2] = self._st_expt.spinbox_rabi_step.value()
        self.rabiparams[3] = self._st_expt.spinbox_rabi_dwell.value()
        self.rabiparams[4] = self._st_expt.spinbox_rabi_rep.value()
        self._pulsed_logic.set_rabi(self.rabiparams)
        return


    def _update_rabibox(self):
        self._st_expt.spinbox_rabi_start.blockSignals(True)
        self._st_expt.spinbox_rabi_start.setValue(self.rabiparams[0])
        self._st_expt.spinbox_rabi_start.blockSignals(False)

        self._st_expt.spinbox_rabi_stop.blockSignals(True)
        self._st_expt.spinbox_rabi_stop.setValue(self.rabiparams[1])
        self._st_expt.spinbox_rabi_stop.blockSignals(False)

        self._st_expt.spinbox_rabi_step.blockSignals(True)
        self._st_expt.spinbox_rabi_step.setValue(self.rabiparams[2])
        self._st_expt.spinbox_rabi_step.blockSignals(False)

        self._st_expt.spinbox_rabi_dwell.blockSignals(True)
        self._st_expt.spinbox_rabi_dwell.setValue(self.rabiparams[3])
        self._st_expt.spinbox_rabi_dwell.blockSignals(False)

        self._st_expt.spinbox_rabi_rep.blockSignals(True)
        self._st_expt.spinbox_rabi_rep.setValue(self.rabiparams[4])
        self._st_expt.spinbox_rabi_rep.blockSignals(False)
        return

    def _update_ramseyvals(self):
        self.ramseyparams[0] = self._st_expt.spinbox_ramsey_start.value()
        self.ramseyparams[1] = self._st_expt.spinbox_ramsey_stop.value()
        self.ramseyparams[2] = self._st_expt.spinbox_ramsey_step.value()
        self.ramseyparams[3] = self._st_expt.spinbox_ramsey_dwell.value()
        self.ramseyparams[4] = self._st_expt.spinbox_ramsey_rep.value()
        self._pulsed_logic.set_ramsey(self.ramseyparams)
        return

    def _update_ramseybox(self):
        self._st_expt.spinbox_ramsey_start.blockSignals(True)
        self._st_expt.spinbox_ramsey_start.setValue(self.ramseyparams[0])
        self._st_expt.spinbox_ramsey_start.blockSignals(False)

        self._st_expt.spinbox_ramsey_stop.blockSignals(True)
        self._st_expt.spinbox_ramsey_stop.setValue(self.ramseyparams[1])
        self._st_expt.spinbox_ramsey_stop.blockSignals(False)

        self._st_expt.spinbox_ramsey_step.blockSignals(True)
        self._st_expt.spinbox_ramsey_step.setValue(self.ramseyparams[2])
        self._st_expt.spinbox_ramsey_step.blockSignals(False)

        self._st_expt.spinbox_ramsey_dwell.blockSignals(True)
        self._st_expt.spinbox_ramsey_dwell.setValue(self.ramseyparams[3])
        self._st_expt.spinbox_ramsey_dwell.blockSignals(False)

        self._st_expt.spinbox_ramsey_rep.blockSignals(True)
        self._st_expt.spinbox_ramsey_rep.setValue(self.ramseyparams[4])
        self._st_expt.spinbox_ramsey_rep.blockSignals(False)
        return

    def _update_hahnvals(self):
        self.hahnparams[0] = self._st_expt.spinbox_hahn_start.value()
        self.hahnparams[1] = self._st_expt.spinbox_hahn_stop.value()
        self.hahnparams[2] = self._st_expt.spinbox_hahn_step.value()
        self.hahnparams[3] = self._st_expt.spinbox_hahn_dwell.value()
        self.hahnparams[4] = self._st_expt.spinbox_hahn_rep.value()
        self._pulsed_logic.set_hahn(self.hahnparams)
        return

    def _update_hahnbox(self):
        self._st_expt.spinbox_hahn_start.blockSignals(True)
        self._st_expt.spinbox_hahn_start.setValue(self.hahnparams[0])
        self._st_expt.spinbox_hahn_start.blockSignals(False)

        self._st_expt.spinbox_hahn_stop.blockSignals(True)
        self._st_expt.spinbox_hahn_stop.setValue(self.hahnparams[1])
        self._st_expt.spinbox_hahn_stop.blockSignals(False)

        self._st_expt.spinbox_hahn_step.blockSignals(True)
        self._st_expt.spinbox_hahn_step.setValue(self.hahnparams[2])
        self._st_expt.spinbox_hahn_step.blockSignals(False)

        self._st_expt.spinbox_hahn_dwell.blockSignals(True)
        self._st_expt.spinbox_hahn_dwell.setValue(self.hahnparams[3])
        self._st_expt.spinbox_hahn_dwell.blockSignals(False)

        self._st_expt.spinbox_hahn_rep.blockSignals(True)
        self._st_expt.spinbox_hahn_rep.setValue(self.hahnparams[4])
        self._st_expt.spinbox_hahn_rep.blockSignals(False)
        return

    def _update_t1vals(self):
        self.t1params[0] = self._st_expt.spinbox_t1_start.value()
        self.t1params[1] = self._st_expt.spinbox_t1_stop.value()
        self.t1params[2] = self._st_expt.spinbox_t1_step.value()
        self.t1params[3] = self._st_expt.spinbox_t1_dwell.value()
        self.t1params[4] = self._st_expt.spinbox_t1_rep.value()
        self._pulsed_logic.set_t1(self.t1params)
        return

    def _update_t1box(self):
        self._st_expt.spinbox_t1_start.blockSignals(True)
        self._st_expt.spinbox_t1_start.setValue(self.t1params[0])
        self._st_expt.spinbox_t1_start.blockSignals(False)

        self._st_expt.spinbox_t1_stop.blockSignals(True)
        self._st_expt.spinbox_t1_stop.setValue(self.t1params[1])
        self._st_expt.spinbox_t1_stop.blockSignals(False)

        self._st_expt.spinbox_t1_step.blockSignals(True)
        self._st_expt.spinbox_t1_step.setValue(self.t1params[2])
        self._st_expt.spinbox_t1_step.blockSignals(False)

        self._st_expt.spinbox_t1_dwell.blockSignals(True)
        self._st_expt.spinbox_t1_dwell.setValue(self.t1params[3])
        self._st_expt.spinbox_t1_dwell.blockSignals(False)

        self._st_expt.spinbox_t1_rep.blockSignals(True)
        self._st_expt.spinbox_t1_rep.setValue(self.t1params[4])
        self._st_expt.spinbox_t1_rep.blockSignals(False)
        return


    def _update_odmrvals(self):
        self.odmrparams[0] = self._st_expt.spinbox_odmr_start.value()
        self.odmrparams[1] = self._st_expt.spinbox_odmr_stop.value()
        self.odmrparams[2] = self._st_expt.spinbox_odmr_step.value()
        self.odmrparams[3] = self._st_expt.spinbox_odmr_dwell.value()
        self.odmrparams[4] = self._st_expt.spinbox_odmr_rep.value()
        self._pulsed_logic.set_odmr(self.odmrparams)
        self.log.info("_update_odmrvals: self.cwparams[4] = {}".format(self.odmrparams[4]))
        return

    def _update_odmrbox(self):
        self._st_expt.spinbox_odmr_start.blockSignals(True)
        self._st_expt.spinbox_odmr_start.setValue(self.odmrparams[0])
        self._st_expt.spinbox_odmr_start.blockSignals(False)

        self._st_expt.spinbox_odmr_stop.blockSignals(True)
        self._st_expt.spinbox_odmr_stop.setValue(self.odmrparams[1])
        self._st_expt.spinbox_odmr_stop.blockSignals(False)

        self._st_expt.spinbox_odmr_step.blockSignals(True)
        self._st_expt.spinbox_odmr_step.setValue(self.odmrparams[2])
        self._st_expt.spinbox_odmr_step.blockSignals(False)

        self._st_expt.spinbox_odmr_dwell.blockSignals(True)
        self._st_expt.spinbox_odmr_dwell.setValue(self.odmrparams[3])
        self._st_expt.spinbox_odmr_dwell.blockSignals(False)

        self._st_expt.spinbox_odmr_rep.blockSignals(True)
        self._st_expt.spinbox_odmr_rep.setValue(self.odmrparams[4])
        self._st_expt.spinbox_odmr_rep.blockSignals(False)
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
            self._mw.mw_freq_spinbox.blockSignals(True)
            self._mw.mw_freq_spinbox.setValue(param)
            self._mw.mw_freq_spinbox.blockSignals(False)

        param = param_dict.get('cw_mw_power')
        if param is not None:
            self._mw.mw_power_spinbox.blockSignals(True)
            self._mw.mw_power_spinbox.setValue(param)
            self._mw.mw_power_spinbox.blockSignals(False)

        param = param_dict.get('DO_states')
        if param is not None:
            self.channel_states = param
            self._update_do_indicators()

        param = param_dict.get('CW_params')
        if param is not None:
            self.cwparams = param
            self._update_cwbox()

        param = param_dict.get('Rabi_params')
        if param is not None:
            self.rabiparams = param
            self._update_rabibox()

        param = param_dict.get('Ramsey_params')
        if param is not None:
            self.ramseyparams = param
            self._update_ramseybox()

        param = param_dict.get('T1_params')
        if param is not None:
            self.t1params = param
            self._update_t1box()

        param = param_dict.get('ODMR_params')
        if param is not None:
            self.odmrparams = param
            self._update_odmrbox()

        param = param_dict.get('Hahn_params')
        if param is not None:
            self.hahnparams = param
            self._update_hahnbox()

        param = param_dict.get('Pulse_lengths')
        if param is not None:
            self.pulselengths = param
            self._update_timingbox()

        param = param_dict.get('Pulse_config')
        if param is not None:
            self.pulseconfigs = param
            self._update_timingbox()

        param = param_dict.get('Pi_pulse')
        if param is not None:
            self.pi_pulse = param
            self._update_pi_pulse()
        return

    def _update_mw_status(self,status):
        if status:
            self._mw.mw_freq_spinbox.setEnabled(False)
            self._mw.mw_power_spinbox.setEnabled(False)

            self._pst.mw_on_button.setChecked(True)
            self._pst.mw_on_button.setText('Push to Turn Microwaves Off')
            self.uw_state = True
        else:
            self._mw.mw_freq_spinbox.setEnabled(True)
            self._mw.mw_power_spinbox.setEnabled(True)

            self._pst.mw_on_button.setChecked(False)
            self._pst.mw_on_button.setText('Push to Turn Microwaves On')
            self.uw_state = False

        return

    def _update_mw_freq(self):
        self._mw.mw_freq_spinbox.setValue(self.uw_freq)
        return

    def _update_mw_power(self):
        self._mw.mw_power_spinbox.setValue(self.uw_power)
        return

    def _update_pi_pulse(self):
        self._mw.mw_pi_pulse.setValue(self.pi_pulse)

    def save_data(self):
        filetag = self._mw.save_tag_LineEdit.text()
        cb_range = self.get_matrix_cb_range()

        # Percentile range is None, unless the percentile scaling is selected in GUI.
        pcile_range = [0, 1]
        # if self._mw.odmr_cb_centiles_RadioButton.isChecked():
        #     low_centile = self._mw.odmr_cb_low_percentile_DoubleSpinBox.value()
        #     high_centile = self._mw.odmr_cb_high_percentile_DoubleSpinBox.value()
        #     pcile_range = [low_centile, high_centile]

        self.sigSaveMeasurement.emit(filetag, cb_range, pcile_range)
        return
