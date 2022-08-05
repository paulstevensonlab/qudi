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
import datetime
import time

from core.connector import Connector
from collections import OrderedDict
from logic.generic_logic import GenericLogic
from qtpy import QtCore
import numpy as np
from core.util.mutex import Mutex
from core.configoption import ConfigOption
from core.statusvariable import StatusVar
import scipy.io
import os.path


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
    fastcounter = Connector(interface='FastCounterInterface')
    odmrlogic1 = Connector(interface='ODMRLogic')
    savelogic = Connector(interface='SaveLogic')
    optimiserlogic = Connector(interface='OptimizerLogic')

    do_channel_states = StatusVar('DO channel states', [0,0,0,0,0,0,0,0])
    pulselengths = StatusVar('Pulse Timing Parameters',[700,10,3000,300]) # aom delay, microwave delay, aom pulse length, integration time, all in nanoseconds
    pulseconfigs = StatusVar('Pulse Channel Configuration',[0,2,1]) # channels for AOM, microwave switch, gate/sync
    #pulseconfigs = StatusVar('Pulse Channel Configuration',[0,2,1,3,4]) # channels for AOM, microwave switch, gate/sync, signal gate, reference gate
    cwparams = StatusVar('CW ODMR params', [2.80e9,2.90e9,1.e6,0.1,1,100.e-6])  # start, stop, step, dwell per point, average, length of ref pulse
    rabiparams = StatusVar('Rabi params',[10., 1000., 10., 0.1, 1])  # start, stop, step, dwell per point, average
    ramseyparams = StatusVar('Ramsey params', [10., 1000., 10., 0.1, 1])  # start, stop, step, dwell per point, average
    hahnparams = StatusVar('Hahn params', [100., 3000., 100., 0.1, 1])  # start, stop, step, dwell per point, average
    t1params = StatusVar('T1 params', [10., 1000., 10., 0.1, 1])  # start, stop, step, dwell per point, average
    odmrparams = StatusVar('Pulsed ODMR params',[2.80e9,2.90e9,1.e6,0.1,1]) # start, stop, step, dwell per point, average
    expt_current = StatusVar('Standard Expt', 'Rabi')
    pi_pulse = StatusVar('Pi Pulse Time',100.)
    autosave = StatusVar('Pulsed Autosave',False)
    _autotrack = StatusVar('autotrack', default=True)
    _trackevery = StatusVar('trackevery',default=3)

    # Define signals
    sigParameterUpdated = QtCore.Signal(dict)
    sigMWStatusChanged = QtCore.Signal(bool)
    sigNextLinePulse = QtCore.Signal()
    sigExptRunningUpdated = QtCore.Signal(bool)
    sigPulsedPlotsUpdated = QtCore.Signal(np.ndarray, np.ndarray, np.ndarray)



    def __init__(self, config, **kwargs):
        """ Create PulsedMasterLogic object with connectors.

          @param dict kwargs: optional parameters
        """
        super().__init__(config=config, **kwargs)
        self.threadlock = Mutex()

        # Dictionary servings as status register
        self.status_dict = dict()
        self.streamer_status = dict()
        self.sequence_dict = dict()
        return

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """

        self._save_logic = self.savelogic()

        # check status of connected instruments
        self.sigNextLinePulse.connect(self._scan_pulse_line, QtCore.Qt.QueuedConnection)
        self.uw_frequency = self.odmrlogic1().cw_mw_frequency
        self.uw_power = self.odmrlogic1().cw_mw_power

        self.odmrlogic1().sigParameterUpdated.connect(self.update_from_odmr,
                                                 QtCore.Qt.QueuedConnection)
        self.odmrlogic1().sigOutputStateUpdated.connect(self.update_mw_status,
                                                       QtCore.Qt.QueuedConnection)

        self.number_of_lines = self.rabiparams[4]
        self.iscw = False # TODO: remove this unused function
        self.fname = ''

        if self.expt_current == 'Rabi':
            self.exptparams = self.rabiparams
        elif self.expt_current == 'Ramsey':
            self.exptparams = self.ramseyparams
        elif self.expt_current == 'T1':
            self.exptparams = self.t1params
        elif self.expt_current == 'T1':
            self.exptparams = self.hahnparams
        else: ## NOTE - this needs to be updated later to account for more event cases
            self.exptparams = self.rabiparams

        self._initialize_pulsed_plots()
        self.pi2_pulse = np.round(self.pi_pulse / 2)
        self.earlyStop = False

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
    def get_streamer_status(self):
        self.streamer_status = self.pulsegenerator().get_connection_status()
        return self.streamer_status

    #######################################################################
    ###             Pulsed methods                          ###
    #######################################################################

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
        # TODO: should we do this?
        # self.uw_frequency = dict_to_pass['cw_mw_frequency']
        # self.uw_power = dict_to_pass['cw_mw_power']
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

    #######################################################################
    ###             Experiment methods                          ###
    #######################################################################

    # TODO: consider removing this unused function.
    def set_iscw(self, iscw):
        if iscw is not None:
            self.iscw = iscw
        return

    def set_expt(self,exptname):
        if exptname is not None:
            self.expt_current = exptname
        return

    def set_autosave(self,saveon):
        self.autosave = saveon
        return

    def set_autotrack(self,trackon,tracknumber):
        self._autotrack = trackon
        self._trackevery = tracknumber


    def set_timing(self,pulselengths=None,pulseconfigs=None):
        if pulselengths is not None:
            self.pulselengths = pulselengths
            update_dict = {'Pulse_lengths': self.pulselengths}
            self.sigParameterUpdated.emit(update_dict)

        if pulseconfigs is not None:
            self.pulseconfigs = pulseconfigs
            update_dict = {'Pulse_config': self.pulseconfigs}
            self.sigParameterUpdated.emit(update_dict)
        return

    def set_pi_pulse(self,pi_length=None):
        if pi_length is not None:
            self.pi_pulse = pi_length
            self.pi2_pulse = np.round(self.pi_pulse/2)
            update_dict = {'Pi_pulse': self.pi_pulse}
            self.sigParameterUpdated.emit(update_dict)
        return

    def change_fname(self,fname=None):
        if fname is not None:
            self.fname = fname
        return

    def set_cw(self, cwparams=None):
        if cwparams is not None:
            self.cwparams = cwparams
        update_dict = {'CW_params': self.cwparams}
        self.sigParameterUpdated.emit(update_dict)
        self.log.info("set_cw(): self.cwparams = {}".format(self.cwparams))
        if type(self.cwparams[4]) is not int:
            self.log.error("self.cwparams[4]) is not int: '{}'".format(self.cwparams[4]))
        return

    def set_rabi(self, rabiparams=None):
        if rabiparams is not None:
            self.rabiparams = rabiparams
        update_dict = {'Rabi_params': self.rabiparams}
        self.sigParameterUpdated.emit(update_dict)
        return

    def set_ramsey(self,ramseyparams=None):
        if ramseyparams is not None:
            self.ramseyparams = ramseyparams
        update_dict = {'Ramsey_params': self.ramseyparams}
        self.sigParameterUpdated.emit(update_dict)
        return

    def set_hahn(self,hahnparams=None):
        if hahnparams is not None:
            self.hahnparams = hahnparams
        update_dict = {'Hahn_params': self.hahnparams}
        self.sigParameterUpdated.emit(update_dict)
        return

    def set_t1(self,t1params=None):
        if t1params is not None:
            self.t1params = t1params
        update_dict = {'T1_params': self.t1params}
        self.sigParameterUpdated.emit(update_dict)
        return

    def set_odmr(self,odmrparams=None):
        if odmrparams is not None:
            self.odmrparams = odmrparams
        update_dict = {'ODMR_params': self.odmrparams}
        self.sigParameterUpdated.emit(update_dict)
        return

    def start_pulsed_scan(self):

        with self.threadlock:
            if self.module_state() == 'locked':
                self.log.error('Can not start another scan. Logic is already locked.')
                return -1

            self.module_state.lock()
            self.stopRequested = False
            self.earlyStop = False
            self.elapsed_sweeps = 0
            # Update these so the scan headers are more likely to be correct.
            self.uw_power = self.odmrlogic1().cw_mw_power
            self.uw_frequency = self.odmrlogic1().cw_mw_frequency

            # Put in statement to read all of the current values from spinboxes


            if self.expt_current == 'Rabi':
                self.scanvar = 'Time'
                self.exptparams = self.rabiparams
                rabi_status = self._setup_rabi()
                if rabi_status < 0:
                    self.module_state.unlock()
                    print("Issue connecting - will put proper error handling in later")
                ########
                self.mw_cw_on()
                ########
            elif self.expt_current == 'Pulsed ODMR':
                self.scanvar = 'Freq'
                self.exptparams = self.odmrparams
                odmr_status = self._setup_odmr()
                if odmr_status < 0:
                    self.module_state.unlock()
                    print("Issue setting up. Investigate")
                #####
            elif self.expt_current == 'Ramsey':
                self.scanvar = 'Time'
                self.exptparams = self.ramseyparams
                ramsey_status = self._setup_ramsey()
                if ramsey_status < 0:
                    self.module_state.unlock()
                    print("Issue connecting - will put proper error handling in later")
                ########
                self.mw_cw_on()
                ########
            elif self.expt_current == 'Hahn Echo':
                self.scanvar = 'Time'
                self.exptparams = self.hahnparams
                hahn_status = self._setup_hahn()
                if hahn_status < 0:
                    self.module_state_unlock()
                    print('Issue with Hahn Echo - stop and think')
                ##
                self.mw_cw_on()
                ##
            elif self.expt_current == 'CW ODMR':
                self.scanvar = 'Freq'
                self.exptparams = self.cwparams
                odmr_status = self._setup_cw()
                if odmr_status < 0:
                    self.module_state.unlock()
                    print("Issue setting up. Investigate")
            elif self.expt_current == 'T1':
            # right now this is an all-optical T1 implementation
                self.scanvar = 'Time'
                self.exptparams = self.t1params
                t1_status = self._setup_t1()
                if t1_status < 0:
                    self.module_state.unlock()
                    print("Issue connecting - will put proper error handling in later")
            else:
                self.stopRequested = True
                print("I don't know what that experiment is")

            self._initialize_pulsed_plots()
            self.exptrunning = self.expt_current

            self.number_of_lines = self.exptparams[4]
            self.log.info("start_pulsed_scan(): self.number_of_lines = {}".format(self.number_of_lines))
            self.pulsed_raw_data = np.zeros((3, self.final_sweep_list.size, self.number_of_lines)) # will save sig, ref, and sig/ref
            # self.tt_output = np.zeros(
            #     (int(self.number_of_lines), int(self.totaltime+1), int(self.final_sweep_list.size)))
            self.sigExptRunningUpdated.emit(True)
            self.sigNextLinePulse.emit()
        return

    def stop_pulsed_scan(self):
        """ Stop the ODMR scan.
        @return int: error code (0:OK, -1:error)
        """
        with self.threadlock:
            if self.module_state() == 'locked':
                self.stopRequested = True
                self.earlyStop = True
        return 0

    def get_sigref(self, fromcounter, histogram):
        if histogram:
            if self.exptrunning == 'CW ODMR':
                st_inds = [0, int(self.cwparams[5] * 1e9) - 1]
                end_inds = [int(self.cwparams[5] * 1e9), int(2 * self.cwparams[5] * 1e9) - 1]
            else:
                st_inds = [100, int(100 + self.pulselengths[3])]
                end_inds = [int(self.pulselengths[2] - self.pulselengths[3]), int(self.pulselengths[2])]
            signal =    np.mean(fromcounter[0, st_inds[0]:st_inds[1]])
            reference = np.mean(fromcounter[0, end_inds[0]:end_inds[1]])
            sig_over_ref = signal/reference
            pulsed_raw_data = [signal, reference, sig_over_ref]
        else:
            signal, reference = from_counter
            sig_over_ref = signal / reference
            pulsed_raw_data = [signal, reference, sig_over_ref]

        return pulsed_raw_data

    def _scan_pulse_line(self):
        with self.threadlock:
            if self.module_state() != 'locked':
                return

            if self.stopRequested:
                self.stopRequested = False
                self.mw_off()
                if self.autosave:
                    self.save_pulsed_data(tag=self.fname)
                self.sigExptRunningUpdated.emit(False)
                self.module_state.unlock()
                return

            if self._autotrack:
                if np.mod(self.elapsed_sweeps, self._trackevery) == 0:
                    if self.scanvar == 'Time':
                        self.pulsegenerator().set_continuous_out([1, 1, 1, 0, 0, 0, 0, 0])
                    self.module_state.unlock()
                    if self.optimiserlogic().module_state() == 'idle':
                        self.optimiserlogic().start_refocus(caller_tag='tracking')
                        time.sleep(12) # note - we really should replace this with something which waits for the signal from the optimizer
                    self.module_state.lock()
                    #do some tracking

                #do some tracking

            # This needs a case for scanning frequency - mostly for pulsed ODMR


            if self.scanvar == 'Time':
                for k, tau in enumerate(self.final_sweep_list):
                    self.sequence_dict['Channels'] = self.pulseconfigs
                    if self.exptrunning == 'Rabi':
                        self.sequence_dict['Levels'] = self.rabi_sequence(tau, self.final_sweep_list.max())
                    elif self.exptrunning == 'Ramsey':
                        self.sequence_dict['Levels'] = self.ramsey_sequence(tau,self.final_sweep_list.max())
                    elif self.exptrunning == 'Hahn Echo':
                        self.sequence_dict['Levels'] = self.hahn_sequence(tau,self.final_sweep_list.max())
                    elif self.exptrunning == 'T1':
                        self.sequence_dict['Levels'] = self.t1_sequence(tau,self.final_sweep_list.max())
                    self.pulsegenerator().direct_write(self.sequence_dict)
                    self.pulsegenerator().pulser_on()
                    # TODO: make the PulseStreamer output high when we want to measure signal
                    # TODO: make the PulseStreamer output high when we want to measure reference
                    self.fromcounter = self.fastcounter().measure_for(self.exptparams[3])
                    self.pulsed_raw_data[:, k, self.elapsed_sweeps] = self.get_sigref(self.fromcounter, histogram=True)

            elif self.scanvar == 'Freq':
                self.odmrlogic1()._mw_device.reset_sweeppos()
                for k in range(int(self.final_sweep_list.shape[0])):
                    self.odmrlogic1()._mw_device.trigger()
                    # TODO: make the PulseStreamer output high when we want to measure signal
                    # TODO: make the PulseStreamer output high when we want to measure reference
                    self.fromcounter = self.fastcounter().measure_for(self.exptparams[3])
                    self.pulsed_raw_data[:, k, self.elapsed_sweeps] = self.get_sigref(self.fromcounter, histogram=True)
                self.odmrlogic1()._mw_device.trigger()

            else:
                print('Error: scanvar not correctly set')

            if self.elapsed_sweeps == 0:
                self.sigPulsedPlotsUpdated.emit(self.pulsed_plot_x,
                                                self.pulsed_raw_data[:, :, 0], self.pulsed_raw_data)
            else:
                self.sigPulsedPlotsUpdated.emit(self.pulsed_plot_x,
                                                np.mean(self.pulsed_raw_data[:,:,:self.elapsed_sweeps],2),
                                                self.pulsed_raw_data)

            self.elapsed_sweeps += 1
            if (self.elapsed_sweeps) >= self.exptparams[4]:
                self.stopRequested = True
                self.earlyStop = False



            self.sigNextLinePulse.emit()
        if self.stopRequested and not self.earlyStop:
            # TODO: figure out how to make the GUI create a pop-up at this point rather than throwing an exception.
            self.log.exception("finished scan.")
        return

    ##################################################
    #### PULSED METHODS ARE HERE #####################
    ##################################################

    def _setup_cw(self):
        self.final_sweep_list = np.arange(self.exptparams[0], self.exptparams[1] + self.exptparams[2],
                                          self.exptparams[2])
        self.sequence_dict['Channels'] = self.pulseconfigs
        self.sequence_dict['Levels'] = self.cw_odmr_sequence()
        self.pulsegenerator().direct_write(self.sequence_dict)
        self.pulsegenerator().pulser_on()

        totaltime = 2*self.cwparams[5]*1e9
        self.fastcounter().configure(1.e-9, 1e-9 * totaltime, 1)

        mode, is_running = self.mw_sweep_on(self.cwparams)
        if not is_running:
            print('Error initializing microwave sweep')
            return -1
        return 0

        return 0


    def _setup_rabi(self):
        """" here we need to setup the integration, and probably pre-generate the pulse sequences to feed to the pulse streamer """""
        self.final_sweep_list = np.arange(self.exptparams[0], self.exptparams[1] + self.exptparams[2],
                                          self.exptparams[2])

        self.sequence_dict['Channels'] = self.pulseconfigs
        self.sequence_dict['Levels'] = self.rabi_sequence(10., self.final_sweep_list.max())
        self.pulsegenerator().direct_write(self.sequence_dict)
        self.pulsegenerator().pulser_on()

        self.totaltime = self.pulselengths[2] + self.final_sweep_list.max() + 50
        self.fastcounter().configure(1.e-9,1e-9*self.totaltime,1)
        return 0

    def _setup_ramsey(self):
        self.final_sweep_list = np.arange(self.exptparams[0], self.exptparams[1] + self.exptparams[2],
                                          self.exptparams[2])

        self.sequence_dict['Channels'] = self.pulseconfigs
        self.sequence_dict['Levels'] = self.ramsey_sequence(10., self.final_sweep_list.max())
        self.pulsegenerator().direct_write(self.sequence_dict)
        self.pulsegenerator().pulser_on()

        self.totaltime = self.pulselengths[2] + self.final_sweep_list.max() + 2*self.pi2_pulse + 50
        self.fastcounter().configure(1.e-9, 1e-9 * self.totaltime, 1)
        return 0

    def _setup_hahn(self):
        self.final_sweep_list = np.arange(self.exptparams[0], self.exptparams[1] + self.exptparams[2],
                                          self.exptparams[2])

        self.sequence_dict['Channels'] = self.pulseconfigs
        self.sequence_dict['Levels'] = self.hahn_sequence(10., self.final_sweep_list.max())
        self.pulsegenerator().direct_write(self.sequence_dict)
        self.pulsegenerator().pulser_on()

        self.totaltime = self.pulselengths[2] + 2*self.final_sweep_list.max() + 2*self.pi2_pulse  + self.pi_pulse + 50
        self.fastcounter().configure(1.e-9, 1e-9 * self.totaltime, 1)
        return 0

    def _setup_t1(self):
        self.final_sweep_list = np.arange(self.exptparams[0], self.exptparams[1] + self.exptparams[2],
                                          self.exptparams[2])

        self.sequence_dict['Channels'] = self.pulseconfigs
        self.sequence_dict['Levels'] = self.t1_sequence(10., self.final_sweep_list.max())
        self.pulsegenerator().direct_write(self.sequence_dict)
        self.pulsegenerator().pulser_on()

        self.totaltime = self.pulselengths[2] + self.final_sweep_list.max()
        self.fastcounter().configure(1.e-9, 1e-9 * self.totaltime, 1)
        return 0

    def _setup_odmr(self):
        self.final_sweep_list = np.arange(self.exptparams[0], self.exptparams[1] + self.exptparams[2],
                                          self.exptparams[2])

        self.sequence_dict['Channels'] = self.pulseconfigs
        self.sequence_dict['Levels'] = self.odmr_sequence()
        self.pulsegenerator().direct_write(self.sequence_dict)
        self.pulsegenerator().pulser_on()

        self.totaltime = self.pulselengths[2] + self.pi_pulse + 50.
        self.fastcounter().configure(1.e-9, 1e-9 * self.totaltime, 1)

        mode, is_running = self.mw_sweep_on(self.odmrparams)
        if not is_running:
            print('Error initializing microwave sweep')
            return -1
        return 0

    ### Sequence definitions

    def rabi_sequence(self,tau=100,taumax=2000):
        totallength = self.pulselengths[2] + taumax + 100
        sync_patt = [(100,1),(int(totallength-100),0)]
        mw_patt = [(int(self.pulselengths[2]+50),0),(int(tau),1),(int(totallength-self.pulselengths[2]-tau),0)]
        laser_patt = [(self.pulselengths[2], 1), (int(totallength - self.pulselengths[2] + 50), 0)]

        laser_rle = self.traj_to_rle(np.roll(self.rle_to_traj(laser_patt), int(-1*self.pulselengths[0])))
        mw_rle = self.traj_to_rle(np.roll(self.rle_to_traj(mw_patt),int(self.pulselengths[1])))
        sync_rle = sync_patt

        return [laser_rle,mw_rle,sync_rle]

    def ramsey_sequence(self,tau=100,taumax=2000):
        totallength = self.pulselengths[2] + taumax + 2*self.pi2_pulse + 100
        sync_patt = [(100, 1), (int(totallength - 100), 0)]
        mw_patt = [(int(self.pulselengths[2] + 50), 0), (int(self.pi2_pulse), 1), (int(tau), 0),
                   (int(self.pi2_pulse), 1),
                   (int(totallength - self.pulselengths[2] - 2 * self.pi2_pulse - tau), 0)]
        laser_patt = [(self.pulselengths[2], 1), (int(totallength - self.pulselengths[2] + 50), 0)]

        laser_rle = self.traj_to_rle(np.roll(self.rle_to_traj(laser_patt), int(-1 * self.pulselengths[0])))
        mw_rle = self.traj_to_rle(np.roll(self.rle_to_traj(mw_patt), int(self.pulselengths[1])))
        sync_rle = sync_patt

        return [laser_rle, mw_rle, sync_rle]

    def hahn_sequence(self,tau=100,taumax=2000):
        totallength = self.pulselengths[2] + 2*taumax + 2*self.pi2_pulse  + self.pi_pulse + 100
        sync_patt = [(100, 1), (int(totallength - 100), 0)]
        mw_patt = [(int(self.pulselengths[2] + 50), 0), (int(self.pi2_pulse), 1), (int(tau), 0), (int(self.pi_pulse),1), (int(tau),0),
                   (int(self.pi2_pulse), 1),
                   (int(totallength - self.pulselengths[2] - 2 * self.pi2_pulse - 2*tau - self.pi_pulse), 0)]
        laser_patt = [(self.pulselengths[2], 1), (int(totallength - self.pulselengths[2]+100), 0)]

        laser_rle = self.traj_to_rle(np.roll(self.rle_to_traj(laser_patt), int(-1 * self.pulselengths[0])))
        mw_rle = self.traj_to_rle(np.roll(self.rle_to_traj(mw_patt), int(self.pulselengths[1])))
        sync_rle = sync_patt

        return [laser_rle, mw_rle, sync_rle]

    def t1_sequence(self,tau=100,taumax=2000):
        totallength = self.pulselengths[2] + tau
        sync_patt = [(100, 1), (int(totallength - 100), 0)]
        mw_patt = [(10, 1), (int(totallength - 10), 0)]
        laser_patt = [(self.pulselengths[2], 1), (int(tau), 0)]

        laser_rle = self.traj_to_rle(np.roll(self.rle_to_traj(laser_patt), int(-1 * self.pulselengths[0])))
        mw_rle = self.traj_to_rle(np.roll(self.rle_to_traj(mw_patt), int(self.pulselengths[1])))
        sync_rle = sync_patt

        return [laser_rle, mw_rle, sync_rle]

    def odmr_sequence(self):
        totallength = self.pulselengths[2] + self.pi_pulse + 100.
        sync_patt = [(100, 1), (int(totallength - 100), 0)]
        mw_patt = [(int(self.pulselengths[2] + 50), 0), (int(self.pi_pulse), 1)]
        laser_patt = [(self.pulselengths[2], 1), (int(totallength - self.pulselengths[2]+100), 0)]

        laser_rle = self.traj_to_rle(np.roll(self.rle_to_traj(laser_patt), int(-1 * self.pulselengths[0])))
        mw_rle = self.traj_to_rle(np.roll(self.rle_to_traj(mw_patt), int(self.pulselengths[1])))
        sync_rle = sync_patt

        return [laser_rle, mw_rle, sync_rle]

    def cw_odmr_sequence(self):
        totallength = 2*self.cwparams[5]*1e9
        sync_patt = [(100, 1), (int(totallength - 100), 0)]
        mw_patt = [(int(self.cwparams[5]*1e9),1),(int(self.cwparams[5]*1e9),0)]
        laser_patt = [(int(totallength-10),1),(int(10),0)]
        signal_patt = mw_patt
        ref_patt = [(int(self.cwparams[5]*1e9),0),(int(self.cwparams[5]*1e9),1)] # inverse of signal_patt

        laser_rle = self.traj_to_rle(np.roll(self.rle_to_traj(laser_patt), int(-1 * self.pulselengths[0])))
        mw_rle = self.traj_to_rle(np.roll(self.rle_to_traj(mw_patt), int(self.pulselengths[1])))
        sync_rle = sync_patt

        return [laser_rle, mw_rle, sync_rle]

    ## Helpful functions

    def _initialize_pulsed_plots(self):
        self.final_sweep_list = np.arange(self.exptparams[0], self.exptparams[1] + self.exptparams[2],
                                          self.exptparams[2])
        self.number_of_lines = int(self.exptparams[4])

        self.pulsed_plot_x = np.array(self.final_sweep_list)
        self.pulsed_plot_y = np.zeros([self.pulsed_plot_x.size, ])
        self.pulsed_plot_xy = np.zeros([self.pulsed_plot_x.size,self.number_of_lines])
        return

    def mw_sweep_on(self,params):
        self.uw_power = self.odmrlogic1().cw_mw_power
        mw_start = params[0]
        mw_stop = params[1]
        mw_step = params[2]

        sweep_return = self.odmrlogic1()._mw_device.set_sweep(
            mw_start, mw_stop, mw_step, self.uw_power)

        err_code = self.odmrlogic1()._mw_device.sweep_on()
        if err_code < 0:
            self.log.error('Activation of microwave output failed.')

        mode, is_running = self.odmrlogic1()._mw_device.get_status()

        return mode, is_running



    ## Helper functions for pulsestreamer
    def rle_to_traj(self,patt):
        totallength = 0.
        for elem in patt:
            totallength += float(elem[0])
        #
        timeax = np.linspace(0, totallength - 0.1, int(totallength))
        hilo = np.zeros_like(timeax)
        stind = 0
        for elem in patt:
            endind = stind + int(elem[0])
            hilo[stind:endind] = int(elem[1])
            stind = endind
        return hilo

    def traj_to_rle(self,traj):
        totaltime = np.size(traj)
        stepinds = np.argwhere(np.abs(np.diff(traj)) > 0.5)
        patt = [(int(stepinds[0] + 1), traj[0])]
        for n in range(1, np.size(stepinds)):
            patt.append((int(stepinds[int(n)] - stepinds[int(n - 1)]), int(traj[int(stepinds[n] - 1)])))
        patt.append((int(totaltime - stepinds[-1] - 1), traj[-1]))
        return patt

    def save_pulsed_data(self, tag=None, colorscale_range=None, percentile_range=None):
        timestamp = datetime.datetime.now()
        filepath = self._save_logic.get_path_for_module(module_name='Pulsed')

        if tag is None:
            tag = ''

        expt_add = (self.exptrunning).replace(" ","")
        filelabel_raw = '{0}_Pulsed_'.format(tag) + expt_add + '_raw'

        data_avg = OrderedDict()


        # General experiment parameters
        parameters = OrderedDict()
        parameters['Microwave Power (dBm)'] = self.uw_power
        parameters['Dwell Time (s)'] = self.exptparams[3]
        parameters['Number of Averages (#)'] = self.exptparams[4]

        if self.exptrunning == 'Rabi':
            parameters['Microwave Frequency (Hz)'] = self.uw_frequency
            parameters['Rabi Start (ns)'] = self.rabiparams[0]
            parameters['Rabi Stop (ns)'] = self.rabiparams[1]
            parameters['Rabi Step (ns)'] = self.rabiparams[2]
            parameters['Experiment Type'] = 'Rabi'
            data_avg['Pulse Length (ns)'] = self.final_sweep_list
        elif self.exptrunning == 'Ramsey':
            parameters['Microwave Frequency (Hz)'] = self.uw_frequency
            parameters['Ramsey Start (ns)'] = self.ramseyparams[0]
            parameters['Ramsey Stop (ns)'] = self.ramseyparams[1]
            parameters['Ramsey Step (ns)'] = self.ramseyparams[2]
            parameters['Experiment Type'] = 'Ramsey'
            data_avg['Pulse Delay (ns)'] = self.final_sweep_list
        elif self.exptrunning == 'Hahn Echo':
            parameters['Microwave Frequency (Hz)'] = self.uw_frequency
            parameters['Echo Start (ns)'] = self.ramseyparams[0]
            parameters['Echo Stop (ns)'] = self.ramseyparams[1]
            parameters['Echo Step (ns)'] = self.ramseyparams[2]
            parameters['Experiment Type'] = 'Hahn Echo'
            data_avg['Pulse Delay (ns)'] = self.final_sweep_list
        elif self.exptrunning == 'T1':
            parameters['T1 Start (ns)'] = self.t1params[0]
            parameters['T1 Stop (ns)'] = self.t1params[1]
            parameters['T1 Step (ns)'] = self.t1params[2]
            parameters['Experiment Type'] = 'T1'
            data_avg['Pulse Delay (ns)'] = self.final_sweep_list
        elif self.exptrunning == 'CW ODMR' or self.exptrunning == 'Pulsed ODMR':
            data_avg['Frequency Axis'] = self.final_sweep_list
        # TODO: add Pi pulse duration to Ramsey and pulsed ODMR

        data_raw = OrderedDict()
        data_raw['Signal data (counts/s)'] = (self.pulsed_raw_data[0, :, :].squeeze()).T
        filelabel_raw = '{0}_Pulsed_'.format(tag) + expt_add + '_rawSig'
        self._save_logic.save_data(data_raw,
                                   filepath=filepath,
                                   parameters=parameters,
                                   filelabel=filelabel_raw,
                                   fmt='%.6e',
                                   delimiter='\t',
                                   timestamp=timestamp)


        data_raw = OrderedDict()
        data_raw['Reference data (counts/s)'] = (self.pulsed_raw_data[1, :, :].squeeze()).T
        filelabel_raw = '{0}_Pulsed_'.format(tag) + expt_add + '_rawRef'
        self._save_logic.save_data(data_raw,
                                   filepath=filepath,
                                   parameters=parameters,
                                   filelabel=filelabel_raw,
                                   fmt='%.6e',
                                   delimiter='\t',
                                   timestamp=timestamp)

        data_raw = OrderedDict()
        data_raw['Normalized data (AU)'] = (self.pulsed_raw_data[2, :, :].squeeze()).T
        filelabel_raw = '{0}_Pulsed_'.format(tag) + expt_add + '_rawNorm'
        self._save_logic.save_data(data_raw,
                                   filepath=filepath,
                                   parameters=parameters,
                                   filelabel=filelabel_raw,
                                   fmt='%.6e',
                                   delimiter='\t',
                                   timestamp=timestamp)



        filelabel_avg = 'Pulsed_' + expt_add
        data_avg['Signal data (counts/s)'] = np.mean(self.pulsed_raw_data[0, :, :], 1)
        data_avg['Reference data (counts/s)'] = np.mean(self.pulsed_raw_data[1, :, :], 1)
        data_avg['Normalized data (AU)'] = np.mean(self.pulsed_raw_data[2, :, :], 1)

        self._save_logic.save_data(data_avg,
                                   filepath=filepath,
                                   parameters=parameters,
                                   filelabel=filelabel_avg,
                                   fmt='%.6e',
                                   delimiter='\t',
                                   timestamp=timestamp)

        # filepath_TT = self._save_logic.get_path_for_module(module_name='TT_raw')
        # if tag is None:
        #     tag = ''
        # expt_add = (self.exptrunning).replace(" ", "")
        # filename_raw_TT = timestamp.strftime('%Y%m%d-%H%M-%S') + '_' + '{0}_Pulsed_'.format(tag) + expt_add + '_TT' + '.npy'
        # fullpath_TT = os.path.join(filepath_TT, filename_raw_TT)
        # # _save_logic.save_data does not permit 3D arrays, see e.g.
        # # "Found data array with dimension >2. Unable to save data."
        # with open(fullpath_TT, 'wb') as fp:
        #     np.save(fp, self.tt_output)
        return




