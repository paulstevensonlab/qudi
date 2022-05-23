# -*- coding: utf-8 -*-
"""

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
from logic.generic_logic import GenericLogic
from qtpy import QtCore
from core.util.mutex import Mutex
from core.configoption import ConfigOption
from core.statusvariable import StatusVar


class GatedCounterLogic(GenericLogic):
    """
    GatedCounterLogic
    """

    # Declare connectors
    pulsegenerator = Connector(interface='PulserInterface')
    fastcounter = Connector(interface='FastCounterInterface')
    odmrlogic1 = Connector(interface='ODMRLogic')
    savelogic = Connector(interface='SaveLogic')
    optimiserlogic = Connector(interface='OptimizerLogic')



    def __init__(self, config, **kwargs):
        """ Create GatedCounterLogic object with connectors.

          @param dict kwargs: optional parameters
        """
        super().__init__(config=config, **kwargs)
        self.threadlock = Mutex()

        return

    def on_activate(self):
        """ Initialisation performed during activation of the module.
        """

        return

    def on_deactivate(self):
        """

        @return:
        """
        # Disconnect all signals
        # Disconnect signals controlling GatedCounterLogic

        return
