#!/usr/bin/env python
# QTVcp Widget - MDI edit line widget
#
# Copyright (c) 2017 Chris Morley
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

import os
from PyQt5.QtWidgets import QLineEdit
from PyQt5.QtCore import Qt, QEvent
from qtvcp.core import Status, Action, Info
from qtvcp.lib.aux_program_loader import Aux_program_loader

# Instiniate the libraries with global reference
# STATUS gives us status messages from linuxcnc
# AUX_PRGM holds helper program loadr
# INI holds ini details
# ACTION gives commands to linuxcnc
STATUS = Status()
AUX_PRGM = Aux_program_loader()
INFO = Info()
ACTION = Action()

# Set up logging
from qtvcp import logger
log = logger.getLogger(__name__)


class Lcnc_MDILine(QLineEdit):
    def __init__(self, parent = None):
        QLineEdit.__init__(self,parent)

        STATUS.connect('state-off', lambda w: self.setEnabled(False))
        STATUS.connect('state-estop', lambda w: self.setEnabled(False))
        STATUS.connect('interp-idle', lambda w: self.setEnabled(STATUS.machine_is_on() and ( STATUS.is_all_homed() or INFO.NO_HOME_REQUIRED ) ))
        STATUS.connect('interp-run', lambda w: self.setEnabled(not STATUS.is_auto_mode() ) )
        STATUS.connect('all-homed', lambda w: self.setEnabled(STATUS.machine_is_on() ) )
        STATUS.connect('mdi-line-selected', self.external_line_selected)
        self.returnPressed.connect(self.submit)

    def submit(self):
        text = str(self.text()).strip()
        if text == '':return
        if text == 'HALMETER':
            AUX_PRGM.load_halmeter()
        elif text == 'STATUS':
            AUX_PRGM.load_status()
        elif text == 'HALSHOW':
            AUX_PRGM.load_halshow()
        elif text == 'CLASSICLADDER':
            AUX_PRGM.load_ladder()
        elif text == 'HALSCOPE':
            AUX_PRGM.load_halscope()
        elif text == 'CALIBRATION':
            AUX_PRGM.load_calibration(self.inifile)
        else:
            ACTION.CALL_MDI(text+'\n')
            try:
                fp = os.path.expanduser(INFO.MDI_HISTORY_PATH)
                fp = open(fp, 'a')
                fp.write(text + "\n")
                fp.close()
            except:
                pass
            STATUS.emit('reload-mdi-history')

    # Gcode widget can emit a signal to this
    def external_line_selected(self, w, text, filename):
        log.debug('Ext line selected: {}, {}'.format(text, filename))
        if filename == INFO.MDI_HISTORY_PATH:
            self.setText(text)

    def keyPressEvent(self, event):
        super(Lcnc_MDILine, self).keyPressEvent(event)
        if event.key() == Qt.Key_Up:
            log.debug('up')
            STATUS.emit('move-text-lineup')
        if event.key() == Qt.Key_Down:
            log.debug('down')
            STATUS.emit('move-text-linedown')

# for testing without editor:
def main():
    import sys
    from PyQt4.QtGui import QApplication

    app = QApplication(sys.argv)
    widget = Lcnc_MDILine()
    widget.show()
    sys.exit(app.exec_())
if __name__ == "__main__":
    main()


