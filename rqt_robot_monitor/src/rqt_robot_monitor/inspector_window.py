# Software License Agreement (BSD License)
#
# Copyright (c) 2012, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Isaac Saito, Ze'ev Klapow

from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QPushButton, QTextEdit, QVBoxLayout
import rospy

from .abst_status_widget import AbstractStatusWidget
from .status_snapshot import StatusSnapshot, level_to_text
from .time_pane import TimelinePane
import util_robot_monitor as util


class InspectorWindow(AbstractStatusWidget):
    _sig_close_window = Signal()

    def __init__(self, status, close_callback):
        """
        :type status: DiagnosticStatus
        :param close_callback: When the instance of this class
                               (InspectorWindow) terminates, this callback gets
                               called.
        """
        #TODO(Isaac) UI construction that currently is done in this method,
        #            needs to be done in .ui file.

        super(InspectorWindow, self).__init__()
        self.status = status
        self._close_callback = close_callback
        self.setWindowTitle(status.name)
        self.paused = False

        self.layout_vertical = QVBoxLayout(self)

        self.disp = StatusSnapshot(parent=self)
        self.snapshot = QPushButton("StatusSnapshot")

        self.timeline_pane = TimelinePane(self)
        self.timeline_pane.set_timeline_data(util.SECONDS_TIMELINE,
                                             self.get_color_for_value,
                                             self.on_pause)

        self.layout_vertical.addWidget(self.disp, 1)
        self.layout_vertical.addWidget(self.timeline_pane, 0)
        self.layout_vertical.addWidget(self.snapshot)

        self.snaps = []
        self.snapshot.clicked.connect(self._take_snapshot)

        self._sig_close_window.connect(self._close_callback)

        self.setLayout(self.layout_vertical)
        # TODO better to be configurable where to appear.
        self.resize(400, 600)
        self.show()
        self.update_status_display(status)

    def closeEvent(self, event):
        # emit signal that should be slotted by StatusItem
        self._sig_close_window.emit()
        self.close()

    def pause(self, msg):
        rospy.logdebug('InspectorWin pause PAUSED')
        self.paused = True
        self.update_status_display(msg)

    def unpause(self, msg):
        rospy.logdebug('InspectorWin pause UN-PAUSED')
        self.paused = False

    def new_diagnostic(self, msg, is_forced=False):
        """
        Overridden from AbstractStatusWidget

        :type status: DiagnosticsStatus
        """
        raise Exception() # TODO(ahendrix): I think this is unused. prove it

        if not self.paused:
            self.update_status_display(msg)
            rospy.logdebug('InspectorWin _cb len of queue=%d self.paused=%s',
                          len(self.timeline_pane._queue_diagnostic),
                          self.paused)
        else:
            if is_forced:
                self.update_status_display(msg, True)
                rospy.logdebug('@@@InspectorWin _cb PAUSED window updated')
            else:
                rospy.logdebug('@@@InspectorWin _cb PAUSED not updated')

    def update_status_display(self, status, is_forced=False):
        """
        :type status: DiagnosticsStatus
        """

        if not self.paused or (self.paused and is_forced):
            scroll_value = self.disp.verticalScrollBar().value()
            self.timeline_pane.new_diagnostic(status)

            rospy.logdebug('InspectorWin update_status_display 1')

            self.status = status
            self.disp.write_status.emit(status)

            if self.disp.verticalScrollBar().maximum() < scroll_value:
                scroll_value = self.disp.verticalScrollBar().maximum()
            self.disp.verticalScrollBar().setValue(scroll_value)

    def _take_snapshot(self):
        snap = StatusSnapshot(status=self.status)
        self.snaps.append(snap)

    def get_color_for_value(self, queue_diagnostic, color_index):
        """
        Overridden from AbstractStatusWidget.

        :type color_index: int
        """

        rospy.logdebug('InspectorWindow get_color_for_value ' +
                       'queue_diagnostic=%d, color_index=%d',
                       len(queue_diagnostic), color_index)
        lv_index = queue_diagnostic[color_index - 1].level
        return util.COLOR_DICT[lv_index]
