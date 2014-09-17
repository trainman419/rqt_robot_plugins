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
#
# TODO:
#   this needs to change pretty considerably
#
#   right now, each instance maintains its own history. this means that
#   each inspector window starts with zero history
#
#   the history should instead be global, and each timeline should have
#   it's own view of the history. I believe this is how the old rx version
#   worked

from collections import deque
from math import floor
import os

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Signal, Slot
from python_qt_binding.QtGui import QGraphicsScene, QWidget
import rospy
import rospkg

from .timeline import Timeline
from .timeline_view import TimelineView

SECONDS_TIMELINE = 30

class TimelinePane(QWidget):
    """
    This class defines the pane where timeline and its related components
    are displayed.
    """

    sig_update = Signal()

    def __init__(self, parent):
        """
        Because this class is intended to be instantiated via Qt's .ui file,
        taking argument other than parent widget is not possible, which is
        ported to set_timeline_data method. That said, set_timeline_data must
        be called (soon) after an object of this is instantiated.
        """
        super(TimelinePane, self).__init__()
        self._parent = parent
        self._timeline = None
        self._last_sec_marker_at = 2

        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_robot_monitor'),
                               'resource',
                               'timelinepane.ui')
        loadUi(ui_file, self)

        self._scene = QGraphicsScene(self._timeline_view)
        self._timeline_view.set_init_data(1, SECONDS_TIMELINE, 5)
        self._timeline_view.setScene(self._scene)
        self._timeline_view.show()

        self.sig_update.connect(self._timeline_view.slot_redraw)

    def set_timeline(self, timeline, name=None):
        assert(self._timeline is None)
        self._timeline = timeline
        self._timeline.message_updated.connect(self.updated)
        self._timeline_view.set_timeline(timeline, name)
        self._pause_button.clicked[bool].connect(self._timeline.set_paused)
        self._timeline.pause_changed[bool].connect(
                self._pause_button.setChecked)

        # bootstrap initial state
        self._pause_button.setChecked(self._timeline.paused)
        self.sig_update.emit()

    def mouse_release(self, event):
        """
        :type event: QMouseEvent
        """
        assert(self._timeline is not None)
        xpos_clicked = event.x()
        width_each_cell_shown = float(
                       self._timeline_view.viewport().width()) / len(
                                                   self._timeline)
        i = int(floor(xpos_clicked / width_each_cell_shown))
        rospy.logdebug('mouse_release i=%d width_each_cell_shown=%s',
                       i, width_each_cell_shown)

        self._timeline.set_position(i)

    def on_slider_scroll(self, evt):
        """

        :type evt: QMouseEvent
        """
        assert(self._timeline is not None)

        xpos_marker = self._timeline_view.get_xpos_marker() - 1
        rospy.logdebug('on_slider_scroll xpos_marker=%s last_sec_marker_at=%s',
                      xpos_marker, self._last_sec_marker_at)
        if xpos_marker == self._last_sec_marker_at:
            # Clicked the same pos as last time.
            return
        elif xpos_marker >= len(self._timeline):
            # When clicked out-of-region
            return

        self._last_sec_marker_at = xpos_marker

        self._timeline.set_paused(True)

        # Fetch corresponding previous DiagsnoticArray instance from queue,
        # and sig_update trees.
        self._timeline.set_position(xpos_marker)

    @Slot()
    def updated(self):
        """ Slot that should be called whenever the underlying Timeline object
        is updated
        """
        assert(self._timeline is not None)
        self._timeline_view.set_range(1, len(self._timeline))
        self.sig_update.emit()

    def redraw(self):
        self.sig_update.emit()
