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
# Author: Isaac Saito, Ze'ev Klapow, Austin Hendrix

from math import floor
import rospy

from python_qt_binding.QtCore import QPointF, Signal, Slot
from python_qt_binding.QtGui import (QColor, QGraphicsPixmapItem,
                                     QGraphicsView, QIcon, QGraphicsScene)

import util_robot_monitor as util


class TimelineView(QGraphicsView):
    """
    This class draws a graphical representation of a timeline.

    This is ONLY the bar and colored boxes.

    When you instantiate this class, do NOT forget to call set_init_data to
    set necessary data.
    """

    _sig_update = Signal()

    MIN_NUM_SECONDS = 1
    MAX_NUM_SECONDS = 30

    def __init__(self, parent):
        """Cannot take args other than parent due to loadUi limitation."""

        super(TimelineView, self).__init__()
        self._parent = parent
        self._timeline_marker = QIcon.fromTheme('system-search')

        self._min_num_seconds = self.MIN_NUM_SECONDS
        self._max_num_seconds = self.MAX_NUM_SECONDS
        self._xpos_marker = 5

        self._timeline_marker_width = 15
        self._timeline_marker_height = 15
        self._last_marker_at = 2

        self._sig_update.connect(self.slot_redraw)

        self._timeline = None

        self.setUpdatesEnabled(True)
        self._scene = QGraphicsScene(self)
        self.setScene(self._scene)

    def set_timeline(self, timeline, name=None):
        assert(self._timeline is None)
        self._name = name
        self._timeline = timeline
        # TODO(ahendrix): connect this to something?
        #self._timeline.message_updated.connect(self.updated)

    def set_range(self, min_val, max_val):
        """
        :param min_val: Smallest second on timeline.
        :param max_val: Largest second on timeline.
        """
        self._min_num_seconds = min_val
        self._max_num_seconds = max_val
        rospy.logdebug(' TimelineView set_range _min_num_seconds=%s max=%s',
                       self._min_num_seconds,
                       self._max_num_seconds)

        # when setting the range, set the position to the end?
        self._xpos_marker = self._clamp(len(self._timeline),
                                       self._min_num_seconds,
                                       self._max_num_seconds)

    def mouseReleaseEvent(self, event):
        """
        :type event: QMouseEvent
        """
        assert(self._timeline is not None)

        xpos_clicked = event.x()
        width_each_cell_shown = float(self.viewport().width()) / len(self._timeline)
        i = int(floor(xpos_clicked / width_each_cell_shown))
        rospy.loginfo('mouse_release i=%d width_each_cell_shown=%s',
                       i, width_each_cell_shown)

        self._timeline.set_position(i)

        self.set_val_from_x(event.pos().x())

        # TODO Figure out what's done by this in wx.
        # I suspect this reads the scroll wheel
        # wx.PostEvent(self.GetEventHandler(),
        #             wx.PyCommandEvent(wx.EVT_COMMAND_SCROLL_CHANGED.typeId,
        #                               self.GetId()))

    def mousePressEvent(self, evt):
        """
        :type event: QMouseEvent
        """
        assert(self._timeline is not None)
        self.set_val_from_x(evt.pos().x())

        # TODO Figure out what's done by this in wx.
        # I suspect this reads the scroll wheel
        # wx.PostEvent(self.GetEventHandler(),
        #             wx.PyCommandEvent(wx.EVT_COMMAND_SCROLL_THUMBTRACK.typeId
        #                               self.GetId()))

        xpos_marker = self._xpos_marker - 1
        rospy.loginfo('on_slider_scroll xpos_marker=%s last_sec_marker_at=%s',
                      xpos_marker, self._last_marker_at)
        if xpos_marker == self._last_marker_at:
            # Clicked the same pos as last time.
            return
        elif xpos_marker >= len(self._timeline):
            # When clicked out-of-region
            return

        self._last_marker_at = xpos_marker

        self._timeline.set_paused(True)

        # Fetch corresponding previous DiagsnoticArray instance from queue,
        # and sig_update trees.
        self._timeline.set_position(xpos_marker)


    def set_val_from_x(self, x):
        """
        Called when marker is moved by user.

        :param x: Position relative to self widget.
        """
        qsize = self.size()
        width = qsize.width()
        # determine value from mouse click
        length_tl_in_second = self._max_num_seconds + 1 - self._min_num_seconds
        width_cell = width / float(length_tl_in_second)
        x_marker_float = x / width_cell + 1
        self.set_marker_pos(x_marker_float)
        rospy.loginfo('TimelineView set_val_from_x x=%s width_cell=%s ' +
                      'length_tl_in_second=%s set_marker_pos=%s',
                      x, width_cell, length_tl_in_second, self._xpos_marker)

    def set_marker_pos(self, val):
        self._xpos_marker = self._clamp(int(val),
                                        self._min_num_seconds,
                                        self._max_num_seconds)
        self._sig_update.emit()

    def _clamp(self, val, min, max):
        """
        Judge if val is within the range given by min & max.
        If not, return either min or max.

        :type val: any number format
        :type min: any number format
        :type max: any number format
        :rtype: int
        """
        if (val < min):
            return min
        if (val > max):
            return max
        return val

    @Slot()
    def slot_redraw(self):
        """
        Gets called either when new msg comes in or when marker is moved by
        user.
        """

        self._scene.clear()

        qsize = self.size()
        width_tl = qsize.width()

        length_tl = ((self._max_num_seconds + 1) -
                     self._min_num_seconds)

        len_queue = length_tl
        w = width_tl / float(len_queue)
        is_enabled = self.isEnabled()


        if self._timeline is not None:
            for i, m in enumerate(self._timeline):
                h = self.viewport().height()

                # Figure out each cell's color.
                qcolor = QColor('grey')
                if is_enabled:
                    qcolor = self.get_color_for_value(m)

#  TODO Use this code for adding gradation to the cell color.
#                end_color = QColor(0.5 * QColor('red').value(),
#                                   0.5 * QColor('green').value(),
#                                   0.5 * QColor('blue').value())

                self._scene.addRect(w * i, 0, w, h,
                                                   QColor('white'), qcolor)
                rospy.logdebug('slot_redraw #%d th loop w=%s width_tl=%s',
                               i, w, width_tl)

        # Setting marker.
        xpos_marker = ((self._xpos_marker - 1) * w +
                       (w / 2.0) - (self._timeline_marker_width / 2.0))
        pos_marker = QPointF(xpos_marker, 0)

        # Need to instantiate marker everytime since it gets deleted
        # in every loop by scene.clear()
        timeline_marker = self._instantiate_tl_icon()
        timeline_marker.setPos(pos_marker)
        self._scene.addItem(timeline_marker)
        rospy.logdebug(' slot_redraw xpos_marker(int)=%s length_tl=%s',
                       int(xpos_marker), length_tl)

    def _instantiate_tl_icon(self):
        timeline_marker_icon = QIcon.fromTheme('system-search')
        timeline_marker_icon_pixmap = timeline_marker_icon.pixmap(
                                                self._timeline_marker_width,
                                                self._timeline_marker_height)
        return QGraphicsPixmapItem(timeline_marker_icon_pixmap)

    def get_color_for_value(self, msg):
        """
        :type msg: DiagnosticArray
        """

        if self._name is not None:
            # look up name in msg; return grey if not found
            status = util.get_status_by_name(msg, self._name)
            if status is not None:
                return util.level_to_color(status.level)
            else:
                return QColor('grey')
        return util.get_color_for_message(msg)
