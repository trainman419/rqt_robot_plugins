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

import os
import rospkg

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer, Signal, Qt, Slot
from python_qt_binding.QtGui import QColor, QPalette, QWidget, QTreeWidgetItem
import rospy

#from .chronologic_state import InstantaneousState
from .inspector_window import InspectorWindow
from .chronologic_state import StatusItem
from .timeline_pane import TimelinePane
from .timeline import Timeline
import util_robot_monitor as util

class MyStatusItem(QTreeWidgetItem):
    def __init__(self, name):
        super(MyStatusItem, self).__init__()
        self.name = name

class Status(object):
    """
    A class that wraps the default QTreeWidgetItem, so that we can manipulate
    all of the nodes in the tree in the same way (even the invisible root node)
    """
    def __init__(self, item=None):
        self._children = {}
        self.updated = False
        if item is not None:
            self._item = item
        else:
            self._item = MyStatusItem("NONAME")

    def update(self, status, displayname):
        self.updated = True
        self.displayname = displayname
        self._item.name = status.name
        self._item.setText(0, self.displayname)
        self._item.setIcon(0, util.level_to_icon(status.level))
        self._item.setText(1, status.message)

    def prune(self):
        stale = []
        for child in self._children:
            if not self._children[child].updated:
                stale.append(child)
            else:
                self._children[child].prune()
        if len(stale) > 0:
            for child in stale:
                self._item.removeChild(self._children[child]._item)
                del self._children[child]
        self.updated = False

    def __getitem__(self, key):
        return self._children[key]

    def __setitem__(self, key, value):
        self._children[key] = value
        self._item.addChild(value._item)

    def __contains__(self, key):
        return key in self._children

    def __iter__(self):
        for key in self._children:
            yield key

class RobotMonitorWidget(QWidget):
    """
    NOTE: RobotMonitorWidget.shutdown function needs to be called
    when the instance of this class terminates.

    RobotMonitorWidget itself doesn't store previous diagnostic states.
    It instead delegates that function to Timeline class.
    """

    _sig_tree_nodes_updated = Signal(int)
    _TREE_ALL = 1
    _TREE_WARN = 2
    _TREE_ERR = 3

    message_updated = Signal(DiagnosticArray)

    def __init__(self, context, topic=None):
        """
        :param context: plugin context hook to enable adding widgets as a
                        ROS_GUI pane, 'PluginContext'
        :param topic: Diagnostic topic to subscribe to 'str'
        """

        super(RobotMonitorWidget, self).__init__()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('rqt_robot_monitor'), 'resource',
                               'robotmonitor_mainwidget.ui')
        loadUi(ui_file, self)

        obj_name = 'Robot Monitor'
        self.setObjectName(obj_name)
        self.setWindowTitle(obj_name)

        self._toplevel_statitems = []  # StatusItem
        self._warn_statusitems = []  # StatusItem. Contains ALL DEGREES
                                # (device top level, device' _sub) in parallel
        self._err_statusitems = []  # StatusItem

        if topic:
            self._timeline = Timeline(topic, DiagnosticArray)
            self._timeline.message_updated.connect(self.message_updated)
            self._timeline.message_updated.connect(self.message_cb)

            self.timeline_pane.set_timeline(self._timeline)

            self.vlayout_top.addWidget(self.timeline_pane)
            self.timeline_pane.show()
        else:
            self._timeline = None

        self._inspectors = {}
        # keep a copy of the current message for opening new inspectors
        self._current_msg = None


        self.tree_all_devices.itemDoubleClicked.connect(self._tree_clicked)
        self.warn_flattree.itemDoubleClicked.connect(self._tree_clicked)
        self.err_flattree.itemDoubleClicked.connect(self._tree_clicked)

        self.tree_all_devices.resizeColumnToContents(0)

        self._sig_tree_nodes_updated.connect(self._tree_nodes_updated)


        self._is_stale = True
        self._last_message_time = 0.0

        self._timer = QTimer()
        self._timer.timeout.connect(self._update_message_state)
        self._timer.start(1000)

        palette = self.tree_all_devices.palette()
        self._original_base_color = palette.base().color()
        self._original_alt_base_color = palette.alternateBase().color()

        self._tree = Status(self.tree_all_devices.invisibleRootItem())
        self._warn_tree = Status(self.warn_flattree.invisibleRootItem())
        self._err_tree = Status(self.err_flattree.invisibleRootItem())

    @Slot(DiagnosticArray)
    def message_cb(self, msg):
        self._current_msg = msg

        # Walk the status array and update the tree
        for status in msg.status:
            # Compute path and walk to appropriate subtree
            path = status.name.split('/')
            if path[0] == '':
                path = path[1:]
            tmp_tree = self._tree
            for p in path:
                if not p in tmp_tree:
                    tmp_tree[p] = Status()
                tmp_tree = tmp_tree[p]
            tmp_tree.update(status, util.get_resource_name(status.name))

            # Check for warnings
            if status.level == DiagnosticStatus.WARN:
                name = status.name
                if not name in self._warn_tree:
                    self._warn_tree[name] = Status()
                self._warn_tree[name].update(status, status.name)

            # Check for errors
            if status.level == DiagnosticStatus.ERROR:
                name = status.name
                if not name in self._err_tree:
                    self._err_tree[name] = Status()
                self._err_tree[name].update(status, status.name)

        # For any items in the tree that were not updated, remove them
        self._tree.prune()
        self._warn_tree.prune()
        self._err_tree.prune()

        # TODO(ahendrix): implement
        # Insight: for any item that is not OK, it only provides additional
        #          information if all of it's children are OK
        #
        #          otherwise, it's just an aggregation of its children
        #          and doesn't provide any additional value when added to
        #          the warning and error flat trees

        self.tree_all_devices.resizeColumnToContents(0)
        self.warn_flattree.resizeColumnToContents(0)
        self.err_flattree.resizeColumnToContents(0)

    def resizeEvent(self, evt):
        """Overridden from QWidget"""
        rospy.logdebug('RobotMonitorWidget resizeEvent')
        self.timeline_pane.redraw()

    @Slot(str)
    def _inspector_closed(self, name):
        """ Called when an inspector window is closed """
        if name in self._inspectors:
            del self._inspectors[name]

    def _tree_clicked(self, item, column):
        """
        Slot to QTreeWidget.itemDoubleClicked

        :type item: QTreeWidgetItem
        :type column: int
        """
        rospy.logdebug('RobotMonitorWidget _tree_clicked col=%d', column)
        if item.name in self._inspectors:
            self._inspectors[item.name].activateWindow()
        else:
            self._inspectors[item.name] = InspectorWindow(self, item.name,
                    self._current_msg, self._timeline)
            self._inspectors[item.name].closed.connect(self._inspector_closed)


    def _tree_nodes_updated(self, tree_type):
        tree_obj = None
        if self._TREE_ALL == tree_type:
            tree_obj = self.tree_all_devices
        elif self._TREE_WARN == tree_type:
            tree_obj = self.warn_flattree
        if self._TREE_ERR == tree_type:
            tree_obj = self.err_flattree
        tree_obj.resizeColumnToContents(0)

    def _get_toplevel_diagnosticstat(self, diag_array):
        """
        Return an array that contains DiagnosticStatus only at the top level of
        the given msg.

        :type msg: DiagnosticArray
        :rtype: DiagnosticStatus[]
        """

        ret = []
        for diagnostic_status in diag_array.status:
            if len(diagnostic_status.name.split('/')) == 2:
                rospy.logdebug(" _get_toplevel_diagnosticstat " +
                "TOP lev %s ", diagnostic_status.name)
                ret.append(diagnostic_status)
            else:
                rospy.logdebug(" _get_toplevel_diagnosticstat " +
                               "Not top lev %s ", diagnostic_status.name)
        return ret

    def _update_message_state(self):
        """ Update the display if it's stale """
        # TODO(ahendrix): push staleness detection down into the Timeline
        current_time = rospy.get_time()
        time_diff = current_time - self._last_message_time
        rospy.logdebug('_update_message_state time_diff= %s ' +
                       'self._last_message_time=%s', time_diff,
                       self._last_message_time)

        previous_stale_state = self._is_stale
        if (time_diff > 10.0):
            self.timeline_pane._msg_label.setText("Last message received " +
                                               "%s seconds ago"
                                               % (int(time_diff)))
            self._is_stale = True
        else:
            seconds_string = "seconds"
            if (int(time_diff) == 1):
                seconds_string = "second"
            self.timeline_pane._msg_label.setText(
                 "Last message received %s %s ago" % (int(time_diff),
                                                      seconds_string))
            self._is_stale = False
        if previous_stale_state != self._is_stale:
            self._update_background_color()

    def _update_background_color(self):
        """ Update the background color based on staleness """
        p = self.tree_all_devices.palette()
        if self._is_stale:
            p.setColor(QPalette.Base, Qt.darkGray)
            p.setColor(QPalette.AlternateBase, Qt.lightGray)
        else:
            p.setColor(QPalette.Base, self._original_base_color)
            p.setColor(QPalette.AlternateBase, self._original_alt_base_color)
        self.tree_all_devices.setPalette(p)
        self.warn_flattree.setPalette(p)
        self.err_flattree.setPalette(p)

    def shutdown(self):
        """
        This needs to be called whenever this class terminates.
        This closes all the instances on all trees.
        Also unregisters ROS' subscriber, stops timer.
        """
        rospy.logdebug('RobotMonitorWidget in shutdown')

        names = self._inspectors.keys()
        for name in names:
            self._inspectors[name].close()

        self._timeline.shutdown()

        self._timer.stop()
        del self._timer

    def save_settings(self, plugin_settings, instance_settings):
        instance_settings.set_value('splitter', self.splitter.saveState())
        # TODO(ahendrix): persist the device paths, positions and sizes of any
        #                 inspector windows

    def restore_settings(self, plugin_settings, instance_settings):
        if instance_settings.contains('splitter'):
            self.splitter.restoreState(instance_settings.value('splitter'))
        else:
            self.splitter.setSizes([100, 100, 200])
        # TODO(ahendrix): restore inspector windows
