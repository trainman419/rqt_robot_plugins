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

class Status(object):
    def __init__(self):
        super(Status, self).__init__()
        self._children = {}
        self.item = None

    def __getitem__(self, key):
        return self._children[key]

    def __setitem__(self, key, value):
        self._children[key] = value

    def __contains__(self, key):
        return key in self._children

    def __str__(self):
        if len(self._children) > 0:
            return "%s (%s)" % ( self.message, str(self._children) )
        else:
            return self.message

    def __repr__(self):
        return self.__str__()

    def __iter__(self):
        for key in self._children:
            yield key

def TreeStatusItem(QTreeWidgetItem):
    def __init__(self, status):
        super(TreeStatusItem, self).__init__(QTreeWidgetItem.UserType)
        name = util.get_resource_name(status.name)
        statusitem.setText(0, name)
        statusitem.setText(1, status.message)
        statusitem.setIcon(0, util.level_to_icon(status.level))


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

    @Slot(DiagnosticArray)
    def message_cb(self, msg):
        self._current_msg = msg

        # How should this ACTUALLY work?

        # Walk the status array and build a tree data structure
        tree = Status()
        for status in msg.status:
            path = status.name.split('/')
            if path[0] == '':
                path = path[1:]
            tmp_tree = tree
            for p in path:
                if not p in tmp_tree:
                    tmp_tree[p] = Status()
                tmp_tree = tmp_tree[p]
            assert(tmp_tree.item is None)
            tmp_tree.item = TreeStatusItem(status)

        # Clear out the trees
        self.tree_all_devices.clear()

        # Walk the tree and create the main tree
        #  additionally, add any warnings to the warnings tree
        #  and any errors to the errors tree
        for item in tree:
            print item
            if tree[item].item is not None:
                print item
                self.tree_all_devices.addTopLevelItem(tree[item].item)

        #self._update_devices_tree(msg)
        #self._update_warns_errors(msg)

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

    def _update_devices_tree(self, diag_array):
        """
        Update the tree from the bottom

        :type diag_array: DiagnosticArray
        """
        # TODO: 11/5/2012 Currently, in case some devices disappear
        #            while running this program, there's no way to remove
        #            those from the device-tree.

        statusnames_curr_toplevel = [util.get_resource_name(status.name)
                                     for status in self._toplevel_statitems]
        # Only the status variable that pops up at the end is
        # processed by util.get_resource_name.

        for status_new in self._get_toplevel_diagnosticstat(diag_array):
            name = util.get_resource_name(status_new.name)
            rospy.logdebug('_update_devices_tree 0 name @ toplevel %s', name)
            dict_status = 0
            if name in statusnames_curr_toplevel:  # No change of names
                                                # in toplevel since last time.
                statusitem = self._toplevel_statitems[
                                        statusnames_curr_toplevel.index(name)]

                dict_status = statusitem.update_children(status_new,
                                                         diag_array)
                times_errors = dict_status[util._DICTKEY_TIMES_ERROR]
                times_warnings = dict_status[util._DICTKEY_TIMES_WARN]
                util.update_status_images(status_new, statusitem)

                base_text = util.get_resource_name(statusitem.status.name)

                if (times_errors > 0 or times_warnings > 0):
                    base_text = "(Err: %s, Wrn: %s) %s %s" % (
                                   times_errors,
                                   times_warnings,
                                   util.get_resource_name(status_new.name),
                                   status_new.message)
                    rospy.logdebug('_update_dev_tree 1 text to show=%s',
                                   base_text)
                    statusitem.setText(0, base_text)
                    statusitem.setText(1, status_new.message)
                else:
                    rospy.logdebug('_update_dev_tree 2 text to show=%s',
                                   base_text)
                    statusitem.setText(0, base_text)
                    statusitem.setText(1, 'OK')

            else:
                new_status_item = StatusItem(status_new)

                new_status_item.update_children(status_new,
                                                diag_array)

                # Figure out if a statusitem and its subtree contains errors.
                # new_status_item.setIcon(0, self._error_icon)
                # This shows NG icon at the beginning of each statusitem.
                util.update_status_images(status_new,
                                           new_status_item)

                self._toplevel_statitems.append(new_status_item)

                rospy.logdebug(' _update_devices_tree 2 ' +
                               'status_new.name %s',
                               new_status_item.name)
                self.tree_all_devices.addTopLevelItem(new_status_item)

        self._sig_tree_nodes_updated.emit(self._TREE_ALL)

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

    def _update_warns_errors(self, diag_array):
        """
        Update the warning and error trees.

        Unlike _update_devices_tree function where all DiagnosticStatus
        need to be inspected constantly, this function is used in a trial
        to reduce unnecessary inspection of the status level for all
        DiagnosticStatus contained in the incoming DiagnosticArray msg.

        :type msg: DiagnosticArray
        """

        self._update_flat_tree(diag_array)

    def _update_flat_tree(self, diag_arr):
        """
        Update the given flat tree (ie. tree that doesn't show children nodes -
        all of its elements will be shown on top level) with
        all the DiagnosticStatus instances contained in the given
        DiagnosticArray, regardless of the level of the device in a device
        category.

        For both warn / error trees, StatusItem instances are newly generated.

        :type diag_arr: DiagnosticArray
        """

        for diag_stat_new in diag_arr.status:
            # Num of loops here should be equal to the num of the top
            # DiagnosticStatus item. Ex. in PR2, 9 or so.

            stat_lv_new = diag_stat_new.level
            dev_name = diag_stat_new.name
            correspondent_warn_curr = util.get_correspondent(
                                         util.get_resource_name(dev_name),
                                         self._warn_statusitems)
            dev_index_warn_curr = correspondent_warn_curr[util._DICTKEY_INDEX]
            rospy.logdebug(' dev_index_warn_curr=%s dev_name=%s',
                           dev_index_warn_curr, dev_name)
            correspondent_err_curr = util.get_correspondent(
                                          util.get_resource_name(dev_name),
                                          self._err_statusitems)
            dev_index_err_curr = correspondent_err_curr[util._DICTKEY_INDEX]
            headline = "%s" % diag_stat_new.name
            if DiagnosticStatus.OK == stat_lv_new:
                if 0 <= dev_index_warn_curr:
                    rospy.logdebug('dev_index_warn_curr=%s name=%s, ' +
                                   'stat_lv_new=%d', dev_index_warn_curr,
                                   dev_name, stat_lv_new)
                    statitem_curr = self._get_statitem(dev_index_warn_curr,
                                                       self._warn_statusitems,
                                                       self.warn_flattree, 1)
                elif 0 <= dev_index_err_curr:
                    statitem_curr = self._get_statitem(dev_index_err_curr,
                                                       self._err_statusitems,
                                                       self.err_flattree, 1)
            elif DiagnosticStatus.WARN == stat_lv_new:
                statitem = None
                if 0 <= dev_index_err_curr:
                    # If the corresponding statusitem is in error tree,
                    # move it to warn tree.
                    statitem = self._get_statitem(dev_index_err_curr,
                                                  self._err_statusitems,
                                                  self.err_flattree)
                    self._add_statitem(statitem, self._warn_statusitems,
                                      self.warn_flattree, headline,
                                      diag_stat_new.message, stat_lv_new)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    # If the corresponding statusitem isn't found,
                    # create new obj.
                    statitem = StatusItem(diag_stat_new)
                    self._add_statitem(statitem, self._warn_statusitems,
                                      self.warn_flattree, headline,
                                      diag_stat_new.message, stat_lv_new)
                    self._warn_statusitems.append(statitem)
                elif (0 < dev_index_warn_curr):
                    # If the corresponding statusitem is already in warn tree,
                    # obtain the instance.
                    statitem = self._get_statitem(dev_index_warn_curr,
                                                  self._warn_statusitems)

                if statitem:  # If not None
                    # Updating statusitem will keep popup window also update.
                    statitem.update_children(diag_stat_new,
                                                           diag_arr)
            elif ((DiagnosticStatus.ERROR == stat_lv_new) or
                  (DiagnosticStatus.STALE == stat_lv_new)):
                statitem = None
                if 0 <= dev_index_warn_curr:
                    # If the corresponding statusitem is in warn tree,
                    # move it to err tree.
                    statitem = self._get_statitem(dev_index_warn_curr,
                                                  self._warn_statusitems,
                                                  self.warn_flattree)
                    self._add_statitem(statitem, self._err_statusitems,
                                      self.err_flattree, headline,
                                      diag_stat_new.message, stat_lv_new)
                elif (0 <= dev_index_err_curr):
                    # If the corresponding statusitem is already in err tree,
                    # obtain the instance.
                    statitem = self._get_statitem(dev_index_err_curr,
                                                  self._err_statusitems)
                elif (dev_index_warn_curr < 0 and dev_index_err_curr < 0):
                    # If the corresponding statusitem isn't found,
                    # create new obj.
                    statitem = StatusItem(diag_stat_new)
                    self._add_statitem(statitem, self._err_statusitems,
                                      self.err_flattree, headline,
                                      diag_stat_new.message, stat_lv_new)

                if statitem:  # If not None
                    # Updating statusitem will keep popup window also update.
                    statitem.update_children(diag_stat_new,
                                                           diag_arr)

        self._sig_tree_nodes_updated.emit(self._TREE_WARN)
        self._sig_tree_nodes_updated.emit(self._TREE_ERR)

    def _add_statitem(self, statusitem, statitem_list,
                      tree, headline, statusmsg, statlevel):
        """
        Setup a status item and add it as a toplevel item in the given tree
        """
        if 'Warning' == statusmsg or 'Error' == statusmsg:
            return

        statusitem.setText(0, headline)
        statusitem.setText(1, statusmsg)
        statusitem.setIcon(0, util.level_to_icon(statlevel))
        statitem_list.append(statusitem)
        tree.addTopLevelItem(statusitem)
        rospy.logdebug(' _add_statitem statitem_list length=%d',
                       len(statitem_list))

    def _get_statitem(self, item_index, item_list, tree=None, mode=2):
        """
        Get a status item; optionally removing it from the tree

        :param mode: 1 = remove from given list, 2 = w/o removing.
        """
        statitem_existing = item_list[item_index]
        if 1 == mode:
            tree.takeTopLevelItem(tree.indexOfTopLevelItem(statitem_existing))
            item_list.pop(item_index)
        return statitem_existing

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


    def _clear(self):
        rospy.logdebug(' RobotMonitorWidget _clear called ')
        self.err_flattree.clear()
        self.warn_flattree.clear()

