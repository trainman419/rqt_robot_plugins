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

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from python_qt_binding.QtCore import Signal
from python_qt_binding.QtGui import QTreeWidgetItem
import rospy

import util_robot_monitor as util


class StatusItem(QTreeWidgetItem):
    """
    Represents a single tree node that is capable of holding children objects
    of its class type.
    """

    def __init__(self, status):
        """
        :type status: DiagnosticStatus
        """
        super(StatusItem, self).__init__(QTreeWidgetItem.UserType)

        self._children_statusitems = []
        self.name = status.name
        self.level = status.level
        self.last_level = None
        self.status = status  # DiagnosticsStatus

        self.setText(0, '/' + util.get_resource_name(self.name))

    def get_name(self):
        return self.name

    def update(self, status):
        """
        Replace old status with the passed one.

        :type status: DiagnosticsStatus
        """
        self.status = status

    def update_children(self, status_new, diag_array):
        """
        Recursive for tree node's children.
        Update text on treeWidgetItem, set icon on it.

        :type status: DiagnosticStatus
        :type msg: DiagnosticArray
        """
        self.status = status_new

        children_diag_statuses = util.get_children(self.name, diag_array)

        names_toplevel_local = [s.name for s in self._children_statusitems]
        errors = 0
        warnings = 0
        for child_diagnostic_status in children_diag_statuses:
            name = child_diagnostic_status.name
            device_name = util.get_resource_name(child_diagnostic_status.name)
            if (child_diagnostic_status.level == DiagnosticStatus.ERROR):
                errors = errors + 1
            elif (child_diagnostic_status.level == DiagnosticStatus.WARN):
                warnings = warnings + 1
            rospy.logdebug(' update_children level= %s',
                           child_diagnostic_status.level)

            if name in names_toplevel_local:
                index_child = names_toplevel_local.index(name)
                status_item = self._children_statusitems[index_child]
                # Recursive call.
                status_item.update_children(
                                           child_diagnostic_status, diag_array)
                util.update_status_images(child_diagnostic_status, status_item)
                rospy.logdebug(' StatusItem update 33 index= %d dev_name= %s',
                               index_child, device_name)
                status_item.setText(0, device_name)
                status_item.setText(1, child_diagnostic_status.message)
            elif len(self.strip_child(name).split('/')) <= 2:
                status_item = StatusItem(child_diagnostic_status)
                # Recursive call.
                status_item.update_children(child_diagnostic_status,
                                            diag_array)
                status_item.setText(0, device_name)
                status_item.setText(1, child_diagnostic_status.message)
                self._children_statusitems.append(status_item)
                self.addChild(status_item)

        rospy.logdebug(' ------ Statusitem.update_children err=%d warn=%d',
                       errors, warnings)
        return {util._DICTKEY_TIMES_ERROR: errors,
                util._DICTKEY_TIMES_WARN: warnings}

    def strip_child(self, child):
        return child.replace(self.name, '')

class State(object):
    """
    A container for StatusItem per timeframe (second).

    Copied from robot_monitor.
    """
    def __init__(self):
        self._items = {}  # dict of StatusItem
        self._msg = None
        self._has_warned_no_name = False

    def reset(self):
        self._items = {}
        self._msg = None

    def get_descendants(self, item):
        rospy.logdebug(' Status get_descendants status.name=%s',
                       item.status.name)
        child_keys = [k for k in self._items.iterkeys()
                      if k.startswith(item.status.name + "/")]
        children = [self._items[k] for k in child_keys]
        return children

    def get_items(self):
        return self._items

    def update(self, msg):
        """

        :type msg: DiagnosticArray
        """
        removed = []
        added = []
        items = {}

        # fill items from new msg, creating new StatusItems for any that don't
        # already exist, and keeping track of those that have been added new
        for s in msg.status:
            # DiagnosticStatus messages without a name are invalid #3806
            if not s.name and not self._has_warned_no_name:
                rospy.logwarn('DiagnosticStatus message with no "name". ' +
                              'Unable to add to robot monitor. Message: ' +
                              '%s, hardware ID: %s, level: %d' %
                              (s.message, s.hardware_id, s.level))
                self._has_warned_no_name = True

            if not s.name:
                continue

            if (len(s.name) > 0 and s.name[0] != '/'):
                s.name = '/' + s.name

            if (s.name not in self._items):
                i = StatusItem(s)
                added.append(i)
                items[s.name] = i
            else:
                i = self._items[s.name]
                i.update(s)
                items[s.name] = i

        # find anything without a parent already in the items, and add it as a
        # dummy item
        to_add = []
        dummy_names = []
        for i in items.itervalues():
            parent = i.status.name
            while (len(parent) != 0):
                parent = util.get_parent_name(parent)
                if (len(parent) > 0 and
                    (parent not in items) and
                    parent not in dummy_names):

                    pi = None
                    if (parent not in self._items):
                        s = DiagnosticStatus()
                        s.name = parent
                        s.message = ""
                        pi = StatusItem(s)
                    else:
                        pi = self._items[parent]

                    to_add.append(pi)
                    dummy_names.append(pi.status.name)

        for a in to_add:
            if (a.status.name not in items):
                items[a.status.name] = a

                if (a.status.name not in self._items):
                    added.append(a)

        for i in self._items.itervalues():
            # determine removed items
            if (i.status.name not in items):
                removed.append(i)

        # remove removed items
        for r in removed:
            del self._items[r.status.name]

        self._items = items
        self._msg = msg

        # sort so that parents are always added before children
        added.sort(cmp=lambda l, r: cmp(l.status.name, r.status.name))
        # sort so that children are always removed before parents
        removed.sort(cmp=lambda l,
                     r: cmp(l.status.name, r.status.name),
                     reverse=True)

        return (added, removed, self._items)
